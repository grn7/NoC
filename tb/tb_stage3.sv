// tb/tb_stage3.sv
// Stage 3 — Global conservation scoreboard
//
// Architecture shift from Stages 1/2:
//   Stages 1–2: one flit at a time, exact-cycle latency assertions.
//   Stage 3:    multiple flits in flight, latency is a LOWER BOUND.
//               Pass/fail based on destination correctness and conservation
//               (no flit lost, none duplicated), not on exact arrival cycle.
//               Deflection = arrives later than minimal hops → OK, logged.
//               Misrouting = arrives at wrong PE → FAIL.
//               Lost flit = pending set non-empty at end of drain → FAIL.
//
// Scoreboard mechanism:
//   - inject_flit() records {src, dest, inject_cycle} in pending[tag].
//   - tag is the flit's 32-bit data field (monotonically increasing int).
//   - Ejection monitor (always @posedge clk) detects all ejections, looks
//     up tag, verifies destination, removes from pending, writes to CSV.
//   - wait_drain() loops until pending is empty or timeout fires.
//
// CSV output (../reports/stage3_flits.csv):
//   tag,src_x,src_y,dest_x,dest_y,eject_x,eject_y,
//   inject_cycle,eject_cycle,hop_count,min_hops,status
//   status: OK_DIRECT | OK_DEFLECTED | WRONG_DEST
//
// Test progression (low to high concurrency):
//   D1: Two disjoint flits, same cycle — sanity-checks the scoreboard itself.
//   D2: Two flits meeting at the same router in the same cycle (contention).
//       First genuine multi-router deflection event in the test history.
//   D3: Four scattered flits across a 2-cycle burst — moderate concurrency.
//   D4: Sustained random traffic for 50 cycles (~25% injection rate per PE).
//       First real empirical deflection-rate dataset for the torus.
//
// Interface-array indexing rule (same as Stages 1/2):
//   pe_if[x][y].signal cannot be indexed with runtime int variables.
//   All pe_if access goes through plain logic/flit_t extraction arrays.

`timescale 1ns/1ps

module tb_stage3;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Flit scoreboard record — stored in associative array keyed by tag
    // ------------------------------------------------------------------
    typedef struct {
        int src_x, src_y;
        int dest_x, dest_y;
        int inject_cycle;
    } flit_record_t;

    // ------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // 2D PE port interface array
    // ------------------------------------------------------------------
    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    // ------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------
    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    // ------------------------------------------------------------------
    // Injection driver arrays + connection to pe_if
    // ------------------------------------------------------------------
    logic  inj_valid_drv [TORUS_X][TORUS_Y];
    flit_t inj_flit_drv  [TORUS_X][TORUS_Y];

    generate
        genvar dx, dy;
        for(dx = 0; dx < TORUS_X; dx++) begin : g_drv_x
            for(dy = 0; dy < TORUS_Y; dy++) begin : g_drv_y
                assign pe_if[dx][dy].inj_valid = inj_valid_drv[dx][dy];
                assign pe_if[dx][dy].inj_flit  = inj_flit_drv[dx][dy];
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Ejection and inj_ready extraction arrays
    // ------------------------------------------------------------------
    logic  ej_valid_arr  [TORUS_X][TORUS_Y];
    flit_t ej_flit_arr   [TORUS_X][TORUS_Y];
    logic  inj_ready_arr [TORUS_X][TORUS_Y];

    generate
        genvar ex, ey;
        for(ex = 0; ex < TORUS_X; ex++) begin : g_ej_x
            for(ey = 0; ey < TORUS_Y; ey++) begin : g_ej_y
                assign ej_valid_arr[ex][ey]  = pe_if[ex][ey].ej_valid;
                assign ej_flit_arr[ex][ey]   = pe_if[ex][ey].ej_flit;
                assign inj_ready_arr[ex][ey] = pe_if[ex][ey].inj_ready;
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Scoreboard and counters
    // ------------------------------------------------------------------
    flit_record_t pending [int];   // keyed by tag (= flit data field)
    int next_tag      = 1;         // monotonic tag allocator
    int total_injected = 0;
    int total_ejected  = 0;
    int total_correct  = 0;
    int pass_count     = 0;
    int fail_count     = 0;

    // ------------------------------------------------------------------
    // Simulation cycle counter
    // Combined with ejection monitor in one always block so sim_cycle is
    // guaranteed incremented (blocking) before the monitor reads it.
    // ------------------------------------------------------------------
    int sim_cycle = 0;

    // ------------------------------------------------------------------
    // CSV file handle — opened in initial block before reset
    // ------------------------------------------------------------------
    int csv_fd;

    // ------------------------------------------------------------------
    // Module-level temporaries for the ejection monitor.
    // Declared here (not inside always) to avoid static-storage ambiguity.
    // The always block is non-reentrant so this is safe.
    // ------------------------------------------------------------------
    int           mon_tag;
    int           mon_eject_cyc;
    int           mon_hops;
    int           mon_min_hops;
    flit_record_t mon_rec;
    logic         mon_dest_ok;
    string        mon_status;

    // ------------------------------------------------------------------
    // Minimum torus hop count — correct Manhattan distance on a torus
    // accounting for wraparound in both dimensions.
    // ------------------------------------------------------------------
    function automatic int torus_min_hops(int sx, sy, dx, dy);
        int dist_x, dist_y, min_dx, min_dy;
        dist_x = (dx >= sx) ? (dx - sx) : (sx - dx);
        dist_y = (dy >= sy) ? (dy - sy) : (sy - dy);
        min_dx = (dist_x <= TORUS_X - dist_x) ? dist_x : (TORUS_X - dist_x);
        min_dy = (dist_y <= TORUS_Y - dist_y) ? dist_y : (TORUS_Y - dist_y);
        return min_dx + min_dy;
    endfunction

    // ------------------------------------------------------------------
    // Ejection monitor — runs every posedge, checks all 16 PE ports.
    //
    // Timing note: hop_count = eject_cycle - inject_cycle - 1
    //   inject_cycle is recorded at sim_cycle value AFTER the previous
    //   posedge (#1 after @posedge clk in initial block). Subtracting 1
    //   corrects for this offset so hop_count == actual network hops.
    //
    // Deflection: hop_count > min_hops → flit took a longer-than-minimal
    //   path due to contention. This is correct behaviour, NOT a failure.
    //   The scoreboard records it as OK_DEFLECTED in the CSV.
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        // Increment cycle counter first (blocking — visible to all code
        // that follows in this same always block execution)
        sim_cycle = sim_cycle + 1;
        #1;  // let this edge's NBA updates (router_core's output registers) settle
             // before sampling ej_valid_arr — otherwise this block races the
             // DUT's own nonblocking FF update on the same edge

        if(rst_n) begin
            for(int pi = 0; pi < TORUS_X; pi++) begin
                for(int pj = 0; pj < TORUS_Y; pj++) begin
                    if(ej_valid_arr[pi][pj]) begin
                        mon_tag       = int'(ej_flit_arr[pi][pj].data);
                        mon_eject_cyc = sim_cycle;

                        if(pending.exists(mon_tag)) begin
                            mon_rec     = pending[mon_tag];
                            mon_dest_ok = (pi == mon_rec.dest_x) && (pj == mon_rec.dest_y);
                            mon_hops    = mon_eject_cyc - mon_rec.inject_cycle - 1;
                            mon_min_hops = torus_min_hops(mon_rec.src_x, mon_rec.src_y,
                                                          mon_rec.dest_x, mon_rec.dest_y);

                            if(!mon_dest_ok) begin
                                mon_status = "WRONG_DEST";
                                $display("  [FAIL] Tag=%0d ejected at PE(%0d,%0d), expected PE(%0d,%0d)",
                                         mon_tag, pi, pj, mon_rec.dest_x, mon_rec.dest_y);
                                fail_count++;
                            end else if(mon_hops > mon_min_hops) begin
                                mon_status = "OK_DEFLECTED";
                                $display("  [EJT ] Tag=%0d PE(%0d,%0d)→PE(%0d,%0d) hops=%0d (min=%0d) [DEFLECTED — correct]",
                                         mon_tag, mon_rec.src_x, mon_rec.src_y,
                                         pi, pj, mon_hops, mon_min_hops);
                                total_correct++;
                            end else begin
                                mon_status = "OK_DIRECT";
                                $display("  [EJT ] Tag=%0d PE(%0d,%0d)→PE(%0d,%0d) hops=%0d",
                                         mon_tag, mon_rec.src_x, mon_rec.src_y,
                                         pi, pj, mon_hops);
                                total_correct++;
                            end

                            $fdisplay(csv_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0s",
                                     mon_tag,
                                     mon_rec.src_x,  mon_rec.src_y,
                                     mon_rec.dest_x, mon_rec.dest_y,
                                     pi, pj,
                                     mon_rec.inject_cycle, mon_eject_cyc,
                                     mon_hops, mon_min_hops,
                                     mon_status);

                            pending.delete(mon_tag);
                            total_ejected++;

                        end else begin
                            $display("  [WARN] Ejection of untracked tag=%0d at PE(%0d,%0d) — spurious flit",
                                     mon_tag, pi, pj);
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Helper functions and tasks
    // ------------------------------------------------------------------
    task automatic check(string name, logic cond);
        if(cond) begin
            $display("  [PASS] %s", name);
            pass_count++;
        end else begin
            $display("  [FAIL] %s", name);
            fail_count++;
        end
    endtask

    function automatic flit_t make_flit(
        logic [X_BITS-1:0]     xd,
        logic [Y_BITS-1:0]     yd,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = xd;
        make_flit.Y_dest = yd;
        make_flit.data   = dat;
    endfunction

    // inject_flit — records flit in scoreboard and drives injection port.
    // Does NOT advance simulation time. Caller must @(posedge clk) after.
    task automatic inject_flit(int sx, sy, destx, desty);
        int tag;
        tag = next_tag++;
        pending[tag].src_x        = sx;
        pending[tag].src_y        = sy;
        pending[tag].dest_x       = destx;
        pending[tag].dest_y       = desty;
        pending[tag].inject_cycle = sim_cycle;
        inj_valid_drv[sx][sy]     = 1'b1;
        inj_flit_drv[sx][sy]      = make_flit(X_BITS'(destx), Y_BITS'(desty), DATA_WIDTH'(tag));
        total_injected++;
        $display("    [INJ ] Tag=%0d PE(%0d,%0d)→PE(%0d,%0d)  min_hops=%0d",
                 tag, sx, sy, destx, desty,
                 torus_min_hops(sx, sy, destx, desty));
    endtask

    // release_injectors — clears all injection drivers after a posedge.
    task automatic release_injectors();
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
    endtask

    // wait_drain — spins until pending set is empty or timeout fires.
    // Conservation check: pending.num() == 0 means every injected flit
    // was ejected exactly once at the correct destination.
    task automatic wait_drain(int timeout_cycles, string test_name);
        int t;
        t = 0;
        while(pending.num() > 0 && t < timeout_cycles) begin
            @(posedge clk); #1;
            t++;
        end
        check($sformatf("%s: conservation — all %0d pending flits ejected",
              test_name, total_injected),
              pending.num() == 0);
        if(pending.num() != 0)
            $display("    %0d flit(s) still pending after %0d cycles (LOST)",
                     pending.num(), timeout_cycles);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin

        // Initialise all injection drivers to 0
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        // Open CSV before reset so the header is always the first line
        csv_fd = $fopen("../reports/stage3_flits.csv", "w");
        $fdisplay(csv_fd,
                  "tag,src_x,src_y,dest_x,dest_y,eject_x,eject_y,inject_cycle,eject_cycle,hop_count,min_hops,status");

        $display("=== tb_stage3: Global Conservation Scoreboard ===");
        $display("    Torus: %0d x %0d  |  CSV: ../reports/stage3_flits.csv", TORUS_X, TORUS_Y);

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        #1;
        $display("Reset released.\n");

        // ==============================================================
        // D1: Two non-interfering flits, injected in the same cycle
        //
        // Flit A: (0,0)→(2,0)  path: East(tie)→East, 2 hops, bottom row
        // Flit B: (3,3)→(1,3)  path: East(wrap tie)→East, 2 hops, top row
        //
        // Paths are in different rows (y=0 vs y=3) and share no routers.
        // Purpose: confirm the scoreboard itself works correctly with two
        // simultaneous in-flight flits before using it to find real bugs.
        // ==============================================================
        $display("====== D1: Two non-interfering flits (disjoint paths) ======");
        inject_flit(0, 0, 2, 0);   // bottom row: (0,0)→East→(1,0)→East→(2,0)
        inject_flit(3, 3, 1, 3);   // top row:    (3,3)→East(wrap)→(0,3)→East→(1,3)
        @(posedge clk); #1;
        release_injectors();
        wait_drain(20, "D1");
        repeat(2) @(posedge clk); #1; // brief idle between tests

        // ==============================================================
        // D2: Two flits meeting at router (0,2) in the same cycle
        //
        // Flit A: (0,1)→(0,3)  path: North→(0,2)→North→(0,3), 2 hops min
        //   At (0,1): dest_x=0=cx → dest_y=3>cy=1, fwd_y=2=bwd_y → North
        //   At (0,2): dest_x=0=cx → dest_y=3>cy=2, fwd_y=1 → North
        //
        // Flit B: (1,2)→(0,3)  path: West→(0,2)→North→(0,3), 2 hops min
        //   At (1,2): dest_x=0 < cx=1, fwd_x=3, bwd_x=1 → West
        //   At (0,2): dest_x=0=cx → dest_y=3>cy=2, fwd_y=1 → North
        //
        // Both A and B arrive at router (0,2) after exactly 1 hop, both
        // wanting the North output — guaranteed contention at (0,2).
        // The deflect_resolver grants North to one, deflects the other.
        // Both must still arrive at (0,3): conservation must hold.
        // The deflected flit will show up as OK_DEFLECTED in the CSV.
        // Stage3 note: do NOT assert exact-cycle latency here — one flit
        // will take more than 2 hops due to the deflection.
        // ==============================================================
        $display("\n====== D2: Contention at router (0,2) — two flits both want North ======");
        inject_flit(0, 1, 0, 3);   // A: arrives at (0,2) from South
        inject_flit(1, 2, 0, 3);   // B: arrives at (0,2) from East
        @(posedge clk); #1;
        release_injectors();
        wait_drain(30, "D2");      // generous timeout: deflected flit takes longer
        repeat(2) @(posedge clk); #1;

        // ==============================================================
        // D3: Four scattered flits across a 2-cycle injection burst
        //
        // Cycle 1: (0,0)→(3,3)  2 hops [West-wrap + South-wrap]
        //          (3,2)→(0,2)  1 hop  [East-wrap]
        // Cycle 2: (1,1)→(3,1)  2 hops [East tie + East]
        //          (0,3)→(2,1)  4 hops [East tie + East + North-wrap + North]
        //
        // Paths cross in places, some contention possible. Run conservation.
        // ==============================================================
        $display("\n====== D3: Four scattered flits, 2-cycle burst ======");

        // Cycle 1 injections
        inject_flit(0, 0, 3, 3);   // (0,0)→West(3,0)→South-wrap(3,3)
        inject_flit(3, 2, 0, 2);   // (3,2)→East-wrap(0,2)
        @(posedge clk); #1;
        release_injectors();
        repeat(1) @(posedge clk); #1; // one idle cycle between bursts

        // Cycle 2 injections
        inject_flit(1, 1, 3, 1);   // (1,1)→East-tie(2,1)→East(3,1)
        inject_flit(0, 3, 2, 1);   // (0,3)→East-tie(1,3)→East(2,3)→North-wrap(2,0)→North(2,1)
        @(posedge clk); #1;
        release_injectors();

        wait_drain(40, "D3");
        repeat(2) @(posedge clk); #1;

        // ==============================================================
        // D4: Sustained random traffic — 50 cycles at ~25% injection rate
        //
        // Each cycle: for each PE, inject with 25% probability (if inj_ready).
        // Injection skipped silently if inj_ready=0 — inj_ready is purely
        // combinational (depends only on transit load, not on inj_valid),
        // so checking it before inject_flit is safe and accurate.
        //
        // Self-destined injection is avoided (wraps dest_x to (rx+1)%TORUS_X).
        //
        // Expected total: ~50 cycles × 16 PEs × 25% × ~95% ready = ~190 flits
        // (actual count printed in summary).
        //
        // This is the first empirical deflection-rate dataset for the torus.
        // The CSV output gives per-flit hop_count vs min_hops for analysis.
        // Latency is NOT asserted here — only conservation and destination
        // correctness are checked, consistent with Stage 3 semantics.
        // ==============================================================
        $display("\n====== D4: Sustained random traffic (50 cycles, ~25%% injection rate) ======");
        begin
            int rdx, rdy;
            for(int cyc = 0; cyc < 50; cyc++) begin
                for(int ri = 0; ri < TORUS_X; ri++) begin
                    for(int rj = 0; rj < TORUS_Y; rj++) begin
                        // ~25% injection probability, only if PE port is ready
                        if($urandom_range(0, 3) == 0 && inj_ready_arr[ri][rj]) begin
                            // Random destination — prevent self-injection
                            rdx = $urandom_range(0, TORUS_X - 1);
                            rdy = $urandom_range(0, TORUS_Y - 1);
                            if(rdx == ri && rdy == rj)
                                rdx = (ri + 1) % TORUS_X; // bump X to avoid self
                            inject_flit(ri, rj, rdx, rdy);
                        end
                    end
                end
                @(posedge clk); #1;
                release_injectors();
            end
        end

        // Drain with generous timeout — allow deflected flits to complete
        wait_drain(100, "D4");

        // ==============================================================
        // Final summary
        // ==============================================================
        $fclose(csv_fd);

        $display("\n==========================================");
        $display("  STAGE 3 SUMMARY");
        $display("  Injected : %0d", total_injected);
        $display("  Ejected  : %0d  (%0d correct destination)", total_ejected, total_correct);
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if(fail_count == 0)
            $display("  ALL STAGE 3 TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above and CSV.");

        $finish;
    end

endmodule
