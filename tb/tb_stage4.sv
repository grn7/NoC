// tb/tb_stage4.sv  (REVISED — stage4_fix.txt applied)
// Stage 4 — Multiple simultaneous flits, moderate load
//
// Fixes applied:
//
// Fix 1 — PRNG: $urandom_range replaced with xorshift32
//   $urandom_range in XSIM produced a period-15 repeating pattern that gave
//   ~56% injection rate against a 50% target — a 6-sigma deviation, impossible
//   for a fair coin. Replaced with a seeded xorshift32 PRNG controlled by
//   PRNG_SEED parameter. Seed printed in log and CSV header for reproducibility.
//   10k-coin-flip validation block printed before tests.
//
// Fix 2 — Deflection metrics: two separate counters
//   Previous "deflection rate" counted only flits with hop_count > min_hops
//   (the "excess-hop rate"). A flit deflected to an alternative minimal path
//   (E1 tag 5: deflected South at (0,0), but South is also a min-hop direction
//   because torus Y-distance=2 is a tie) is missed by this metric.
//
//   Two metrics now tracked:
//     total_excess_hop      — flits with hop_count > min_hops (latency impact)
//     total_deflection_events — sum of deflect_resolver.deflected[i] bits across
//                               all 16 routers every cycle (true event count)
//
//   defl_arr[TORUS_X][TORUS_Y] extracts the internal RTL deflected[3:0] signals
//   via genvar-based continuous assigns — the same pattern used for ej_valid_arr
//   so that runtime int indices can be used safely in the always block.
//   Hierarchy: dut.gen_col[gx].gen_row[gy].u_router.u_resolver.deflected
//
// Fix 3 — Hop count statistics
//   All mon_hops values pushed to hop_counts[$] queue.
//   At end: sort ascending, report min/max/99th-percentile.
//   Max and p99 are the first empirical data points for the Phase 10
//   worst-case latency bound analysis.
//
// CSV header now includes PRNG seed line.
// E3 inj_ready handshake unchanged (the Stage 3 D4 fix carried over correctly).

`timescale 1ns/1ps

module tb_stage4;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam logic [31:0] PRNG_SEED = 32'hDEAD_BEEF; // change to vary run
    localparam int          E3_CYCLES = 150;

    // ------------------------------------------------------------------
    // Flit scoreboard record
    // ------------------------------------------------------------------
    typedef struct {
        int src_x,  src_y;
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
    // Injection driver arrays
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
    // Deflection-event extraction — same genvar pattern as ej_valid_arr.
    // defl_arr[x][y] mirrors deflect_resolver.deflected[3:0] for router (x,y).
    // deflected[i]=1 means transit flit i was deflected in the current cycle.
    // Using continuous assign with constant genvar indices so that the
    // always block can index defl_arr with runtime int variables safely.
    //
    // Hierarchy: dut.gen_col[gx].gen_row[gy].u_router.u_resolver.deflected
    //   gen_col / gen_row  : generate loop labels in torus_top.sv
    //   u_router           : router_core instance name in gen_col/gen_row
    //   u_resolver         : deflect_resolver instance name in router_core
    //   deflected          : logic [3:0] module-level signal in deflect_resolver
    // ------------------------------------------------------------------
    logic [3:0] defl_arr [TORUS_X][TORUS_Y];

    generate
        genvar gx, gy;
        for(gx = 0; gx < TORUS_X; gx++) begin : g_defl_x
            for(gy = 0; gy < TORUS_Y; gy++) begin : g_defl_y
                assign defl_arr[gx][gy] =
                    dut.gen_col[gx].gen_row[gy].u_router.u_resolver.deflected;
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Scoreboard and counters
    // ------------------------------------------------------------------
    flit_record_t pending [int];
    int next_tag               = 1;
    int total_injected         = 0;
    int total_ejected          = 0;
    int total_correct          = 0;
    // excess-hop rate: flits with hop_count > min_hops (latency-relevant)
    int total_excess_hop       = 0;
    // true deflection count: sum of RTL deflected[] bits across all routers/cycles
    int total_deflection_events = 0;
    // per-input-port deflection breakdown — reveals priority bias.
    // deflected[0]=SRC_N, [1]=SRC_S, [2]=SRC_E, [3]=SRC_W.
    // With N>S>E>W priority, West-input flits are deflected most often;
    // i.e. East-going flits entering the next router's west_in are disadvantaged.
    int defl_from_n            = 0;  // North-input flits deflected
    int defl_from_s            = 0;  // South-input flits deflected
    int defl_from_e            = 0;  // East-input flits deflected
    int defl_from_w            = 0;  // West-input flits deflected (expected highest)
    // injection attempt count before inj_ready check — for attempted vs achieved display
    int total_attempted        = 0;
    int pass_count             = 0;
    int fail_count             = 0;

    // ------------------------------------------------------------------
    // Hop count queue for statistics (min/max/p99 at end of run)
    // ------------------------------------------------------------------
    int hop_counts [$];

    // ------------------------------------------------------------------
    // xorshift32 PRNG — seeded from PRNG_SEED.
    // prng_state is module-level (static). prng_next() modifies it on every
    // call. Called only from the initial block, so no concurrent-access issue.
    // ------------------------------------------------------------------
    logic [31:0] prng_state;

    function automatic logic [31:0] prng_next();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

    // ------------------------------------------------------------------
    // Simulation cycle counter and CSV handle
    // ------------------------------------------------------------------
    int sim_cycle = 0;
    int csv_fd;

    // ------------------------------------------------------------------
    // Module-level temporaries for ejection monitor
    // ------------------------------------------------------------------
    int           mon_tag;
    int           mon_eject_cyc;
    int           mon_hops;
    int           mon_min_hops;
    flit_record_t mon_rec;
    logic         mon_dest_ok;
    string        mon_status;

    // ------------------------------------------------------------------
    // Minimum torus Manhattan distance
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
    // Combined always block:
    //   1. sim_cycle++ (blocking — runs before anything else in this block)
    //   2. #1 — lets router_core's nonblocking FF updates AND deflect_resolver's
    //      always_comb re-evaluation complete before sampling any DUT signals.
    //   3. Count deflection events: $countones over defl_arr, all 16 routers.
    //      ($countones fallback if unsupported: bit-by-bit loop over [3:0])
    //   4. Ejection monitor: tag lookup, classification, CSV write.
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        sim_cycle = sim_cycle + 1;
        #1; // wait for NBA region + always_comb re-evaluation

        if(rst_n) begin

            // --- Deflection event counting (RTL flags) ---
            for(int cx = 0; cx < TORUS_X; cx++) begin
                for(int cy = 0; cy < TORUS_Y; cy++) begin
                    total_deflection_events += $countones(defl_arr[cx][cy]);
                    defl_from_n += int'(defl_arr[cx][cy][0]); // SRC_N input deflected
                    defl_from_s += int'(defl_arr[cx][cy][1]); // SRC_S input deflected
                    defl_from_e += int'(defl_arr[cx][cy][2]); // SRC_E input deflected
                    defl_from_w += int'(defl_arr[cx][cy][3]); // SRC_W input deflected
                end
            end

            // --- Ejection monitor ---
            for(int pi = 0; pi < TORUS_X; pi++) begin
                for(int pj = 0; pj < TORUS_Y; pj++) begin
                    if(ej_valid_arr[pi][pj]) begin
                        mon_tag       = int'(ej_flit_arr[pi][pj].data);
                        mon_eject_cyc = sim_cycle;

                        if(pending.exists(mon_tag)) begin
                            mon_rec      = pending[mon_tag];
                            mon_dest_ok  = (pi == mon_rec.dest_x) && (pj == mon_rec.dest_y);
                            mon_hops     = mon_eject_cyc - mon_rec.inject_cycle - 1;
                            mon_min_hops = torus_min_hops(mon_rec.src_x, mon_rec.src_y,
                                                          mon_rec.dest_x, mon_rec.dest_y);

                            hop_counts.push_back(mon_hops); // for statistics

                            if(!mon_dest_ok) begin
                                mon_status = "WRONG_DEST";
                                $display("  [FAIL] Tag=%0d ejected at PE(%0d,%0d), expected PE(%0d,%0d)",
                                         mon_tag, pi, pj, mon_rec.dest_x, mon_rec.dest_y);
                                fail_count++;
                            end else if(mon_hops > mon_min_hops) begin
                                mon_status = "OK_DEFLECTED";
                                $display("  [EJT ] Tag=%0d PE(%0d,%0d)→PE(%0d,%0d) hops=%0d (min=%0d) [excess-hop]",
                                         mon_tag, mon_rec.src_x, mon_rec.src_y,
                                         pi, pj, mon_hops, mon_min_hops);
                                total_correct++;
                                total_excess_hop++;
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
                            $display("  [WARN] Ejection of untracked tag=%0d at PE(%0d,%0d)",
                                     mon_tag, pi, pj);
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Helper tasks and functions
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

    // inject_flit — deterministic injection for E1 and E2.
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
        $display("    [INJ ] Tag=%0d PE(%0d,%0d)→PE(%0d,%0d) min_hops=%0d",
                 tag, sx, sy, destx, desty,
                 torus_min_hops(sx, sy, destx, desty));
    endtask

    task automatic release_injectors();
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
    endtask

    task automatic wait_drain(int timeout_cycles, string test_name);
        int t;
        t = 0;
        while(pending.num() > 0 && t < timeout_cycles) begin
            @(posedge clk); #1;
            t++;
        end
        check($sformatf("%s: conservation — all pending flits ejected", test_name),
              pending.num() == 0);
        if(pending.num() != 0)
            $display("    %0d flit(s) still pending after %0d cycles (LOST)",
                     pending.num(), timeout_cycles);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin

        // Init all injection drivers
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        // Seed PRNG before anything else
        prng_state = PRNG_SEED;

        // Open CSV — seed in header so data and RNG state are co-located
        csv_fd = $fopen("../reports/stage4_flits.csv", "w");
        $fdisplay(csv_fd, "# Stage 4  |  PRNG seed: 0x%08h  |  E3 cycles: %0d",
                  PRNG_SEED, E3_CYCLES);
        $fdisplay(csv_fd,
                  "tag,src_x,src_y,dest_x,dest_y,eject_x,eject_y,inject_cycle,eject_cycle,hop_count,min_hops,status");

        $display("=== tb_stage4: Multiple Simultaneous Flits, Moderate Load (REVISED) ===");
        $display("    Torus: %0d x %0d  |  PRNG seed: 0x%08h  |  E3 cycles: %0d",
                 TORUS_X, TORUS_Y, PRNG_SEED, E3_CYCLES);

        // --- PRNG validation ---
        // Run 10k coin flips (prng_next()[0]) before the main tests.
        // A real xorshift32 should land between 4750 and 5250 (±5% of 5000).
        // This also consumes 10k PRNG steps before E3, giving the stream
        // a warm-up period that helps avoid short-period startup artifacts.
        begin
            int ones;
            logic [31:0] pv;
            ones = 0;
            for(int k = 0; k < 10000; k++) begin
                pv = prng_next();
                if(pv[0]) ones++;
            end
            $display("PRNG validation: %0d/10000 ones  (range [4750,5250] = ±5%% of 5000)", ones);
            if(ones < 4750 || ones > 5250)
                $display("  [WARN] Coin bias — check seed or xorshift implementation.");
            else
                $display("  [OK  ] PRNG coin is fair. Repeating-pattern check visible in CSV.");
        end

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        #1;
        $display("Reset released.\n");

        // ==============================================================
        // E1: Three simultaneous, geometrically separate collision points
        //
        // All 6 flits injected in the same cycle. Three separate routers
        // each handle a contention event at the same posedge (posedge 2).
        //
        // Group 1 — collision at (0,2):
        //   (0,1)→(0,3): North 1-hop → (0,2), wants North
        //   (1,2)→(0,3): West  1-hop → (0,2), wants North
        //
        // Group 2 — collision at (2,2):
        //   (2,1)→(2,3): North 1-hop → (2,2), wants North
        //   (3,2)→(2,3): West  1-hop → (2,2), wants North
        //
        // Group 3 — collision at (0,0) via wraparound links:
        //   (3,0)→(0,2): East-wrap 1-hop → (0,0), wants North
        //   (1,0)→(0,2): West      1-hop → (0,0), wants North
        //
        // NOTE (from stage4_fix.txt hand-trace):
        //   Groups 1 and 3 are not fully independent — tag 6 wins at (0,0)
        //   but then loses at (0,1) to tag 2 which was deflected from group 1.
        //   This is an accidental second cascade. It is still a valid test.
        //   Tag 5 shows OK_DIRECT despite being deflected, because South is also
        //   a minimal direction for a Y-distance=2 path on a 4-ring (tie).
        // ==============================================================
        $display("====== E1: Three simultaneous collision groups ======");
        $display("  Group 1: (0,2)   Group 2: (2,2)   Group 3: (0,0)");

        inject_flit(0, 1, 0, 3);   // Group 1A — North → (0,2)
        inject_flit(1, 2, 0, 3);   // Group 1B — West  → (0,2)
        inject_flit(2, 1, 2, 3);   // Group 2A — North → (2,2)
        inject_flit(3, 2, 2, 3);   // Group 2B — West  → (2,2)
        inject_flit(3, 0, 0, 2);   // Group 3A — East-wrap → (0,0)
        inject_flit(1, 0, 0, 2);   // Group 3B — West      → (0,0)

        @(posedge clk); #1;
        release_injectors();
        wait_drain(40, "E1");
        repeat(3) @(posedge clk); #1;

        // ==============================================================
        // E2: Cascaded/chained deflection — 3 flits, staggered injection
        //
        // Cycle 0: inject G1 and G2 simultaneously.
        //   G1: (0,1)→(0,3)  North→(0,2)→North→(0,3)       min 2 hops
        //   G2: (1,2)→(0,3)  West→(0,2)→North→(0,3)        min 2 hops
        //   At posedge 2: G1(SRC_S) and G2(SRC_E) collide at (0,2), both want
        //   North. Priority SRC_S > SRC_E → G1 wins. G2 deflected South → (0,1).
        //
        // Cycle 1: inject G3 one cycle later.
        //   G3: (0,0)→(0,2)  North→(0,1)→North→(0,2)       min 2 hops
        //   At posedge 2: (0,0) captures G3 → exits North via link_v[0][0].
        //   At posedge 3: G3 arrives at (0,1) from South (SRC_S).
        //                 G2 (deflected from (0,2)) arrives at (0,1) from
        //                 North (SRC_N). Both want North.
        //                 Priority SRC_N > SRC_S → G2 wins. G3 deflected.
        //                 Second deflection event = cascade confirmed.
        //
        // Expected CSV: G1 hop_count=2 (OK_DIRECT), G2 hop_count=4 (OK_DEFLECTED),
        //               G3 hop_count=4 (OK_DEFLECTED). All reach correct PE.
        // ==============================================================
        $display("\n====== E2: Cascaded deflection (staggered by 1 cycle) ======");
        $display("  First collision at (0,2) posedge 2 ; cascade at (0,1) posedge 3");

        inject_flit(0, 1, 0, 3);   // G1
        inject_flit(1, 2, 0, 3);   // G2 — deflected at (0,2), cascades at (0,1)
        @(posedge clk); #1;
        release_injectors();

        inject_flit(0, 0, 0, 2);   // G3 — meets G2 at (0,1) one cycle later
        @(posedge clk); #1;
        release_injectors();

        wait_drain(40, "E2");
        repeat(3) @(posedge clk); #1;

        // ==============================================================
        // E3: Sustained random traffic — xorshift32 PRNG, 150 cycles
        //
        // Injection protocol (correct inj_ready handshake):
        //   Phase 1: for each PE, flip PRNG coin; if heads, drive inj_valid+flit
        //            and record tag optimistically in pending[].
        //   Phase 2: #1 to settle combinational logic (deflect_resolver.inj_ready
        //            is now valid because inj_valid=1 has propagated). Roll back
        //            any PE where inj_ready=0: delete from pending, decrement
        //            total_injected, clear inj_valid_drv before posedge.
        //   Phase 3: @posedge clk captures accepted injections. #1, release all.
        //
        // PRNG usage per PE per cycle:
        //   prng_next()[0]    — coin flip (inject this PE?)
        //   prng_next() % TORUS_X — rdx
        //   prng_next() % TORUS_Y — rdy  (only if coin came up heads)
        // ==============================================================
        $display("\n====== E3: Random traffic (%0d cycles, ~50%% target rate) ======",
                 E3_CYCLES);
        begin
            // --- ALL variable declarations first (XSIM requires no statements before declarations) ---
            int pre_e3_inj;
            int pre_e3_ej;
            int pre_e3_xhop;
            int pre_e3_devt;
            int pre_e3_att;
            int pre_e3_dfn;
            int pre_e3_dfs;
            int pre_e3_dfe;
            int pre_e3_dfw;
            int rdx, rdy, tag;
            logic [31:0] pv;
            real attempted_rate_pct, achieved_rate_pct, excess_hop_rate_pct;
            real deflection_evt_per_flit;
            int e3_att, e3_inj, e3_rej, e3_ej, e3_xhop, e3_devt;
            int e3_dfn, e3_dfs, e3_dfe, e3_dfw;

            // --- Snapshot all counters before E3 injection loop starts ---
            pre_e3_inj  = total_injected;
            pre_e3_ej   = total_ejected;
            pre_e3_xhop = total_excess_hop;
            pre_e3_devt = total_deflection_events;
            pre_e3_att  = total_attempted;
            pre_e3_dfn  = defl_from_n;
            pre_e3_dfs  = defl_from_s;
            pre_e3_dfe  = defl_from_e;
            pre_e3_dfw  = defl_from_w;

            for(int cyc = 0; cyc < E3_CYCLES; cyc++) begin

                // --- Phase 1: select PEs and drive ports ---
                for(int ri = 0; ri < TORUS_X; ri++) begin
                    for(int rj = 0; rj < TORUS_Y; rj++) begin
                        pv = prng_next();           // coin flip
                        if(pv[0]) begin
                            pv  = prng_next();
                            rdx = int'(pv % TORUS_X);
                            pv  = prng_next();
                            rdy = int'(pv % TORUS_Y);
                            // prevent self-destined injection
                            if(rdx == ri && rdy == rj) rdx = (ri + 1) % TORUS_X;
                            // count attempt before inj_ready check
                            total_attempted++;
                            // optimistic scoreboard entry
                            tag = next_tag++;
                            pending[tag].src_x        = ri;
                            pending[tag].src_y        = rj;
                            pending[tag].dest_x       = rdx;
                            pending[tag].dest_y       = rdy;
                            pending[tag].inject_cycle = sim_cycle;
                            total_injected++;
                            // drive injection port
                            inj_valid_drv[ri][rj] = 1'b1;
                            inj_flit_drv[ri][rj]  = make_flit(X_BITS'(rdx), Y_BITS'(rdy),
                                                               DATA_WIDTH'(tag));
                        end
                    end
                end

                // --- Phase 2: settle, rollback rejected injections ---
                // After #1, inj_ready_arr reflects the deflect_resolver's
                // Pass 4 decision (with inj_valid=1 driving the always_comb).
                // Any PE where inj_ready=0 is rolled back before posedge.
                #1;
                for(int ri = 0; ri < TORUS_X; ri++) begin
                    for(int rj = 0; rj < TORUS_Y; rj++) begin
                        if(inj_valid_drv[ri][rj] && !inj_ready_arr[ri][rj]) begin
                            tag = int'(inj_flit_drv[ri][rj].data);
                            pending.delete(tag);
                            total_injected--;
                            inj_valid_drv[ri][rj] = 1'b0;
                            inj_flit_drv[ri][rj]  = '0;
                        end
                    end
                end

                // --- Phase 3: clock edge + clear ---
                @(posedge clk); #1;
                release_injectors();
            end

            // Drain timeout scales with torus size and load.
            // 200 cycles is sufficient for 4x4 at 50%; revisit for Stage 5+.
            wait_drain(200, "E3");

            // E3-specific statistics
            e3_att  = total_attempted         - pre_e3_att;
            e3_inj  = total_injected          - pre_e3_inj;
            e3_rej  = e3_att                  - e3_inj;   // rejected by inj_ready
            e3_ej   = total_ejected           - pre_e3_ej;
            e3_xhop = total_excess_hop        - pre_e3_xhop;
            e3_devt = total_deflection_events - pre_e3_devt;
            e3_dfn  = defl_from_n             - pre_e3_dfn;
            e3_dfs  = defl_from_s             - pre_e3_dfs;
            e3_dfe  = defl_from_e             - pre_e3_dfe;
            e3_dfw  = defl_from_w             - pre_e3_dfw;

            attempted_rate_pct     = real'(e3_att)  * 100.0 /
                                      real'(E3_CYCLES * TORUS_X * TORUS_Y);
            achieved_rate_pct      = real'(e3_inj)  * 100.0 /
                                      real'(E3_CYCLES * TORUS_X * TORUS_Y);
            excess_hop_rate_pct    = (e3_ej > 0) ?
                                      real'(e3_xhop) * 100.0 / real'(e3_ej) : 0.0;
            deflection_evt_per_flit = (e3_ej > 0) ?
                                       real'(e3_devt) / real'(e3_ej) : 0.0;

            $display("\n  --- E3 Statistics ---");
            $display("  Target injection rate    : ~50%%");
            $display("  Attempted injection rate : %.1f%%  (%0d attempts / %0d PE-slot-cycles)",
                     attempted_rate_pct, e3_att, E3_CYCLES * TORUS_X * TORUS_Y);
            $display("  Achieved injection rate  : %.1f%%  (%0d accepted, %0d rejected by inj_ready)",
                     achieved_rate_pct, e3_inj, e3_rej);
            $display("  Flits ejected            : %0d", e3_ej);
            $display("  Excess-hop rate          : %.1f%%  (%0d flits with hop_count>min_hops)",
                     excess_hop_rate_pct, e3_xhop);
            $display("  Deflection events (RTL)  : %0d  (%.2f events/flit)",
                     e3_devt, deflection_evt_per_flit);
            $display("  Per-port deflection breakdown (N>S>E>W priority — W expected highest):");
            $display("    from North input : %0d  (%.1f%%)",
                     e3_dfn, (e3_devt>0) ? real'(e3_dfn)*100.0/real'(e3_devt) : 0.0);
            $display("    from South input : %0d  (%.1f%%)",
                     e3_dfs, (e3_devt>0) ? real'(e3_dfs)*100.0/real'(e3_devt) : 0.0);
            $display("    from East  input : %0d  (%.1f%%)",
                     e3_dfe, (e3_devt>0) ? real'(e3_dfe)*100.0/real'(e3_devt) : 0.0);
            $display("    from West  input : %0d  (%.1f%%)",
                     e3_dfw, (e3_devt>0) ? real'(e3_dfw)*100.0/real'(e3_devt) : 0.0);
            $display("  Note: excess-hop rate undercounts deflections where a deflected");
            $display("        flit reaches dest via an equally-short alternative path.");
        end

        // ==============================================================
        // Hop count statistics — full run (E1 + E2 + E3)
        // ==============================================================
        begin
            int sorted_hops [];
            int p99_idx;
            int p999_idx;

            sorted_hops = hop_counts;   // queue → dynamic array
            sorted_hops.sort();         // ascending

            if(sorted_hops.size() > 0) begin
                p99_idx  = int'(real'(sorted_hops.size() - 1) * 0.990);
                p999_idx = int'(real'(sorted_hops.size() - 1) * 0.999);
                $display("\n  --- Hop Count Statistics (all %0d ejected flits) ---",
                         sorted_hops.size());
                $display("  Min hop count    : %0d", sorted_hops[0]);
                $display("  Max hop count    : %0d", sorted_hops[sorted_hops.size()-1]);
                $display("  99th pct hops    : %0d  (rank %0d / %0d)",
                         sorted_hops[p99_idx],  p99_idx,  sorted_hops.size()-1);
                $display("  99.9th pct hops  : %0d  (rank %0d / %0d)",
                         sorted_hops[p999_idx], p999_idx, sorted_hops.size()-1);
                $display("  (Min/Max/p99/p99.9 logged every run — baseline for Phase 10 worst-case bound)");
            end
        end


        // ==============================================================
        // Final summary
        // ==============================================================
        $fclose(csv_fd);

        $display("\n==========================================");
        $display("  STAGE 4 SUMMARY");
        $display("  Attempted             : %0d  (before inj_ready check)", total_attempted);
        $display("  Injected (accepted)   : %0d", total_injected);
        $display("  Ejected               : %0d  (%0d correct destination)",
                 total_ejected, total_correct);
        $display("  Excess-hop flits      : %0d  (hop_count > min_hops — latency impact)",
                 total_excess_hop);
        $display("  Deflection events     : %0d  (RTL deflected[] flags, all routers, all cycles)",
                 total_deflection_events);
        $display("    N-input: %0d  S-input: %0d  E-input: %0d  W-input: %0d",
                 defl_from_n, defl_from_s, defl_from_e, defl_from_w);
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if(fail_count == 0)
            $display("  ALL STAGE 4 TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above and CSV.");

        $finish;
    end

endmodule
