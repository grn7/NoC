// tb/tb_stage7_l2.sv
// Stage 7 — L2 "strict + skip-ahead" Ablation Testbench (4x4)
//
// L2 policy (per mech1_update.txt item 0):
//   Like L1 strict (no fallback — PE only injects when desired port is free),
//   but the PE looks ahead up to LOOKAHEAD=4 flits in its queue and injects the
//   first one whose desired port is free this cycle.
//
// Implementation:
//   Each PE presents candidate flits from its window to the resolver each cycle.
//   Because the resolver is purely combinational, we can probe it with different
//   inj_flit values within the same cycle. We drive the head flit first; if
//   inj_ready=0 (desired port taken), we try the next one, up to LOOKAHEAD.
//   We use the L1-strict resolver (no fallback sub-pass): inj_ready=1 iff the
//   desired port is free, so inj_ready=0 unambiguously means "desired port busy".
//
// Reordering note:
//   L2 can inject flit[k] before flit[0..k-1] from the same PE.
//   The scoreboard handles this correctly — each flit has its own tag.
//
// This testbench uses the same RTL as L1 strict (rtl_strict/ tree).
// Run via scripts/sim_stage7_l2.tcl.

`timescale 1ns/1ps

module tb_stage7_l2;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Sweep Parameters
    // ------------------------------------------------------------------
    localparam int LOOKAHEAD  = 4;   // window size for skip-ahead

    localparam int NUM_DEPTHS = 4;
    localparam int DEPTHS [NUM_DEPTHS] = '{4, 8, 16, 32};
    localparam int MAX_DEPTH  = 32;

    localparam int NUM_SEEDS = 4;
    localparam logic [31:0] SEEDS [NUM_SEEDS] = '{
        32'hDEAD_BEEF,
        32'h1234_5678,
        32'hCAFE_BABE,
        32'hA5A5_5A5A
    };

    localparam int DRAIN_TIMEOUT = 10000;

    // ------------------------------------------------------------------
    // Flit scoreboard record
    // ------------------------------------------------------------------
    typedef struct {
        int src_x,  src_y;
        int dest_x, dest_y;
        int inject_cycle;
    } flit_record_t;

    // ------------------------------------------------------------------
    // Clock and Reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // 2D PE Port Interface Array
    // ------------------------------------------------------------------
    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    // ------------------------------------------------------------------
    // DUT Instantiation (L1 strict tree — no fallback sub-pass)
    // ------------------------------------------------------------------
    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    // ------------------------------------------------------------------
    // Injection Driver Arrays
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
    // Ejection & inj_ready Extraction Arrays
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
    // Deflection-Event Extraction Array
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
    // Workload Queue Storage (Per-PE Fixed Queues)
    // ------------------------------------------------------------------
    flit_t workload_queue [TORUS_X][TORUS_Y][MAX_DEPTH];
    logic  flit_injected  [TORUS_X][TORUS_Y][MAX_DEPTH]; // which slots have been injected
    int    pe_inj_count   [TORUS_X][TORUS_Y];            // how many flits this PE has injected
    int    pe_done_cycle  [TORUS_X][TORUS_Y];

    // ------------------------------------------------------------------
    // Scoreboard and Per-Run Tracking
    // ------------------------------------------------------------------
    flit_record_t pending [int];

    int sim_cycle             = 0;
    int run_attempted         = 0;
    int run_injected          = 0;
    int run_ejected           = 0;
    int run_correct           = 0;
    int run_excess_hop        = 0;
    int run_deflection_events = 0;
    int run_defl_from_n       = 0;
    int run_defl_from_s       = 0;
    int run_defl_from_e       = 0;
    int run_defl_from_w       = 0;
    int run_fail              = 0;

    int hop_counts [$];

    // Helper functions
    function automatic flit_t make_flit(logic [X_BITS-1:0] x, logic [Y_BITS-1:0] y, logic [DATA_WIDTH-1:0] d);
        flit_t f;
        f.X_dest = x;
        f.Y_dest = y;
        f.data   = d;
        return f;
    endfunction

    function automatic int torus_min_hops(int sx, sy, dx, dy);
        int dist_x, dist_y, min_dx, min_dy;
        dist_x = (dx >= sx) ? (dx - sx) : (sx - dx);
        dist_y = (dy >= sy) ? (dy - sy) : (sy - dy);
        min_dx = (dist_x <= TORUS_X - dist_x) ? dist_x : (TORUS_X - dist_x);
        min_dy = (dist_y <= TORUS_Y - dist_y) ? dist_y : (TORUS_Y - dist_y);
        return min_dx + min_dy;
    endfunction

    // ------------------------------------------------------------------
    // Deterministic Workload Generation (identical to tb_stage7_completion)
    // ------------------------------------------------------------------
    function automatic logic [31:0] generate_workload(int depth, logic [31:0] seed);
        logic [31:0] l_state;
        logic [31:0] chk;
        int dx, dy;
        int tag;

        l_state = seed;
        chk     = 32'h0;

        // Warm up PRNG
        for(int k = 0; k < 1000; k++) begin
            l_state ^= l_state << 13;
            l_state ^= l_state >> 17;
            l_state ^= l_state << 5;
        end

        tag = 1;
        for(int k = 0; k < depth; k++) begin
            for(int x = 0; x < TORUS_X; x++) begin
                for(int y = 0; y < TORUS_Y; y++) begin
                    do begin
                        l_state ^= l_state << 13;
                        l_state ^= l_state >> 17;
                        l_state ^= l_state << 5;
                        dx = int'(l_state % TORUS_X);

                        l_state ^= l_state << 13;
                        l_state ^= l_state >> 17;
                        l_state ^= l_state << 5;
                        dy = int'(l_state % TORUS_Y);

                        if(dx == x && dy == y) begin
                            dx = (x + 1) % TORUS_X;
                        end
                    end while (dx == x && dy == y);

                    workload_queue[x][y][k] = make_flit(X_BITS'(dx), Y_BITS'(dy), DATA_WIDTH'(tag));
                    chk ^= (logic'(dx) << 24) ^ (logic'(dy) << 16) ^ logic'(tag);
                    tag++;
                end
            end
        end
        return chk;
    endfunction

    // ------------------------------------------------------------------
    // Ejection & Deflection Monitor (Sampled at posedge clk + #1)
    // ------------------------------------------------------------------
    int           mon_tag;
    int           mon_eject_cyc;
    int           mon_hops;
    int           mon_min_hops;
    flit_record_t mon_rec;
    logic         mon_dest_ok;

    always @(posedge clk) begin
        sim_cycle = sim_cycle + 1;
        #1;

        if(rst_n) begin
            // 1. Deflection event counting across all 16 routers
            for(int pi = 0; pi < TORUS_X; pi++) begin
                for(int pj = 0; pj < TORUS_Y; pj++) begin
                    if(defl_arr[pi][pj][0]) begin run_deflection_events++; run_defl_from_n++; end
                    if(defl_arr[pi][pj][1]) begin run_deflection_events++; run_defl_from_s++; end
                    if(defl_arr[pi][pj][2]) begin run_deflection_events++; run_defl_from_e++; end
                    if(defl_arr[pi][pj][3]) begin run_deflection_events++; run_defl_from_w++; end
                end
            end

            // 2. Ejection monitor across all 16 PEs
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

                            hop_counts.push_back(mon_hops);

                            if(!mon_dest_ok) begin
                                $display("  [FAIL] Tag=%0d ejected at (%0d,%0d), expected (%0d,%0d)",
                                         mon_tag, pi, pj, mon_rec.dest_x, mon_rec.dest_y);
                                run_fail++;
                            end else if(mon_hops > mon_min_hops) begin
                                run_correct++;
                                run_excess_hop++;
                            end else begin
                                run_correct++;
                            end

                            pending.delete(mon_tag);
                            run_ejected++;

                        end else begin
                            $display("  [FAIL] Untracked ejection tag=%0d at (%0d,%0d) — stray/duplicate",
                                     mon_tag, pi, pj);
                            run_fail++;
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Main Simulation Process
    // ------------------------------------------------------------------
    initial begin
        int d_idx, s_idx;
        int cur_depth;
        logic [31:0] cur_seed;
        logic [31:0] workload_chk;
        int total_workload_flits;

        int run_start_cycle;
        int all_inj_cycle;
        int makespan_cycle;
        int drain_delay;
        int run_done;
        int run_timed_out;

        int acc_tag;
        flit_t acc_flit;

        real mean_hops;
        real p99_hops;
        int  max_hops;
        int  min_hops_recorded;
        real devt_per_flit;
        real excess_hop_pct;
        real achieved_inj_rate;
        real defl_n_pct, defl_s_pct, defl_e_pct, defl_w_pct;

        int sorted_hops [$];
        int n_hops;
        int p99_idx;

        int csv_fd;
        string csv_filename;

        // L2 skip-ahead per-PE state
        int   pe_skip_slot [TORUS_X][TORUS_Y]; // which slot we're currently trying this cycle
        logic found_slot;
        int   tried;

        csv_filename = "../reports/stage7_l2_completion.csv";
        if($value$plusargs("CSV_OUT=%s", csv_filename)) begin
            $display("Using CSV filename from plusarg: %s", csv_filename);
        end

        // Initial driver state
        for(int i = 0; i < TORUS_X; i++) begin
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        // Open CSV
        csv_fd = $fopen(csv_filename, "w");
        $fdisplay(csv_fd, "depth,seed,total_flits,inj_complete_cycles,makespan_cycles,drain_delay_cycles,injection_rate_pct,mean_latency,p99_latency,max_latency,devt_per_flit,defl_n_pct,defl_s_pct,defl_e_pct,defl_w_pct,excess_hop_pct,pass_count,fail_count,timed_out");

        $display("=========================================================================================");
        $display("=== STAGE 7 L2: Strict + Skip-Ahead (LOOKAHEAD=%0d) Ablation Benchmark             ===", LOOKAHEAD);
        $display("=== Queue Depths: 4, 8, 16, 32 flits/PE  |  4 Seeds per Depth = 16 Benchmark Points===");
        $display("=== L1-strict resolver (no fallback). PE picks first ready-to-go flit in window.   ===");
        $display("=========================================================================================");

        for(d_idx = 0; d_idx < NUM_DEPTHS; d_idx++) begin
            cur_depth = DEPTHS[d_idx];
            total_workload_flits = TORUS_X * TORUS_Y * cur_depth;

            for(s_idx = 0; s_idx < NUM_SEEDS; s_idx++) begin
                cur_seed = SEEDS[s_idx];

                workload_chk = generate_workload(cur_depth, cur_seed);

                // Reset DUT & TB state
                rst_n = 0;
                repeat(5) @(posedge clk);
                rst_n = 1;
                repeat(2) @(posedge clk);
                #1;

                for(int i = 0; i < TORUS_X; i++) begin
                    for(int j = 0; j < TORUS_Y; j++) begin
                        inj_valid_drv[i][j] = 1'b0;
                        inj_flit_drv[i][j]  = '0;
                        pe_inj_count[i][j]  = 0;
                        pe_done_cycle[i][j] = -1;
                        for(int k = 0; k < MAX_DEPTH; k++)
                            flit_injected[i][j][k] = 1'b0;
                    end
                end

                pending.delete();
                hop_counts.delete();

                run_attempted         = 0;
                run_injected          = 0;
                run_ejected           = 0;
                run_correct           = 0;
                run_excess_hop        = 0;
                run_deflection_events = 0;
                run_defl_from_n       = 0;
                run_defl_from_s       = 0;
                run_defl_from_e       = 0;
                run_defl_from_w       = 0;
                run_fail              = 0;

                run_start_cycle       = sim_cycle;
                all_inj_cycle         = -1;
                makespan_cycle        = -1;
                run_done              = 0;
                run_timed_out         = 0;

                while (!run_done && !run_timed_out) begin

                    // Step A: L2 skip-ahead injection selection
                    // For each PE, find the first un-injected slot in the LOOKAHEAD window
                    // whose desired port is free (inj_ready=1 with the strict resolver).
                    // We iterate over the window slots, driving each and probing inj_ready after #0.
                    // The accepted flit wins the slot for this cycle.

                    // First, drive nothing (deassert all)
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            inj_valid_drv[ri][rj] = 1'b0;
                            inj_flit_drv[ri][rj]  = '0;
                        end
                    end

                    // For each PE: walk through LOOKAHEAD un-injected slots, try each
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            tried      = 0;
                            found_slot = 1'b0;

                            for(int k = 0; k < cur_depth && tried < LOOKAHEAD && !found_slot; k++) begin
                                if(!flit_injected[ri][rj][k]) begin
                                    // Try presenting this flit
                                    inj_valid_drv[ri][rj] = 1'b1;
                                    inj_flit_drv[ri][rj]  = workload_queue[ri][rj][k];
                                    tried++;

                                    // Settle combinational logic
                                    #0;

                                    // Check if this flit's desired port is free (strict: inj_ready=1 iff desired free)
                                    if(inj_ready_arr[ri][rj]) begin
                                        found_slot = 1'b1; // keep this flit driven; break inner loop
                                    end else begin
                                        // This flit's desired port is busy; try next
                                        inj_valid_drv[ri][rj] = 1'b0;
                                        inj_flit_drv[ri][rj]  = '0;
                                    end
                                end
                            end

                            if(!found_slot) begin
                                // No flit in window can inject — PE idles this cycle
                                inj_valid_drv[ri][rj] = 1'b0;
                                inj_flit_drv[ri][rj]  = '0;
                            end

                            if(pe_inj_count[ri][rj] < cur_depth)
                                run_attempted++;
                        end
                    end

                    // Step B: Settle combinational logic
                    #1;

                    // Step C: Check inj_ready_arr and record accepted flits
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            if(inj_valid_drv[ri][rj] && inj_ready_arr[ri][rj]) begin
                                acc_flit = inj_flit_drv[ri][rj];
                                acc_tag  = int'(acc_flit.data);

                                // Find which slot this tag came from and mark injected
                                for(int k = 0; k < cur_depth; k++) begin
                                    if(int'(workload_queue[ri][rj][k].data) == acc_tag &&
                                       !flit_injected[ri][rj][k]) begin
                                        flit_injected[ri][rj][k] = 1'b1;
                                        break;
                                    end
                                end

                                pending[acc_tag].src_x        = ri;
                                pending[acc_tag].src_y        = rj;
                                pending[acc_tag].dest_x       = int'(acc_flit.X_dest);
                                pending[acc_tag].dest_y       = int'(acc_flit.Y_dest);
                                pending[acc_tag].inject_cycle = sim_cycle;

                                run_injected++;
                                pe_inj_count[ri][rj]++;

                                if(pe_inj_count[ri][rj] == cur_depth) begin
                                    pe_done_cycle[ri][rj] = sim_cycle;
                                end
                            end
                        end
                    end

                    // Check if all flits have been injected (off-by-one fix applied)
                    if(run_injected == total_workload_flits && all_inj_cycle == -1) begin
                        all_inj_cycle = sim_cycle - run_start_cycle - 1;
                        if(all_inj_cycle < 1) all_inj_cycle = 1;
                    end

                    // Step D: Advance clock
                    @(posedge clk);
                    #1;

                    // Step E: Check completion / timeout conditions
                    if(run_ejected == total_workload_flits && pending.size() == 0) begin
                        run_done = 1;
                        makespan_cycle = sim_cycle - run_start_cycle;
                    end else if((sim_cycle - run_start_cycle) >= DRAIN_TIMEOUT) begin
                        run_timed_out = 1;
                        makespan_cycle = sim_cycle - run_start_cycle;
                    end
                end

                // Compute run metrics
                drain_delay = (all_inj_cycle >= 0) ? (makespan_cycle - all_inj_cycle) : -1;
                achieved_inj_rate = (all_inj_cycle > 0) ?
                    (real'(total_workload_flits) * 100.0 / (real'(TORUS_X * TORUS_Y) * real'(all_inj_cycle))) : 0.0;

                if(achieved_inj_rate > 100.0) begin
                    $display("  [ASSERT FAIL] Injection rate %0.2f%% > 100%% — all_inj_cycle=%0d",
                             achieved_inj_rate, all_inj_cycle);
                end

                devt_per_flit = (run_ejected > 0) ? real'(run_deflection_events) / real'(run_ejected) : 0.0;
                excess_hop_pct = (run_ejected > 0) ? (real'(run_excess_hop) * 100.0 / real'(run_ejected)) : 0.0;

                if(run_deflection_events > 0) begin
                    defl_n_pct = real'(run_defl_from_n) * 100.0 / real'(run_deflection_events);
                    defl_s_pct = real'(run_defl_from_s) * 100.0 / real'(run_deflection_events);
                    defl_e_pct = real'(run_defl_from_e) * 100.0 / real'(run_deflection_events);
                    defl_w_pct = real'(run_defl_from_w) * 100.0 / real'(run_deflection_events);
                end else begin
                    defl_n_pct = 0.0; defl_s_pct = 0.0; defl_e_pct = 0.0; defl_w_pct = 0.0;
                end

                // Latency percentiles
                n_hops = hop_counts.size();
                if(n_hops > 0) begin
                    sorted_hops = hop_counts;
                    sorted_hops.sort();

                    min_hops_recorded = sorted_hops[0];
                    max_hops          = sorted_hops[n_hops-1];

                    mean_hops = 0.0;
                    for(int k = 0; k < n_hops; k++) mean_hops += real'(sorted_hops[k]);
                    mean_hops /= real'(n_hops);

                    p99_idx  = int'(real'(n_hops - 1) * 0.99);
                    p99_hops = real'(sorted_hops[p99_idx]);
                end else begin
                    min_hops_recorded = 0;
                    max_hops          = 0;
                    mean_hops         = 0.0;
                    p99_hops          = 0.0;
                end

                // Display Run Summary
                $display("-----------------------------------------------------------------------------------------");
                $display(" [L2] Depth=%0d/PE | Seed=0x%08h (chk=0x%08h) | Flits=%0d",
                         cur_depth, cur_seed, workload_chk, total_workload_flits);
                $display("   Status:      %s (injected=%0d, ejected=%0d, undrained=%0d)",
                         run_timed_out ? "[TIMEOUT]" : "[COMPLETED]",
                         run_injected, run_ejected, pending.size());
                $display("   Makespan:    %0d cycles  |  Inj Finish: %0d cycles  |  Drain Delay: %0d cycles",
                         makespan_cycle, all_inj_cycle, drain_delay);
                $display("   Achieved Inj Rate: %0.2f%%", achieved_inj_rate);
                $display("   Latency:     Mean=%0.2f, P99=%0.1f, Max=%0d  |  Excess Hops=%0.1f%%",
                         mean_hops, p99_hops, max_hops, excess_hop_pct);
                $display("   Deflections: %0d total (%0.2f/flit)  |  S=%0.1f%%, E=%0.1f%%, W=%0.1f%%",
                         run_deflection_events, devt_per_flit, defl_s_pct, defl_e_pct, defl_w_pct);

                // Write row to CSV
                $fdisplay(csv_fd, "%0d,0x%08h,%0d,%0d,%0d,%0d,%0.4f,%0.4f,%0.4f,%0d,%0.4f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0d,%0d,%0d",
                          cur_depth, cur_seed, total_workload_flits,
                          all_inj_cycle, makespan_cycle, drain_delay,
                          achieved_inj_rate, mean_hops, p99_hops, max_hops,
                          devt_per_flit, defl_n_pct, defl_s_pct, defl_e_pct, defl_w_pct,
                          excess_hop_pct, run_correct, run_fail, run_timed_out);
            end
        end

        $fclose(csv_fd);
        $display("=========================================================================================");
        $display("=== STAGE 7 L2 BENCHMARK COMPLETED: Results written to %s", csv_filename);
        $display("=========================================================================================");

        $finish;
    end

endmodule
