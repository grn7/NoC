// tb/tb_stage7_completion.sv
// Stage 7 — Fixed-Workload Completion-Time & Makespan Benchmark (4x4)
//
// Head-to-Head Benchmark: Baseline vs Mechanism 1 (Predictor)
// In this benchmark, every PE has a fixed queue of N flits (destinations decided
// once up front, shared identically between Baseline and Mechanism 1).
// Each PE retries the head of its queue every cycle until accepted, then advances.
//
// Both architectures MUST deliver the EXACT SAME total number of packets (16 * N).
// Measures:
//   - Injection completion time (cycles until last PE finishes injecting)
//   - Total makespan / completion time (cycles until entire network is drained)
//   - Packet delivery latency (mean, P99, max)
//   - Deflections per delivered flit
//
// Sweeps:
//   - Queue depths: 4, 8, 16, 32 flits per PE
//   - Seeds: 0xDEADBEEF, 0x12345678, 0xCAFEBABE, 0xA5A55A5A (4 seeds)
//   - Total: 16 benchmark points per architecture

`timescale 1ns/1ps

module tb_stage7_completion;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Sweep Parameters
    // ------------------------------------------------------------------
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

    localparam int DRAIN_TIMEOUT = 10000; // Safety timeout for livelock / deadlock

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
    // DUT Instantiation
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
    // Hierarchy: dut.gen_col[gx].gen_row[gy].u_router.u_resolver.deflected
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
    int    queue_head     [TORUS_X][TORUS_Y];
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
    // Deterministic Workload Generation (100% Identical Across Trees)
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
        // Strictly declare ALL local variables at top of block (XSIM rule)
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

        // Check if plusarg specified output CSV filename
        csv_filename = "../reports/stage7_completion.csv";
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
        $display("=== STAGE 7: Fixed-Workload Completion-Time & Makespan Benchmark                       ===");
        $display("=== Queue Depths: 4, 8, 16, 32 flits/PE  |  4 Seeds per Depth = 16 Benchmark Points   ===");
        $display("=== Head-of-Line Retry Discipline: Same Pre-Generated Flits Fed to Both Architectures ===");
        $display("=========================================================================================");

        // Iterate over Queue Depths
        for(d_idx = 0; d_idx < NUM_DEPTHS; d_idx++) begin
            cur_depth = DEPTHS[d_idx];
            total_workload_flits = TORUS_X * TORUS_Y * cur_depth;

            // Iterate over Seeds
            for(s_idx = 0; s_idx < NUM_SEEDS; s_idx++) begin
                cur_seed = SEEDS[s_idx];

                // 1. Generate workload deterministically (identical across Baseline & Mech1)
                workload_chk = generate_workload(cur_depth, cur_seed);

                // 2. Reset DUT & TB state
                rst_n = 0;
                repeat(5) @(posedge clk);
                rst_n = 1;
                repeat(2) @(posedge clk);
                #1;

                // Reset per-run queues & scoreboards
                for(int i = 0; i < TORUS_X; i++) begin
                    for(int j = 0; j < TORUS_Y; j++) begin
                        inj_valid_drv[i][j] = 1'b0;
                        inj_flit_drv[i][j]  = '0;
                        queue_head[i][j]    = 0;
                        pe_done_cycle[i][j] = -1;
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

                // Active injection & drainage loop
                while (!run_done && !run_timed_out) begin

                    // Step A: Drive head flit for each PE that still has flits in queue
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            if(queue_head[ri][rj] < cur_depth) begin
                                inj_valid_drv[ri][rj] = 1'b1;
                                inj_flit_drv[ri][rj]  = workload_queue[ri][rj][queue_head[ri][rj]];
                                run_attempted++;
                            end else begin
                                inj_valid_drv[ri][rj] = 1'b0;
                                inj_flit_drv[ri][rj]  = '0;
                            end
                        end
                    end

                    // Step B: Settle combinational logic (#1)
                    #1;

                    // Step C: Check inj_ready_arr and advance queue for accepted flits
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            if(inj_valid_drv[ri][rj] && inj_ready_arr[ri][rj]) begin
                                acc_flit = inj_flit_drv[ri][rj];
                                acc_tag  = int'(acc_flit.data);

                                pending[acc_tag].src_x        = ri;
                                pending[acc_tag].src_y        = rj;
                                pending[acc_tag].dest_x       = int'(acc_flit.X_dest);
                                pending[acc_tag].dest_y       = int'(acc_flit.Y_dest);
                                pending[acc_tag].inject_cycle = sim_cycle;

                                run_injected++;
                                queue_head[ri][rj]++;

                                if(queue_head[ri][rj] == cur_depth) begin
                                    pe_done_cycle[ri][rj] = sim_cycle;
                                end
                            end
                            // If !inj_ready_arr, queue_head is held; PE retries same flit next cycle.
                        end
                    end

                    // Check if all flits have been injected
                    // off-by-one fix: accepts are registered in step C which runs in the same
                    // logical cycle as the posedge we are about to advance past, so sim_cycle
                    // already reflects the cycle whose clock edge closes this injection.
                    // Subtract 1 so that "1 flit in 1 cycle" correctly gives rate = 100%.
                    if(run_injected == total_workload_flits && all_inj_cycle == -1) begin
                        all_inj_cycle = sim_cycle - run_start_cycle - 1;
                        if(all_inj_cycle < 1) all_inj_cycle = 1; // guard against zero
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

                // Sanity check: injection rate must never exceed 100% (one flit per PE per cycle maximum)
                if(achieved_inj_rate > 100.0) begin
                    $display("  [ASSERT FAIL] Injection rate %0.2f%% > 100%% — all_inj_cycle=%0d total_flits=%0d PEs=%0d",
                             achieved_inj_rate, all_inj_cycle, total_workload_flits, TORUS_X * TORUS_Y);
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
                $display(" Depth=%0d/PE | Seed=0x%08h (chk=0x%08h) | Flits=%0d",
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
        $display("=== STAGE 7 BENCHMARK COMPLETED: Results written to %s", csv_filename);
        $display("=========================================================================================");

        $finish;
    end

endmodule
