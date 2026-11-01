// tb/tb_stage6.sv
// Stage 6 — Fine-grained load-latency and deflection curve characterization (4x4 baseline)
//
// Sweeps offered load finely across 13 injection probability points (prob_16 = 1..13 / 16,
// corresponding to ~6.25% to ~81.25% offered load) across 4 distinct PRNG seeds
// (0xDEADBEEF, 0x12345678, 0xCAFEBABE, 0xA5A55A5A) for a total of 52 characterization points.
//
// Per point:
//   - 500 active injection cycles (long run for stable tail statistics)
//   - Clean DUT reset and PRNG re-seeding + 10k warm-up flips
//   - Queue hop_counts cleared per run (zero cross-point contamination)
//   - DRAIN_TIMEOUT = 5000 cycles + quiet-period stray flit check
//   - Complete logging to ../reports/stage6_curve.csv

`timescale 1ns/1ps

module tb_stage6;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Sweep Parameters
    // ------------------------------------------------------------------
    localparam int NUM_PROBS = 13;
    localparam int PROBS [NUM_PROBS] = '{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

    localparam int NUM_SEEDS = 4;
    localparam logic [31:0] SEEDS [NUM_SEEDS] = '{
        32'hDEAD_BEEF,
        32'h1234_5678,
        32'hCAFE_BABE,
        32'hA5A5_5A5A
    };

    localparam int E6_CYCLES     = 500;  // 500 active cycles for stable tail percentiles
    localparam int DRAIN_TIMEOUT = 5000; // 5000 cycles safety margin for high-load non-aging priority drains

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
    // Deflection-event extraction
    // defl_arr[x][y] = deflect_resolver.deflected[3:0] for router (x,y)
    //   bit 0 = SRC_N deflected, 1 = SRC_S, 2 = SRC_E, 3 = SRC_W
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
    // Scoreboard and Per-Run Counters
    // ------------------------------------------------------------------
    flit_record_t pending [int];
    int next_tag                = 1;

    // Cumulative test suite counters
    int total_pass_count        = 0;
    int total_fail_count        = 0;

    // Current run counters (reset before each seed/load point)
    int run_attempted           = 0;
    int run_injected            = 0;
    int run_ejected             = 0;
    int run_correct             = 0;
    int run_excess_hop          = 0;
    int run_deflection_events   = 0;
    int run_defl_from_n         = 0;
    int run_defl_from_s         = 0;
    int run_defl_from_e         = 0;
    int run_defl_from_w         = 0;
    int run_pass                = 0;
    int run_fail                = 0;

    // Hop count queue for the CURRENT run only
    int hop_counts [$];

    // ------------------------------------------------------------------
    // xorshift32 PRNG
    // ------------------------------------------------------------------
    logic [31:0] prng_state;

    function automatic logic [31:0] prng_next();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

    // ------------------------------------------------------------------
    // Simulation cycle counter and file handle
    // ------------------------------------------------------------------
    int sim_cycle = 0;
    int curve_fd;

    // ------------------------------------------------------------------
    // Module-level ejection monitor temporaries
    // ------------------------------------------------------------------
    int           mon_tag;
    int           mon_eject_cyc;
    int           mon_hops;
    int           mon_min_hops;
    flit_record_t mon_rec;
    logic         mon_dest_ok;

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
    //   1. sim_cycle++
    //   2. #1 — wait for NBA region & always_comb logic to settle
    //   3. Deflection event counting across all 16 routers and per-port
    //   4. Ejection monitor: match tag, calculate hops, check untracked
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        sim_cycle = sim_cycle + 1;
        #1;

        if(rst_n) begin

            // Deflection event counting
            for(int cx = 0; cx < TORUS_X; cx++) begin
                for(int cy = 0; cy < TORUS_Y; cy++) begin
                    run_deflection_events += $countones(defl_arr[cx][cy]);
                    run_defl_from_n += int'(defl_arr[cx][cy][0]);
                    run_defl_from_s += int'(defl_arr[cx][cy][1]);
                    run_defl_from_e += int'(defl_arr[cx][cy][2]);
                    run_defl_from_w += int'(defl_arr[cx][cy][3]);
                end
            end

            // Ejection monitor
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
                                total_fail_count++;
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
                            // Untracked ejection is FAIL (strict safety invariant)
                            $display("  [FAIL] Untracked ejection tag=%0d at (%0d,%0d) — stray or duplicate",
                                     mon_tag, pi, pj);
                            total_fail_count++;
                            run_fail++;
                        end
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // Helper Tasks
    // ------------------------------------------------------------------
    function automatic flit_t make_flit(
        logic [X_BITS-1:0]     xd,
        logic [Y_BITS-1:0]     yd,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = xd;
        make_flit.Y_dest = yd;
        make_flit.data   = dat;
    endfunction

    task automatic release_injectors();
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
    endtask

    task automatic wait_drain(int timeout_cycles, string phase_label, output int drain_cyc);
        int t;
        t = 0;
        while(pending.num() > 0 && t < timeout_cycles) begin
            @(posedge clk); #1;
            t++;
        end
        drain_cyc = t;
        if(pending.num() != 0) begin
            $display("  [FAIL] %s: %0d flit(s) lost/undrained after %0d drain cycles",
                     phase_label, pending.num(), timeout_cycles);
            foreach(pending[ptag]) begin
                $display("         Pending tag=%0d src=(%0d,%0d) dest=(%0d,%0d) inj_cyc=%0d",
                         ptag, pending[ptag].src_x, pending[ptag].src_y,
                         pending[ptag].dest_x, pending[ptag].dest_y,
                         pending[ptag].inject_cycle);
            end
            total_fail_count++;
            run_fail++;
        end else begin
            total_pass_count++;
            run_pass++;
        end
    endtask

    task automatic stray_flit_check(string phase_label);
        int wait_cyc;
        logic had_stray;
        wait_cyc  = 2 * (TORUS_X + TORUS_Y);
        had_stray = 1'b0;
        repeat(wait_cyc) begin
            @(posedge clk); #1;
            for(int pi = 0; pi < TORUS_X; pi++)
                for(int pj = 0; pj < TORUS_Y; pj++)
                    if(ej_valid_arr[pi][pj]) had_stray = 1'b1;
        end
        if(had_stray) begin
            $display("  [FAIL] %s: stray ejections during quiet period", phase_label);
            total_fail_count++;
            run_fail++;
        end else begin
            total_pass_count++;
            run_pass++;
        end
        if(pending.num() != 0) begin
            $display("  [FAIL] %s: pending not empty after quiet period", phase_label);
            total_fail_count++;
            run_fail++;
        end else begin
            total_pass_count++;
            run_pass++;
        end
    endtask

    // ------------------------------------------------------------------
    // Main Characterization Sequence
    // ------------------------------------------------------------------
    initial begin
        // ALL variable declarations at the start of initial block (XSIM rule)
        int p_idx, s_idx;
        int cur_prob;
        logic [31:0] cur_seed;
        int rdx, rdy, tag;
        logic [31:0] pv;
        int actual_drain;
        int run_rejected;
        real target_rate_pct, attempted_rate_pct, achieved_rate_pct;
        real excess_hop_pct, devt_per_flit;
        real defl_n_pct, defl_s_pct, defl_e_pct, defl_w_pct;
        int sorted_hops [];
        int p99i, p999i;
        real sum_h, mean_h, min_h, max_h, p99_h, p999_h;
        int total_runs_completed;

        total_runs_completed = 0;

        // Init drivers
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        // Open Stage 6 Curve CSV
        curve_fd = $fopen("../reports/stage6_curve.csv", "w");
        $fdisplay(curve_fd, "prob_16,seed,target_rate_pct,attempted_rate_pct,achieved_rate_pct,attempted_flits,accepted_flits,rejected_flits,ejected_flits,excess_hop_pct,devt_per_flit,defl_n_pct,defl_s_pct,defl_e_pct,defl_w_pct,min_hops,mean_hops,p99_hops,p999_hops,max_hops,drain_cycles,pass_count,fail_count");

        $display("=========================================================================================");
        $display("=== STAGE 6: Load-Latency and Deflection Curve Sweep (4x4 Baseline)                   ===");
        $display("=== Offered Load: 1/16 to 13/16 (13 levels) x 4 seeds = 52 characterization points     ===");
        $display("=== Cycles per point: %0d  |  Drain Timeout: %0d                                    ===",
                 E6_CYCLES, DRAIN_TIMEOUT);
        $display("=========================================================================================");

        // Iterate over offered load levels
        for(p_idx = 0; p_idx < NUM_PROBS; p_idx++) begin
            cur_prob = PROBS[p_idx];
            target_rate_pct = real'(cur_prob) * 100.0 / 16.0;

            $display("\n>>> Load Point %0d/%0d: prob=%0d/16 (~%.2f%% offered load) <<<",
                     p_idx + 1, NUM_PROBS, cur_prob, target_rate_pct);

            // Iterate over seeds
            for(s_idx = 0; s_idx < NUM_SEEDS; s_idx++) begin
                cur_seed = SEEDS[s_idx];

                // 1. Reset DUT and TB state
                rst_n = 0;
                repeat(5) @(posedge clk);
                rst_n = 1;
                repeat(2) @(posedge clk);
                #1;

                release_injectors();
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
                run_pass              = 0;
                run_fail              = 0;

                // Seed and warm up PRNG (10k coin flips)
                prng_state = cur_seed;
                for(int k = 0; k < 10000; k++) begin
                    pv = prng_next();
                end

                // 2. Active injection loop
                for(int cyc = 0; cyc < E6_CYCLES; cyc++) begin

                    // Phase 1: Select PEs and drive injection ports
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            pv = prng_next();
                            if(pv[3:0] < cur_prob) begin
                                pv  = prng_next();
                                rdx = int'(pv % TORUS_X);
                                pv  = prng_next();
                                rdy = int'(pv % TORUS_Y);
                                if(rdx == ri && rdy == rj) rdx = (ri + 1) % TORUS_X;

                                run_attempted++;
                                tag = next_tag++;
                                pending[tag].src_x        = ri;
                                pending[tag].src_y        = rj;
                                pending[tag].dest_x       = rdx;
                                pending[tag].dest_y       = rdy;
                                pending[tag].inject_cycle = sim_cycle;
                                run_injected++;

                                inj_valid_drv[ri][rj] = 1'b1;
                                inj_flit_drv[ri][rj]  = make_flit(X_BITS'(rdx), Y_BITS'(rdy),
                                                                   DATA_WIDTH'(tag));
                            end
                        end
                    end

                    // Phase 2: Settle combinational logic (#1), rollback rejected flits
                    #1;
                    for(int ri = 0; ri < TORUS_X; ri++) begin
                        for(int rj = 0; rj < TORUS_Y; rj++) begin
                            if(inj_valid_drv[ri][rj] && !inj_ready_arr[ri][rj]) begin
                                tag = int'(inj_flit_drv[ri][rj].data);
                                pending.delete(tag);
                                run_injected--;
                                inj_valid_drv[ri][rj] = 1'b0;
                                inj_flit_drv[ri][rj]  = '0;
                            end
                        end
                    end

                    // Phase 3: Clock edge capture, release injection drivers
                    @(posedge clk); #1;
                    release_injectors();
                end

                // 3. Drain and verify quiet network
                wait_drain(DRAIN_TIMEOUT, $sformatf("P%0d_S%0d", cur_prob, s_idx), actual_drain);
                stray_flit_check($sformatf("P%0d_S%0d", cur_prob, s_idx));

                // 4. Calculate point metrics
                run_rejected      = run_attempted - run_injected;
                attempted_rate_pct = real'(run_attempted) * 100.0 / real'(E6_CYCLES * TORUS_X * TORUS_Y);
                achieved_rate_pct  = real'(run_injected)  * 100.0 / real'(E6_CYCLES * TORUS_X * TORUS_Y);
                excess_hop_pct     = (run_ejected > 0) ?
                                     real'(run_excess_hop) * 100.0 / real'(run_ejected) : 0.0;
                devt_per_flit      = (run_ejected > 0) ?
                                     real'(run_deflection_events) / real'(run_ejected)  : 0.0;

                defl_n_pct = (run_deflection_events > 0) ?
                             real'(run_defl_from_n) * 100.0 / real'(run_deflection_events) : 0.0;
                defl_s_pct = (run_deflection_events > 0) ?
                             real'(run_defl_from_s) * 100.0 / real'(run_deflection_events) : 0.0;
                defl_e_pct = (run_deflection_events > 0) ?
                             real'(run_defl_from_e) * 100.0 / real'(run_deflection_events) : 0.0;
                defl_w_pct = (run_deflection_events > 0) ?
                             real'(run_defl_from_w) * 100.0 / real'(run_deflection_events) : 0.0;

                // Sort hop counts of this run
                sorted_hops = hop_counts;
                sorted_hops.sort();

                if(sorted_hops.size() > 0) begin
                    p99i  = int'(real'(sorted_hops.size() - 1) * 0.990);
                    p999i = int'(real'(sorted_hops.size() - 1) * 0.999);
                    min_h = real'(sorted_hops[0]);
                    max_h = real'(sorted_hops[sorted_hops.size() - 1]);
                    p99_h = real'(sorted_hops[p99i]);
                    p999_h = real'(sorted_hops[p999i]);

                    sum_h = 0.0;
                    for(int k = 0; k < sorted_hops.size(); k++) sum_h += real'(sorted_hops[k]);
                    mean_h = sum_h / real'(sorted_hops.size());
                end else begin
                    min_h = 0.0; max_h = 0.0; p99_h = 0.0; p999_h = 0.0; mean_h = 0.0;
                end

                // Write to CSV
                $fdisplay(curve_fd,
                    "%0d,0x%08h,%.2f,%.2f,%.2f,%0d,%0d,%0d,%0d,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.0f,%.2f,%.0f,%.0f,%.0f,%0d,%0d,%0d",
                    cur_prob, cur_seed, target_rate_pct,
                    attempted_rate_pct, achieved_rate_pct,
                    run_attempted, run_injected, run_rejected, run_ejected,
                    excess_hop_pct, devt_per_flit,
                    defl_n_pct, defl_s_pct, defl_e_pct, defl_w_pct,
                    min_h, mean_h, p99_h, p999_h, max_h,
                    actual_drain, run_pass, run_fail
                );

                // Print concise progress line
                $display("  [prob=%2d/16 seed=0x%08h] Att=%.1f%% Ach=%.1f%% (rej=%0d) | MeanHops=%.2f p99=%.0f Max=%.0f | XHop=%.1f%% Devt/flit=%.2f | N:%.0f%% S:%.0f%% E:%.0f%% W:%.0f%% [drain=%0d cyc]",
                         cur_prob, cur_seed,
                         attempted_rate_pct, achieved_rate_pct, run_rejected,
                         mean_h, p99_h, max_h,
                         excess_hop_pct, devt_per_flit,
                         defl_n_pct, defl_s_pct, defl_e_pct, defl_w_pct,
                         actual_drain);

                total_runs_completed++;
            end
        end

        $fclose(curve_fd);

        $display("\n=========================================================================================");
        $display("=== STAGE 6 COMPLETE: 52/52 runs finished                                             ===");
        $display("=== Total Pass Checks: %0d  |  Total Fail Checks: %0d                                 ===",
                 total_pass_count, total_fail_count);
        $display("=== Curve CSV: ../reports/stage6_curve.csv                                            ===");
        $display("=========================================================================================");

        if(total_fail_count == 0)
            $display(">>> ALL STAGE 6 RUNS PASSED CONSERVATION AND STRAY CHECKS.");
        else
            $display(">>> FAILURES DETECTED IN STAGE 6 RUNS.");

        $finish;
    end

endmodule
