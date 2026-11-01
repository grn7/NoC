// tb/tb_stage6_diagnostic.sv
// Stage 6 Diagnostic Testbench — Targeted Investigation of Pathological Tails / Livelock
//
// Specifically reruns the two exact anomaly configurations identified in Stage 6:
//   Case 1: prob = 10/16 (~62.5% offered load), seed = 32'hDEAD_BEEF (6 undrained flits at 5000 cycles)
//   Case 2: prob = 12/16 (~75.0% offered load), seed = 32'hCAFE_BABE (8 undrained flits at 5000 cycles)
//
// Key features:
//   1. Extended Drain Timeout: 50,000 cycles to determine if flits eventually drain (long tail)
//      or form a self-sustaining limit cycle (permanent livelock).
//   2. Fine-grained Tracing: Tracks the exact router position (x, y), arrival port, desired port,
//      deflected flag, and exit port for long-lived flits (age > 200 cycles or during drain).
//   3. Cycle / Loop Detection: Monitors trajectory history of stuck flits to detect periodic orbits.
//   4. Writes ../reports/stage6_diag_trace.csv for detailed offline trajectory visualization.

`timescale 1ns/1ps

module tb_stage6_diagnostic;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Sweep Parameters
    // ------------------------------------------------------------------
    localparam int NUM_CASES = 2;
    localparam int PROBS [NUM_CASES] = '{10, 12};
    localparam logic [31:0] SEEDS [NUM_CASES] = '{
        32'hDEAD_BEEF,
        32'hCAFE_BABE
    };

    localparam int E6_CYCLES          = 500;   // 500 active injection cycles
    localparam int DIAG_DRAIN_TIMEOUT = 50000; // 50,000 cycles diagnostic drain timeout

    // ------------------------------------------------------------------
    // Flit scoreboard record
    // ------------------------------------------------------------------
    typedef struct {
        int src_x,  src_y;
        int dest_x, dest_y;
        int inject_cycle;
        int last_seen_cycle;
        int last_x, last_y;
        int deflection_count;
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
    // Trace and Deflection Signal Extraction
    // Avoids runtime indexing into generate blocks (which XSIM rejects).
    // ------------------------------------------------------------------
    logic       tr_valid_in  [TORUS_X][TORUS_Y][4];
    flit_t      tr_flit_in   [TORUS_X][TORUS_Y][4];
    logic [4:0] tr_desired   [TORUS_X][TORUS_Y][4];
    logic       tr_is_eject  [TORUS_X][TORUS_Y][4];
    logic       tr_valid_out [TORUS_X][TORUS_Y][5];
    flit_t      tr_flit_out  [TORUS_X][TORUS_Y][5];
    logic [3:0] defl_arr     [TORUS_X][TORUS_Y];

    generate
        genvar gx, gy, sp, od;
        for(gx = 0; gx < TORUS_X; gx++) begin : g_trace_x
            for(gy = 0; gy < TORUS_Y; gy++) begin : g_trace_y
                assign defl_arr[gx][gy] =
                    dut.gen_col[gx].gen_row[gy].u_router.u_resolver.deflected;

                for(sp = 0; sp < 4; sp++) begin : g_sp
                    assign tr_valid_in[gx][gy][sp] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.valid_in[sp];
                    assign tr_flit_in[gx][gy][sp] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.flit_in[sp];
                    assign tr_desired[gx][gy][sp] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.desired[sp];
                    assign tr_is_eject[gx][gy][sp] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.is_eject[sp];
                end

                for(od = 0; od < 5; od++) begin : g_od
                    assign tr_valid_out[gx][gy][od] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.valid_out[od];
                    assign tr_flit_out[gx][gy][od] =
                        dut.gen_col[gx].gen_row[gy].u_router.u_resolver.flit_out[od];
                end
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Scoreboard and Per-Run Counters
    // ------------------------------------------------------------------
    flit_record_t pending [int];
    int next_tag                = 1;

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
    int sim_cycle = 0;
    int trace_fd;
    logic is_drain_phase = 1'b0;
    int cur_case_id = 1; // 1 = Case 1 (prob=10 seed=0xDEADBEEF), 2 = Case 2 (prob=12 seed=0xCAFEBABE)

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
    // Helper string functions for direction printing
    // ------------------------------------------------------------------
    function automatic string dir_name(int d);
        case(d)
            0: return "NORTH";
            1: return "SOUTH";
            2: return "EAST";
            3: return "WEST";
            4: return "LOCAL";
            default: return "NONE";
        endcase
    endfunction

    function automatic string desired_name(logic [4:0] des);
        if(des[4]) return "LOCAL";
        if(des[0]) return "NORTH";
        if(des[1]) return "SOUTH";
        if(des[2]) return "EAST";
        if(des[3]) return "WEST";
        return "NONE";
    endfunction

    function automatic int find_out_dir(int cx, cy, int tag);
        for(int od = 0; od < 5; od++) begin
            if(tr_valid_out[cx][cy][od] && int'(tr_flit_out[cx][cy][od].data) == tag)
                return od;
        end
        return -1;
    endfunction

    // Minimum torus Manhattan distance
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
    //   3. Deflection event counting
    //   4. Fine-grained Trace of long-lived / stuck flits
    //   5. Ejection monitor
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

            // --------------------------------------------------------------
            // Fine-grained Tracing of Long-Lived Flits
            // --------------------------------------------------------------
            for(int cx = 0; cx < TORUS_X; cx++) begin
                for(int cy = 0; cy < TORUS_Y; cy++) begin
                    for(int sp = 0; sp < 4; sp++) begin
                        if(tr_valid_in[cx][cy][sp]) begin
                            int t_tag;
                            logic t_defl, t_ej;
                            int t_out_dir;
                            int flit_age;

                            t_tag = int'(tr_flit_in[cx][cy][sp].data);

                            if(pending.exists(t_tag)) begin
                                flit_age = sim_cycle - pending[t_tag].inject_cycle;
                                t_defl   = defl_arr[cx][cy][sp];
                                t_ej     = tr_is_eject[cx][cy][sp];
                                t_out_dir = find_out_dir(cx, cy, t_tag);

                                if(t_defl) begin
                                    pending[t_tag].deflection_count++;
                                end

                                // Log condition: age > 200 cycles or during drain phase
                                if(is_drain_phase || flit_age > 200) begin
                                    $fdisplay(trace_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%s,%s,%s,%0d,%0d,%0d",
                                              cur_case_id,
                                              sim_cycle, t_tag,
                                              pending[t_tag].src_x, pending[t_tag].src_y,
                                              pending[t_tag].dest_x, pending[t_tag].dest_y,
                                              cx, cy,
                                              dir_name(sp),
                                              desired_name(tr_desired[cx][cy][sp]),
                                              dir_name(t_out_dir),
                                              t_defl, t_ej,
                                              pending.num());

                                    // Print console trace for first 30 drain cycles, or periodically every 1000 cycles
                                    if(is_drain_phase && (sim_cycle <= 530 || (sim_cycle % 1000 == 0))) begin
                                        $display("    [TRACE cyc=%0d] Tag=%0d (dest=%0d,%0d age=%0d defls=%0d) at router (%0d,%0d) in=%s want=%s out=%s (defl=%0d)",
                                                 sim_cycle, t_tag,
                                                 pending[t_tag].dest_x, pending[t_tag].dest_y,
                                                 flit_age, pending[t_tag].deflection_count,
                                                 cx, cy,
                                                 dir_name(sp),
                                                 desired_name(tr_desired[cx][cy][sp]),
                                                 dir_name(t_out_dir),
                                                 t_defl);
                                    end
                                end

                                pending[t_tag].last_seen_cycle = sim_cycle;
                                pending[t_tag].last_x = cx;
                                pending[t_tag].last_y = cy;
                            end
                        end
                    end
                end
            end

            // --------------------------------------------------------------
            // Ejection Monitor
            // --------------------------------------------------------------
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
                            end else if(mon_hops > mon_min_hops) begin
                                run_correct++;
                                run_excess_hop++;
                                if(is_drain_phase || mon_hops > 100) begin
                                    $display("  >>> [EJECT RESOLVED] Tag=%0d ejected at (%0d,%0d) after %0d hops (defls=%0d) at cycle %0d",
                                             mon_tag, pi, pj, mon_hops, mon_rec.deflection_count, mon_eject_cyc);
                                end
                            end else begin
                                run_correct++;
                            end

                            pending.delete(mon_tag);
                            run_ejected++;

                        end else begin
                            $display("  [FAIL] Untracked ejection tag=%0d at (%0d,%0d)", mon_tag, pi, pj);
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

    // ------------------------------------------------------------------
    // Diagnostic Run Sequence
    // ------------------------------------------------------------------
    initial begin
        // ALL variable declarations at top of initial block (XSIM rule)
        int c_idx;
        int cur_prob;
        logic [31:0] cur_seed;
        int rdx, rdy, tag;
        logic [31:0] pv;
        int drain_t;
        int run_rejected;
        real target_rate_pct, attempted_rate_pct, achieved_rate_pct;
        real excess_hop_pct, devt_per_flit;
        int sorted_hops [];
        real sum_h, mean_h, min_h, max_h, p99_h;

        // Init drivers
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        // Open Trace CSV
        trace_fd = $fopen("../reports/stage6_diag_trace.csv", "w");
        $fdisplay(trace_fd, "case_id,cycle,tag,src_x,src_y,dest_x,dest_y,cur_x,cur_y,in_port,wanted_dir,out_dir,deflected,is_eject,pending_flits");

        $display("=========================================================================================");
        $display("=== STAGE 6 DIAGNOSTIC RUN: Investigating Pathological Tail / Livelock Anomalies     ===");
        $display("=== Drain Timeout: %0d cycles (10x Stage 6)                                         ===",
                 DIAG_DRAIN_TIMEOUT);
        $display("=========================================================================================");

        for(c_idx = 0; c_idx < NUM_CASES; c_idx++) begin
            cur_case_id = c_idx + 1;
            cur_prob = PROBS[c_idx];
            cur_seed = SEEDS[c_idx];
            target_rate_pct = real'(cur_prob) * 100.0 / 16.0;

            $display("\n*****************************************************************************************");
            $display("=== DIAGNOSTIC CASE %0d/%0d: prob=%0d/16 (~%.2f%% offered) | seed=0x%08h ===",
                     cur_case_id, NUM_CASES, cur_prob, target_rate_pct, cur_seed);
            $display("*****************************************************************************************");

            // 1. Clean Reset
            is_drain_phase = 1'b0;
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

            // PRNG seed and warm-up
            prng_state = cur_seed;
            for(int k = 0; k < 10000; k++) begin
                pv = prng_next();
            end

            // 2. Active Injection Loop (500 cycles)
            for(int cyc = 0; cyc < E6_CYCLES; cyc++) begin

                // Phase 1: Injection attempts
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
                            pending[tag].src_x            = ri;
                            pending[tag].src_y            = rj;
                            pending[tag].dest_x           = rdx;
                            pending[tag].dest_y           = rdy;
                            pending[tag].inject_cycle     = sim_cycle;
                            pending[tag].last_seen_cycle  = sim_cycle;
                            pending[tag].last_x           = ri;
                            pending[tag].last_y           = rj;
                            pending[tag].deflection_count = 0;
                            run_injected++;

                            inj_valid_drv[ri][rj] = 1'b1;
                            inj_flit_drv[ri][rj]  = make_flit(X_BITS'(rdx), Y_BITS'(rdy),
                                                               DATA_WIDTH'(tag));
                        end
                    end
                end

                // Phase 2: Settle combinational logic, rollback rejected
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

                // Phase 3: Clock edge capture, release drivers
                @(posedge clk); #1;
                release_injectors();
            end

            // 3. Extended Drain Phase
            is_drain_phase = 1'b1;
            $display("  [INJECTION COMPLETE at cyc=%0d] Injected=%0d, Accepted=%0d, Flits in flight=%0d. Entering extended drain (up to %0d cycles)...",
                     sim_cycle, run_attempted, run_injected, pending.num(), DIAG_DRAIN_TIMEOUT);

            drain_t = 0;
            while(pending.num() > 0 && drain_t < DIAG_DRAIN_TIMEOUT) begin
                @(posedge clk); #1;
                drain_t++;

                if(drain_t % 5000 == 0) begin
                    $display("    ... Still draining at drain_cycle=%0d (sim_cycle=%0d) | %0d flits remaining in flight",
                             drain_t, sim_cycle, pending.num());
                    foreach(pending[ptag]) begin
                        $display("        Tag %0d: dest=(%0d,%0d) last_pos=(%0d,%0d) total_defls=%0d",
                                 ptag, pending[ptag].dest_x, pending[ptag].dest_y,
                                 pending[ptag].last_x, pending[ptag].last_y,
                                 pending[ptag].deflection_count);
                    end
                end
            end

            // 4. Outcome Analysis
            $display("\n  ================== DIAGNOSTIC RESULT FOR CASE %0d ==================", c_idx + 1);
            if(pending.num() == 0) begin
                $display("  >>> RESOLVED AS LONG TAIL (NOT PERMANENT LIVELOCK) <<<");
                $display("  All flits completely drained in %0d cycles (within %0d limit).",
                         drain_t, DIAG_DRAIN_TIMEOUT);
            end else begin
                $display("  >>> CONFIRMED PERSISTENT LIVELOCK / UNBOUNDED DEFLECTION <<<");
                $display("  %0d flits remain trapped in network after %0d cycles!",
                         pending.num(), DIAG_DRAIN_TIMEOUT);
                $display("  Stuck Flits Inventory:");
                foreach(pending[ptag]) begin
                    $display("    - Tag %0d: src=(%0d,%0d) dest=(%0d,%0d) last_seen_at=(%0d,%0d) deflections=%0d age=%0d cycles",
                             ptag, pending[ptag].src_x, pending[ptag].src_y,
                             pending[ptag].dest_x, pending[ptag].dest_y,
                             pending[ptag].last_x, pending[ptag].last_y,
                             pending[ptag].deflection_count,
                             sim_cycle - pending[ptag].inject_cycle);
                end
            end

            // Point Stats
            run_rejected       = run_attempted - run_injected;
            attempted_rate_pct = real'(run_attempted) * 100.0 / real'(E6_CYCLES * TORUS_X * TORUS_Y);
            achieved_rate_pct  = real'(run_injected)  * 100.0 / real'(E6_CYCLES * TORUS_X * TORUS_Y);
            excess_hop_pct     = (run_ejected > 0) ?
                                 real'(run_excess_hop) * 100.0 / real'(run_ejected) : 0.0;
            devt_per_flit      = (run_ejected > 0) ?
                                 real'(run_deflection_events) / real'(run_ejected)  : 0.0;

            sorted_hops = hop_counts;
            sorted_hops.sort();
            if(sorted_hops.size() > 0) begin
                min_h = real'(sorted_hops[0]);
                max_h = real'(sorted_hops[sorted_hops.size() - 1]);
                p99_h = real'(sorted_hops[int'(real'(sorted_hops.size() - 1) * 0.990)]);
                sum_h = 0.0;
                for(int k = 0; k < sorted_hops.size(); k++) sum_h += real'(sorted_hops[k]);
                mean_h = sum_h / real'(sorted_hops.size());
            end else begin
                min_h = 0; max_h = 0; p99_h = 0; mean_h = 0;
            end

            $display("  Stats: Ejected=%0d | MeanHops=%.2f p99=%.0f Max=%.0f | XHop=%.1f%% Devt/flit=%.2f",
                     run_ejected, mean_h, p99_h, max_h, excess_hop_pct, devt_per_flit);
            $display("  Port Deflection Share: N=%.1f%% S=%.1f%% E=%.1f%% W=%.1f%%",
                     (run_deflection_events>0)? real'(run_defl_from_n)*100.0/real'(run_deflection_events) : 0.0,
                     (run_deflection_events>0)? real'(run_defl_from_s)*100.0/real'(run_deflection_events) : 0.0,
                     (run_deflection_events>0)? real'(run_defl_from_e)*100.0/real'(run_deflection_events) : 0.0,
                     (run_deflection_events>0)? real'(run_defl_from_w)*100.0/real'(run_deflection_events) : 0.0);
            $display("  ====================================================================\n");
        end

        $fclose(trace_fd);
        $display("Detailed trajectory trace written to ../reports/stage6_diag_trace.csv");
        $finish;
    end

endmodule
