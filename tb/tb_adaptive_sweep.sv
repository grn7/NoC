// tb/tb_adaptive_sweep.sv
// Stage 6/7-Style Benchmark Sweep for Adaptive Cluster-Shared Buffer Pool
//
// Capabilities:
// 1. Supports 3 traffic patterns via +TRAFFIC=<uniform|hotspot|migrating>:
//    - uniform   : standard uniform random traffic (URT)
//    - hotspot   : static hotspot directed at node (2,2)
//    - migrating : migrating hotspot shifting every HOTSPOT_PERIOD cycles across clusters
// 2. Sweeps offered load finely across 13 injection probability points (1..13 / 16)
//    across 4 deterministic PRNG seeds (0xDEADBEEF, 0x12345678, 0xCAFEBABE, 0xA5A55A5A).
// 3. Complete performance metrics logged to CSV:
//    - throughput, mean latency, p99 latency, max latency, deflection rate
//    - buffer requests and grants
//    - single-port bottleneck losses vs quota/budget exhaustion losses (Review Point 2)
// 4. Fully compliant with XSIM variable declaration rules and no $urandom_range.

`timescale 1ns/1ps

module tb_adaptive_sweep;

    import noc_pkg::*;

    // -------------------------------------------------------------------------
    // Sweep Parameters
    // -------------------------------------------------------------------------
    localparam int NUM_PROBS = 13;
    localparam int PROBS [NUM_PROBS] = '{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

    localparam int NUM_SEEDS = 4;
    localparam logic [31:0] SEEDS [NUM_SEEDS] = '{
        32'hDEAD_BEEF,
        32'h1234_5678,
        32'hCAFE_BABE,
        32'hA5A5_5A5A
    };

    localparam int ACTIVE_CYCLES   = 500;
    localparam int DRAIN_TIMEOUT   = 5000;
    localparam int HOTSPOT_PERIOD  = 100;

    // -------------------------------------------------------------------------
    // Clock and Reset
    // -------------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // Interfaces & DUT
    // -------------------------------------------------------------------------
    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    // -------------------------------------------------------------------------
    // Static Driver & Monitor Extraction Arrays
    // -------------------------------------------------------------------------
    logic  inj_valid_drv [TORUS_X][TORUS_Y];
    flit_t inj_flit_drv  [TORUS_X][TORUS_Y];

    logic  ej_valid_arr  [TORUS_X][TORUS_Y];
    flit_t ej_flit_arr   [TORUS_X][TORUS_Y];
    logic  inj_ready_arr [TORUS_X][TORUS_Y];

    generate
        genvar x, y;
        for (x = 0; x < TORUS_X; x++) begin : g_map_x
            for (y = 0; y < TORUS_Y; y++) begin : g_map_y
                assign pe_if[x][y].inj_valid = inj_valid_drv[x][y];
                assign pe_if[x][y].inj_flit  = inj_flit_drv[x][y];
                assign ej_valid_arr[x][y]    = pe_if[x][y].ej_valid;
                assign ej_flit_arr[x][y]     = pe_if[x][y].ej_flit;
                assign inj_ready_arr[x][y]   = pe_if[x][y].inj_ready;
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Diagnostic Extraction Arrays
    // -------------------------------------------------------------------------
    logic [2:0] deflect_count_arr [TORUS_X][TORUS_Y];
    logic pool_req_arr            [TORUS_X][TORUS_Y];
    logic pool_grant_arr          [TORUS_X][TORUS_Y];

    generate
        genvar rx, ry;
        for (rx = 0; rx < TORUS_X; rx++) begin : g_rdiag_x
            for (ry = 0; ry < TORUS_Y; ry++) begin : g_rdiag_y
                assign deflect_count_arr[rx][ry] = dut.gen_col[rx].gen_row[ry].u_router.u_resolver.actual_deflect_count;
                assign pool_req_arr[rx][ry]      = dut.gen_col[rx].gen_row[ry].u_router.pool_port.req;
                assign pool_grant_arr[rx][ry]    = dut.gen_col[rx].gen_row[ry].u_router.pool_port.grant;
            end
        end
    endgenerate

    logic [ROUTERS_PER_CLUSTER-1:0] port_loss_arr  [CLUSTERS_X][CLUSTERS_Y];
    logic [ROUTERS_PER_CLUSTER-1:0] quota_loss_arr [CLUSTERS_X][CLUSTERS_Y];

    generate
        genvar cx, cy;
        if (POOL_ENABLE) begin : g_pool_diag
            for (cx = 0; cx < CLUSTERS_X; cx++) begin : g_cdiag_x
                for (cy = 0; cy < CLUSTERS_Y; cy++) begin : g_cdiag_y
                    assign port_loss_arr[cx][cy]  = dut.gen_c_col[cx].gen_c_row[cy].port_bottleneck_loss;
                    assign quota_loss_arr[cx][cy] = dut.gen_c_col[cx].gen_c_row[cy].quota_exhausted_loss;
                end
            end
        end else begin : g_no_pool_diag
            for (cx = 0; cx < CLUSTERS_X; cx++) begin : g_cdiag_x
                for (cy = 0; cy < CLUSTERS_Y; cy++) begin : g_cdiag_y
                    assign port_loss_arr[cx][cy]  = '0;
                    assign quota_loss_arr[cx][cy] = '0;
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Scoreboard Record
    // -------------------------------------------------------------------------
    typedef struct {
        int src_x,  src_y;
        int dest_x, dest_y;
        int inject_cycle;
    } flit_record_t;

    flit_record_t pending [int];
    int           hop_counts [$];

    int sim_cycle             = 0;
    int run_attempted         = 0;
    int run_injected          = 0;
    int run_ejected           = 0;
    int run_deflections       = 0;
    int run_buf_reqs          = 0;
    int run_buf_grants        = 0;
    int run_port_loss         = 0;
    int run_quota_loss        = 0;
    int run_fail              = 0;

    // Helper functions
    function automatic flit_t make_flit(int dx, int dy, int tag);
        flit_t f;
        f.X_dest = X_BITS'(dx);
        f.Y_dest = Y_BITS'(dy);
        f.data   = DATA_WIDTH'(tag);
        return f;
    endfunction

    // -------------------------------------------------------------------------
    // Ejection & Event Monitor
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        sim_cycle++;
        #1;
        if (rst_n) begin
            // 1. Deflections and pool event counters
            for (int x = 0; x < TORUS_X; x++) begin
                for (int y = 0; y < TORUS_Y; y++) begin
                    run_deflections           += int'(deflect_count_arr[x][y]);
                    if (pool_req_arr[x][y])    run_buf_reqs++;
                    if (pool_grant_arr[x][y])  run_buf_grants++;
                end
            end

            // 2. Diagnostic loss breakdown
            for (int cx = 0; cx < CLUSTERS_X; cx++) begin
                for (int cy = 0; cy < CLUSTERS_Y; cy++) begin
                    for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                        if (port_loss_arr[cx][cy][r])  run_port_loss++;
                        if (quota_loss_arr[cx][cy][r]) run_quota_loss++;
                    end
                end
            end

            // 3. Ejection tracking
            for (int x = 0; x < TORUS_X; x++) begin
                for (int y = 0; y < TORUS_Y; y++) begin
                    if (ej_valid_arr[x][y]) begin
                        int tag;
                        tag = int'(ej_flit_arr[x][y].data);
                        if (pending.exists(tag)) begin
                            flit_record_t rec;
                            int latency;
                            rec = pending[tag];
                            latency = sim_cycle - rec.inject_cycle;
                            hop_counts.push_back(latency);

                            if ((x != rec.dest_x) || (y != rec.dest_y)) begin
                                $display("  [FAIL] Flit tag %0d ejected at (%0d,%0d), expected (%0d,%0d)",
                                         tag, x, y, rec.dest_x, rec.dest_y);
                                run_fail++;
                            end
                            pending.delete(tag);
                            run_ejected++;
                        end else begin
                            $display("  [FAIL] Untracked ejection of tag %0d at (%0d,%0d)", tag, x, y);
                            run_fail++;
                        end
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Main Sweep Process
    // -------------------------------------------------------------------------
    initial begin
        // Variable declarations at top per XSIM rules
        int p_idx, s_idx;
        int cur_prob;
        logic [31:0] cur_seed;
        logic [31:0] l_state;
        string traffic_mode;
        string csv_filename;
        int csv_fd;
        int status_arg;
        int csv_append_mode;

        int active_cycles_cnt;
        int drain_cycles_cnt;
        int run_timed_out;
        int tag_seq;
        int tag;

        real offered_pct;
        real achieved_inj_rate;
        real throughput_pct;
        real mean_latency;
        real p99_latency;
        int  max_latency;
        real defl_per_flit;
        real buf_grant_rate;

        int sorted_hops [$];
        int n_hops;
        int p99_idx;

        // Parse plusargs
        traffic_mode = "uniform";
        status_arg   = $value$plusargs("TRAFFIC=%s", traffic_mode);

        csv_filename = "../reports/adaptive_sweep.csv";
        status_arg   = $value$plusargs("CSV_OUT=%s", csv_filename);

        csv_append_mode = 0;
        status_arg   = $value$plusargs("CSV_APPEND=%d", csv_append_mode);

        // Initialize drivers
        for (int i = 0; i < TORUS_X; i++) begin
            for (int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        // Open CSV and write header (or append without re-writing header)
        if (csv_append_mode != 0) begin
            csv_fd = $fopen(csv_filename, "a");
        end else begin
            csv_fd = $fopen(csv_filename, "w");
            $fdisplay(csv_fd, "torus_x,torus_y,pool_enable,cluster_x,cluster_y,slots_per_router,pool_size,alpha_q,beta_q,hyst_thresh,traffic,prob_16,offered_pct,seed,attempted,injected,ejected,throughput_pct,inj_rate_pct,mean_lat,p99_lat,max_lat,deflections,defl_per_flit,buf_reqs,buf_grants,port_loss,quota_loss,timed_out,errors");
        end

        $display("=========================================================================================");
        $display("=== ADAPTIVE CLUSTER-SHARED BUFFER POOL BENCHMARK SWEEP                               ===");
        $display("=== POOL_ENABLE=%0d | CLUSTER=%0dx%0d | POOL_SIZE=%0d | TRAFFIC=%s                   ===",
                 POOL_ENABLE, CLUSTER_X, CLUSTER_Y, POOL_SIZE, traffic_mode);
        $display("=== Sweep: %0d probabilities x %0d seeds = %0d characterization points                 ===",
                 NUM_PROBS, NUM_SEEDS, NUM_PROBS * NUM_SEEDS);
        $display("=========================================================================================");

        for (p_idx = 0; p_idx < NUM_PROBS; p_idx++) begin
            cur_prob = PROBS[p_idx];
            offered_pct = (real'(cur_prob) / 16.0) * 100.0;

            for (s_idx = 0; s_idx < NUM_SEEDS; s_idx++) begin
                cur_seed = SEEDS[s_idx];
                l_state  = cur_seed;

                // Warm up PRNG
                for (int w = 0; w < 1000; w++) begin
                    l_state ^= l_state << 13;
                    l_state ^= l_state >> 17;
                    l_state ^= l_state << 5;
                end

                // Reset DUT
                rst_n = 0;
                repeat (5) @(posedge clk);
                rst_n = 1;
                repeat (2) @(posedge clk);
                #1;

                // Reset run metrics
                pending.delete();
                hop_counts.delete();
                run_attempted   = 0;
                run_injected    = 0;
                run_ejected     = 0;
                run_deflections = 0;
                run_buf_reqs    = 0;
                run_buf_grants  = 0;
                run_port_loss   = 0;
                run_quota_loss  = 0;
                run_fail        = 0;
                run_timed_out   = 0;
                tag_seq         = 1;

                // Active injection phase
                for (active_cycles_cnt = 0; active_cycles_cnt < ACTIVE_CYCLES; active_cycles_cnt++) begin
                    int hot_x, hot_y;

                    // Compute migrating hotspot target if in migrating mode
                    if (traffic_mode == "migrating") begin
                        // Reuse Gate 4's verified 4-cluster representative sequence:
                        // Epoch 0: (0,0)           [Cluster (0,0)]
                        // Epoch 1: (TORUS_X-1, 0)   [Cluster (1,0)]
                        // Epoch 2: (TORUS_X-1, TORUS_Y-1) [Cluster (1,1)]
                        // Epoch 3: (0, TORUS_Y-1)   [Cluster (0,1)]
                        // Guaranteed to visit every distinct cluster across 4x4 torus
                        int hot_step;
                        hot_step = (active_cycles_cnt / HOTSPOT_PERIOD) % 4;
                        case (hot_step)
                            0: begin hot_x = 0;           hot_y = 0;           end
                            1: begin hot_x = TORUS_X - 1; hot_y = 0;           end
                            2: begin hot_x = TORUS_X - 1; hot_y = TORUS_Y - 1; end
                            3: begin hot_x = 0;           hot_y = TORUS_Y - 1; end
                            default: begin hot_x = 0; hot_y = 0; end
                        endcase
                    end else begin
                        hot_x = TORUS_X / 2;
                        hot_y = TORUS_Y / 2;
                    end

                    for (int sx = 0; sx < TORUS_X; sx++) begin
                        for (int sy = 0; sy < TORUS_Y; sy++) begin
                            logic [31:0] r;
                            r = l_state;
                            l_state ^= l_state << 13;
                            l_state ^= l_state >> 17;
                            l_state ^= l_state << 5;

                            // Bernoulli injection decision
                            if (int'(r[3:0]) < cur_prob) begin
                                int dx, dy;
                                run_attempted++;

                                // Determine destination
                                if ((traffic_mode == "hotspot" || traffic_mode == "migrating") && r[4]) begin
                                    // 50% chance to target the hotspot node
                                    if (sx != hot_x || sy != hot_y) begin
                                        dx = hot_x;
                                        dy = hot_y;
                                    end else begin
                                        dx = (hot_x + 1) % TORUS_X;
                                        dy = (hot_y + 1) % TORUS_Y;
                                    end
                                end else begin
                                    // Uniform random destination
                                    do begin
                                        r = l_state;
                                        l_state ^= l_state << 13;
                                        l_state ^= l_state >> 17;
                                        l_state ^= l_state << 5;
                                        dx = int'(r % TORUS_X);
                                        r = l_state;
                                        l_state ^= l_state << 13;
                                        l_state ^= l_state >> 17;
                                        l_state ^= l_state << 5;
                                        dy = int'(r % TORUS_Y);
                                    end while (dx == sx && dy == sy);
                                end

                                tag = tag_seq++;
                                pending[tag] = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inject_cycle:sim_cycle};
                                run_injected++;

                                inj_valid_drv[sx][sy] = 1'b1;
                                inj_flit_drv[sx][sy]  = make_flit(dx, dy, tag);
                            end else begin
                                inj_valid_drv[sx][sy] = 1'b0;
                                inj_flit_drv[sx][sy]  = '0;
                            end
                        end
                    end

                    // Phase 2: Settle combinational logic (#1), rollback rejected flits
                    #1;
                    for (int sx = 0; sx < TORUS_X; sx++) begin
                        for (int sy = 0; sy < TORUS_Y; sy++) begin
                            if (inj_valid_drv[sx][sy] && !inj_ready_arr[sx][sy]) begin
                                tag = int'(inj_flit_drv[sx][sy].data);
                                pending.delete(tag);
                                run_injected--;
                                inj_valid_drv[sx][sy] = 1'b0;
                                inj_flit_drv[sx][sy]  = '0;
                            end
                        end
                    end

                    @(posedge clk); #1;
                end

                // Deassert all injections for drain phase
                for (int sx = 0; sx < TORUS_X; sx++) begin
                    for (int sy = 0; sy < TORUS_Y; sy++) begin
                        inj_valid_drv[sx][sy] = 1'b0;
                        inj_flit_drv[sx][sy]  = '0;
                    end
                end

                // Drain phase
                drain_cycles_cnt = 0;
                while (pending.size() > 0 && drain_cycles_cnt < DRAIN_TIMEOUT) begin
                    @(posedge clk); #1;
                    drain_cycles_cnt++;
                end

                if (pending.size() > 0) begin
                    run_timed_out = 1;
                    $display("  [WARN] Point prob=%0d/16 seed=0x%08h: TIMEOUT with %0d flits undrained!",
                             cur_prob, cur_seed, pending.size());
                end

                // Compute point metrics
                achieved_inj_rate = (real'(run_injected) * 100.0) / real'(TORUS_X * TORUS_Y * ACTIVE_CYCLES);
                throughput_pct    = (real'(run_ejected)  * 100.0) / real'(TORUS_X * TORUS_Y * (ACTIVE_CYCLES + drain_cycles_cnt));
                defl_per_flit     = (run_ejected > 0) ? real'(run_deflections) / real'(run_ejected) : 0.0;
                buf_grant_rate    = (run_buf_reqs > 0) ? (real'(run_buf_grants) * 100.0) / real'(run_buf_reqs) : 0.0;

                n_hops = hop_counts.size();
                if (n_hops > 0) begin
                    sorted_hops = hop_counts;
                    sorted_hops.sort();
                    max_latency = sorted_hops[n_hops - 1];

                    mean_latency = 0.0;
                    for (int h = 0; h < n_hops; h++) mean_latency += real'(sorted_hops[h]);
                    mean_latency /= real'(n_hops);

                    p99_idx = int'(real'(n_hops - 1) * 0.99);
                    p99_latency = real'(sorted_hops[p99_idx]);
                end else begin
                    mean_latency = 0.0;
                    p99_latency  = 0.0;
                    max_latency  = 0;
                end

                $display("  [%s] Prob=%2d/16 (%4.1f%%) | Seed=0x%08h | Inj=%0d Ej=%0d | Tput=%4.1f%% | MeanLat=%4.2f P99=%4.1f | Defl/flit=%4.2f | Grants=%0d/%0d (%4.1f%%) | PortLoss=%0d QuotaLoss=%0d",
                         traffic_mode, cur_prob, offered_pct, cur_seed, run_injected, run_ejected, throughput_pct,
                         mean_latency, p99_latency, defl_per_flit, run_buf_grants, run_buf_reqs, buf_grant_rate,
                         run_port_loss, run_quota_loss);

                // Write row to CSV
                $fdisplay(csv_fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%s,%0d,%0.2f,0x%08h,%0d,%0d,%0d,%0.2f,%0.2f,%0.2f,%0.2f,%0d,%0d,%0.2f,%0d,%0d,%0d,%0d,%0d,%0d",
                          TORUS_X, TORUS_Y, POOL_ENABLE, CLUSTER_X, CLUSTER_Y, SLOTS_PER_ROUTER, POOL_SIZE,
                          int'(ALPHA_Q), int'(BETA_Q), int'(HYST_THRESH),
                          traffic_mode, cur_prob, offered_pct, cur_seed,
                          run_attempted, run_injected, run_ejected, throughput_pct, achieved_inj_rate,
                          mean_latency, p99_latency, max_latency,
                          run_deflections, defl_per_flit,
                          run_buf_reqs, run_buf_grants, run_port_loss, run_quota_loss,
                          run_timed_out, run_fail);
            end
        end

        $fclose(csv_fd);
        $display("=========================================================================");
        $display("=== SWEEP COMPLETED: Results written to %s", csv_filename);
        $display("=========================================================================");
        $finish;
    end

endmodule
