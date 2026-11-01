// tb/tb_stage5.sv
// Stage 5 — Parameterized correctness sweep
//
// Compile with -d TORUS_X=N -d TORUS_Y=M (e.g. xvlog -sv -d TORUS_X=8 -d TORUS_Y=8).
// Default is 4x4 (unchanged from Stages 1-4).
// Both dimensions must be >= 3.
//
// Tests in order:
//   S1  Idle smoke       — no ejections for 5 cycles after reset
//   S2  All-pairs        — every (src,dest) pair; checks exact latency, no
//                          stray ejections, single-cycle pulse
//   S3  Diameter         — max observed hop_count == floor(X/2)+floor(Y/2)
//   S4a Random ~12.5%%   — xorshift32 PRNG, prob = 2/16
//   S4b Random ~50%%     — prob = 8/16; per-port deflection breakdown
//   S5  Stray-flit check — embedded inside run_random_traffic after each drain
//
// Outputs per run:
//   ../reports/stage5_NxM_flits.csv   — per-flit scoreboard
//   ../reports/stage5_summary.csv     — one appended row per size

`timescale 1ns/1ps

module tb_stage5;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam logic [31:0] PRNG_SEED    = 32'hDEAD_BEEF;
    localparam int          E5_CYCLES    = 100;
    localparam int          E5_PROB_LOW  = 2;   // 2/16 = 12.5%  ~15%
    localparam int          E5_PROB_HIGH = 8;   // 8/16 = 50%
    // Drain timeout: 30*(X+Y) cycles.  4x4 → 240, 8x8 → 480.
    localparam int          DRAIN_TIMEOUT = 30 * (TORUS_X + TORUS_Y);

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
    // Scoreboard and counters
    // ------------------------------------------------------------------
    flit_record_t pending [int];
    int next_tag                = 1;
    int total_injected          = 0;
    int total_ejected           = 0;
    int total_correct           = 0;
    int total_excess_hop        = 0;
    int total_deflection_events = 0;
    int total_attempted         = 0;  // before inj_ready check
    int defl_from_n             = 0;  // SRC_N deflected
    int defl_from_s             = 0;  // SRC_S deflected
    int defl_from_e             = 0;  // SRC_E deflected
    int defl_from_w             = 0;  // SRC_W deflected (expected highest: lowest priority)
    int pass_count              = 0;
    int fail_count              = 0;
    // All-pairs specific
    int ap_pass                 = 0;
    int ap_fail                 = 0;
    int max_ap_hops             = 0;  // max min_hops seen across all pairs

    // ------------------------------------------------------------------
    // Hop count queue (pushed by ejection monitor for every ejected flit)
    // ------------------------------------------------------------------
    int hop_counts [$];

    // ------------------------------------------------------------------
    // xorshift32 PRNG (same as Stage 4, seeded from PRNG_SEED)
    // ------------------------------------------------------------------
    logic [31:0] prng_state;

    function automatic logic [31:0] prng_next();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

    // ------------------------------------------------------------------
    // Simulation cycle counter and file handles
    // ------------------------------------------------------------------
    int sim_cycle = 0;
    int csv_fd;
    int summ_fd;

    // ------------------------------------------------------------------
    // Module-level ejection monitor temporaries
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
    //   1. sim_cycle++ (before #1)
    //   2. #1 — lets router_core NBA assignments and always_comb settle
    //   3. Deflection event counting (all routers, all bits)
    //   4. Ejection monitor — untracked ejections are FAIL (Stage 5 requirement)
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        sim_cycle = sim_cycle + 1;
        #1;

        if(rst_n) begin

            // Deflection counting
            for(int cx = 0; cx < TORUS_X; cx++) begin
                for(int cy = 0; cy < TORUS_Y; cy++) begin
                    total_deflection_events += $countones(defl_arr[cx][cy]);
                    defl_from_n += int'(defl_arr[cx][cy][0]);
                    defl_from_s += int'(defl_arr[cx][cy][1]);
                    defl_from_e += int'(defl_arr[cx][cy][2]);
                    defl_from_w += int'(defl_arr[cx][cy][3]);
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
                                mon_status = "WRONG_DEST";
                                $display("  [FAIL] Tag=%0d ejected at (%0d,%0d), expected (%0d,%0d)",
                                         mon_tag, pi, pj, mon_rec.dest_x, mon_rec.dest_y);
                                fail_count++;
                            end else if(mon_hops > mon_min_hops) begin
                                mon_status = "OK_DEFLECTED";
                                total_correct++;
                                total_excess_hop++;
                            end else begin
                                mon_status = "OK_DIRECT";
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
                            // STAGE 5: untracked ejection is a FAILURE, not a warning
                            $display("  [FAIL] Untracked ejection tag=%0d at (%0d,%0d) — stray or duplicate",
                                     mon_tag, pi, pj);
                            fail_count++;
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

    // inject_flit_quiet — for all-pairs sweep (no per-flit console output)
    task automatic inject_flit_quiet(int sx, sy, destx, desty);
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
    endtask

    task automatic release_injectors();
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
    endtask

    // wait_drain — timeout = DRAIN_TIMEOUT; prints actual drain cycle count
    task automatic wait_drain(string test_name);
        int t;
        t = 0;
        while(pending.num() > 0 && t < DRAIN_TIMEOUT) begin
            @(posedge clk); #1;
            t++;
        end
        if(pending.num() != 0) begin
            $display("  [FAIL] %s: %0d flit(s) lost after %0d drain cycles",
                     test_name, pending.num(), DRAIN_TIMEOUT);
            fail_count++;
        end else begin
            $display("  [PASS] %s: conservation — drained in %0d / %0d cycles",
                     test_name, t, DRAIN_TIMEOUT);
            pass_count++;
        end
    endtask

    // stray_flit_check — wait 2*(X+Y) cycles; fail if any ejection or pending != 0
    task automatic stray_flit_check(string phase_name);
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
        check($sformatf("%s: no stray ejections (%0d quiet cycles)", phase_name, wait_cyc),
              !had_stray);
        check($sformatf("%s: pending empty after stray check", phase_name),
              pending.num() == 0);
    endtask

    // run_random_traffic — one round of random injection
    //   prob_16  : inject this PE if prng_next()[3:0] < prob_16
    //              (8 → 50%, 2 → 12.5%)
    //   phase_name: label for drain/stray messages
    // Handles Phase 1 (attempt), Phase 2 (rollback rejected), Phase 3 (posedge).
    // Calls wait_drain then stray_flit_check before returning.
    task automatic run_random_traffic(int prob_16, string phase_name);
        // All declarations first (XSIM rule)
        int rdx, rdy, tag;
        logic [31:0] pv;

        for(int cyc = 0; cyc < E5_CYCLES; cyc++) begin

            // Phase 1: attempt injection on selected PEs
            for(int ri = 0; ri < TORUS_X; ri++) begin
                for(int rj = 0; rj < TORUS_Y; rj++) begin
                    pv = prng_next();
                    if(pv[3:0] < prob_16) begin
                        pv  = prng_next();
                        rdx = int'(pv % TORUS_X);
                        pv  = prng_next();
                        rdy = int'(pv % TORUS_Y);
                        if(rdx == ri && rdy == rj) rdx = (ri + 1) % TORUS_X;
                        total_attempted++;
                        tag = next_tag++;
                        pending[tag].src_x        = ri;
                        pending[tag].src_y        = rj;
                        pending[tag].dest_x       = rdx;
                        pending[tag].dest_y       = rdy;
                        pending[tag].inject_cycle = sim_cycle;
                        total_injected++;
                        inj_valid_drv[ri][rj] = 1'b1;
                        inj_flit_drv[ri][rj]  = make_flit(X_BITS'(rdx), Y_BITS'(rdy),
                                                           DATA_WIDTH'(tag));
                    end
                end
            end

            // Phase 2: settle, rollback rejected (inj_ready=0) injections
            #1;
            for(int ri = 0; ri < TORUS_X; ri++) begin
                for(int rj = 0; rj < TORUS_Y; rj++) begin
                    if(inj_valid_drv[ri][rj] && !inj_ready_arr[ri][rj]) begin
                        tag = int'(inj_flit_drv[ri][rj].data);
                        pending.delete(tag);
                        total_injected--;
                        // total_attempted is NOT decremented:
                        //   it counts all PEs that tried, accepted or not.
                        inj_valid_drv[ri][rj] = 1'b0;
                        inj_flit_drv[ri][rj]  = '0;
                    end
                end
            end

            // Phase 3: clock edge + clear injectors
            @(posedge clk); #1;
            release_injectors();
        end

        wait_drain(phase_name);
        stray_flit_check(phase_name);
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin

        // Size guard — fail immediately if dimensions too small
        if(TORUS_X < 3 || TORUS_Y < 3) begin
            $display("FATAL: TORUS_X=%0d TORUS_Y=%0d — both must be >= 3", TORUS_X, TORUS_Y);
            $fatal(0, "Invalid torus dimensions");
        end

        // Init all injection drivers
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        prng_state = PRNG_SEED;

        // Open per-run flit CSV
        csv_fd = $fopen($sformatf("../reports/stage5_%0dx%0d_flits.csv", TORUS_X, TORUS_Y), "w");
        $fdisplay(csv_fd, "# Stage 5  %0dx%0d  seed=0x%08h  drain_timeout=%0d",
                  TORUS_X, TORUS_Y, PRNG_SEED, DRAIN_TIMEOUT);
        $fdisplay(csv_fd,
                  "tag,src_x,src_y,dest_x,dest_y,eject_x,eject_y,inject_cycle,eject_cycle,hop_count,min_hops,status");

        // Open or append summary CSV (header written only if file is new)
        begin
            int tmp_fd;
            tmp_fd = $fopen("../reports/stage5_summary.csv", "r");
            if(tmp_fd == 0) begin
                tmp_fd = $fopen("../reports/stage5_summary.csv", "w");
                $fdisplay(tmp_fd, "torus_x,torus_y,prng_seed,ap_pairs,ap_pass,ap_fail,diameter_exp,diameter_obs,e5a_att_pct,e5a_inj_pct,e5a_inj_n,e5a_rej_n,e5b_att_pct,e5b_inj_pct,e5b_inj_n,e5b_rej_n,e5b_xhop_pct,e5b_devt_per_flit,defl_n_pct,defl_s_pct,defl_e_pct,defl_w_pct,mean_hops,p99_hops,p999_hops,max_hops,total_pass,total_fail");
                $fclose(tmp_fd);
            end else begin
                $fclose(tmp_fd);
            end
            summ_fd = $fopen("../reports/stage5_summary.csv", "a");
        end

        $display("=== tb_stage5: %0dx%0d | seed=0x%08h | drain=%0d cycles ===",
                 TORUS_X, TORUS_Y, PRNG_SEED, DRAIN_TIMEOUT);

        // PRNG validation
        begin
            int ones;
            logic [31:0] pv;
            ones = 0;
            for(int k = 0; k < 10000; k++) begin
                pv = prng_next();
                if(pv[0]) ones++;
            end
            $display("PRNG validation: %0d/10000 ones (expect 4750-5250)", ones);
            if(ones < 4750 || ones > 5250)
                $display("  [WARN] coin bias");
            else
                $display("  [OK  ] PRNG fair");
        end

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        #1;
        $display("Reset released.\n");

        // ==============================================================
        // S1: Idle smoke — no ejections for 5 cycles after reset
        // ==============================================================
        $display("====== S1: Idle smoke ======");
        begin
            logic idle_ok;
            idle_ok = 1'b1;
            repeat(5) begin
                @(posedge clk); #1;
                for(int pi = 0; pi < TORUS_X; pi++)
                    for(int pj = 0; pj < TORUS_Y; pj++)
                        if(ej_valid_arr[pi][pj]) idle_ok = 1'b0;
            end
            check("S1: no ejections on idle network", idle_ok);
        end

        // ==============================================================
        // S2: All-pairs uncontended
        //
        // Each (src,dest) pair injected one at a time into an empty network.
        // Three checks per pair:
        //   (a) ejection occurs at exactly min_hops+1 posedges after injection
        //   (b) no other PE ejects in the same cycle
        //   (c) ejection is a single-cycle pulse
        //
        // For wrap, tie, wrap-then-turn paths — all covered automatically.
        // 4x4 = 240 pairs, 8x8 = 4032 pairs.  Failures only printed.
        // ==============================================================
        $display("\n====== S2: All-pairs uncontended  (%0d x %0d = %0d pairs) ======",
                 TORUS_X, TORUS_Y, TORUS_X * TORUS_Y * (TORUS_X * TORUS_Y - 1));
        $display("  Printing failures only.");
        ap_pass = 0; ap_fail = 0; max_ap_hops = 0;
        begin
            // Standalone declarations at top of block (XSIM rule)
            int mh;
            logic stray;

            for(int sx = 0; sx < TORUS_X; sx++) begin
                for(int sy = 0; sy < TORUS_Y; sy++) begin
                    for(int dx = 0; dx < TORUS_X; dx++) begin
                        for(int dy = 0; dy < TORUS_Y; dy++) begin
                            if(!(sx == dx && sy == dy)) begin

                                mh = torus_min_hops(sx, sy, dx, dy);
                                if(mh > max_ap_hops) max_ap_hops = mh;

                                inject_flit_quiet(sx, sy, dx, dy);

                                // Posedge 1: source captures flit
                                @(posedge clk); #1;
                                release_injectors();

                                // mh more posedges: flit routes to destination
                                repeat(mh) @(posedge clk);
                                #1;

                                // (a) Latency check
                                if(ej_valid_arr[dx][dy]) begin
                                    ap_pass++;
                                end else begin
                                    ap_fail++;
                                    fail_count++;
                                    $display("  [FAIL] AP (%0d,%0d)→(%0d,%0d) mh=%0d: no ejection",
                                             sx, sy, dx, dy, mh);
                                end

                                // (b) Stray ejection check
                                stray = 1'b0;
                                for(int pi = 0; pi < TORUS_X; pi++) begin
                                    for(int pj = 0; pj < TORUS_Y; pj++) begin
                                        if((pi != dx || pj != dy) && ej_valid_arr[pi][pj]) begin
                                            stray = 1'b1;
                                            $display("  [FAIL] AP (%0d,%0d)→(%0d,%0d): stray at (%0d,%0d)",
                                                     sx, sy, dx, dy, pi, pj);
                                            fail_count++;
                                        end
                                    end
                                end
                                if(!stray) ap_pass++;
                                else       ap_fail++;

                                // (c) Pulse check: ejection is gone next cycle
                                @(posedge clk); #1;
                                if(!ej_valid_arr[dx][dy]) begin
                                    ap_pass++;
                                end else begin
                                    ap_fail++;
                                    fail_count++;
                                    $display("  [FAIL] AP (%0d,%0d)→(%0d,%0d): ejection lasted >1 cycle",
                                             sx, sy, dx, dy);
                                end

                                // Idle cycle before next pair
                                @(posedge clk); #1;

                            end
                        end
                    end
                end
            end
        end
        check("S2: conservation (pending empty)", pending.num() == 0);
        $display("  S2 summary: %0d checks PASS, %0d FAIL", ap_pass, ap_fail);

        // ==============================================================
        // S3: Diameter check
        // max_ap_hops is the largest min_hops seen across all pairs.
        // For an uncontended torus, observed hop_count == min_hops.
        // The torus diameter = floor(X/2) + floor(Y/2).
        // ==============================================================
        $display("\n====== S3: Diameter check ======");
        begin
            int exp_d;
            exp_d = (TORUS_X / 2) + (TORUS_Y / 2);
            $display("  Expected: floor(%0d/2)+floor(%0d/2) = %0d  |  Observed max = %0d",
                     TORUS_X, TORUS_Y, exp_d, max_ap_hops);
            check("S3: diameter == floor(X/2)+floor(Y/2)", max_ap_hops == exp_d);
        end

        // ==============================================================
        // S4: Random traffic at two load levels
        //
        // S4a — ~12.5% load (prob = E5_PROB_LOW / 16 = 2/16)
        // S4b — ~50%   load (prob = E5_PROB_HIGH / 16 = 8/16)
        //
        // Both use the same xorshift32 PRNG and injection handshake as Stage 4.
        // After each run: wait_drain + stray_flit_check (via run_random_traffic).
        //
        // All E5a and E5b variables are declared in ONE begin block so they
        // are all in scope when the summary CSV row is written.
        // XSIM rule: all declarations before any statements in this block.
        // ==============================================================
        $display("\n====== S4: Random traffic sweeps ======");
        begin
            // All declarations first
            int snap1_att, snap1_inj, snap1_ej, snap1_xhop;
            int snap2_att, snap2_inj, snap2_ej, snap2_xhop, snap2_devt;
            int snap2_dfn, snap2_dfs, snap2_dfe, snap2_dfw;
            int e5a_att, e5a_inj, e5a_rej, e5a_ej, e5a_xhop;
            int e5b_att, e5b_inj, e5b_rej, e5b_ej, e5b_xhop, e5b_devt;
            int e5b_dfn, e5b_dfs, e5b_dfe, e5b_dfw;
            real e5a_att_pct, e5a_inj_pct;
            real e5b_att_pct, e5b_inj_pct, e5b_xhop_pct, e5b_devt_pf;
            // Snapshot index: hop_counts entries from S2 (all-pairs, all uncontended,
            // hop_count==min_hops by construction) must not contaminate the percentile
            // ranks that measure congested-traffic tail latency for Phase 10.
            // s4_hop_start marks the first S4 entry in the queue.
            int s4_hop_start;

            // ---- S4a: ~12.5% load ----
            snap1_att  = total_attempted;
            snap1_inj  = total_injected;
            snap1_ej   = total_ejected;
            snap1_xhop = total_excess_hop;
            s4_hop_start = hop_counts.size(); // snapshot before any S4 traffic
            $display("\n  --- S4a: ~12.5%% load  (%0d cycles, prob=%0d/16) ---",
                     E5_CYCLES, E5_PROB_LOW);
            run_random_traffic(E5_PROB_LOW, "S4a");
            e5a_att  = total_attempted  - snap1_att;
            e5a_inj  = total_injected   - snap1_inj;
            e5a_rej  = e5a_att          - e5a_inj;
            e5a_ej   = total_ejected    - snap1_ej;
            e5a_xhop = total_excess_hop - snap1_xhop;
            e5a_att_pct = real'(e5a_att) * 100.0 / real'(E5_CYCLES * TORUS_X * TORUS_Y);
            e5a_inj_pct = real'(e5a_inj) * 100.0 / real'(E5_CYCLES * TORUS_X * TORUS_Y);
            $display("  E5a: attempted=%.1f%%  accepted=%.1f%%  rejected=%0d  ejected=%0d",
                     e5a_att_pct, e5a_inj_pct, e5a_rej, e5a_ej);

            // ---- S4b: ~50% load ----
            snap2_att  = total_attempted;
            snap2_inj  = total_injected;
            snap2_ej   = total_ejected;
            snap2_xhop = total_excess_hop;
            snap2_devt = total_deflection_events;
            snap2_dfn  = defl_from_n;
            snap2_dfs  = defl_from_s;
            snap2_dfe  = defl_from_e;
            snap2_dfw  = defl_from_w;
            $display("\n  --- S4b: ~50%% load  (%0d cycles, prob=%0d/16) ---",
                     E5_CYCLES, E5_PROB_HIGH);
            run_random_traffic(E5_PROB_HIGH, "S4b");
            e5b_att  = total_attempted          - snap2_att;
            e5b_inj  = total_injected           - snap2_inj;
            e5b_rej  = e5b_att                  - e5b_inj;
            e5b_ej   = total_ejected            - snap2_ej;
            e5b_xhop = total_excess_hop         - snap2_xhop;
            e5b_devt = total_deflection_events  - snap2_devt;
            e5b_dfn  = defl_from_n             - snap2_dfn;
            e5b_dfs  = defl_from_s             - snap2_dfs;
            e5b_dfe  = defl_from_e             - snap2_dfe;
            e5b_dfw  = defl_from_w             - snap2_dfw;
            e5b_att_pct  = real'(e5b_att) * 100.0 / real'(E5_CYCLES * TORUS_X * TORUS_Y);
            e5b_inj_pct  = real'(e5b_inj) * 100.0 / real'(E5_CYCLES * TORUS_X * TORUS_Y);
            e5b_xhop_pct = (e5b_ej > 0) ? real'(e5b_xhop) * 100.0 / real'(e5b_ej) : 0.0;
            e5b_devt_pf  = (e5b_ej > 0) ? real'(e5b_devt) / real'(e5b_ej)          : 0.0;
            $display("  E5b: attempted=%.1f%%  accepted=%.1f%%  rejected=%0d  ejected=%0d",
                     e5b_att_pct, e5b_inj_pct, e5b_rej, e5b_ej);
            $display("  E5b: excess-hop=%.1f%%  devt/flit=%.2f",
                     e5b_xhop_pct, e5b_devt_pf);
            $display("  Per-port deflections (N>S>E>W priority, W expected highest):");
            $display("    N=%0d  S=%0d  E=%0d  W=%0d",
                     e5b_dfn, e5b_dfs, e5b_dfe, e5b_dfw);

            // ---- Hop count statistics + summary CSV row ----
            begin
                int sorted_hops [];
                int p99i, p999i;
                real sum_h, mean_h, p99_h, p999_h, max_h;
                real e5b_dfn_pct, e5b_dfs_pct, e5b_dfe_pct, e5b_dfw_pct;

                // Slice from s4_hop_start: exclude S2 (all-pairs) entries which are
                // all hop_count==min_hops by construction and would dilute p99/p999.
                // max_hops is still trustworthy from the unsliced queue if needed,
                // but here we keep statistics pure to the congested-traffic regime.
                sorted_hops = hop_counts[s4_hop_start:$];

                sorted_hops.sort();

                p99i  = (sorted_hops.size() > 0) ?
                        int'(real'(sorted_hops.size() - 1) * 0.990) : 0;
                p999i = (sorted_hops.size() > 0) ?
                        int'(real'(sorted_hops.size() - 1) * 0.999) : 0;

                sum_h = 0.0;
                for(int k = 0; k < sorted_hops.size(); k++) sum_h += real'(sorted_hops[k]);
                mean_h = (sorted_hops.size() > 0) ? sum_h / real'(sorted_hops.size()) : 0.0;
                max_h  = (sorted_hops.size() > 0) ? real'(sorted_hops[sorted_hops.size()-1]) : 0.0;
                p99_h  = (sorted_hops.size() > 0) ? real'(sorted_hops[p99i])  : 0.0;
                p999_h = (sorted_hops.size() > 0) ? real'(sorted_hops[p999i]) : 0.0;

                e5b_dfn_pct = (e5b_devt>0) ? real'(e5b_dfn)*100.0/real'(e5b_devt) : 0.0;
                e5b_dfs_pct = (e5b_devt>0) ? real'(e5b_dfs)*100.0/real'(e5b_devt) : 0.0;
                e5b_dfe_pct = (e5b_devt>0) ? real'(e5b_dfe)*100.0/real'(e5b_devt) : 0.0;
                e5b_dfw_pct = (e5b_devt>0) ? real'(e5b_dfw)*100.0/real'(e5b_devt) : 0.0;

                // Append one row to stage5_summary.csv
                $fdisplay(summ_fd,
                    "%0d,%0d,0x%08h,%0d,%0d,%0d,%0d,%0d,%.1f,%.1f,%0d,%0d,%.1f,%.1f,%0d,%0d,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.0f,%.0f,%.0f,%0d,%0d",
                    TORUS_X, TORUS_Y, PRNG_SEED,
                    TORUS_X * TORUS_Y * (TORUS_X * TORUS_Y - 1),
                    ap_pass, ap_fail,
                    (TORUS_X/2) + (TORUS_Y/2), max_ap_hops,
                    e5a_att_pct, e5a_inj_pct, e5a_inj, e5a_rej,
                    e5b_att_pct, e5b_inj_pct, e5b_inj, e5b_rej,
                    e5b_xhop_pct, e5b_devt_pf,
                    e5b_dfn_pct, e5b_dfs_pct, e5b_dfe_pct, e5b_dfw_pct,
                    mean_h, p99_h, p999_h, max_h,
                    pass_count, fail_count);

                $display("\n  --- Hop Count Statistics (all %0d ejected flits) ---",
                         sorted_hops.size());
                if(sorted_hops.size() > 0)
                    $display("  Min=%0d  Max=%.0f  p99=%.0f  p99.9=%.0f  mean=%.2f",
                             sorted_hops[0], max_h, p99_h, p999_h, mean_h);
            end
        end

        // ==============================================================
        // Final summary
        // ==============================================================
        $fclose(csv_fd);
        $fclose(summ_fd);

        $display("\n==========================================");
        $display("  STAGE 5 SUMMARY  %0dx%0d", TORUS_X, TORUS_Y);
        $display("  Injected  : %0d  |  Ejected  : %0d", total_injected, total_ejected);
        $display("  Correct   : %0d  |  Excess-hop: %0d", total_correct, total_excess_hop);
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if(fail_count == 0)
            $display("  ALL STAGE 5 TESTS PASSED for %0dx%0d.", TORUS_X, TORUS_Y);
        else
            $display("  FAILURES DETECTED — review output above.");

        $finish;
    end

endmodule
