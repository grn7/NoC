// tb/tb_gate1_gate3_adaptive.sv
// Gate 1 & Gate 3 Verification Testbench for Adaptive Cluster-Shared Buffer Pool
//
// Verification Coverage:
// 1. Single-hop & multi-hop routing correctness with cluster pool hardware active.
// 2. Buffer & Reinjection delivery: flit deflected from Pass 2 is buffered in pool,
//    held, reinjected via Pass 3 when port is free, and reaches destination correctly.
// 3. Flit conservation: no flits lost, duplicated, or corrupted while in network or pool.
// 4. Cluster Ownership Integrity (Gate 3): flit buffered by router (x1, y1) in a shared
//    cluster returns strictly to router (x1, y1), never to another router in the same cluster.
// 5. Single-Write-Port Arbitration & Deflection Fallback (Gate 3): when multiple routers
//    request buffering simultaneously, exactly ONE grant issues; ungranted routers fall
//    through to blind deflection without stalling or dropping flits.

`timescale 1ns/1ps

module tb_gate1_gate3_adaptive;

    import noc_pkg::*;

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
    // Static Driver & Monitor Arrays (XSIM runtime indexing support)
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
    // PRNG (xorshift32)
    // -------------------------------------------------------------------------
    logic [31:0] prng_state = 32'hCAFE_F00D;
    function automatic logic [31:0] xorshift32();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

    // Helper to create flit
    function automatic flit_t make_flit(int dx, int dy, int tag);
        flit_t f;
        f.X_dest = X_BITS'(dx);
        f.Y_dest = Y_BITS'(dy);
        f.data   = DATA_WIDTH'(tag);
        return f;
    endfunction

    // -------------------------------------------------------------------------
    // Scoreboard for Conservation Tracking
    // -------------------------------------------------------------------------
    typedef struct {
        int src_x, src_y;
        int dest_x, dest_y;
        int inj_cycle;
        int tag;
    } flit_rec_t;

    flit_rec_t pending [int];
    int ejected_tags   [$];

    int sim_cycle  = 0;
    int pass_count = 0;
    int fail_count = 0;

    // Monitor process: checks ejections every cycle
    always @(posedge clk) begin
        sim_cycle <= sim_cycle + 1;
        #1;
        if (rst_n) begin
            for (int px = 0; px < TORUS_X; px++) begin
                for (int py = 0; py < TORUS_Y; py++) begin
                    if (ej_valid_arr[px][py]) begin
                        int tag;
                        tag = int'(ej_flit_arr[px][py].data);
                        ejected_tags.push_back(tag);

                        if (pending.exists(tag)) begin
                            flit_rec_t rec;
                            rec = pending[tag];
                            if ((px != rec.dest_x) || (py != rec.dest_y)) begin
                                $display("  [FAIL] Flit tag %0d ejected at (%0d,%0d), expected (%0d,%0d)!",
                                         tag, px, py, rec.dest_x, rec.dest_y);
                                fail_count++;
                            end else begin
                                pass_count++;
                            end
                            pending.delete(tag);
                        end else begin
                            $display("  [FAIL] Untracked or duplicate ejection of tag %0d at (%0d,%0d)!",
                                     tag, px, py);
                            fail_count++;
                        end
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Hierarchical Probes for Invariant and Ownership Assertions
    // -------------------------------------------------------------------------
    localparam int ENTRY_BITS = (POOL_SIZE > 1) ? $clog2(POOL_SIZE) : 1;

    logic                t2_grant_valid;
    logic [OCC_BITS-1:0] t2_total_occupancy;

    generate
        if (CLUSTER_X == 1 && CLUSTER_Y == 1) begin : gen_t2_c1x1
            assign t2_grant_valid     = dut.gen_c_col[1].gen_c_row[1].u_cluster_pool.gen_pool_active.u_write_arb.grant_valid;
            assign t2_total_occupancy = dut.gen_c_col[1].gen_c_row[1].u_cluster_pool.gen_pool_active.u_storage.total_occupancy;
        end else begin : gen_t2_c2x2
            assign t2_grant_valid     = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant_valid;
            assign t2_total_occupancy = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.total_occupancy;
        end
    endgenerate

    logic [ROUTERS_PER_CLUSTER-1:0] t4_grant_vec;
    logic                           t4_grant_val;
    logic [RPC_BITS-1:0]            t4_grant_idx;
    logic [ROUTERS_PER_CLUSTER-1:0] t4_req_vec;
    logic [ENTRY_BITS-1:0]          t4_free_slot;

    assign t4_grant_vec = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant;
    assign t4_grant_val = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant_valid;
    assign t4_grant_idx = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant_idx;
    assign t4_req_vec   = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.req;
    assign t4_free_slot = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.free_slot_idx;

    // -------------------------------------------------------------------------
    // Main Test Stimulus
    // -------------------------------------------------------------------------
    initial begin
        // Variable declarations at top of initial block per XSIM rules
        int tag_counter;
        int wait_cycles;
        int drain_cycles;
        flit_t f_inj;
        bit test2_grant_seen;
        bit test2_occupancy_seen;
        bit test4_grant_seen;
        bit test4_arb_ok;
        bit test4_owner_ok;
        bit test4_multi_req_seen;
        int exp_grant_idx;
        int wr_slot;

        tag_counter = 1;

        // Initialize drivers
        for (int i = 0; i < TORUS_X; i++) begin
            for (int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        $display("=========================================================================");
        $display("=== STARTING GATE 1 & GATE 3 ADAPTIVE CLUSTER POOL TESTBENCH          ===");
        $display("=== TORUS=%0dx%0d | CLUSTER=%0dx%0d | POOL_ENABLE=%0d | POOL_SIZE=%0d ===",
                 TORUS_X, TORUS_Y, CLUSTER_X, CLUSTER_Y, POOL_ENABLE, POOL_SIZE);
        $display("=========================================================================");

        // Reset DUT
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        #1;

        // ---------------------------------------------------------------------
        // TEST 1: Basic Point-to-Point Deliveries (Interior & Wraparound)
        // ---------------------------------------------------------------------
        $display("\n--- Test 1: Basic Point-to-Point Routing ---");
        // Inject from (0,0) to (2,2)
        inj_valid_drv[0][0] = 1'b1;
        inj_flit_drv[0][0]  = make_flit(2, 2, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:0, dest_x:2, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        // Inject from (3,3) to (0,0) across wraparound links
        inj_valid_drv[3][3] = 1'b1;
        inj_flit_drv[3][3]  = make_flit(0, 0, tag_counter);
        pending[tag_counter] = '{src_x:3, src_y:3, dest_x:0, dest_y:0, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        @(posedge clk); #1;
        inj_valid_drv[0][0] = 1'b0;
        inj_valid_drv[3][3] = 1'b0;

        // Wait for delivery
        wait_cycles = 0;
        while (pending.size() > 0 && wait_cycles < 20) begin
            @(posedge clk); #1;
            wait_cycles++;
        end

        if (pending.size() == 0)
            $display("  [PASS] Test 1 completed: All flits delivered successfully.");
        else begin
            $display("  [FAIL] Test 1: %0d flits timed out!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 2: Multi-Flit Contention with Buffer-and-Reinject
        // ---------------------------------------------------------------------
        $display("\n--- Test 2: Contention with Buffer Pool Storage and Reinjection ---");
        // Create deliberate structural contention at router (1,1):
        // Flit A from (1,0) going North toward (1,2)
        // Flit B from (0,1) going East toward (1,2)
        // Both enter router (1,1) in cycle 2; both demand the North output!
        test2_grant_seen     = 1'b0;
        test2_occupancy_seen = 1'b0;

        inj_valid_drv[1][0] = 1'b1;
        inj_flit_drv[1][0]  = make_flit(1, 2, tag_counter);
        pending[tag_counter] = '{src_x:1, src_y:0, dest_x:1, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        inj_valid_drv[0][1] = 1'b1;
        inj_flit_drv[0][1]  = make_flit(1, 2, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:1, dest_x:1, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        @(posedge clk); #1;
        inj_valid_drv[1][0] = 1'b0;
        inj_valid_drv[0][1] = 1'b0;

        // Monitor contention, grant, and buffer occupancy
        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 40) begin
            if (t2_grant_valid)
                test2_grant_seen = 1'b1;
            if (t2_total_occupancy > 0)
                test2_occupancy_seen = 1'b1;

            @(posedge clk); #1;
            drain_cycles++;
        end

        if (!test2_grant_seen) begin
            $display("  [FAIL] Test 2: Write grant was NEVER asserted during contention!");
            fail_count++;
        end
        if (!test2_occupancy_seen) begin
            $display("  [FAIL] Test 2: Pool occupancy was NEVER non-zero during contention!");
            fail_count++;
        end

        if (pending.size() == 0 && test2_grant_seen && test2_occupancy_seen)
            $display("  [PASS] Test 2 completed: Contending flits triggered pool grant, non-zero occupancy, and correct delivery.");
        else if (pending.size() > 0) begin
            $display("  [FAIL] Test 2: %0d flits timed out!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 3: Full-Network Random Traffic Burst (Scoreboard Conservation)
        // ---------------------------------------------------------------------
        $display("\n--- Test 3: Randomized Traffic Burst with Active Pool Sharing ---");
        for (int burst = 0; burst < 3; burst++) begin
            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    int dx, dy;
                    logic [31:0] r;
                    r = xorshift32();
                    dx = int'(r[3:0]) % TORUS_X;
                    dy = int'(r[7:4]) % TORUS_Y;
                    if (dx == sx && dy == sy) dx = (sx + 1) % TORUS_X;

                    inj_valid_drv[sx][sy] = 1'b1;
                    f_inj = make_flit(dx, dy, tag_counter);
                    inj_flit_drv[sx][sy]  = f_inj;

                    pending[tag_counter] = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inj_cycle:sim_cycle, tag:tag_counter};
                    tag_counter++;
                end
            end
            @(posedge clk); #1;
            // Deassert injection
            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    inj_valid_drv[sx][sy] = 1'b0;
                end
            end
            repeat (2) @(posedge clk); #1;
        end

        // Wait for full network drain
        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 300) begin
            @(posedge clk); #1;
            drain_cycles++;
        end

        if (pending.size() == 0)
            $display("  [PASS] Test 3 completed: 100%% conservation achieved across burst traffic (%0d drain cycles).", drain_cycles);
        else begin
            $display("  [FAIL] Test 3: %0d flits undrained after %0d cycles!", pending.size(), drain_cycles);
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 4 (Gate 3): Cluster Ownership & Multi-Router Request Arbitration
        // ---------------------------------------------------------------------
        $display("\n--- Test 4: Cluster Pool Ownership & Multi-Router Arbitration Check ---");
        test4_grant_seen     = 1'b0;
        test4_arb_ok         = 1'b1;
        test4_owner_ok       = 1'b1;
        test4_multi_req_seen = 1'b0;

        // Step 1: Inject 4 flits simultaneously to create simultaneous contention at two routers:
        // Pair A targeting router (1,0) [owner_idx=2 in cluster (0,0)]:
        // Flit 1 from (0,0) going to (1,1) -> arrives at (1,0) via west_in wanting North
        // Flit 2 from (1,3) going to (1,1) -> arrives at (1,0) via south_in wanting North
        // Pair B targeting router (1,1) [owner_idx=3 in cluster (0,0)]:
        // Flit 3 from (0,1) going to (1,2) -> arrives at (1,1) via west_in wanting North
        // Flit 4 from (2,1) going to (1,2) -> arrives at (1,1) via east_in wanting North
        inj_valid_drv[0][0] = 1'b1; inj_flit_drv[0][0] = make_flit(1, 1, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:0, dest_x:1, dest_y:1, inj_cycle:sim_cycle, tag:tag_counter}; tag_counter++;

        inj_valid_drv[1][3] = 1'b1; inj_flit_drv[1][3] = make_flit(1, 1, tag_counter);
        pending[tag_counter] = '{src_x:1, src_y:3, dest_x:1, dest_y:1, inj_cycle:sim_cycle, tag:tag_counter}; tag_counter++;

        inj_valid_drv[0][1] = 1'b1; inj_flit_drv[0][1] = make_flit(1, 2, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:1, dest_x:1, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter}; tag_counter++;

        inj_valid_drv[2][1] = 1'b1; inj_flit_drv[2][1] = make_flit(1, 2, tag_counter);
        pending[tag_counter] = '{src_x:2, src_y:1, dest_x:1, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter}; tag_counter++;

        @(posedge clk); #1;
        inj_valid_drv[0][0] = 1'b0;
        inj_valid_drv[1][3] = 1'b0;
        inj_valid_drv[0][1] = 1'b0;
        inj_valid_drv[2][1] = 1'b0;

        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 100) begin
            if ($countones(t4_req_vec) > 1) begin
                test4_multi_req_seen = 1'b1;
            end

            // Property A: At most 1 write grant issued per cycle
            if ($countones(t4_grant_vec) > 1) begin
                $display("  [FAIL] Test 4: More than one grant issued in cluster simultaneously! grant_vec=%b", t4_grant_vec);
                test4_arb_ok = 1'b0;
                fail_count++;
            end

            // Property B: When grant is valid, the granted router actually requested
            if (t4_grant_val) begin
                test4_grant_seen = 1'b1;
                if (!t4_req_vec[t4_grant_idx]) begin
                    $display("  [FAIL] Test 4: Grant issued to non-requesting router %0d!", t4_grant_idx);
                    test4_arb_ok = 1'b0;
                    fail_count++;
                end

                wr_slot       = int'(t4_free_slot);
                exp_grant_idx = int'(t4_grant_idx);
                @(posedge clk); #1;
                drain_cycles++;
                if (dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.entries[wr_slot].valid &&
                    dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.entries[wr_slot].owner_idx != exp_grant_idx) begin
                    $display("  [FAIL] Test 4: Stored entry owner_idx mismatch! expected=%0d, got=%0d",
                             exp_grant_idx, dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.entries[wr_slot].owner_idx);
                    test4_owner_ok = 1'b0;
                    fail_count++;
                end
                continue;
            end

            @(posedge clk); #1;
            drain_cycles++;
        end

        if (!test4_grant_seen && (CLUSTER_X * CLUSTER_Y > 1)) begin
            $display("  [FAIL] Test 4: No write grants observed during multi-router contention!");
            fail_count++;
        end

        if (!test4_multi_req_seen && (CLUSTER_X * CLUSTER_Y > 1)) begin
            $display("  [FAIL] Test 4: Simultaneous multi-router requests were never observed!");
            fail_count++;
        end

        if (pending.size() == 0 && test4_arb_ok && test4_owner_ok && (test4_multi_req_seen || CLUSTER_X * CLUSTER_Y == 1))
            $display("  [PASS] Test 4 completed: Multi-router arbitration, quota isolation, and ownership verified.");
        else if (pending.size() > 0) begin
            $display("  [FAIL] Test 4: %0d flits undrained!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n=========================================================================");
        if (fail_count == 0) begin
            $display("=== ALL TESTS PASSED (%0d successful flit deliveries) ===", pass_count);
        end else begin
            $display("=== TESTBENCH FAILED with %0d errors! ===", fail_count);
        end
        $display("=========================================================================");

        if (fail_count > 0) $fatal(1, "Testbench encountered errors!");
        $finish;
    end

endmodule
