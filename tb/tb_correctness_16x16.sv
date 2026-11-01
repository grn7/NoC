// tb/tb_correctness_16x16.sv
// Lightweight 16x16 Correctness Testbench for Adaptive Cluster-Shared Buffer NoC
//
// Verification Scope:
// 1. Instantiates 16x16 (256-node) torus with 4x4 clusters (16 clusters, 16 routers/cluster).
// 2. Test 1: Single-hop, multi-hop interior, and toroidal wraparound deliveries.
// 3. Test 2: Local contention verifying cluster pool grant, non-zero occupancy, and reinjection.
// 4. Test 3: Moderate randomized burst verifying 100% flit conservation on 256-node scale.
//
// Compliant with XSIM SystemVerilog declaration rules and Vivado 2026.1.

`timescale 1ns/1ps

module tb_correctness_16x16;

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
    // Driver & Monitor Probing Arrays
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
    // Diagnostic Probes for Cluster (0,0)
    // -------------------------------------------------------------------------
    logic                t2_grant_valid;
    logic [OCC_BITS-1:0] t2_total_occupancy;

    assign t2_grant_valid     = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant_valid;
    assign t2_total_occupancy = dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_storage.total_occupancy;

    // -------------------------------------------------------------------------
    // PRNG (xorshift32)
    // -------------------------------------------------------------------------
    logic [31:0] prng_state = 32'h1616_BEEF;
    function automatic logic [31:0] xorshift32();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

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
    // Main Stimulus Sequence
    // -------------------------------------------------------------------------
    initial begin
        // Variable declarations at top of block per XSIM rules
        int tag_counter;
        int wait_cycles;
        int drain_cycles;
        bit test2_grant_seen;
        bit test2_occupancy_seen;

        tag_counter = 1;

        // Initialize drivers
        for (int i = 0; i < TORUS_X; i++) begin
            for (int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        $display("=========================================================================");
        $display("=== STARTING 16x16 (256-NODE) CORRECTNESS VERIFICATION TEST           ===");
        $display("=== TORUS=%0dx%0d | CLUSTER=%0dx%0d | POOL_SIZE=%0d | SLOTS=%0d      ===",
                 TORUS_X, TORUS_Y, CLUSTER_X, CLUSTER_Y, POOL_SIZE, SLOTS_PER_ROUTER);
        $display("=========================================================================");

        // Reset DUT
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        #1;

        // ---------------------------------------------------------------------
        // TEST 1: Point-to-Point Deliveries (Single-hop, Multi-hop, Wraparound)
        // ---------------------------------------------------------------------
        $display("\n--- Test 1: 16x16 Point-to-Point & Toroidal Wraparound Deliveries ---");

        // 1. Single hop East: (0,0) -> (1,0)
        inj_valid_drv[0][0] = 1'b1;
        inj_flit_drv[0][0]  = make_flit(1, 0, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:0, dest_x:1, dest_y:0, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        // 2. Single hop South: (0,0) -> (0,1)
        inj_valid_drv[0][1] = 1'b1;
        inj_flit_drv[0][1]  = make_flit(0, 2, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:1, dest_x:0, dest_y:2, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        // 3. Multi-cluster interior diagonal hop: (2,2) -> (8,8)
        inj_valid_drv[2][2] = 1'b1;
        inj_flit_drv[2][2]  = make_flit(8, 8, tag_counter);
        pending[tag_counter] = '{src_x:2, src_y:2, dest_x:8, dest_y:8, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        // 4. Maximum wraparound hop: (15,15) -> (0,0)
        inj_valid_drv[15][15] = 1'b1;
        inj_flit_drv[15][15]  = make_flit(0, 0, tag_counter);
        pending[tag_counter] = '{src_x:15, src_y:15, dest_x:0, dest_y:0, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        // 5. Cross-boundary wraparound: (0,15) -> (15,0)
        inj_valid_drv[0][15] = 1'b1;
        inj_flit_drv[0][15]  = make_flit(15, 0, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:15, dest_x:15, dest_y:0, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        @(posedge clk); #1;
        inj_valid_drv[0][0]   = 1'b0;
        inj_valid_drv[0][1]   = 1'b0;
        inj_valid_drv[2][2]   = 1'b0;
        inj_valid_drv[15][15] = 1'b0;
        inj_valid_drv[0][15]  = 1'b0;

        // Drain Test 1
        wait_cycles = 0;
        while (pending.size() > 0 && wait_cycles < 60) begin
            @(posedge clk); #1;
            wait_cycles++;
        end

        if (pending.size() == 0) begin
            $display("  [PASS] Test 1: All interior and wraparound flits delivered cleanly.");
        end else begin
            $display("  [FAIL] Test 1: %0d flits timed out!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 2: In-Cluster Contention with Buffer-and-Reinject
        // ---------------------------------------------------------------------
        $display("\n--- Test 2: In-Cluster Contention & Pool Reinjection at 16x16 ---");
        // Flit A from (1,0) destined for (1,3)
        // Flit B from (0,1) destined for (1,3)
        // Both enter router (1,1) in cycle 2; both demand South!
        test2_grant_seen     = 1'b0;
        test2_occupancy_seen = 1'b0;

        inj_valid_drv[1][0] = 1'b1;
        inj_flit_drv[1][0]  = make_flit(1, 3, tag_counter);
        pending[tag_counter] = '{src_x:1, src_y:0, dest_x:1, dest_y:3, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        inj_valid_drv[0][1] = 1'b1;
        inj_flit_drv[0][1]  = make_flit(1, 3, tag_counter);
        pending[tag_counter] = '{src_x:0, src_y:1, dest_x:1, dest_y:3, inj_cycle:sim_cycle, tag:tag_counter};
        tag_counter++;

        @(posedge clk); #1;
        inj_valid_drv[1][0] = 1'b0;
        inj_valid_drv[0][1] = 1'b0;

        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 50) begin
            if (t2_grant_valid)
                test2_grant_seen = 1'b1;
            if (t2_total_occupancy > 0)
                test2_occupancy_seen = 1'b1;

            @(posedge clk); #1;
            drain_cycles++;
        end

        if (!test2_grant_seen) begin
            $display("  [FAIL] Test 2: Pool grant was NEVER asserted during contention!");
            fail_count++;
        end
        if (!test2_occupancy_seen) begin
            $display("  [FAIL] Test 2: Pool occupancy was NEVER non-zero during contention!");
            fail_count++;
        end

        if (pending.size() == 0 && test2_grant_seen && test2_occupancy_seen) begin
            $display("  [PASS] Test 2: Contention triggered pool grant, non-zero occupancy, and delivery.");
        end else if (pending.size() > 0) begin
            $display("  [FAIL] Test 2: %0d flits timed out!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 3: Moderate Random Burst with Full Scoreboard Conservation
        // ---------------------------------------------------------------------
        $display("\n--- Test 3: Moderate Randomized Traffic Burst (Conservation Scoreboard) ---");
        // Inject 64 flits across selected nodes across all 4 quadrants of the 16x16 network
        for (int burst = 0; burst < 64; burst++) begin
            logic [31:0] r1, r2;
            int sx, sy, dx, dy;

            r1 = xorshift32();
            r2 = xorshift32();
            sx = int'(r1[3:0]);
            sy = int'(r1[7:4]);

            do begin
                r2 = xorshift32();
                dx = int'(r2[3:0]);
                dy = int'(r2[7:4]);
            end while (dx == sx && dy == sy);

            inj_valid_drv[sx][sy] = 1'b1;
            inj_flit_drv[sx][sy]  = make_flit(dx, dy, tag_counter);
            pending[tag_counter] = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inj_cycle:sim_cycle, tag:tag_counter};
            tag_counter++;

            @(posedge clk); #1;
            inj_valid_drv[sx][sy] = 1'b0;
        end

        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 200) begin
            @(posedge clk); #1;
            drain_cycles++;
        end

        if (pending.size() == 0) begin
            $display("  [PASS] Test 3: 64/64 random burst flits delivered with 100%% conservation.");
        end else begin
            $display("  [FAIL] Test 3: %0d flits timed out or dropped in 16x16 network!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n=========================================================================");
        $display("=== 16x16 CORRECTNESS TEST SUMMARY                                    ===");
        $display("=== Total Flits Ejected Correctly : %0d", pass_count);
        $display("=== Total Failures / Anomalies   : %0d", fail_count);
        if (fail_count == 0) begin
            $display("=== [PASS] 16x16 ARCHITECTURE VERIFIED FUNCTIONALLY CORRECT!          ===");
        end else begin
            $display("=== [FAIL] 16x16 ARCHITECTURE FAILED VERIFICATION!                    ===");
        end
        $display("=========================================================================");
        $finish;
    end

endmodule
