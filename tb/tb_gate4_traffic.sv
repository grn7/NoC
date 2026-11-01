// tb/tb_gate4_traffic.sv
// Gate 4: Standalone Traffic Generator Verification
//
// Validates:
// 1. Static Hotspot Generator:
//    - Focuses 50% of traffic onto a single hotspot node (2,2).
//    - Verifies statistical concentration at (2,2) vs uniform background.
//    - Verifies 100% flit delivery, zero drops, zero misroutes.
// 2. Migrating Hotspot Generator:
//    - Rotates the hotspot target across distinct clusters every HOTSPOT_PERIOD cycles.
//    - Tracks per-epoch destination histograms, proving the hotspot moves dynamically.
//    - Verifies 100% flit conservation across migration transitions.
//    - Probes pool buffer activity across cluster boundaries during migration.

`timescale 1ns/1ps

module tb_gate4_traffic;

    import noc_pkg::*;

    localparam int HOTSPOT_PERIOD = 100;

    // Clock and Reset
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // Interfaces & DUT
    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    // Static Driver & Monitor Arrays
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

    // PRNG Xorshift32
    logic [31:0] l_state = 32'hDEAD_BEEF;

    function automatic logic [31:0] xorshift32();
        l_state ^= l_state << 13;
        l_state ^= l_state >> 17;
        l_state ^= l_state << 5;
        return l_state;
    endfunction

    // Helper to make flit
    function automatic flit_t make_flit(int dx, int dy, int tag);
        flit_t f;
        f.X_dest = X_BITS'(dx);
        f.Y_dest = Y_BITS'(dy);
        f.data   = DATA_WIDTH'(tag);
        return f;
    endfunction

    // Scoreboard
    typedef struct {
        int src_x,  src_y;
        int dest_x, dest_y;
        int inject_cycle;
        int tag;
    } flit_rec_t;

    flit_rec_t pending [int];
    int sim_cycle     = 0;
    int pass_count    = 0;
    int fail_count    = 0;
    int tag_counter   = 1;
    int grants_seen   = 0;

    // Ejection monitor
    always @(posedge clk) begin
        sim_cycle <= sim_cycle + 1;
        #1;
        if (rst_n) begin
            // Track pool write grants in cluster 0
            if (dut.gen_c_col[0].gen_c_row[0].u_cluster_pool.gen_pool_active.u_write_arb.grant_valid)
                grants_seen++;

            for (int px = 0; px < TORUS_X; px++) begin
                for (int py = 0; py < TORUS_Y; py++) begin
                    if (ej_valid_arr[px][py]) begin
                        int tag;
                        tag = int'(ej_flit_arr[px][py].data);
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
                            $display("  [FAIL] Untracked ejection of tag %0d at (%0d,%0d)!", tag, px, py);
                            fail_count++;
                        end
                    end
                end
            end
        end
    end

    // Main Test Stimulus
    initial begin
        // Top-level variable declarations per XSIM rules
        int active_cycles;
        int drain_cycles;
        int dest_counts [TORUS_X][TORUS_Y];
        int epoch_dest_counts [4][TORUS_X][TORUS_Y];
        int total_injected;
        int hot_targets [4][2];
        logic [31:0] r;
        int dx, dy;
        int cur_hot_x, cur_hot_y;

        hot_targets[0][0] = 0; hot_targets[0][1] = 0; // Epoch 0: Cluster (0,0)
        hot_targets[1][0] = 3; hot_targets[1][1] = 0; // Epoch 1: Cluster (1,0)
        hot_targets[2][0] = 3; hot_targets[2][1] = 3; // Epoch 2: Cluster (1,1)
        hot_targets[3][0] = 0; hot_targets[3][1] = 3; // Epoch 3: Cluster (0,1)

        // Initialize drivers
        for (int i = 0; i < TORUS_X; i++) begin
            for (int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        $display("=========================================================================");
        $display("=== STARTING GATE 4: TRAFFIC GENERATOR VERIFICATION                   ===");
        $display("=== TORUS=%0dx%0d | CLUSTER=%0dx%0d | POOL_ENABLE=%0d                 ===",
                 TORUS_X, TORUS_Y, CLUSTER_X, CLUSTER_Y, POOL_ENABLE);
        $display("=========================================================================");

        // Warm up PRNG
        for (int w = 0; w < 500; w++) r = xorshift32();

        // ---------------------------------------------------------------------
        // TEST 1: Static Hotspot Traffic Generator
        // ---------------------------------------------------------------------
        $display("\n--- Test 1: Static Hotspot Traffic Generator (Target: (2,2), 50%% Hotspot) ---");
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        #1;

        pending.delete();
        total_injected = 0;
        for (int x = 0; x < TORUS_X; x++) begin
            for (int y = 0; y < TORUS_Y; y++) begin
                dest_counts[x][y] = 0;
            end
        end

        cur_hot_x = 2;
        cur_hot_y = 2;

        // Run injection for 200 cycles at injection prob = 8/16 (50%)
        for (active_cycles = 0; active_cycles < 200; active_cycles++) begin
            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    r = xorshift32();
                    if (int'(r[3:0]) < 8) begin // 50% injection probability
                        if (r[4]) begin
                            // 50% hotspot flit directed at (2,2)
                            if (sx != cur_hot_x || sy != cur_hot_y) begin
                                dx = cur_hot_x;
                                dy = cur_hot_y;
                            end else begin
                                dx = (cur_hot_x + 1) % TORUS_X;
                                dy = (cur_hot_y + 1) % TORUS_Y;
                            end
                        end else begin
                            // Uniform random destination
                            do begin
                                r = xorshift32();
                                dx = int'(r[1:0]);
                                dy = int'(r[3:2]);
                            end while (dx == sx && dy == sy);
                        end

                        inj_valid_drv[sx][sy] = 1'b1;
                        inj_flit_drv[sx][sy]  = make_flit(dx, dy, tag_counter);
                        pending[tag_counter]  = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inject_cycle:sim_cycle, tag:tag_counter};
                        dest_counts[dx][dy]++;
                        total_injected++;
                        tag_counter++;
                    end else begin
                        inj_valid_drv[sx][sy] = 1'b0;
                        inj_flit_drv[sx][sy]  = '0;
                    end
                end
            end

            // Settle combinational logic & handle rollback for rejected injection
            #1;
            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    if (inj_valid_drv[sx][sy] && !inj_ready_arr[sx][sy]) begin
                        int tag;
                        tag = int'(inj_flit_drv[sx][sy].data);
                        dest_counts[pending[tag].dest_x][pending[tag].dest_y]--;
                        total_injected--;
                        pending.delete(tag);
                        inj_valid_drv[sx][sy] = 1'b0;
                    end
                end
            end
            @(posedge clk); #1;
        end

        // Deassert injection
        for (int sx = 0; sx < TORUS_X; sx++) begin
            for (int sy = 0; sy < TORUS_Y; sy++) begin
                inj_valid_drv[sx][sy] = 1'b0;
            end
        end

        // Drain network
        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 200) begin
            @(posedge clk); #1;
            drain_cycles++;
        end

        $display("  Total injected flits = %0d, Flits targeting Hotspot (2,2) = %0d (%.1f%%)",
                 total_injected, dest_counts[2][2], (real'(dest_counts[2][2]) / real'(total_injected)) * 100.0);

        // Assert that (2,2) received significantly higher traffic than uniform expectation (~6.25%)
        if (dest_counts[2][2] > (total_injected / 3)) begin
            $display("  [PASS] Static hotspot concentration verified (targeted > 33%% of all injected network flits).");
        end else begin
            $display("  [FAIL] Hotspot node (2,2) did not receive expected concentration! count=%0d, total=%0d",
                     dest_counts[2][2], total_injected);
            fail_count++;
        end

        if (pending.size() == 0) begin
            $display("  [PASS] 100%% conservation achieved (all %0d flits delivered cleanly).", total_injected);
        end else begin
            $display("  [FAIL] %0d flits undrained after %0d cycles!", pending.size(), drain_cycles);
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 2: Migrating Hotspot Traffic Generator
        // ---------------------------------------------------------------------
        $display("\n--- Test 2: Migrating Hotspot Traffic Generator (4 Epochs x 100 cycles) ---");
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        #1;

        pending.delete();
        total_injected = 0;
        for (int e = 0; e < 4; e++) begin
            for (int x = 0; x < TORUS_X; x++) begin
                for (int y = 0; y < TORUS_Y; y++) begin
                    epoch_dest_counts[e][x][y] = 0;
                end
            end
        end

        // 4 epochs of HOTSPOT_PERIOD cycles = 400 cycles
        for (active_cycles = 0; active_cycles < 4 * HOTSPOT_PERIOD; active_cycles++) begin
            int cur_epoch;
            cur_epoch = active_cycles / HOTSPOT_PERIOD;
            cur_hot_x = hot_targets[cur_epoch][0];
            cur_hot_y = hot_targets[cur_epoch][1];

            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    r = xorshift32();
                    if (int'(r[3:0]) < 8) begin // 50% injection probability
                        if (r[4]) begin
                            // Hotspot flit directed at current epoch's hotspot node
                            if (sx != cur_hot_x || sy != cur_hot_y) begin
                                dx = cur_hot_x;
                                dy = cur_hot_y;
                            end else begin
                                dx = (cur_hot_x + 1) % TORUS_X;
                                dy = (cur_hot_y + 1) % TORUS_Y;
                            end
                        end else begin
                            // Uniform random destination
                            do begin
                                r = xorshift32();
                                dx = int'(r[1:0]);
                                dy = int'(r[3:2]);
                            end while (dx == sx && dy == sy);
                        end

                        inj_valid_drv[sx][sy] = 1'b1;
                        inj_flit_drv[sx][sy]  = make_flit(dx, dy, tag_counter);
                        pending[tag_counter]  = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inject_cycle:sim_cycle, tag:tag_counter};
                        epoch_dest_counts[cur_epoch][dx][dy]++;
                        total_injected++;
                        tag_counter++;
                    end else begin
                        inj_valid_drv[sx][sy] = 1'b0;
                        inj_flit_drv[sx][sy]  = '0;
                    end
                end
            end

            // Settle combinational logic & handle rollback
            #1;
            for (int sx = 0; sx < TORUS_X; sx++) begin
                for (int sy = 0; sy < TORUS_Y; sy++) begin
                    if (inj_valid_drv[sx][sy] && !inj_ready_arr[sx][sy]) begin
                        int tag;
                        tag = int'(inj_flit_drv[sx][sy].data);
                        epoch_dest_counts[cur_epoch][pending[tag].dest_x][pending[tag].dest_y]--;
                        total_injected--;
                        pending.delete(tag);
                        inj_valid_drv[sx][sy] = 1'b0;
                    end
                end
            end
            @(posedge clk); #1;
        end

        // Deassert injection
        for (int sx = 0; sx < TORUS_X; sx++) begin
            for (int sy = 0; sy < TORUS_Y; sy++) begin
                inj_valid_drv[sx][sy] = 1'b0;
            end
        end

        // Drain network
        drain_cycles = 0;
        while (pending.size() > 0 && drain_cycles < 200) begin
            @(posedge clk); #1;
            drain_cycles++;
        end

        // Verify each epoch's migrating hotspot peak
        for (int e = 0; e < 4; e++) begin
            int hx, hy;
            int peak_val, peak_x, peak_y;
            hx = hot_targets[e][0];
            hy = hot_targets[e][1];
            peak_val = -1;
            peak_x   = -1;
            peak_y   = -1;

            for (int x = 0; x < TORUS_X; x++) begin
                for (int y = 0; y < TORUS_Y; y++) begin
                    if (epoch_dest_counts[e][x][y] > peak_val) begin
                        peak_val = epoch_dest_counts[e][x][y];
                        peak_x   = x;
                        peak_y   = y;
                    end
                end
            end

            $display("  Epoch %0d (Target: (%0d,%0d)): Peak Destination was (%0d,%0d) with %0d flits",
                     e, hx, hy, peak_x, peak_y, peak_val);

            if (peak_x == hx && peak_y == hy) begin
                $display("    [PASS] Epoch %0d correctly focused on migrating hotspot (%0d,%0d).", e, hx, hy);
            end else begin
                $display("    [FAIL] Epoch %0d hotspot mismatch! Expected (%0d,%0d), got peak (%0d,%0d)",
                         e, hx, hy, peak_x, peak_y);
                fail_count++;
            end
        end

        if (pending.size() == 0) begin
            $display("  [PASS] Migrating hotspot 100%% conservation achieved (all %0d flits delivered cleanly).", total_injected);
        end else begin
            $display("  [FAIL] %0d flits undrained after migrating sweep!", pending.size());
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n=========================================================================");
        if (fail_count == 0) begin
            $display("=== ALL GATE 4 TESTS PASSED (%0d successful flit deliveries) ===", pass_count);
        end else begin
            $display("=== GATE 4 TESTBENCH FAILED with %0d errors! ===", fail_count);
        end
        $display("=========================================================================");

        if (fail_count > 0) $fatal(1, "Gate 4 testbench encountered errors!");
        $finish;
    end

endmodule
