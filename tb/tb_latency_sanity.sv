// tb/tb_latency_sanity.sv
// Sanity check verifying exact latency calculation and NBA timing semantics
//
// In a pipelined HopLite/HopliteRT torus router:
// Hop latency formula for H network hops: Latency = H + 1 cycles.
// - 1-hop delivery: (0,0) -> (1,0): H=1 -> exactly 2 cycles.
// - 2-hop delivery: (0,0) -> (2,0): H=2 -> exactly 3 cycles.
// - 3-hop delivery: (0,0) -> (2,1): H=3 -> exactly 4 cycles.

`timescale 1ns/1ps

module tb_latency_sanity;

    import noc_pkg::*;

    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    logic  inj_valid_drv [TORUS_X][TORUS_Y];
    flit_t inj_flit_drv  [TORUS_X][TORUS_Y];
    logic  ej_valid_arr  [TORUS_X][TORUS_Y];
    flit_t ej_flit_arr   [TORUS_X][TORUS_Y];

    generate
        genvar x, y;
        for (x = 0; x < TORUS_X; x++) begin : g_map_x
            for (y = 0; y < TORUS_Y; y++) begin : g_map_y
                assign pe_if[x][y].inj_valid = inj_valid_drv[x][y];
                assign pe_if[x][y].inj_flit  = inj_flit_drv[x][y];
                assign ej_valid_arr[x][y]    = pe_if[x][y].ej_valid;
                assign ej_flit_arr[x][y]     = pe_if[x][y].ej_flit;
            end
        end
    endgenerate

    int sim_cycle = 0;

    // Use blocking update matching standard benchmark harness
    always @(posedge clk) begin
        sim_cycle++;
    end

    typedef struct {
        int src_x, src_y;
        int dest_x, dest_y;
        int inject_cycle;
        int hops;
    } flit_rec_t;

    flit_rec_t pending [int];
    int measured_latencies [int];
    int pass_count = 0;
    int fail_count = 0;

    // Ejection monitor
    always @(posedge clk) begin
        #1;
        if (rst_n) begin
            for (int px = 0; px < TORUS_X; px++) begin
                for (int py = 0; py < TORUS_Y; py++) begin
                    if (ej_valid_arr[px][py]) begin
                        int tag;
                        tag = int'(ej_flit_arr[px][py].data);
                        if (pending.exists(tag)) begin
                            int lat;
                            lat = sim_cycle - pending[tag].inject_cycle;
                            measured_latencies[tag] = lat;
                            pending.delete(tag);
                        end
                    end
                end
            end
        end
    end

    function automatic flit_t make_flit(int dx, int dy, int tag);
        flit_t f;
        f.X_dest = X_BITS'(dx);
        f.Y_dest = Y_BITS'(dy);
        f.data   = DATA_WIDTH'(tag);
        return f;
    endfunction

    task automatic test_single_delivery(int sx, int sy, int dx, int dy, int tag, int expected_hops);
        int expected_lat;
        expected_lat = expected_hops + 1;

        inj_valid_drv[sx][sy] = 1'b1;
        inj_flit_drv[sx][sy]  = make_flit(dx, dy, tag);
        pending[tag]          = '{src_x:sx, src_y:sy, dest_x:dx, dest_y:dy, inject_cycle:sim_cycle, hops:expected_hops};

        @(posedge clk); #1;
        inj_valid_drv[sx][sy] = 1'b0;
        inj_flit_drv[sx][sy]  = '0;

        // Wait for delivery
        while (pending.exists(tag)) begin
            @(posedge clk); #1;
        end

        if (measured_latencies[tag] == expected_lat) begin
            $display("  [PASS] (%0d,%0d) -> (%0d,%0d): %0d hops -> Latency = %0d cycles (Exact match: expected %0d)",
                     sx, sy, dx, dy, expected_hops, measured_latencies[tag], expected_lat);
            pass_count++;
        end else begin
            $display("  [FAIL] (%0d,%0d) -> (%0d,%0d): %0d hops -> Latency = %0d cycles (MISMATCH: expected %0d)",
                     sx, sy, dx, dy, expected_hops, measured_latencies[tag], expected_lat);
            fail_count++;
        end
    endtask

    initial begin
        for (int i = 0; i < TORUS_X; i++) begin
            for (int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end
        end

        $display("=========================================================================");
        $display("=== STARTING LATENCY AND CYCLE-TIMING SANITY CHECK                   ===");
        $display("=========================================================================");

        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        #1;

        // Test 1: 1-hop delivery (0,0) -> (1,0) (East link)
        test_single_delivery(0, 0, 1, 0, 101, 1);

        // Test 2: 2-hop delivery (0,0) -> (2,0) (East-East)
        test_single_delivery(0, 0, 2, 0, 102, 2);

        // Test 3: 3-hop delivery (0,0) -> (2,1) (East-East-North)
        test_single_delivery(0, 0, 2, 1, 103, 3);

        // Test 4: 4-hop delivery (0,0) -> (2,2) (East-East-North-North)
        test_single_delivery(0, 0, 2, 2, 104, 4);

        $display("=========================================================================");
        if (fail_count == 0) begin
            $display("=== ALL SANITY CHECKS PASSED: Timing and latency arithmetic confirmed! ===");
        end else begin
            $display("=== SANITY CHECK FAILED with %0d errors! ===", fail_count);
        end
        $display("=========================================================================");

        if (fail_count > 0) $fatal(1, "Latency sanity check failed!");
        $finish;
    end

endmodule
