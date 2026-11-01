// Step 0 smoke test — elaboration and idle check only.
//
// Purpose: confirm that the 2D pe_port_if array port on torus_top
// elaborates correctly in XSIM (this was flagged as a real risk when
// the array-of-interfaces plan was made). No real traffic is injected.
// All PE injection ports are held idle. After reset we just check that
// every ejection output stays at 0 for 10 cycles.
//
// If this passes: the foundation (interface arrays, generate loops,
// modport connections) is confirmed sound and we can build on top of it.
// If this fails with an elaboration error: we need a flattening workaround
// before investing in any further testbench logic.

`timescale 1ns/1ps

module tb_torus_smoke;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // 2D PE port interface array — one per router, clk shared
    // These are the "doorways" into the torus for the testbench.
    // ------------------------------------------------------------------
    pe_port_if pe_if [TORUS_X][TORUS_Y] (.clk(clk));

    // ------------------------------------------------------------------
    // DUT: the full torus
    // ------------------------------------------------------------------
    torus_top dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .pe_ports(pe_if)
    );

    // ------------------------------------------------------------------
    // Hold all PE injection ports idle — nothing is injected this test
    // ------------------------------------------------------------------
    generate
        genvar tx, ty;
        for(tx = 0; tx < TORUS_X; tx++) begin : g_idle_x
            for(ty = 0; ty < TORUS_Y; ty++) begin : g_idle_y
                assign pe_if[tx][ty].inj_valid = 1'b0;
                assign pe_if[tx][ty].inj_flit  = '0;
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Extract ej_valid from each interface into a regular logic array.
    //
    // WHY: Interface arrays cannot be indexed with runtime variables
    // (e.g., int i in a for loop) — the elaborator must resolve interface
    // indices to constants at compile time. Regular logic arrays have no
    // such restriction and can be indexed freely in procedural code.
    // The generate block below uses genvar constants to do the one-time
    // mapping from pe_if[x][y].ej_valid → ej_arr[x][y].
    // ------------------------------------------------------------------
    logic ej_arr [TORUS_X][TORUS_Y];

    generate
        genvar ex, ey;
        for(ex = 0; ex < TORUS_X; ex++) begin : g_ej_x
            for(ey = 0; ey < TORUS_Y; ey++) begin : g_ej_y
                assign ej_arr[ex][ey] = pe_if[ex][ey].ej_valid;
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // Smoke test sequence
    // ------------------------------------------------------------------
    int fail_count = 0;

    initial begin
        $display("=== Elaboration Smoke Test ===");
        $display("    Torus size: %0d x %0d  (%0d routers total)",
                 TORUS_X, TORUS_Y, TORUS_X * TORUS_Y);

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        $display("    Reset released. Running 10 idle cycles...");
        repeat(10) @(posedge clk);
        #1; // settle

        // Check every PE ejection output is 0
        // ej_arr is a regular logic array — safe to index with int i, j
        $display("    Checking all ejection outputs...");
        for(int i = 0; i < TORUS_X; i++) begin
            for(int j = 0; j < TORUS_Y; j++) begin
                if(ej_arr[i][j] !== 1'b0) begin
                    $display("  [FAIL] PE(%0d,%0d) ej_valid=%0b (expected 0)",
                             i, j, ej_arr[i][j]);
                    fail_count++;
                end
            end
        end

        if(fail_count == 0)
            $display("  [PASS] All %0d PE ejection ports idle after reset",
                     TORUS_X * TORUS_Y);

        $display("\n==========================================");
        $display("  SMOKE TEST SUMMARY");
        $display("  FAIL: %0d", fail_count);
        $display("==========================================");

        if(fail_count == 0)
            $display("  ELABORATION SMOKE TEST PASSED.");
        else
            $display("  FAILURES DETECTED — review above.");

        $finish;
    end

endmodule
