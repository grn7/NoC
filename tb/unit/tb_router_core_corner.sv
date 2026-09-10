// tb/unit/tb_router_core_corner.sv
// Corner-coordinate unit testbench for router_core.
// Tests two router instances in isolation:
//   DUT_A: cx=0, cy=0  (bottom-left corner)
//   DUT_B: cx=3, cy=3  (top-right corner)
//
// Purpose: verify splitter wraparound arithmetic at boundary coordinates.
// An interior router like (1,1) never exercises the "else" branch of fwd_x/fwd_y
// computation (X_dest + TORUS_X - cx) for cases where going the wrap direction
// is actually shorter. These corner cases do.
//
// Key wraparound cases proven by hand (TORUS_X=4, TORUS_Y=4):
//
// At (0,0):
//   dest(3,0): fwd_x=3, bwd_x=1 → WEST  (wrap shorter, not the naive "go East")
//   dest(0,3): fwd_y=3, bwd_y=1 → SOUTH (wrap shorter, not the naive "go North")
//   dest(2,0): fwd_x=2, bwd_x=2 → EAST  (tie breaks East)
//   dest(0,2): fwd_y=2, bwd_y=2 → NORTH (tie breaks North)
//
// At (3,3):
//   dest(0,3): fwd_x=1, bwd_x=3 → EAST  (wrap shorter, not the naive "go West")
//   dest(3,0): fwd_y=1, bwd_y=3 → NORTH (wrap shorter, not the naive "go South")
//   dest(1,3): fwd_x=2, bwd_x=2 → EAST  (tie breaks East)
//   dest(3,1): fwd_y=2, bwd_y=2 → NORTH (tie breaks North)
//
// Key rule: output pipeline register adds ONE cycle.
// Drive inputs at cycle N → sample outputs at cycle N+1.

`timescale 1ns/1ps

module tb_router_core_corner;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // DUT_A interfaces — router at (cx=0, cy=0)
    // ------------------------------------------------------------------
    router_link_if a_north_in_if  ();
    router_link_if a_north_out_if ();
    router_link_if a_south_in_if  ();
    router_link_if a_south_out_if ();
    router_link_if a_east_in_if   ();
    router_link_if a_east_out_if  ();
    router_link_if a_west_in_if   ();
    router_link_if a_west_out_if  ();
    pe_port_if     a_pe_if (.clk(clk));

    localparam logic [1:0] CX_A = 2'd0;
    localparam logic [1:0] CY_A = 2'd0;

    router_core dut_a (
        .clk       (clk),
        .rst_n     (rst_n),
        .cx        (CX_A),
        .cy        (CY_A),
        .north_in  (a_north_in_if.snk),
        .north_out (a_north_out_if.src),
        .south_in  (a_south_in_if.snk),
        .south_out (a_south_out_if.src),
        .east_in   (a_east_in_if.snk),
        .east_out  (a_east_out_if.src),
        .west_in   (a_west_in_if.snk),
        .west_out  (a_west_out_if.src),
        .pe_port   (a_pe_if.router)
    );

    // ------------------------------------------------------------------
    // DUT_B interfaces — router at (cx=3, cy=3)
    // ------------------------------------------------------------------
    router_link_if b_north_in_if  ();
    router_link_if b_north_out_if ();
    router_link_if b_south_in_if  ();
    router_link_if b_south_out_if ();
    router_link_if b_east_in_if   ();
    router_link_if b_east_out_if  ();
    router_link_if b_west_in_if   ();
    router_link_if b_west_out_if  ();
    pe_port_if     b_pe_if (.clk(clk));

    localparam logic [1:0] CX_B = 2'd3;
    localparam logic [1:0] CY_B = 2'd3;

    router_core dut_b (
        .clk       (clk),
        .rst_n     (rst_n),
        .cx        (CX_B),
        .cy        (CY_B),
        .north_in  (b_north_in_if.snk),
        .north_out (b_north_out_if.src),
        .south_in  (b_south_in_if.snk),
        .south_out (b_south_out_if.src),
        .east_in   (b_east_in_if.snk),
        .east_out  (b_east_out_if.src),
        .west_in   (b_west_in_if.snk),
        .west_out  (b_west_out_if.src),
        .pe_port   (b_pe_if.router)
    );

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    task idle_a();
        a_north_in_if.valid = 0; a_north_in_if.flit = '0;
        a_south_in_if.valid = 0; a_south_in_if.flit = '0;
        a_east_in_if.valid  = 0; a_east_in_if.flit  = '0;
        a_west_in_if.valid  = 0; a_west_in_if.flit  = '0;
        a_pe_if.inj_valid   = 0; a_pe_if.inj_flit   = '0;
    endtask

    task idle_b();
        b_north_in_if.valid = 0; b_north_in_if.flit = '0;
        b_south_in_if.valid = 0; b_south_in_if.flit = '0;
        b_east_in_if.valid  = 0; b_east_in_if.flit  = '0;
        b_west_in_if.valid  = 0; b_west_in_if.flit  = '0;
        b_pe_if.inj_valid   = 0; b_pe_if.inj_flit   = '0;
    endtask

    task tick(int n = 1);
        repeat(n) @(posedge clk);
        #1;
    endtask

    function automatic flit_t make_flit(
        logic [X_BITS-1:0] dx,
        logic [Y_BITS-1:0] dy,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = dx;
        make_flit.Y_dest = dy;
        make_flit.data   = dat;
    endfunction

    int pass_count = 0;
    int fail_count = 0;

    task check(string test_name, logic cond);
        if (cond) begin
            $display("  [PASS] %s", test_name);
            pass_count++;
        end else begin
            $display("  [FAIL] %s", test_name);
            fail_count++;
        end
    endtask

    // ------------------------------------------------------------------
    // MAIN TEST SEQUENCE
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_router_core_corner: Wraparound Tests ===");
        $display("    DUT_A: cx=0, cy=0   DUT_B: cx=3, cy=3");

        rst_n = 0;
        idle_a(); idle_b();
        tick(3);
        rst_n = 1;
        tick(2);

        // ==============================================================
        // DUT_A (cx=0, cy=0) TESTS
        // ==============================================================
        $display("\n====== DUT_A: router at (0,0) ======");

        // --------------------------------------------------------------
        // Test A1: WEST wraparound — dest_x=3 from cx=0
        // fwd_x = 3 (go East 3 hops), bwd_x = 1 (go West 1 hop, wraps 0→3)
        // Correct: WEST. Wrong (naive): East.
        // This is THE critical test — proves the else-branch of fwd_x fires.
        // --------------------------------------------------------------
        $display("\n--- A1: WRAP — dest(3,0) from (0,0) → must go WEST ---");
        a_north_in_if.valid = 1;
        a_north_in_if.flit  = make_flit(2'd3, 2'd0, 32'hA1AA_0001);
        tick(1);
        idle_a();
        check("Wraparound West: flit exits West-out",
              a_west_out_if.valid === 1 && a_west_out_if.flit.data === 32'hA1AA_0001);
        check("Wraparound West: NOT East-out (naive wrong answer)",
              a_east_out_if.valid === 0 ||
              (a_east_out_if.valid === 1 && a_east_out_if.flit.data !== 32'hA1AA_0001));
        check("Wraparound West: NOT ejected",
              !(a_pe_if.ej_valid && a_pe_if.ej_flit.data === 32'hA1AA_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test A2: SOUTH wraparound — dest_y=3 from cy=0
        // fwd_y = 3 (go North 3 hops), bwd_y = 1 (go South 1 hop, wraps 0→3)
        // Correct: SOUTH. Wrong (naive): North.
        // --------------------------------------------------------------
        $display("\n--- A2: WRAP — dest(0,3) from (0,0) → must go SOUTH ---");
        a_north_in_if.valid = 1;
        a_north_in_if.flit  = make_flit(2'd0, 2'd3, 32'hA2AA_0001);
        tick(1);
        idle_a();
        check("Wraparound South: flit exits South-out",
              a_south_out_if.valid === 1 && a_south_out_if.flit.data === 32'hA2AA_0001);
        check("Wraparound South: NOT North-out (naive wrong answer)",
              a_north_out_if.valid === 0 ||
              (a_north_out_if.valid === 1 && a_north_out_if.flit.data !== 32'hA2AA_0001));
        check("Wraparound South: NOT ejected",
              !(a_pe_if.ej_valid && a_pe_if.ej_flit.data === 32'hA2AA_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test A3: Normal East — dest_x=1 from cx=0
        // fwd_x = 1, bwd_x = 3 → correct: East. No wraparound needed.
        // Confirm normal path still works at corner.
        // --------------------------------------------------------------
        $display("\n--- A3: Normal East from (0,0) — dest(1,0) ---");
        a_south_in_if.valid = 1;
        a_south_in_if.flit  = make_flit(2'd1, 2'd0, 32'hA3AA_0001);
        tick(1);
        idle_a();
        check("Normal East: flit exits East-out",
              a_east_out_if.valid === 1 && a_east_out_if.flit.data === 32'hA3AA_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test A4: Normal North — dest_y=1 from cy=0
        // fwd_y = 1, bwd_y = 3 → correct: North.
        // --------------------------------------------------------------
        $display("\n--- A4: Normal North from (0,0) — dest(0,1) ---");
        a_south_in_if.valid = 1;
        a_south_in_if.flit  = make_flit(2'd0, 2'd1, 32'hA4AA_0001);
        tick(1);
        idle_a();
        check("Normal North: flit exits North-out",
              a_north_out_if.valid === 1 && a_north_out_if.flit.data === 32'hA4AA_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test A5: X tie-break East — dest_x=2 from cx=0
        // fwd_x = 2, bwd_x = 2 → tie → East (fwd_x <= bwd_x)
        // --------------------------------------------------------------
        $display("\n--- A5: X tie-break East — dest(2,0) from (0,0) ---");
        a_north_in_if.valid = 1;
        a_north_in_if.flit  = make_flit(2'd2, 2'd0, 32'hA5AA_0001);
        tick(1);
        idle_a();
        check("X tie breaks East: flit exits East-out",
              a_east_out_if.valid === 1 && a_east_out_if.flit.data === 32'hA5AA_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test A6: Y tie-break North — dest_y=2 from cy=0
        // fwd_y = 2, bwd_y = 2 → tie → North (fwd_y <= bwd_y)
        // --------------------------------------------------------------
        $display("\n--- A6: Y tie-break North — dest(0,2) from (0,0) ---");
        a_south_in_if.valid = 1;
        a_south_in_if.flit  = make_flit(2'd0, 2'd2, 32'hA6AA_0001);
        tick(1);
        idle_a();
        check("Y tie breaks North: flit exits North-out",
              a_north_out_if.valid === 1 && a_north_out_if.flit.data === 32'hA6AA_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test A7: Ejection at (0,0) — a flit arriving with dest (0,0)
        // --------------------------------------------------------------
        $display("\n--- A7: Ejection at (0,0) ---");
        a_east_in_if.valid = 1;
        a_east_in_if.flit  = make_flit(2'd0, 2'd0, 32'hA7AA_0001);
        tick(1);
        idle_a();
        check("Eject at corner (0,0): appears on ej_valid",
              a_pe_if.ej_valid === 1 && a_pe_if.ej_flit.data === 32'hA7AA_0001);
        check("Eject at corner (0,0): NOT on any network output",
              !(a_north_out_if.valid && a_north_out_if.flit.data === 32'hA7AA_0001) &&
              !(a_south_out_if.valid && a_south_out_if.flit.data === 32'hA7AA_0001) &&
              !(a_east_out_if.valid  && a_east_out_if.flit.data  === 32'hA7AA_0001) &&
              !(a_west_out_if.valid  && a_west_out_if.flit.data  === 32'hA7AA_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test A8: Both wrap directions simultaneously
        // North-in: dest(3,0) → West  (X-wrap)
        // South-in: dest(0,3) → South (Y-wrap, via X-aligned path)
        // Two inputs, no contention (different outputs) → clean permutation
        // --------------------------------------------------------------
        $display("\n--- A8: Both wrap directions simultaneously from (0,0) ---");
        a_north_in_if.valid = 1; a_north_in_if.flit = make_flit(2'd3, 2'd0, 32'hA8AA_0001);
        a_south_in_if.valid = 1; a_south_in_if.flit = make_flit(2'd0, 2'd3, 32'hA8AA_0002);
        tick(1);
        idle_a();
        check("Simultaneous wrap: X-wrap flit on West-out",
              a_west_out_if.valid  && a_west_out_if.flit.data  === 32'hA8AA_0001);
        check("Simultaneous wrap: Y-wrap flit on South-out",
              a_south_out_if.valid && a_south_out_if.flit.data === 32'hA8AA_0002);
        tick(1);

        // ==============================================================
        // DUT_B (cx=3, cy=3) TESTS
        // ==============================================================
        $display("\n====== DUT_B: router at (3,3) ======");

        // --------------------------------------------------------------
        // Test B1: EAST wraparound — dest_x=0 from cx=3
        // fwd_x = 1 (go East 1 hop, wraps 3→0), bwd_x = 3 (go West 3 hops)
        // Correct: EAST. Wrong (naive): West.
        // --------------------------------------------------------------
        $display("\n--- B1: WRAP — dest(0,3) from (3,3) → must go EAST ---");
        b_north_in_if.valid = 1;
        b_north_in_if.flit  = make_flit(2'd0, 2'd3, 32'hB1BB_0001);
        tick(1);
        idle_b();
        check("Wraparound East: flit exits East-out",
              b_east_out_if.valid === 1 && b_east_out_if.flit.data === 32'hB1BB_0001);
        check("Wraparound East: NOT West-out (naive wrong answer)",
              b_west_out_if.valid === 0 ||
              (b_west_out_if.valid === 1 && b_west_out_if.flit.data !== 32'hB1BB_0001));
        check("Wraparound East: NOT ejected",
              !(b_pe_if.ej_valid && b_pe_if.ej_flit.data === 32'hB1BB_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test B2: NORTH wraparound — dest_y=0 from cy=3
        // fwd_y = 1 (go North 1 hop, wraps 3→0), bwd_y = 3 (go South 3 hops)
        // Correct: NORTH. Wrong (naive): South.
        // --------------------------------------------------------------
        $display("\n--- B2: WRAP — dest(3,0) from (3,3) → must go NORTH ---");
        b_west_in_if.valid = 1;
        b_west_in_if.flit  = make_flit(2'd3, 2'd0, 32'hB2BB_0001);
        tick(1);
        idle_b();
        check("Wraparound North: flit exits North-out",
              b_north_out_if.valid === 1 && b_north_out_if.flit.data === 32'hB2BB_0001);
        check("Wraparound North: NOT South-out (naive wrong answer)",
              b_south_out_if.valid === 0 ||
              (b_south_out_if.valid === 1 && b_south_out_if.flit.data !== 32'hB2BB_0001));
        check("Wraparound North: NOT ejected",
              !(b_pe_if.ej_valid && b_pe_if.ej_flit.data === 32'hB2BB_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test B3: Normal West — dest_x=2 from cx=3
        // fwd_x = 3, bwd_x = 1 → correct: West.
        // --------------------------------------------------------------
        $display("\n--- B3: Normal West from (3,3) — dest(2,3) ---");
        b_north_in_if.valid = 1;
        b_north_in_if.flit  = make_flit(2'd2, 2'd3, 32'hB3BB_0001);
        tick(1);
        idle_b();
        check("Normal West: flit exits West-out",
              b_west_out_if.valid === 1 && b_west_out_if.flit.data === 32'hB3BB_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test B4: Normal South — dest_y=2 from cy=3
        // fwd_y = 3, bwd_y = 1 → correct: South.
        // --------------------------------------------------------------
        $display("\n--- B4: Normal South from (3,3) — dest(3,2) ---");
        b_east_in_if.valid = 1;
        b_east_in_if.flit  = make_flit(2'd3, 2'd2, 32'hB4BB_0001);
        tick(1);
        idle_b();
        check("Normal South: flit exits South-out",
              b_south_out_if.valid === 1 && b_south_out_if.flit.data === 32'hB4BB_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test B5: X tie-break East — dest_x=1 from cx=3
        // fwd_x = 2 (3→0→1), bwd_x = 2 (3→2→1) → tie → East
        // --------------------------------------------------------------
        $display("\n--- B5: X tie-break East — dest(1,3) from (3,3) ---");
        b_south_in_if.valid = 1;
        b_south_in_if.flit  = make_flit(2'd1, 2'd3, 32'hB5BB_0001);
        tick(1);
        idle_b();
        check("X tie breaks East at (3,3): flit exits East-out",
              b_east_out_if.valid === 1 && b_east_out_if.flit.data === 32'hB5BB_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test B6: Y tie-break North — dest_y=1 from cy=3
        // fwd_y = 2 (3→0→1), bwd_y = 2 (3→2→1) → tie → North
        // --------------------------------------------------------------
        $display("\n--- B6: Y tie-break North — dest(3,1) from (3,3) ---");
        b_south_in_if.valid = 1;
        b_south_in_if.flit  = make_flit(2'd3, 2'd1, 32'hB6BB_0001);
        tick(1);
        idle_b();
        check("Y tie breaks North at (3,3): flit exits North-out",
              b_north_out_if.valid === 1 && b_north_out_if.flit.data === 32'hB6BB_0001);
        tick(1);

        // --------------------------------------------------------------
        // Test B7: Ejection at (3,3)
        // --------------------------------------------------------------
        $display("\n--- B7: Ejection at (3,3) ---");
        b_west_in_if.valid = 1;
        b_west_in_if.flit  = make_flit(2'd3, 2'd3, 32'hB7BB_0001);
        tick(1);
        idle_b();
        check("Eject at corner (3,3): appears on ej_valid",
              b_pe_if.ej_valid === 1 && b_pe_if.ej_flit.data === 32'hB7BB_0001);
        check("Eject at corner (3,3): NOT on any network output",
              !(b_north_out_if.valid && b_north_out_if.flit.data === 32'hB7BB_0001) &&
              !(b_south_out_if.valid && b_south_out_if.flit.data === 32'hB7BB_0001) &&
              !(b_east_out_if.valid  && b_east_out_if.flit.data  === 32'hB7BB_0001) &&
              !(b_west_out_if.valid  && b_west_out_if.flit.data  === 32'hB7BB_0001));
        tick(1);

        // --------------------------------------------------------------
        // Test B8: Both wrap directions simultaneously at (3,3)
        // North-in: dest(0,3) → East (X-wrap)
        // South-in: dest(3,0) → North (Y-wrap)
        // Two inputs, no contention (East vs North) → clean permutation
        // --------------------------------------------------------------
        $display("\n--- B8: Both wrap directions simultaneously from (3,3) ---");
        b_north_in_if.valid = 1; b_north_in_if.flit = make_flit(2'd0, 2'd3, 32'hB8BB_0001);
        b_south_in_if.valid = 1; b_south_in_if.flit = make_flit(2'd3, 2'd0, 32'hB8BB_0002);
        tick(1);
        idle_b();
        check("Simultaneous wrap at (3,3): X-wrap flit on East-out",
              b_east_out_if.valid  && b_east_out_if.flit.data  === 32'hB8BB_0001);
        check("Simultaneous wrap at (3,3): Y-wrap flit on North-out",
              b_north_out_if.valid && b_north_out_if.flit.data === 32'hB8BB_0002);
        tick(1);

        // ==============================================================
        // SUMMARY
        // ==============================================================
        $display("\n==========================================");
        $display("  CORNER TEST SUMMARY");
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count == 0)
            $display("  ALL CORNER TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above.");

        $finish;
    end

endmodule
