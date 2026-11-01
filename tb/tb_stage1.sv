// tb/tb_stage1.sv
// Stage 1 — Single-hop, single-flit delivery
//
// Tests:
//   Part A: Interior router (1,1) — one hop East, West, North, South.
//           No wraparound. Exercises the basic routing path end-to-end
//           across a connected torus for the first time.
//
//   Part B: Wraparound hops — the highest-priority cases in this stage.
//           First time link_hp and link_vp carry real flit data.
//           (3,1)→(0,1) East-wrap  via link_h  (fwd_x=1 < bwd_x=3)
//           (0,1)→(3,1) West-wrap  via link_hp (bwd_x=1 < fwd_x=3)
//           (1,3)→(1,0) North-wrap via link_v  (fwd_y=1 < bwd_y=3)
//           (1,0)→(1,3) South-wrap via link_vp (bwd_y=1 < fwd_y=3)
//
// Expected latency for 1 hop = 2 clock cycles:
//   Posedge N   : source router's output register latches flit → link carries it
//   Posedge N+1 : destination router's output register latches ej_valid
//   (After #1)  : ej_valid visible to testbench
//
// Latency assertion: check NOT ejected at cycle 1 (no early arrival),
// then IS ejected at cycle 2 (exact match). Ejection clears at cycle 3.
//
// Interface-array indexing rule (same as smoke test):
//   pe_if[x][y].signal cannot be indexed with runtime int variables.
//   Solution: generate block maps interface members → plain logic/flit_t
//   arrays (inj_valid_drv, ej_valid_arr, etc.) that tasks CAN index freely.

`timescale 1ns/1ps

module tb_stage1;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // 2D PE port interface array — one per router
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
    //
    // Plain logic/flit_t arrays driven procedurally (including inside
    // tasks with runtime int indices). Connected to pe_if via continuous
    // assigns in the generate block below — so the DUT sees them as if
    // the testbench were driving the interface directly.
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
    // Ejection and inj_ready extraction
    //
    // Same genvar-generate pattern as smoke test. Allows procedural code
    // (including tasks) to check signals with runtime int indices.
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
    // Scoreboard
    // ------------------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

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
        logic [X_BITS-1:0]    xd,
        logic [Y_BITS-1:0]    yd,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = xd;
        make_flit.Y_dest = yd;
        make_flit.data   = dat;
    endfunction

    // ------------------------------------------------------------------
    // One-hop delivery task
    //
    // Injects one flit at source PE (sx,sy) destined for (destx,desty).
    // Asserts:
    //   1. inj_ready = 1 when flit is presented (no contention)
    //   2. ej_valid = 0 after tick 1 — flit has NOT arrived early
    //   3. ej_valid = 1 after tick 2 — exact 2-cycle latency
    //   4. ej_flit.data = payload — no corruption in transit
    //   5. ej_valid = 0 after tick 3 — ejection is a one-cycle pulse
    //
    // Timing:
    //   drive inj_valid=1
    //   #1          : allow continuous assign + combinational logic to settle
    //   check inj_ready (combinational, still valid)
    //   @posedge    : [tick 1] source router latches flit into output link
    //   #1
    //   clear inj_valid (flit captured — prevents double injection)
    //   check ej_valid=0  (not arrived yet)
    //   @posedge    : [tick 2] dest router ejects flit to PE
    //   #1
    //   check ej_valid=1 + payload match
    //   @posedge    : [tick 3] dest router sees idle link, clears ej_valid
    //   #1
    //   check ej_valid=0 (ejection cleared — confirms one-cycle pulse)
    // ------------------------------------------------------------------
    task automatic one_hop_test(
        string test_name,
        int    sx, sy,          // source PE coordinates
        int    destx, desty,    // destination PE coordinates
        logic [DATA_WIDTH-1:0] payload
    );
        // Drive injection
        inj_valid_drv[sx][sy] = 1'b1;
        inj_flit_drv[sx][sy]  = make_flit(X_BITS'(destx), Y_BITS'(desty), payload);
        #1; // allow continuous assign + router combinational logic to settle

        // inj_ready is combinational — check it while inj_valid is high
        check($sformatf("%s: inj_ready at PE(%0d,%0d)", test_name, sx, sy),
              inj_ready_arr[sx][sy] === 1'b1);

        // Tick 1: source router latches flit into output link
        @(posedge clk); #1;

        // Flit has been captured by the source router's output register.
        // Clear injection so the source doesn't try to inject again next cycle.
        inj_valid_drv[sx][sy] = 1'b0;
        inj_flit_drv[sx][sy]  = '0;

        // Latency check part 1: flit should NOT have arrived yet
        check($sformatf("%s: ej_valid=0 at PE(%0d,%0d) after 1 cycle (no early arrival)",
              test_name, destx, desty),
              ej_valid_arr[destx][desty] === 1'b0);

        // Tick 2: destination router ejects flit
        @(posedge clk); #1;

        // Latency check part 2: flit must arrive at exactly 2 cycles
        check($sformatf("%s: ej_valid=1 at PE(%0d,%0d) after 2 cycles",
              test_name, destx, desty),
              ej_valid_arr[destx][desty] === 1'b1);

        // Payload integrity check
        check($sformatf("%s: payload correct at PE(%0d,%0d)",
              test_name, destx, desty),
              ej_flit_arr[destx][desty].data === payload);

        // Tick 3: destination router sees idle link, ej_valid should clear
        @(posedge clk); #1;

        // One-cycle pulse check: ejection must NOT persist
        check($sformatf("%s: ej_valid=0 at PE(%0d,%0d) after 3 cycles (one-cycle pulse)",
              test_name, destx, desty),
              ej_valid_arr[destx][desty] === 1'b0);

    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin

        // Initialise all injection drivers to 0
        // (plain logic array — int loop index is allowed here)
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        $display("=== tb_stage1: Single-Hop Delivery Tests ===");
        $display("    Torus: %0d x %0d", TORUS_X, TORUS_Y);

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        #1;
        $display("Reset released.\n");

        // ==============================================================
        // PART A: Interior router (1,1) — one hop in each direction
        //
        // No wraparound involved. Both the source and destination are
        // interior nodes. Tests that the basic injection → splitter →
        // resolver → link → neighbor → ejection path works correctly
        // for all four directions.
        // ==============================================================
        $display("====== Part A: Interior Router (1,1) — Normal Hops ======");

        // A1: East  (1,1) → (2,1)
        // dest_x=2 > cx=1 : fwd_x=1, bwd_x=3 → East. No wraparound.
        one_hop_test("A1 East  (1,1)→(2,1)", 1, 1, 2, 1, 32'hAA01_0001);

        // A2: West  (1,1) → (0,1)
        // dest_x=0 < cx=1 : fwd_x=3, bwd_x=1 → West. No wraparound.
        one_hop_test("A2 West  (1,1)→(0,1)", 1, 1, 0, 1, 32'hBB02_0001);

        // A3: North (1,1) → (1,2)
        // X-aligned. dest_y=2 > cy=1 : fwd_y=1, bwd_y=3 → North.
        one_hop_test("A3 North (1,1)→(1,2)", 1, 1, 1, 2, 32'hCC03_0001);

        // A4: South (1,1) → (1,0)
        // X-aligned. dest_y=0 < cy=1 : fwd_y=3, bwd_y=1 → South.
        one_hop_test("A4 South (1,1)→(1,0)", 1, 1, 1, 0, 32'hDD04_0001);

        // ==============================================================
        // PART B: Wraparound hops — highest priority in this stage
        //
        // First time link_hp (west-going) and link_vp (south-going) carry
        // real flit data. Also the first end-to-end test of the modular
        // indexing in torus_top's generate block at network boundaries.
        //
        // Route reasoning for each case is annotated below using the
        // splitter's fwd/bwd arithmetic (TORUS_X=4, TORUS_Y=4).
        // ==============================================================
        $display("\n====== Part B: Wraparound Hops ======");

        // B1: East-wrap  (3,1) → (0,1)
        // cx=3, dest_x=0: fwd_x=(0-3+4)%4=1, bwd_x=3 → EAST (wrap)
        // Flit exits east_out of (3,1) via link_h[3][1] → west_in of (0,1)
        one_hop_test("B1 East-wrap  (3,1)→(0,1)", 3, 1, 0, 1, 32'hEE05_0001);

        // B2: West-wrap  (0,1) → (3,1)
        // cx=0, dest_x=3: fwd_x=3, bwd_x=1 → WEST (wrap)
        // Flit exits west_out of (0,1) via link_hp[0][1] → east_in of (3,1)
        one_hop_test("B2 West-wrap  (0,1)→(3,1)", 0, 1, 3, 1, 32'hFF06_0001);

        // B3: North-wrap (1,3) → (1,0)
        // X-aligned. cy=3, dest_y=0: fwd_y=(0-3+4)%4=1, bwd_y=3 → NORTH (wrap)
        // Flit exits north_out of (1,3) via link_v[1][3] → south_in of (1,0)
        one_hop_test("B3 North-wrap (1,3)→(1,0)", 1, 3, 1, 0, 32'hAA07_0001);

        // B4: South-wrap (1,0) → (1,3)
        // X-aligned. cy=0, dest_y=3: fwd_y=3, bwd_y=1 → SOUTH (wrap)
        // Flit exits south_out of (1,0) via link_vp[1][0] → north_in of (1,3)
        one_hop_test("B4 South-wrap (1,0)→(1,3)", 1, 0, 1, 3, 32'hBB08_0001);

        // ==============================================================
        // Summary
        // ==============================================================
        $display("\n==========================================");
        $display("  STAGE 1 SUMMARY");
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if(fail_count == 0)
            $display("  ALL STAGE 1 TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above.");

        $finish;
    end

endmodule
