// tb/tb_stage2.sv
// Stage 2 — Multi-hop delivery, no contention
//
// Formula: N hops → N+1 clock cycles latency.
//   Each additional transit router adds exactly one pipeline register stage.
//   This is verified explicitly for each case — never assumed.
//
// Test cases:
//   C1  2-hop straight X    (0,1)→(2,1)  via (1,1)              latency 3
//   C2  2-hop X→Y turn      (0,0)→(1,1)  via (1,0)              latency 3
//   C3  3-hop 2X+1Y         (0,0)→(2,1)  via (1,0),(2,0)        latency 4
//   C4  2-hop wrap transit  (3,1)→(1,1)  via (0,1)  [WRAP]      latency 3
//   C5  2-hop straight Y    (1,0)→(1,2)  via (1,1)              latency 3
//   C6  3-hop X+2Y          (0,1)→(1,3)  via (1,1),(1,2)        latency 4
//
// Splitter branch coverage at transit routers:
//   dest_x != cx at transit → still routing X  : C1, C3(hop1), C4(hop1)
//   dest_x == cx at transit → switch to Y phase : C2, C3(hop2), C5, C6(hop1,2)
//
// Latency assertion strategy (from stage2.txt):
//   - Check ej_valid=0 at cycle 1 (no early arrival — valid for all N)
//   - Check ej_valid=1 at EXACTLY cycle N+1 (deflection detector — if a flit
//     were deflected it would arrive late and this check would fail)
//   - Check ej_valid=0 at cycle N+2 (one-cycle pulse — no sticky output)
//
// The multi_hop_test task generalises the one_hop_test pattern from Stage 1:
//   same boilerplate, takes hop_count instead of hardcoding 2 cycles.
//
// Interface-array indexing rule carried over from Stage 1:
//   inj_valid_drv / ej_valid_arr / ej_flit_arr / inj_ready_arr are plain
//   logic/flit_t arrays extracted from pe_if via generate — tasks can
//   index them freely with runtime int parameters.

`timescale 1ns/1ps

module tb_stage2;

    import noc_pkg::*;

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
    // Plain logic/flit_t arrays — connected to pe_if via continuous
    // assigns below. Tasks index them with runtime int parameters freely.
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
        logic [X_BITS-1:0]     xd,
        logic [Y_BITS-1:0]     yd,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = xd;
        make_flit.Y_dest = yd;
        make_flit.data   = dat;
    endfunction

    // ------------------------------------------------------------------
    // Multi-hop delivery task
    //
    // Generalisation of one_hop_test from Stage 1.
    // Takes hop_count; expected latency = hop_count + 1 cycles.
    //
    // Timing:
    //   drive inj_valid=1, #1 settle
    //   check inj_ready (combinational)
    //   tick 1 : source latches → clear inj_valid
    //            check ej_valid=0 (no early arrival at cycle 1)
    //   repeat(hop_count) ticks   [total elapsed = 1 + hop_count = N+1]
    //   check ej_valid=1 and payload (exact cycle N+1)
    //   tick N+2 : check ej_valid=0 (one-cycle pulse)
    //
    // Deflection detection:
    //   If a flit is deflected it arrives LATER than N+1 cycles. The exact-
    //   cycle check at N+1 will see ej_valid=0 and FAIL, catching the problem
    //   without needing a separate deflection counter. Since Stage 2 has no
    //   contention, any such failure indicates a routing or wiring bug.
    // ------------------------------------------------------------------
    task automatic multi_hop_test(
        string test_name,
        int    sx, sy,          // source PE coordinates
        int    destx, desty,    // destination PE coordinates
        int    hop_count,       // number of hops (latency = hop_count + 1)
        logic [DATA_WIDTH-1:0] payload
    );
        $display("\n--- %s ---", test_name);
        $display("    Source: PE(%0d,%0d)  Dest: PE(%0d,%0d)  hops=%0d  expected_latency=%0d",
                 sx, sy, destx, desty, hop_count, hop_count + 1);

        // Drive injection
        inj_valid_drv[sx][sy] = 1'b1;
        inj_flit_drv[sx][sy]  = make_flit(X_BITS'(destx), Y_BITS'(desty), payload);
        #1; // allow continuous assign + combinational logic to settle

        // inj_ready is combinational — check while inj_valid is still high
        check($sformatf("%s: inj_ready at PE(%0d,%0d)", test_name, sx, sy),
              inj_ready_arr[sx][sy] === 1'b1);

        // Tick 1: source router latches flit into its output link
        @(posedge clk); #1;

        // Flit captured by source register — clear injection so source
        // does not attempt a second injection in the next cycle
        inj_valid_drv[sx][sy] = 1'b0;
        inj_flit_drv[sx][sy]  = '0;

        // Cycle-1 check: flit cannot have reached destination yet
        // (minimum latency for any path is 2 cycles)
        check($sformatf("%s: ej_valid=0 at PE(%0d,%0d) cycle 1 (no early arrival)",
              test_name, destx, desty),
              ej_valid_arr[destx][desty] === 1'b0);

        // Wait remaining ticks — total elapsed after this block = 1 + hop_count = N+1
        repeat(hop_count) @(posedge clk);
        #1;

        // Exact-cycle latency check — both a correctness and deflection assertion.
        // If ej_valid is 0 here, the flit either hasn't arrived yet (deflected/slow)
        // or went to the wrong PE (misrouted). Both are failures at this stage.
        check($sformatf("%s: ej_valid=1 at PE(%0d,%0d) at exact cycle %0d",
              test_name, destx, desty, hop_count + 1),
              ej_valid_arr[destx][desty] === 1'b1);

        // Payload integrity — first time payload passes through more than one
        // router's registered path; checks for width-mismatch or packing bugs
        // at transit routers (not just source/destination as in Stage 1)
        check($sformatf("%s: payload 0x%08h correct at PE(%0d,%0d)",
              test_name, payload, destx, desty),
              ej_flit_arr[destx][desty].data === payload);

        // One-cycle pulse check — ejection must not persist
        @(posedge clk); #1;
        check($sformatf("%s: ej_valid=0 at PE(%0d,%0d) cycle %0d (one-cycle pulse)",
              test_name, destx, desty, hop_count + 2),
              ej_valid_arr[destx][desty] === 1'b0);

    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin

        // Initialise all injection drivers to 0
        for(int i = 0; i < TORUS_X; i++)
            for(int j = 0; j < TORUS_Y; j++) begin
                inj_valid_drv[i][j] = 1'b0;
                inj_flit_drv[i][j]  = '0;
            end

        $display("=== tb_stage2: Multi-Hop Delivery Tests ===");
        $display("    Torus: %0d x %0d  |  Formula: N hops = N+1 cycles", TORUS_X, TORUS_Y);

        // Reset
        rst_n = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        repeat(2) @(posedge clk);
        #1;
        $display("Reset released.\n");

        // ==============================================================
        // C1: 2-hop straight X — (0,1) → (2,1) via (1,1)
        //
        // (0,1): dest_x=2, cx=0: fwd_x=2, bwd_x=2 → tie → East
        // (1,1): dest_x=2, cx=1: fwd_x=1, bwd_x=3 → East  [straight X transit]
        // (2,1): dest=(2,1)=(cx,cy) → eject
        //
        // Transit router (1,1) stays in X-phase: exercises dest_x != cx branch.
        // ==============================================================
        multi_hop_test("C1 2-hop straight X (0,1)→(2,1)", 0, 1, 2, 1, 2, 32'hAA11_0001);

        // ==============================================================
        // C2: 2-hop X→Y turn — (0,0) → (1,1) via (1,0)
        //
        // (0,0): dest_x=1, cx=0: fwd_x=1, bwd_x=3 → East
        // (1,0): dest_x=1=cx → dest_y=1>cy=0: fwd_y=1, bwd_y=3 → North  [X→Y turn]
        // (1,1): dest=(1,1)=(cx,cy) → eject
        //
        // Transit router (1,0) switches from X to Y phase — the first time
        // a transit router (not source/destination) evaluates dest_x == cx.
        // ==============================================================
        multi_hop_test("C2 2-hop X→Y turn  (0,0)→(1,1)", 0, 0, 1, 1, 2, 32'hBB22_0001);

        // ==============================================================
        // C3: 3-hop 2X+1Y — (0,0) → (2,1) via (1,0), (2,0)
        //
        // (0,0): dest_x=2, cx=0: fwd_x=2, bwd_x=2 → tie → East
        // (1,0): dest_x=2, cx=1: fwd_x=1, bwd_x=3 → East  [straight X transit]
        // (2,0): dest_x=2=cx → dest_y=1>cy=0: fwd_y=1, bwd_y=3 → North  [X→Y turn]
        // (2,1): dest=(2,1)=(cx,cy) → eject
        //
        // Confirms N+1 formula at N=3. Two consecutive transit routers compose
        // correctly — straight-X transit then X→Y-turn transit.
        // ==============================================================
        multi_hop_test("C3 3-hop 2X+1Y     (0,0)→(2,1)", 0, 0, 2, 1, 3, 32'hCC33_0001);

        // ==============================================================
        // C4: 2-hop wraparound transit — (3,1) → (1,1) via (0,1)
        //
        // (3,1): dest_x=1, cx=3: fwd_x=(1-3+4)%4=2, bwd_x=2 → tie → East (wrap)
        //        exits east_out → link_h[3][1] → west_in of (0,1)
        // (0,1): dest_x=1, cx=0: fwd_x=1, bwd_x=3 → East  [straight X transit]
        // (1,1): dest=(1,1)=(cx,cy) → eject
        //
        // Highest-value case in Stage 2: link_h[3][1] carries TRANSIT traffic
        // for the first time (Stage 1 only used wraparound links for final delivery).
        // Confirms the modular wiring in torus_top composes correctly across hops.
        // ==============================================================
        multi_hop_test("C4 2-hop wrap      (3,1)→(1,1)", 3, 1, 1, 1, 2, 32'hDD44_0001);

        // ==============================================================
        // C5: 2-hop straight Y — (1,0) → (1,2) via (1,1)
        //
        // (1,0): dest_x=1=cx → dest_y=2>cy=0: fwd_y=2, bwd_y=2 → tie → North
        // (1,1): dest_x=1=cx → dest_y=2>cy=1: fwd_y=1, bwd_y=3 → North  [straight Y]
        // (1,2): dest=(1,2)=(cx,cy) → eject
        //
        // Transit router stays in Y-phase the whole way. Exercises dest_x==cx
        // at a transit router on a path that never needed X-routing at all.
        // ==============================================================
        multi_hop_test("C5 2-hop straight Y (1,0)→(1,2)", 1, 0, 1, 2, 2, 32'hEE55_0001);

        // ==============================================================
        // C6: 3-hop 1X+2Y — (0,1) → (1,3) via (1,1), (1,2)
        //
        // (0,1): dest_x=1, cx=0: fwd_x=1, bwd_x=3 → East
        // (1,1): dest_x=1=cx → dest_y=3>cy=1: fwd_y=2, bwd_y=2 → tie → North  [X→Y turn]
        // (1,2): dest_x=1=cx → dest_y=3>cy=2: fwd_y=1, bwd_y=3 → North  [straight Y]
        // (1,3): dest=(1,3)=(cx,cy) → eject
        //
        // N=3, latency=4. X→Y turn at first transit, straight Y at second.
        // Three distinct router behaviours in one path: source, turn-transit,
        // straight-transit, destination.
        // ==============================================================
        multi_hop_test("C6 3-hop 1X+2Y     (0,1)→(1,3)", 0, 1, 1, 3, 3, 32'hFF66_0001);

        // ==============================================================
        // Summary
        // ==============================================================
        $display("\n==========================================");
        $display("  STAGE 2 SUMMARY");
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if(fail_count == 0)
            $display("  ALL STAGE 2 TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above.");

        $finish;
    end

endmodule
