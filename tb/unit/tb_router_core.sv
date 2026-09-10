// tb/unit/tb_router_core.sv
// Directed unit testbench for router_core.
// Tests one router in isolation at coordinates (cx=1, cy=1) in a 4x4 torus.
// All neighbors are driven directly by this testbench — no real adjacent routers.
//
// Key rule: output pipeline register adds ONE cycle.
// Drive inputs at cycle N → sample outputs at cycle N+1.

`timescale 1ns/1ps

module tb_router_core;

    import noc_pkg::*;

    // ------------------------------------------------------------------
    // Clock and reset
    // ------------------------------------------------------------------
    logic clk;
    logic rst_n;

    // 10ns clock period
    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // Interface instantiations
    // All 4 router-to-router links + 1 PE port
    // ------------------------------------------------------------------
    router_link_if north_in_if  ();
    router_link_if north_out_if ();
    router_link_if south_in_if  ();
    router_link_if south_out_if ();
    router_link_if east_in_if   ();
    router_link_if east_out_if  ();
    router_link_if west_in_if   ();
    router_link_if west_out_if  ();

    pe_port_if pe_if (.clk(clk));

    // ------------------------------------------------------------------
    // DUT: router at position (cx=1, cy=1)
    // Interior router — all 4 directions are real neighbors.
    // ------------------------------------------------------------------
    localparam logic [1:0] CX = 2'd1;
    localparam logic [1:0] CY = 2'd1;

    router_core dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .cx        (CX),
        .cy        (CY),
        .north_in  (north_in_if.snk),
        .north_out (north_out_if.src),
        .south_in  (south_in_if.snk),
        .south_out (south_out_if.src),
        .east_in   (east_in_if.snk),
        .east_out  (east_out_if.src),
        .west_in   (west_in_if.snk),
        .west_out  (west_out_if.src),
        .pe_port   (pe_if.router)
    );

    // ------------------------------------------------------------------
    // Helper task: drive all inputs idle (safe default between tests)
    // ------------------------------------------------------------------
    task idle_all();
        north_in_if.valid  = 0; north_in_if.flit  = '0;
        south_in_if.valid  = 0; south_in_if.flit  = '0;
        east_in_if.valid   = 0; east_in_if.flit   = '0;
        west_in_if.valid   = 0; west_in_if.flit   = '0;
        pe_if.inj_valid    = 0; pe_if.inj_flit    = '0;
    endtask

    // ------------------------------------------------------------------
    // Helper task: tick clock N times
    // ------------------------------------------------------------------
    task tick(int n = 1);
        repeat(n) @(posedge clk);
        #1; // small delay so outputs are stable before we sample
    endtask

    // ------------------------------------------------------------------
    // Helper function: build a flit quickly
    // ------------------------------------------------------------------
    function automatic flit_t make_flit(
        logic [X_BITS-1:0] dx,
        logic [Y_BITS-1:0] dy,
        logic [DATA_WIDTH-1:0] dat
    );
        make_flit.X_dest = dx;
        make_flit.Y_dest = dy;
        make_flit.data   = dat;
    endfunction

    // ------------------------------------------------------------------
    // Pass/fail counter
    // ------------------------------------------------------------------
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
    // SELF-INJECTION ASSERTION (always running)
    // Sep8 feedback: document assumption as a loud, checked property.
    // Fires anytime the PE tries to inject a flit to its own router.
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (pe_if.inj_valid && pe_if.inj_flit.X_dest == CX && pe_if.inj_flit.Y_dest == CY) begin
            $error("[%0t] SELF-INJECTION: PE at (%0d,%0d) injected flit destined for itself!",
                   $time, CX, CY);
            fail_count++;
        end
    end

    // ------------------------------------------------------------------
    // MAIN TEST SEQUENCE
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_router_core: Directed Tests (cx=1, cy=1) ===");

        // ---- RESET ----
        rst_n = 0;
        idle_all();
        tick(3);
        rst_n = 1;
        tick(2);

        // ==============================================================
        // TEST 1: Reset state — all outputs must be deasserted
        // ==============================================================
        $display("\n--- Test 1: Reset State ---");
        check("north_out idle after reset", north_out_if.valid === 0);
        check("south_out idle after reset", south_out_if.valid === 0);
        check("east_out  idle after reset", east_out_if.valid  === 0);
        check("west_out  idle after reset", west_out_if.valid  === 0);
        check("ej_valid  idle after reset", pe_if.ej_valid     === 0);

        // ==============================================================
        // TEST 2: Single source — each direction, one at a time
        // Router at (1,1).
        // North output:  dest_x=1, dest_y=2  (one hop North from cy=1)
        // South output:  dest_x=1, dest_y=0  (one hop South)
        // East output:   dest_x=2, dest_y=1  (one hop East)
        // West output:   dest_x=0, dest_y=1  (one hop West)
        // Ejection:      dest_x=1, dest_y=1  NOT TESTED (self-dest assumption)
        //
        // For each: drive one input, idle all others, wait 1 cycle, check.
        // ==============================================================
        $display("\n--- Test 2a: Single flit from North-in to East-out ---");
        // Flit arrives from North, wants to go East (dest_x=2, dest_y=1)
        north_in_if.valid = 1;
        north_in_if.flit  = make_flit(2'd2, 2'd1, 32'hAAAA_0001);
        tick(1);
        idle_all();
        check("flit from North routes East",
              east_out_if.valid === 1 && east_out_if.flit.data === 32'hAAAA_0001);
        check("no other output asserted (N-in to E-out test)",
              north_out_if.valid === 0 && south_out_if.valid === 0 &&
              west_out_if.valid  === 0 && pe_if.ej_valid     === 0);
        tick(1);

        $display("\n--- Test 2b: Single flit from South-in to West-out ---");
        south_in_if.valid = 1;
        south_in_if.flit  = make_flit(2'd0, 2'd1, 32'hBBBB_0002);
        tick(1);
        idle_all();
        check("flit from South routes West",
              west_out_if.valid === 1 && west_out_if.flit.data === 32'hBBBB_0002);
        check("no other output asserted (S-in to W-out test)",
              north_out_if.valid === 0 && south_out_if.valid === 0 &&
              east_out_if.valid  === 0 && pe_if.ej_valid     === 0);
        tick(1);

        $display("\n--- Test 2c: Single flit from East-in to North-out ---");
        east_in_if.valid = 1;
        east_in_if.flit  = make_flit(2'd1, 2'd2, 32'hCCCC_0003);
        tick(1);
        idle_all();
        check("flit from East routes North",
              north_out_if.valid === 1 && north_out_if.flit.data === 32'hCCCC_0003);
        check("no other output asserted (E-in to N-out test)",
              south_out_if.valid === 0 && east_out_if.valid  === 0 &&
              west_out_if.valid  === 0 && pe_if.ej_valid     === 0);
        tick(1);

        $display("\n--- Test 2d: Single flit from West-in to South-out ---");
        west_in_if.valid = 1;
        west_in_if.flit  = make_flit(2'd1, 2'd0, 32'hDDDD_0004);
        tick(1);
        idle_all();
        check("flit from West routes South",
              south_out_if.valid === 1 && south_out_if.flit.data === 32'hDDDD_0004);
        check("no other output asserted (W-in to S-out test)",
              north_out_if.valid === 0 && east_out_if.valid  === 0 &&
              west_out_if.valid  === 0 && pe_if.ej_valid     === 0);
        tick(1);

        $display("\n--- Test 2e: Single flit from North-in — EJECT to PE ---");
        // Arrival ejection from network (dest == this router). NOT self-injection.
        north_in_if.valid = 1;
        north_in_if.flit  = make_flit(2'd1, 2'd1, 32'hEEEE_0005);
        tick(1);
        idle_all();
        check("flit from North ejects to PE",
              pe_if.ej_valid === 1 && pe_if.ej_flit.data === 32'hEEEE_0005);
        check("no network output asserted on ejection",
              north_out_if.valid === 0 && south_out_if.valid === 0 &&
              east_out_if.valid  === 0 && west_out_if.valid  === 0);
        tick(1);

        $display("\n--- Test 2f: PE injection alone to East-out ---");
        pe_if.inj_valid = 1;
        pe_if.inj_flit  = make_flit(2'd2, 2'd1, 32'hFFFF_0006);
        tick(1);
        check("PE injection accepted (inj_ready high)", pe_if.inj_ready === 1);
        idle_all();
        check("PE flit routed East",
              east_out_if.valid === 1 && east_out_if.flit.data === 32'hFFFF_0006);
        tick(1);

        // ==============================================================
        // TEST 3: Two-source contention — N and S both want East
        // N > S priority: North wins East, South is deflected.
        // ==============================================================
        $display("\n--- Test 3: Contention — N and S both want East, N wins ---");
        north_in_if.valid = 1;
        north_in_if.flit  = make_flit(2'd2, 2'd1, 32'h1111_AAAA);
        south_in_if.valid = 1;
        south_in_if.flit  = make_flit(2'd2, 2'd1, 32'h2222_BBBB);
        tick(1);
        idle_all();
        check("North flit wins East in contention",
              east_out_if.valid === 1 && east_out_if.flit.data === 32'h1111_AAAA);
        begin
            automatic int south_found = 0;
            if (north_out_if.valid && north_out_if.flit.data === 32'h2222_BBBB) south_found++;
            if (south_out_if.valid && south_out_if.flit.data === 32'h2222_BBBB) south_found++;
            if (west_out_if.valid  && west_out_if.flit.data  === 32'h2222_BBBB) south_found++;
            check("South flit deflected to exactly one free output", south_found === 1);
        end
        tick(1);

        // ==============================================================
        // TEST 4: All 4 transit inputs active, no contention
        // Each wants a different direction → clean permutation.
        // ==============================================================
        $display("\n--- Test 4: All 4 transit inputs, no contention ---");
        north_in_if.valid = 1; north_in_if.flit = make_flit(2'd2, 2'd1, 32'hA100_DEAD); // East
        south_in_if.valid = 1; south_in_if.flit = make_flit(2'd0, 2'd1, 32'hB200_BEEF); // West
        east_in_if.valid  = 1; east_in_if.flit  = make_flit(2'd1, 2'd2, 32'hC300_CAFE); // North
        west_in_if.valid  = 1; west_in_if.flit  = make_flit(2'd1, 2'd0, 32'hD400_BABE); // South
        tick(1);
        idle_all();
        check("All 4 transit: North-in to East-out",
              east_out_if.valid  && east_out_if.flit.data  === 32'hA100_DEAD);
        check("All 4 transit: South-in to West-out",
              west_out_if.valid  && west_out_if.flit.data  === 32'hB200_BEEF);
        check("All 4 transit: East-in to North-out",
              north_out_if.valid && north_out_if.flit.data === 32'hC300_CAFE);
        check("All 4 transit: West-in to South-out",
              south_out_if.valid && south_out_if.flit.data === 32'hD400_BABE);
        check("No ejection in Test 4", pe_if.ej_valid === 0);
        tick(1);

        // ==============================================================
        // TEST 5: One ejector among 4 transit inputs
        // ==============================================================
        $display("\n--- Test 5: One ejector among 4 transit inputs ---");
        north_in_if.valid = 1; north_in_if.flit = make_flit(2'd1, 2'd1, 32'hECEC_0001); // eject
        south_in_if.valid = 1; south_in_if.flit = make_flit(2'd2, 2'd1, 32'hBAA0_0002); // East
        east_in_if.valid  = 1; east_in_if.flit  = make_flit(2'd1, 2'd0, 32'hCBB0_0003); // South
        west_in_if.valid  = 1; west_in_if.flit  = make_flit(2'd0, 2'd1, 32'hDCC0_0004); // West
        tick(1);
        idle_all();
        check("Ejector appears on ej_valid",
              pe_if.ej_valid === 1 && pe_if.ej_flit.data === 32'hECEC_0001);
        check("Ejector does NOT appear on any network output",
              !(north_out_if.valid && north_out_if.flit.data === 32'hECEC_0001) &&
              !(south_out_if.valid && south_out_if.flit.data === 32'hECEC_0001) &&
              !(east_out_if.valid  && east_out_if.flit.data  === 32'hECEC_0001) &&
              !(west_out_if.valid  && west_out_if.flit.data  === 32'hECEC_0001));
        tick(1);

        // ==============================================================
        // TEST 6: Two simultaneous ejectors
        // North wins ejection. South is deflected (not dropped).
        // ==============================================================
        $display("\n--- Test 6: Two simultaneous ejectors ---");
        north_in_if.valid = 1; north_in_if.flit = make_flit(2'd1, 2'd1, 32'hECEC_0001);
        south_in_if.valid = 1; south_in_if.flit = make_flit(2'd1, 2'd1, 32'hECEC_0002);
        tick(1);
        idle_all();
        check("North wins ejection in dual-ejector case",
              pe_if.ej_valid === 1 && pe_if.ej_flit.data === 32'hECEC_0001);
        begin
            automatic int south_ejector_found = 0;
            if (north_out_if.valid && north_out_if.flit.data === 32'hECEC_0002) south_ejector_found++;
            if (south_out_if.valid && south_out_if.flit.data === 32'hECEC_0002) south_ejector_found++;
            if (east_out_if.valid  && east_out_if.flit.data  === 32'hECEC_0002) south_ejector_found++;
            if (west_out_if.valid  && west_out_if.flit.data  === 32'hECEC_0002) south_ejector_found++;
            check("Second ejector deflected (not dropped)", south_ejector_found === 1);
        end
        tick(1);

        // ==============================================================
        // TEST 7: PE injection contends with transit for same output
        // Transit wins East. PE falls back to any other free slot.
        // ==============================================================
        $display("\n--- Test 7: PE injection contends with transit for East ---");
        east_in_if.valid = 1; east_in_if.flit = make_flit(2'd2, 2'd1, 32'hEA57_0001);
        pe_if.inj_valid  = 1; pe_if.inj_flit  = make_flit(2'd2, 2'd1, 32'h00EE_0001);
        tick(1);
        check("PE accepted even when desired output taken", pe_if.inj_ready === 1);
        idle_all();
        check("Transit flit on East output",
              east_out_if.valid === 1 && east_out_if.flit.data === 32'hEA57_0001);
        check("PE flit on a different free output",
              (north_out_if.valid && north_out_if.flit.data === 32'h00EE_0001) ||
              (south_out_if.valid && south_out_if.flit.data === 32'h00EE_0001) ||
              (west_out_if.valid  && west_out_if.flit.data  === 32'h00EE_0001));
        tick(1);

        // ==============================================================
        // TEST 8: Full saturation — all 4 transit + PE → PE stalls
        // ==============================================================
        $display("\n--- Test 8: Full saturation — PE must stall (inj_ready=0) ---");
        north_in_if.valid = 1; north_in_if.flit = make_flit(2'd2, 2'd1, 32'h5A70_0001);
        south_in_if.valid = 1; south_in_if.flit = make_flit(2'd0, 2'd1, 32'h5A70_0002);
        east_in_if.valid  = 1; east_in_if.flit  = make_flit(2'd1, 2'd2, 32'h5A70_0003);
        west_in_if.valid  = 1; west_in_if.flit  = make_flit(2'd1, 2'd0, 32'h5A70_0004);
        pe_if.inj_valid   = 1; pe_if.inj_flit   = make_flit(2'd2, 2'd0, 32'h5A70_0005);
        @(posedge clk); #1;
        check("PE stalls when all transit slots full (inj_ready=0)", pe_if.inj_ready === 0);
        // Clear transit — PE still holds inj_valid high, retries next cycle
        north_in_if.valid = 0;
        south_in_if.valid = 0;
        east_in_if.valid  = 0;
        west_in_if.valid  = 0;
        @(posedge clk); #1;
        check("PE accepted on retry when transit clears (inj_ready=1)", pe_if.inj_ready === 1);
        pe_if.inj_valid = 0;
        tick(1);

        // ==============================================================
        // TEST 9: Idle cycle
        // ==============================================================
        $display("\n--- Test 9: Idle cycle ---");
        idle_all();
        tick(1);
        check("North-out idle", north_out_if.valid === 0);
        check("South-out idle", south_out_if.valid === 0);
        check("East-out idle",  east_out_if.valid  === 0);
        check("West-out idle",  west_out_if.valid  === 0);
        check("Ejection idle",  pe_if.ej_valid     === 0);
        check("inj_ready idle", pe_if.inj_ready    === 0);

        // ==============================================================
        // TEST 10: Reset mid-stream
        // ==============================================================
        $display("\n--- Test 10: Reset mid-stream ---");
        north_in_if.valid = 1; north_in_if.flit = make_flit(2'd2, 2'd1, 32'h0057_0001);
        @(posedge clk); #1;
        rst_n = 0;
        @(posedge clk); #1;
        check("North-out clears on reset", north_out_if.valid === 0);
        check("South-out clears on reset", south_out_if.valid === 0);
        check("East-out clears on reset",  east_out_if.valid  === 0);
        check("West-out clears on reset",  west_out_if.valid  === 0);
        check("Ejection clears on reset",  pe_if.ej_valid     === 0);
        rst_n = 1;
        idle_all();
        tick(2);

        // ==============================================================
        // TEST 11: Boundary coordinate note
        // Corner router tests need a separate DUT instance with cx=0,cy=0
        // and cx=3,cy=3 to exercise wraparound splitter arithmetic.
        // See tb_router_core_corner.sv.
        // ==============================================================
        $display("\n--- Test 11: Boundary Coordinates ---");
        $display("  [NOTE] Corner tests (cx=0,cy=0 and cx=3,cy=3) in tb_router_core_corner.sv");

        // ==============================================================
        // SUMMARY
        // ==============================================================
        $display("\n==========================================");
        $display("  DIRECTED TEST SUMMARY");
        $display("  PASS: %0d  |  FAIL: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count == 0)
            $display("  ALL DIRECTED TESTS PASSED.");
        else
            $display("  FAILURES DETECTED — review above.");

        $finish;
    end

endmodule
