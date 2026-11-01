// tb/tb_injection_predictor.sv
// Unit-level testbench for injection_predictor.sv (Mechanism 1)
//
// Directed tests covering Part 4 of ip_idea.txt:
//   Test 1: Single clean hit -> counter decrements / stays 0, throttle_hint stays low
//   Test 2: Single fallback event -> counter increments to 1
//   Test 3: Two consecutive fallback events -> counter reaches throttle threshold (2), throttle_hint goes high
//   Test 4: Full rejection (last_inj_accepted=0) -> counter increments same as fallback
//   Test 5: Idle for 64+ cycles -> confirm decay timer triggers and counter decrements
//   Test 6: Saturated counter returns to not-throttled within the 127-cycle bound
//   Test 7: was_gated=0 suppresses feedback processing (no false learning when throttled)
//   Test 8: Asymmetric independence (all 4 directions operate independently)

`timescale 1ns/1ps

module tb_injection_predictor;

    import noc_pkg::*;

    // Clock and Reset
    logic clk;
    logic rst_n;

    // Router coordinates (testing node (0,0))
    logic [X_BITS-1:0] cx;
    logic [Y_BITS-1:0] cy;

    // Candidate destination
    logic [X_BITS-1:0] cand_dest_x;
    logic [Y_BITS-1:0] cand_dest_y;

    // Feedback inputs
    logic pe_took_fallback;
    logic last_inj_accepted;
    logic was_gated;

    // Output
    logic throttle_hint;

    // Instantiate DUT
    injection_predictor dut (
        .clk(clk),
        .rst_n(rst_n),
        .cx(cx),
        .cy(cy),
        .cand_dest_x(cand_dest_x),
        .cand_dest_y(cand_dest_y),
        .pe_took_fallback(pe_took_fallback),
        .last_inj_accepted(last_inj_accepted),
        .was_gated(was_gated),
        .throttle_hint(throttle_hint)
    );

    // Clock generator: 100 MHz (10ns period)
    initial clk = 0;
    always #5 clk = ~clk;

    // Helper task to step one cycle
    task automatic step_cycle(
        input logic [X_BITS-1:0] dx,
        input logic [Y_BITS-1:0] dy,
        input logic wg,
        input logic acc,
        input logic fb
    );
        cand_dest_x       = dx;
        cand_dest_y       = dy;
        was_gated         = wg;
        last_inj_accepted = acc;
        pe_took_fallback  = fb;
        @(posedge clk);
        #1;
    endtask

    // Reset task
    task automatic do_reset();
        rst_n             = 1'b0;
        cx                = '0;
        cy                = '0;
        cand_dest_x       = '0;
        cand_dest_y       = '0;
        pe_took_fallback  = 1'b0;
        last_inj_accepted = 1'b0;
        was_gated         = 1'b0;
        repeat (2) @(posedge clk);
        #1;
        rst_n = 1'b1;
        @(posedge clk);
        #1;
    endtask

    // Destination coordinates for node (0,0) in 4x4 torus:
    // East:  (1,0) -> DIR_E = 2
    // West:  (3,0) -> DIR_W = 3
    // North: (0,1) -> DIR_N = 0
    // South: (0,3) -> DIR_S = 1
    localparam logic [X_BITS-1:0] DEST_E_X = 2'd1, DEST_E_Y = 2'd0;
    localparam logic [X_BITS-1:0] DEST_W_X = 2'd3, DEST_W_Y = 2'd0;
    localparam logic [X_BITS-1:0] DEST_N_X = 2'd0, DEST_N_Y = 2'd1;
    localparam logic [X_BITS-1:0] DEST_S_X = 2'd0, DEST_S_Y = 2'd3;

    initial begin
        // All local variable declarations first (strict XSIM rule)
        int pass_count;
        int fail_count;
        int idle_cyc;
        int unthrottled_at;

        pass_count = 0;
        fail_count = 0;

        $display("===================================================================");
        $display("=== Unit Testbench: injection_predictor (Mechanism 1)           ===");
        $display("===================================================================");

        // -------------------------------------------------------------
        // Test 1: Single clean hit (East)
        // -------------------------------------------------------------
        $display("\n--- Test 1: Single clean hit on East ---");
        do_reset();

        // Cycle 1: PE sets candidate destination East
        step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);

        // Cycle 2: Feedback arrives: accepted cleanly (pe_took_fallback=0)
        step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b0);

        if (dut.counter[DIR_E] == 2'b00 && throttle_hint == 1'b0) begin
            $display("  [PASS] Clean hit: counter[E]=%0d, throttle_hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            pass_count++;
        end else begin
            $display("  [FAIL] Clean hit: expected counter[E]=0, throttle_hint=0, got counter[E]=%0d, hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 2: Single fallback event (East)
        // -------------------------------------------------------------
        $display("\n--- Test 2: Single fallback event on East ---");
        do_reset();

        // Cycle 1: register candidate East
        step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);

        // Cycle 2: feedback: accepted but took fallback slot
        step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b1);

        if (dut.counter[DIR_E] == 2'b01 && throttle_hint == 1'b0) begin
            $display("  [PASS] Single fallback: counter[E]=%0d, throttle_hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            pass_count++;
        end else begin
            $display("  [FAIL] Single fallback: expected counter[E]=1, hint=0, got counter[E]=%0d, hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 3: Two consecutive fallback events (East) -> throttle fires
        // -------------------------------------------------------------
        $display("\n--- Test 3: Two consecutive fallbacks (East) -> throttle fires ---");
        do_reset();

        // Cycle 1: candidate East
        step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);

        // Cycle 2: 1st fallback -> counter[E] becomes 1
        step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b1);

        // Cycle 3: 2nd fallback -> counter[E] becomes 2, throttle_hint goes HI
        step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b1);

        if (dut.counter[DIR_E] == 2'b10 && throttle_hint == 1'b1) begin
            $display("  [PASS] Two fallbacks: counter[E]=%0d, throttle_hint=%0b (HI)",
                     dut.counter[DIR_E], throttle_hint);
            pass_count++;
        end else begin
            $display("  [FAIL] Two fallbacks: expected counter[E]=2, hint=1, got counter[E]=%0d, hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            fail_count++;
        end

        // Cycle 4: 3rd fallback -> saturates at 3, throttle_hint remains HI
        step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b1);

        if (dut.counter[DIR_E] == 2'b11 && throttle_hint == 1'b1) begin
            $display("  [PASS] Saturation at 3: counter[E]=%0d, throttle_hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            pass_count++;
        end else begin
            $display("  [FAIL] Saturation: expected counter[E]=3, hint=1, got counter[E]=%0d, hint=%0b",
                     dut.counter[DIR_E], throttle_hint);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 4: Full rejection on West (last_inj_accepted=0)
        // -------------------------------------------------------------
        $display("\n--- Test 4: Full rejection on West (last_inj_accepted=0) ---");
        do_reset();

        // Cycle 1: candidate West
        step_cycle(DEST_W_X, DEST_W_Y, 1'b0, 1'b0, 1'b0);

        // Cycle 2: full rejection (was_gated=1, last_inj_accepted=0)
        step_cycle(DEST_W_X, DEST_W_Y, 1'b1, 1'b0, 1'b0);

        if (dut.counter[DIR_W] == 2'b01) begin
            $display("  [PASS] Full rejection 1: counter[W]=%0d", dut.counter[DIR_W]);
            pass_count++;
        end else begin
            $display("  [FAIL] Full rejection 1: expected counter[W]=1, got %0d", dut.counter[DIR_W]);
            fail_count++;
        end

        // Cycle 3: second rejection -> reaches 2, throttle fires for West
        step_cycle(DEST_W_X, DEST_W_Y, 1'b1, 1'b0, 1'b0);

        if (dut.counter[DIR_W] == 2'b10 && throttle_hint == 1'b1) begin
            $display("  [PASS] Full rejection 2: counter[W]=%0d, throttle_hint=%0b (HI)",
                     dut.counter[DIR_W], throttle_hint);
            pass_count++;
        end else begin
            $display("  [FAIL] Full rejection 2: expected counter[W]=2, hint=1, got counter[W]=%0d, hint=%0b",
                     dut.counter[DIR_W], throttle_hint);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 5: Idle for 64+ cycles -> decay timer triggers
        // -------------------------------------------------------------
        $display("\n--- Test 5: Forced decay after 64 idle cycles ---");
        do_reset();

        // Pre-saturate South to 3
        step_cycle(DEST_S_X, DEST_S_Y, 1'b0, 1'b0, 1'b0);
        repeat (4) step_cycle(DEST_S_X, DEST_S_Y, 1'b1, 1'b1, 1'b1);

        if (dut.counter[DIR_S] == 2'b11) begin
            $display("  South pre-saturated to %0d", dut.counter[DIR_S]);
        end

        // Idle without feedback until decay timer reaches 63 (first decay tick)
        while (dut.decay_timer != 6'd63) begin
            step_cycle(DEST_S_X, DEST_S_Y, 1'b0, 1'b0, 1'b0);
        end

        // Next clock edge triggers the decay decrement
        step_cycle(DEST_S_X, DEST_S_Y, 1'b0, 1'b0, 1'b0);

        if (dut.counter[DIR_S] == 2'b10) begin
            $display("  [PASS] Decay fired at tick 63: counter[S] decremented 3 -> %0d",
                     dut.counter[DIR_S]);
            pass_count++;
        end else begin
            $display("  [FAIL] Decay: expected counter[S]=2 after tick, got %0d", dut.counter[DIR_S]);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 6: 127-cycle liveness bound verification
        // -------------------------------------------------------------
        $display("\n--- Test 6: Worst-case liveness bound (<= 127 cycles) ---");
        do_reset();

        // Pre-saturate East to 3
        step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);
        repeat (4) step_cycle(DEST_E_X, DEST_E_Y, 1'b1, 1'b1, 1'b1);

        // Now run idle with cand_dest=East until throttle_hint drops to 0
        unthrottled_at = -1;
        for (idle_cyc = 0; idle_cyc < 200; idle_cyc++) begin
            step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);
            if (throttle_hint == 1'b0 && unthrottled_at == -1) begin
                unthrottled_at = idle_cyc;
                break;
            end
        end

        if (unthrottled_at >= 0 && unthrottled_at <= 127) begin
            $display("  [PASS] Unthrottled at idle cycle %0d (<= 127 cycles bound met)",
                     unthrottled_at);
            pass_count++;
        end else begin
            $display("  [FAIL] Liveness bound: unthrottled_at=%0d (expected <= 127)",
                     unthrottled_at);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 7: was_gated=0 suppresses feedback processing (no false learning)
        // -------------------------------------------------------------
        $display("\n--- Test 7: was_gated=0 suppression ---");
        do_reset();

        // Candidate North
        step_cycle(DEST_N_X, DEST_N_Y, 1'b0, 1'b0, 1'b0);

        // Present feedback signals with was_gated=0 (PE was throttled last cycle)
        // Even though accepted=1 and fallback=1 are present, was_gated=0 must suppress counter update
        repeat (5) begin
            step_cycle(DEST_N_X, DEST_N_Y, 1'b0, 1'b1, 1'b1);
        end

        if (dut.counter[DIR_N] == 2'b00 && throttle_hint == 1'b0) begin
            $display("  [PASS] was_gated=0: counter[N] stayed 0 despite fallback inputs",
                     dut.counter[DIR_N]);
            pass_count++;
        end else begin
            $display("  [FAIL] was_gated=0: expected counter[N]=0, got %0d", dut.counter[DIR_N]);
            fail_count++;
        end

        // -------------------------------------------------------------
        // Test 8: Directional independence
        // -------------------------------------------------------------
        $display("\n--- Test 8: Asymmetric independence across 4 directions ---");
        do_reset();

        // Saturate West to 3
        step_cycle(DEST_W_X, DEST_W_Y, 1'b0, 1'b0, 1'b0);
        repeat (4) step_cycle(DEST_W_X, DEST_W_Y, 1'b1, 1'b1, 1'b1);

        // Check West is throttled, but North, South, East are NOT throttled
        step_cycle(DEST_W_X, DEST_W_Y, 1'b0, 1'b0, 1'b0);
        if (throttle_hint != 1'b1) begin
            $display("  [FAIL] West expected throttled");
            fail_count++;
        end else begin
            // Check East
            step_cycle(DEST_E_X, DEST_E_Y, 1'b0, 1'b0, 1'b0);
            if (throttle_hint != 1'b0) begin
                $display("  [FAIL] East false throttled");
                fail_count++;
            end else begin
                // Check North
                step_cycle(DEST_N_X, DEST_N_Y, 1'b0, 1'b0, 1'b0);
                if (throttle_hint != 1'b0) begin
                    $display("  [FAIL] North false throttled");
                    fail_count++;
                end else begin
                    // Check South
                    step_cycle(DEST_S_X, DEST_S_Y, 1'b0, 1'b0, 1'b0);
                    if (throttle_hint != 1'b0) begin
                        $display("  [FAIL] South false throttled");
                        fail_count++;
                    end else begin
                        $display("  [PASS] Only West is throttled (N=0, S=0, E=0, W=%0d)",
                                 dut.counter[DIR_W]);
                        pass_count++;
                    end
                end
            end
        end

        // -------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------
        $display("\n===================================================================");
        $display("=== Summary: %0d PASSED, %0d FAILED                             ===",
                 pass_count, fail_count);
        $display("===================================================================");

        if (fail_count == 0)
            $display("=== ALL PREDICTOR UNIT TESTS PASSED ===");
        else
            $display("=== SOME PREDICTOR UNIT TESTS FAILED ===");

        $finish;
    end

endmodule
