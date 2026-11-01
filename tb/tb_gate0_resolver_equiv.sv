// tb/tb_gate0_resolver_equiv.sv
// Gate 0: POOL_ENABLE = 0 Equivalence Regression Testbench
// Instantiates baseline deflect_resolver and rtl_adaptive deflect_resolver side by side,
// drives identical random stimulus into both, and asserts that valid_out, flit_out,
// and inj_ready match bit-for-bit on every evaluation.

`timescale 1ns/1ps

module tb_gate0_resolver_equiv;

    import noc_pkg::*;

    // Inputs to both resolvers (5 sources: N=0, S=1, E=2, W=3, PE=4)
    logic       valid_in [5];
    flit_t      flit_in  [5];
    logic [4:0] desired  [5];
    logic       is_eject [5];

    // Outputs from baseline resolver
    logic       valid_out_orig [5];
    flit_t      flit_out_orig  [5];
    logic       inj_ready_orig;

    // Outputs from adaptive resolver (POOL_ENABLE=0)
    logic       valid_out_adapt [5];
    flit_t      flit_out_adapt  [5];
    logic       inj_ready_adapt;
    logic       pool_req_adapt;
    flit_t      pool_cand_flit_adapt;
    logic       pool_offer_accepted_adapt;
    logic [2:0] process_count_adapt;
    logic [2:0] potential_deflect_count_adapt;
    logic [2:0] actual_deflect_count_adapt;
    logic [2:0] deflect_count_adapt;
    logic       did_deflect_adapt;
    logic       did_process_adapt;
    logic       did_buf_request_adapt;
    logic       did_buf_request_fail_adapt;

    // Instantiate baseline resolver
    deflect_resolver u_orig (
        .valid_in  (valid_in),
        .flit_in   (flit_in),
        .desired   (desired),
        .is_eject  (is_eject),
        .valid_out (valid_out_orig),
        .flit_out  (flit_out_orig),
        .inj_ready (inj_ready_orig)
    );

    // Instantiate adaptive resolver compiled with -d DEFLECT_RESOLVER_MODNAME=deflect_resolver_adapt
    deflect_resolver_adapt #(
        .POOL_ENABLE (0),
        .NUM_SOURCES (5)
    ) u_adapt (
        .valid_in                (valid_in),
        .flit_in                 (flit_in),
        .desired                 (desired),
        .is_eject                (is_eject),
        .valid_out               (valid_out_adapt),
        .flit_out                (flit_out_adapt),
        .inj_ready               (inj_ready_adapt),
        .pool_req                (pool_req_adapt),
        .pool_cand_flit          (pool_cand_flit_adapt),
        .pool_grant              (1'b0),
        .pool_offer_accepted     (pool_offer_accepted_adapt),
        .process_count           (process_count_adapt),
        .potential_deflect_count (potential_deflect_count_adapt),
        .actual_deflect_count    (actual_deflect_count_adapt),
        .deflect_count           (deflect_count_adapt),
        .did_deflect             (did_deflect_adapt),
        .did_process             (did_process_adapt),
        .did_buf_request         (did_buf_request_adapt),
        .did_buf_request_fail    (did_buf_request_fail_adapt)
    );

    // PRNG xorshift32
    logic [31:0] prng_state = 32'hBADC_0FFE;
    function automatic logic [31:0] xorshift32();
        prng_state ^= prng_state << 13;
        prng_state ^= prng_state >> 17;
        prng_state ^= prng_state << 5;
        return prng_state;
    endfunction

    // Test loop
    localparam int NUM_VECTORS = 10000;

    initial begin
        // ALL variable declarations at top of block per XSIM rule
        int pass_count;
        int fail_count;
        int dir;
        bit flit_match;
        logic [31:0] rnd;

        pass_count = 0;
        fail_count = 0;

        $display("=========================================================================");
        $display("=== GATE 0: deflect_resolver POOL_ENABLE=0 Bit-Exact Equivalence Test ===");
        $display("=== Testing %0d randomized vectors against untouched baseline...       ===", NUM_VECTORS);
        $display("=========================================================================");

        for (int v = 0; v < NUM_VECTORS; v++) begin
            // Generate stimulus for 5 ports
            for (int i = 0; i < 5; i++) begin
                rnd = xorshift32();
                valid_in[i] = rnd[0];
                is_eject[i] = (i < 4) ? rnd[1] : 1'b0; // PE never ejects locally

                // Desired direction: 1-hot
                desired[i] = 5'b0;
                if (valid_in[i]) begin
                    if (is_eject[i]) begin
                        desired[i][4] = 1'b1; // Local
                    end else begin
                        dir = int'(rnd[4:2]) % 4; // N, S, E, or W
                        desired[i][dir] = 1'b1;
                    end
                end

                // Random flit payload
                rnd = xorshift32();
                flit_in[i].X_dest = rnd[1:0];
                flit_in[i].Y_dest = rnd[3:2];
                flit_in[i].data   = rnd;
            end

            // Settle combinational logic
            #1;

            // Assert exact equivalence
            if ((valid_out_orig !== valid_out_adapt) ||
                (inj_ready_orig !== inj_ready_adapt)) begin
                $display("[FAIL Vector %0d] valid_out mismatch: orig=%b%b%b%b%b adapt=%b%b%b%b%b, inj_ready orig=%b adapt=%b",
                         v, valid_out_orig[4], valid_out_orig[3], valid_out_orig[2], valid_out_orig[1], valid_out_orig[0],
                            valid_out_adapt[4], valid_out_adapt[3], valid_out_adapt[2], valid_out_adapt[1], valid_out_adapt[0],
                         inj_ready_orig, inj_ready_adapt);
                fail_count++;
            end else begin
                // Check flit_out on all active outputs
                flit_match = 1'b1;
                for (int d = 0; d < 5; d++) begin
                    if (valid_out_orig[d] && (flit_out_orig[d] !== flit_out_adapt[d])) begin
                        $display("[FAIL Vector %0d] Port %0d flit mismatch! orig=%h adapt=%h",
                                 v, d, flit_out_orig[d], flit_out_adapt[d]);
                        flit_match = 1'b0;
                    end
                end
                if (flit_match) pass_count++;
                else fail_count++;
            end
        end

        $display("=========================================================================");
        if (fail_count == 0) begin
            $display("=== GATE 0 PASSED: 100%% bit-exact match across %0d vectors!           ===", pass_count);
        end else begin
            $display("=== GATE 0 FAILED: %0d mismatches detected!                            ===", fail_count);
        end
        $display("=========================================================================");

        if (fail_count > 0) $fatal(1, "Gate 0 equivalence check failed!");
        $finish;
    end

endmodule
