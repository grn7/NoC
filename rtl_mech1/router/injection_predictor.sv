// Injection Throttling Predictor - Mechanism 1
// Per-direction saturating counters that learn congestion state from real injection outcomes.
//
// Key design decisions (from ip_idea.txt Part 3):
//   - 2-bit counters: filters single-event noise (same reason branch predictors use 2-bit)
//   - 4 counters (one per direction): respects measured asymmetry (N always 0%, W heaviest)
//   - was_gated check: prevents "learning" from cycles where the PE was itself throttled
//   - Decay tick every 64 cycles: hard upper bound of ~127 cycles on any throttle duration
//     (2 decrements needed to clear the top bit from saturation: 3->2->1,
//      up to 63 cycles to the first decay tick + 64 to the second)

module injection_predictor
    import noc_pkg::*;
(
    input  logic clk,
    input  logic rst_n,

    // This router's coordinates - same as router_core's cx/cy
    input  logic [X_BITS-1:0] cx,
    input  logic [Y_BITS-1:0] cy,

    // Destination the PE currently wants to inject toward
    // Read every cycle regardless of inj_valid - purely combinational lookup
    input  logic [X_BITS-1:0] cand_dest_x,
    input  logic [Y_BITS-1:0] cand_dest_y,

    // Feedback from deflect_resolver (mech1 variant)
    // pe_took_fallback: valid only when last_inj_accepted = 1
    input  logic pe_took_fallback,

    // Was last cycle's gated_inj_valid actually 1?
    // (i.e. did we actually present a request to the resolver last cycle?)
    // Without this, a throttled PE could corrupt counters with feedback from a non-attempt
    input  logic last_inj_accepted,

    // Was the PE's request actually gated through to the resolver last cycle?
    // (== gated_inj_valid from the previous cycle, registered in router_core)
    input  logic was_gated,

    // Throttle hint output - combinational, drives the gate in router_core this cycle
    output logic throttle_hint
);

    // 2-bit saturating counters, one per direction
    // Index 0=N, 1=S, 2=E, 3=W (matches noc_pkg DIR_N/DIR_S/DIR_E/DIR_W)
    logic [1:0] counter [4];

    // Free-running 6-bit decay timer - wraps every 64 cycles
    logic [5:0] decay_timer;

    // Registered copies of last cycle's candidate direction and gated_inj_valid
    // (we need to know LAST cycle's direction when the feedback arrives THIS cycle)
    logic [1:0] cand_dir_last;

    // Combinational: compute desired direction for the CURRENT candidate flit
    logic [1:0] cand_dir;

    always_comb begin
        cand_dir     = compute_direction(cx, cy, cand_dest_x, cand_dest_y);
        // Throttle hint: top bit of the counter for the direction this candidate wants
        throttle_hint = counter[cand_dir][1];
    end

    // Sequential state machine
    always_ff @(posedge clk or negedge rst_n) begin
        int d;
        if (!rst_n) begin
            for (d = 0; d < 4; d++) 
                counter[d]  <= 2'b00;
            decay_timer    <= 6'd0;
            cand_dir_last  <= 2'd0;
        end else begin
            // Advance decay timer (free-running, always)
            decay_timer <= decay_timer + 6'd1;

            // Register this cycle's candidate direction for next cycle's feedback processing
            cand_dir_last <= cand_dir;

            // Update each direction's counter based on last cycle's outcome
            for (d = 0; d < 4; d++) begin
                if (was_gated && last_inj_accepted && (d == int'(cand_dir_last))) begin
                    // Real feedback: we presented a request last cycle and it was accepted
                    if (pe_took_fallback)
                        // Got a slot but not the desired direction - contention signal
                        counter[d] <= (counter[d] == 2'b11) ? 2'b11 : counter[d] + 2'b01;
                    else
                        // Got exactly the desired direction - things are fine, back off
                        counter[d] <= (counter[d] == 2'b00) ? 2'b00 : counter[d] - 2'b01;
                end else if (was_gated && !last_inj_accepted && (d == int'(cand_dir_last))) begin
                    // Fully rejected - no slot at all - strongest congestion signal
                    counter[d] <= (counter[d] == 2'b11) ? 2'b11 : counter[d] + 2'b01;
                end else if (decay_timer == 6'd63) begin
                    // No real feedback this direction this cycle - forced decay
                    // Guarantees counter[d] unthrottles in <= 127 idle cycles (and reaches 0 in <= 191 cycles)
                    counter[d] <= (counter[d] == 2'b00) ? 2'b00 : counter[d] - 2'b01;
                end
                // else: hold (no change needed - implicit in always_ff)
            end
        end
    end

endmodule
