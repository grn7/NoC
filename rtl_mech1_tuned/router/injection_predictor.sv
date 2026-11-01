// Injection Throttling Predictor - Mechanism 1 Tuned Variant
// Parameterized version of injection_predictor.sv with two tuning knobs:
//
//   HOLD_K      : bounded-hold counter K (item 1 from mech1_update.txt)
//                 When throttle_hint=1, the PE is still blocked for up to K consecutive
//                 cycles. On cycle K+1, a forced probe is let through regardless of the
//                 counter. This forced attempt produces real feedback, breaking the
//                 head-of-line stall without waiting for the 64-cycle decay tick.
//                 Set HOLD_K=0 to disable (never force — identical to original behaviour).
//                 Sweep values: {0, 1, 2, 4, 8}.
//
//   DECAY_PERIOD: the number of cycles between forced counter decrements (item 2).
//                 Original value was 64. A shorter period means faster recovery.
//                 Must be a power of two; implemented as a free-running counter
//                 that wraps when it reaches DECAY_PERIOD-1.
//                 Sweep values: {4, 8, 16, 32, 64}.
//
// All other logic (2-bit counters, was_gated suppression, counter update rules)
// is unchanged from the original injection_predictor.sv.

module injection_predictor
    import noc_pkg::*;
#(
    parameter int HOLD_K       = noc_pkg::HOLD_K,  // 0 = no forced probe (original behaviour)
    parameter int DECAY_PERIOD = noc_pkg::DECAY_PERIOD  // original value; must be power of two, 4..64
)
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
    input  logic last_inj_accepted,

    // Was the PE's request actually gated through to the resolver last cycle?
    input  logic was_gated,

    // Throttle hint output - combinational, drives the gate in router_core this cycle
    output logic throttle_hint
);
    // Compute DECAY_PERIOD counter width at elaboration time
    localparam int DECAY_BITS = $clog2(DECAY_PERIOD); // e.g. period=64 -> 6 bits

    // 2-bit saturating counters, one per direction
    // Index 0=N, 1=S, 2=E, 3=W (matches noc_pkg DIR_N/DIR_S/DIR_E/DIR_W)
    logic [1:0] counter [4];

    // Free-running decay timer - wraps every DECAY_PERIOD cycles
    logic [DECAY_BITS-1:0] decay_timer;

    // Registered copy of last cycle's candidate direction
    logic [1:0] cand_dir_last;

    // Bounded-hold counter: counts consecutive cycles where we've been throttling
    // the current head flit. When it reaches HOLD_K, we emit throttle_hint=0 (force probe).
    // 4 bits: supports K up to 15 (sweep goes to K=8, so 4 bits is sufficient).
    logic [3:0] hold_cnt;

    // Combinational: compute desired direction for the CURRENT candidate flit
    logic [1:0] cand_dir;

    // Localparams derived from parameters (XSIM-safe: no direct parameter slicing in expressions)
    localparam logic [3:0] HOLD_K_CNT = 4'(HOLD_K);      // 4-bit version of HOLD_K for hold_cnt compare
    localparam int         DECAY_WRAP = DECAY_PERIOD - 1; // decay fires when timer == this value

    always_comb begin
        cand_dir     = compute_direction(cx, cy, cand_dest_x, cand_dest_y);
        // Throttle hint: top bit of the counter for the direction this candidate wants.
        // If HOLD_K > 0 and we've been holding for >= HOLD_K consecutive cycles, force through (hint=0).
        if (HOLD_K > 0)
            throttle_hint = counter[cand_dir][1] & ~(hold_cnt >= HOLD_K_CNT);
        else
            throttle_hint = counter[cand_dir][1];
    end

    // Sequential state machine
    always_ff @(posedge clk or negedge rst_n) begin
        int d;
        if (!rst_n) begin
            for (d = 0; d < 4; d++)
                counter[d]  <= 2'b00;
            decay_timer    <= '0;
            cand_dir_last  <= 2'd0;
            hold_cnt       <= 4'd0;
        end else begin
            // Advance decay timer (free-running, always)
            decay_timer <= decay_timer + 1'b1;

            // Register this cycle's candidate direction for next cycle's feedback processing
            cand_dir_last <= cand_dir;

            // Bounded-hold counter update
            // We're throttling when the raw counter says throttle AND hold_cnt < HOLD_K (i.e. no force)
            if (HOLD_K > 0) begin
                if (counter[cand_dir][1] && hold_cnt < HOLD_K_CNT)
                    hold_cnt <= hold_cnt + 4'd1; // accumulate held cycles
                else
                    hold_cnt <= 4'd0;            // reset when not throttling or after forced probe
            end

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
                end else if (decay_timer == DECAY_BITS'(DECAY_WRAP)) begin
                    // Forced decay: fires every DECAY_PERIOD cycles
                    counter[d] <= (counter[d] == 2'b00) ? 2'b00 : counter[d] - 2'b01;
                end
                // else: hold (no change needed - implicit in always_ff)
            end
        end
    end

endmodule
