// Runs every cycle; fully combinational
// Deflect Resolver for Adaptive Cluster-Shared Buffer Pool NoC
//
// 5-Pass Priority Algorithm:
// Pass 1 - Ejection first: arrived flits claim Local unconditionally (Transit > Pool Offer)
// Pass 2 - Transit claim: N/S/E/W transit flits claim desired output in priority order (N > S > E > W)
// Candidate selection (NEW): First deflected transit flit (N > S > E > W) requests pool buffering
// Pass 3 - Pool-Offer Reinjection (NEW): Offered flit claims desired port (Transit > Pool Offer > PE)
// Pass 4 - Deflection: Granted candidate is absorbed into pool; remaining losers take free network ports
// Pass 5 - PE Injection: Lowest priority, gets leftover slot or stalls (inj_ready = 0)
//
// When POOL_ENABLE = 0:
// Pass 3 and pool candidate logic are disabled; behavior is bit-identical to baseline HopliteRT.

`ifndef DEFLECT_RESOLVER_MODNAME
  `define DEFLECT_RESOLVER_MODNAME deflect_resolver
`endif

module `DEFLECT_RESOLVER_MODNAME
    import noc_pkg::*;
#(
    parameter int POOL_ENABLE = noc_pkg::POOL_ENABLE,
    parameter int NUM_SOURCES = (POOL_ENABLE) ? 6 : 5
)
(
    // Inputs from splitters:
    // 0=N, 1=S, 2=E, 3=W, 4=PE, 5=POOL_OFFER (when NUM_SOURCES=6)
    input  logic             valid_in [NUM_SOURCES],
    input  flit_t            flit_in  [NUM_SOURCES],
    input  logic [4:0]       desired  [NUM_SOURCES],
    input  logic             is_eject [NUM_SOURCES],

    // Outputs to network and Local ejection:
    // 0=N, 1=S, 2=E, 3=W, 4=L
    output logic             valid_out [5],
    output flit_t            flit_out  [5],

    // Backpressure to PE
    output logic             inj_ready,

    // Cluster Pool Write Request (Router -> Pool Arbiter)
    output logic             pool_req,
    output flit_t            pool_cand_flit,
    input  logic             pool_grant,

    // Cluster Pool Reinjection Feedback (Router -> Pool Storage)
    output logic             pool_offer_accepted,

    // Congestion Event Counts and Pulses (Router -> Congestion Controller)
    output logic [2:0]       process_count,
    output logic [2:0]       potential_deflect_count,
    output logic [2:0]       actual_deflect_count,
    output logic [2:0]       deflect_count,
    output logic             did_deflect,
    output logic             did_process,
    output logic             did_buf_request,
    output logic             did_buf_request_fail
);

    // Direction indices
    localparam int DIR_N = 0;
    localparam int DIR_S = 1;
    localparam int DIR_E = 2;
    localparam int DIR_W = 3;
    localparam int DIR_L = 4; // Local (ejection output)

    // Source port indices
    localparam int SRC_N    = 0;
    localparam int SRC_S    = 1;
    localparam int SRC_E    = 2;
    localparam int SRC_W    = 3;
    localparam int SRC_PE   = 4;
    localparam int SRC_POOL = 5;

    // Internal state signals per combinational evaluation
    logic [4:0] out_taken; // out_taken[d] = 1 once output direction claimed
    logic [3:0] deflected; // deflected[i] = 1 if transit flit i needs assignment
    int         cand_src;  // index of transit flit selected as candidate for pool
    int         num_defl_transit;
    int         free_net_ports;

    always_comb begin

        // ---------------------------------------------------------------------
        // Initialize clean slate
        // ---------------------------------------------------------------------
        for (int d = 0; d < 5; d++) begin
            valid_out[d] = 1'b0;
            flit_out[d]  = '0;
        end
        out_taken = 5'b0;
        deflected = 4'b0;
        inj_ready = 1'b0;

        pool_req            = 1'b0;
        pool_cand_flit      = '0;
        pool_offer_accepted = 1'b0;
        cand_src            = -1;
        num_defl_transit    = 0;
        free_net_ports      = 0;

        process_count           = 3'd0;
        potential_deflect_count = 3'd0;
        actual_deflect_count    = 3'd0;
        deflect_count           = 3'd0;
        did_deflect             = 1'b0;
        did_process             = 1'b0;
        did_buf_request         = 1'b0;
        did_buf_request_fail    = 1'b0;

        // Processed count: count active transit flits received this cycle (0..4)
        for (int i = 0; i < 4; i++) begin
            if (valid_in[i]) process_count = process_count + 3'd1;
        end
        did_process = (process_count > 0);

        // ---------------------------------------------------------------------
        // Pass 1 - Ejection first: arrived flits claim Local unconditionally
        // ---------------------------------------------------------------------
        // Transit inputs (0..3) have highest priority for Local
        for (int i = 0; i < 4; i++) begin
            if (valid_in[i] && is_eject[i]) begin
                if (!out_taken[DIR_L]) begin
                    out_taken[DIR_L] = 1'b1;
                    valid_out[DIR_L] = 1'b1;
                    flit_out[DIR_L]  = flit_in[i];
                end else begin
                    deflected[i] = 1'b1;
                end
            end
        end

        // Reinjected pool flit also checks if it has arrived at its destination
        if (POOL_ENABLE && (NUM_SOURCES >= 6)) begin
            if (valid_in[SRC_POOL] && is_eject[SRC_POOL]) begin
                if (!out_taken[DIR_L]) begin
                    out_taken[DIR_L]    = 1'b1;
                    valid_out[DIR_L]    = 1'b1;
                    flit_out[DIR_L]     = flit_in[SRC_POOL];
                    pool_offer_accepted = 1'b1;
                end
                // If Local already taken, offered flit stays in pool (not accepted)
            end
        end

        // ---------------------------------------------------------------------
        // Pass 2 - Transit flits claim desired output
        // Fixed HopliteRT priority: N(0) > S(1) > E(2) > W(3)
        // ---------------------------------------------------------------------
        for (int i = 0; i < 4; i++) begin
            if (valid_in[i] && !is_eject[i]) begin
                for (int d = 0; d < 4; d++) begin
                    if (desired[i][d]) begin
                        if (!out_taken[d]) begin
                            out_taken[d] = 1'b1;
                            valid_out[d] = 1'b1;
                            flit_out[d]  = flit_in[i];
                        end else begin
                            deflected[i] = 1'b1;
                        end
                        break;
                    end
                end
            end
        end

        // ---------------------------------------------------------------------
        // Potential Deflections (raw demand before buffer admission)
        // Captured immediately after Pass 2 across transit losers
        // ---------------------------------------------------------------------
        potential_deflect_count = 3'($countones(deflected));

        // ---------------------------------------------------------------------
        // Candidate Selection for Pool Buffering (POOL_ENABLE active)
        // Scan deflected[0..3] in fixed N > S > E > W order.
        // First match is submitted as this router's single pool-write candidate.
        // ---------------------------------------------------------------------
        if (POOL_ENABLE) begin
            for (int i = 0; i < 4; i++) begin
                if (deflected[i]) begin
                    pool_req       = 1'b1;
                    pool_cand_flit = flit_in[i];
                    cand_src       = i;
                    break;
                end
            end
            did_buf_request      = pool_req;
            did_buf_request_fail = pool_req && !pool_grant;
        end

        // ---------------------------------------------------------------------
        // Pass 3 - Pool-offer Reinjection (POOL_ENABLE active)
        // Priority: below fresh transit (Pass 2) but above PE injection (Pass 5)
        // Guard: Reinject ONLY if free network ports exceed deflected transit flits,
        // guaranteeing that every transit flit has an available network port (no drops).
        // ---------------------------------------------------------------------
        if (POOL_ENABLE && (NUM_SOURCES >= 6)) begin
            if (valid_in[SRC_POOL] && !is_eject[SRC_POOL] && !pool_offer_accepted) begin
                num_defl_transit = 0;
                for (int i = 0; i < 4; i++) begin
                    if (deflected[i]) num_defl_transit++;
                end

                // If candidate is absorbed into pool, it won't need a network port
                if (pool_grant && (cand_src >= 0)) begin
                    num_defl_transit--;
                end

                free_net_ports = 0;
                for (int d = 0; d < 4; d++) begin
                    if (!out_taken[d]) free_net_ports++;
                end

                if (free_net_ports > num_defl_transit) begin
                    for (int d = 0; d < 4; d++) begin
                        if (desired[SRC_POOL][d]) begin
                            if (!out_taken[d]) begin
                                out_taken[d]        = 1'b1;
                                valid_out[d]        = 1'b1;
                                flit_out[d]         = flit_in[SRC_POOL];
                                pool_offer_accepted = 1'b1;
                            end
                            // If desired output taken: flit stays in pool (offer_accepted stays 0)
                            break;
                        end
                    end
                end
            end
        end

        // ---------------------------------------------------------------------
        // Pass 4 - Deflection of losers to free network outputs
        // ---------------------------------------------------------------------
        // If pool write was granted, candidate flit is absorbed into pool buffer!
        // It does not claim a network output.
        if (POOL_ENABLE && pool_grant && (cand_src >= 0)) begin
            deflected[cand_src] = 1'b0;
        end

        // All remaining deflected flits must take any free network slot
        for (int i = 0; i < 4; i++) begin
            if (deflected[i]) begin
                for (int d = 0; d < 4; d++) begin
                    if (!out_taken[d]) begin
                        out_taken[d]         = 1'b1;
                        valid_out[d]         = 1'b1;
                        flit_out[d]          = flit_in[i];
                        actual_deflect_count = actual_deflect_count + 3'd1;
                        break;
                    end
                end
            end
        end
        deflect_count = actual_deflect_count;
        did_deflect   = (actual_deflect_count > 0);

        // ---------------------------------------------------------------------
        // Pass 5 - PE Injection: Lowest priority
        // ---------------------------------------------------------------------
        if (valid_in[SRC_PE]) begin
            // 1st - Try PE's desired direction
            for (int d = 0; d < 4; d++) begin
                if (desired[SRC_PE][d] && !out_taken[d]) begin
                    out_taken[d] = 1'b1;
                    valid_out[d] = 1'b1;
                    flit_out[d]  = flit_in[SRC_PE];
                    inj_ready    = 1'b1;
                    break;
                end
            end

            // 2nd - Fall back to any free slot
            if (!inj_ready) begin
                for (int d = 0; d < 4; d++) begin
                    if (!out_taken[d]) begin
                        out_taken[d] = 1'b1;
                        valid_out[d] = 1'b1;
                        flit_out[d]  = flit_in[SRC_PE];
                        inj_ready    = 1'b1;
                        break;
                    end
                end
            end
        end

    end

endmodule
