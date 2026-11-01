// Runs every cycle ; fully combinational
// Every transit flit that enters must EXIT
// No flit is ever dropped

// 4-Pass Priority Algorithm (HopLite RT):
// Pass 1 - Ejection first : arrived flits claim Local unconditionally
// Pass 2 - Transit claim : N/S/E/W flits claim desired output in priority order
// Pass 3 - Deflection : losers from Pass 2 grab any remaining free slot
// Pass 4 - PE Injection : gets its desired slot only — NO fallback (L1 strict variant)
//          If desired slot is taken or no slot at all — inj_ready = 0, PE retries.
//
// L1 "strict" ablation: the fallback sub-pass from Pass 4 is deleted.
// This means the PE never uses a non-desired port. The question this answers:
// does avoiding all fallback injections improve makespan vs baseline?

module deflect_resolver

    import noc_pkg::*;

(
    // Inputs from 5 splitter instances (one per input port)
    // Index mapping: 0=N, 1=S, 2=E, 3=W, 4=PE
    input logic valid_in [5],
    input flit_t flit_in [5],

    // desired[i] is a 5-bit one hot vector - which output flit i wants
    // Index mapping: 0=N, 1=S, 2=E, 3=W, 4=L
    // Produced by router_core by packing splitter's individual outputs
    input logic[4:0] desired [5],

    // is_eject[i] = 1 means flit i has reached its destination
    // This is the same signal as to_local from the splitter , named separately here so the intent is explicit as this level
    input logic is_eject [5],

    // Output to 5 outputs , same index mapping as before
    output logic valid_out [5],
    output flit_t flit_out [5],

    // Backpressure to PE - valid/ready handshake at PE boundary
    // 1 - PE flit accepted this cycle; 0 - no free slot in network, PE must hold its flit and retry next cycle
    output logic inj_ready
);

    // Direction indices - must match 'desired' bit encoding above
    localparam int DIR_N = 0;
    localparam int DIR_S = 1;
    localparam int DIR_E = 2;
    localparam int DIR_W = 3;
    localparam int DIR_L = 4; // Local (ejection output)

    // Source port indices
    localparam int SRC_N = 0;
    localparam int SRC_S = 1;
    localparam int SRC_E = 2;
    localparam int SRC_W = 3;
    localparam int SRC_PE = 4;

    // Internal signals
    logic [4:0] out_taken; // out_taken[d] = 1 once the output direction has been claimed
    logic [3:0] deflected; // deflected[i] = 1 if transit flit i lost contention in Pass 2 and still needs to be assigned to some output in Pass 3

    always_comb begin

        // Initialise clean slate every combinational evaulation
        for(int d = 0; d<5; d++) begin
            valid_out[d] = 1'b0;
            flit_out[d] = '0;
        end
        out_taken = 5'b0;
        deflected = 4'b0;
        inj_ready = 1'b0;

        // Pass 1 - Ejection first
        // Any transit flit that has reached its destination (is_eject=1) unconditonally claims Local output - Highest priority
        // Ejection first breaks the possibility of looping forever
        for(int i=0; i<4; i++) begin // transit inputs only (not PE)
            if(valid_in[i] && is_eject[i]) begin
                if(!out_taken[DIR_L]) begin // first highest priority ejector claims local
                out_taken[DIR_L] = 1'b1;
                valid_out[DIR_L] = 1'b1;
                flit_out[DIR_L] = flit_in[i];
                end
                else begin
                    // Second or upcoming ejectors lost contention for Local; must be deflected
                    deflected[i] = 1'b1;
                end
            end
        end

        // Pass 2 - Transit flits claim desired output
        // Fixed HopliteRT priority - N(0) > S(1) > E(2) > W(3)
        // For each transit flit in priority order -
        // Find desired dest, if output is free claim it else mark flit as deflected
        for(int i=0; i<4 ; i++) begin
            if(valid_in[i] && !is_eject[i]) begin
                for(int d=0; d<4; d++) begin // Local is already resolved in Pass 1
                    if(desired[i][d]) begin // desired direction
                        if(!out_taken[d]) begin // if free - claim it
                            out_taken[d] = 1'b1;
                            valid_out[d] = 1'b1;
                            flit_out[d] = flit_in[i];
                        end
                        else begin // slot taken - needs deflection
                            deflected[i] = 1'b1;
                        end
                        break; // once the one hot one is either claimed or deflected , checking remaining directions is pointless
                    end
                end
            end
        end

        // Pass 3 - Deflect losers to any remaining free output
        // Every flit marked deflected[i] = 1 must be assigned somewhere, we can't drop in-flight packets
        // We scan output slots from lowest index (DIR_N=0) upward and assign to 1st free one
        // Deflected flits can go to any direction - even back to where they came from
        for (int i=0; i<4; i++) begin
            if(deflected[i]) begin
                for(int d=0; d<4; d++) begin // shouldn't go to local
                    if(!out_taken[d]) begin
                        out_taken[d] = 1'b1;
                        valid_out[d] = 1'b1;
                        flit_out[d] = flit_in[i];
                        break; // exactly one slot per deflected flit
                    end
                end
            end
        end

        // Pass 4: PE injection - Lowest priority - L1 STRICT: desired port only, no fallback
        // The PE's new flit is only placed if its XY-desired output port is free.
        // The fallback sub-pass (any remaining slot) is intentionally removed.
        // Effect: every injection that reaches the network travels its first hop productively.
        // Cost: PE stalls more often — inj_ready=0 whenever desired port is taken even if another slot is free.
        if(valid_in[SRC_PE]) begin
            for(int d=0; d<4; d++) begin // d<4 - network outputs only
                if(desired[SRC_PE][d] && !out_taken[d]) begin
                    out_taken[d] = 1'b1;
                    valid_out[d] = 1'b1;
                    flit_out[d] = flit_in[SRC_PE];
                    inj_ready   = 1'b1;
                    break;
                end
            end
            // No fallback sub-pass - inj_ready stays 0 if desired port was taken
        end
    end


endmodule
