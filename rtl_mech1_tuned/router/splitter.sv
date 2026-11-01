// Purely combinational routing logic for a single flit 
// Uses XY routing 
// One instance per input port per router(5 per router)
// mech1 variant: internal inline direction math replaced by noc_pkg::compute_direction()
// Routing behavior is byte-for-byte identical to baseline splitter.sv

module splitter

    import noc_pkg::* ;

    (
        input logic [X_BITS-1:0] cx, // This router's x coord
        input logic [Y_BITS-1:0] cy, 
        input logic valid_in, // Is a flit present 
        input flit_t flit_in,

        output logic to_north,
        output logic to_south,
        output logic to_east,
        output logic to_west,
        output logic to_local, // dest reached , eject to PE
        output logic is_ejection // same as to_local, named separately so that deflect resolver can read the intent

    );

    // Routing decision 
    // X first then Y , then local 
    // All outputs gated by valid_in ; no flit present = all outputs 0 

    always_comb begin 
        // set defaults 
        to_north   = 1'b0;
        to_south   = 1'b0;
        to_east    = 1'b0;
        to_west    = 1'b0;
        to_local   = 1'b0;
        is_ejection = 1'b0;

        if(valid_in) begin
            if(flit_in.X_dest != cx || flit_in.Y_dest != cy) begin
                // Not at destination yet - call shared function for direction
                unique case (compute_direction(cx, cy, flit_in.X_dest, flit_in.Y_dest))
                    DIR_N: to_north = 1'b1;
                    DIR_S: to_south = 1'b1;
                    DIR_E: to_east  = 1'b1;
                    DIR_W: to_west  = 1'b1;
                    default: ; // unreachable for valid non-local flits
                endcase
            end else begin // reached destination 
                to_local    = 1'b1;
                is_ejection = 1'b1;
            end
        end
    end

endmodule
