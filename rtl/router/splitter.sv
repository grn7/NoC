// Purely combinational routing logic for a single flit 
// Uses XY routing 
// One instance per input port per router(5 per router)

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

    // X dimension 
    // fwd_x is hops going East, bwd_x is hops going West

    // Why X_BITS + 1? 
    // Capacity: Coordinates range from 0-3 (fits in 2 bits), but the grid size 
    // (TORUS_X) is 4 (3'b100), which requires a 3rd bit to store without truncating to 0.
    // Safe Math: Casting to a wider bit-width before subtraction prevents binary 
    // underflow (wrap-around) when calculating reverse routes (e.g., Dest 1 - Curr 3).

    // Example of what happens without the extra bit:
    // 4x4 Torus. X_BITS = 2. TORUS_X = 4.
    // Let cx = 1, X_dest = 1.
    // 
    // The flit is already at its destination, so fwd_x = 0 hops.
    // Now we calculate the reverse path: bwd_x = TORUS_X - fwd_x
    // bwd_x = 4 - 0 = 4. 
    //
    // If bwd_x was only 2 bits wide, the number 4 (binary 100) would 
    // truncate and silently wrap around to 0 (binary 00).
    // The router would think both the forward AND backward paths are 0 hops,
    // destroying the actual distance metrics


    logic [X_BITS:0] fwd_x;
    logic [X_BITS:0] bwd_x;

    always_comb begin
        if(flit_in.X_dest >= cx) 
            fwd_x = (X_BITS+1)'(flit_in.X_dest) - (X_BITS+1)'(cx);
        else 
            fwd_x = (X_BITS+1)'(flit_in.X_dest) + (X_BITS+1)'(TORUS_X) - (X_BITS+1)'(cx);

        bwd_x = (X_BITS+1)'(TORUS_X) - fwd_x;
    end

    // Y dimension 
    // fwd_y is hops going North, bwd_y is hops going South
    logic [Y_BITS:0] fwd_y;
    logic [Y_BITS:0] bwd_y;

    always_comb begin
        if(flit_in.Y_dest >= cy) 
            fwd_y = (Y_BITS+1)'(flit_in.Y_dest) - (Y_BITS+1)'(cy);
        else 
            fwd_y = (Y_BITS+1)'(flit_in.Y_dest) + (Y_BITS+1)'(TORUS_Y) - (Y_BITS+1)'(cy);

        bwd_y = (Y_BITS+1)'(TORUS_Y) - fwd_y;
    end

    // Routing decision 
    // X first then Y , then local 
    // All outputs gated by valid_in ; no flit present = all outputs 0 

    always_comb begin 
        // set defaults 
        to_north = 1'b0;
        to_south = 1'b0;
        to_east = 1'b0;
        to_west = 1'b0;
        to_local = 1'b0;
        is_ejection = 1'b0;

        if(valid_in) begin
            if(flit_in.X_dest != cx) begin
            // X first 
            // Go east if shorter or equal distance 
            if(fwd_x <= bwd_x) 
                to_east = 1'b1;
            else 
                to_west = 1'b1;
            end

            else if (flit_in.Y_dest != cy) begin
                if(fwd_y <= bwd_y)
                    to_north = 1'b1;
                else 
                    to_south = 1'b1;
            end

            else begin // reached destination 
                to_local = 1'b1;
                is_ejection = 1'b1;
            end
        end
    end
            


endmodule