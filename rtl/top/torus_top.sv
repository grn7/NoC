// Top level module for torus NoC


// Internal link naming convention - 
// link_h[x][y] - carries traffic going EAST from router (x,y)
// link_hp[x][y] - carries traffic going WEST from router (x,y)
// link_v[x][y] - carries traffic going NORTH from router (x,y)
// link_vp[x][y] - carries traffic going SOUTH from router (x,y)

// link_hp , link_vp are complements of link_h, link_v (same horizontal axis , opposite direction)


// For router at (x,y):
// east_out -> link_h[x][y] (this router drives it)
// east_in <- link_hp[(x+1)%TORUS_X][y] (east neighbor drives it west) ; dont need +TORUS_X since it will never be -ve
// west_out -> link_hp[x][y] (this router drives it west)
// west_in <- link_h[(x-1+TORUS_X)%TORUS_X][y] (west neighbor drives it east)
// north_out -> link_v[x][y]
// north_in <- link_vp[x][(y+1)%TORUS_Y] (north neighbor drives it south); dont need +TORUS_Y since it will never be -ve
// south_out -> link_vp[x][y] I(this router drives it south)
// south_in <- link_v[x][(y-1+TORUS_Y)%TORUS_Y] (south neighbor drives it)

// Wrap around is handle by modular indexing ; no special case wiring needed for edge routers

module torus_top 
    import noc_pkg::*;
    (
        input logic clk,
        input logic rst_n,

        // PE interface array - one per router 
        // Used for injection and ejection by testbench or logic 
        pe_port_if.router pe_ports[TORUS_X][TORUS_Y]
    );

    // Internal link arrays 
    // 16 east + 16 west + 16 north + 16 south = 64 router link interfaces needed 
    router_link_if link_h [TORUS_X][TORUS_Y] (); // East going links 
    router_link_if link_hp [TORUS_X][TORUS_Y] (); // West going links
    router_link_if link_v [TORUS_X][TORUS_Y] (); // North going links
    router_link_if link_vp [TORUS_X][TORUS_Y] (); // South going links

    // Router instantiation 
    generate
        genvar x,y;

        for(x=0; x<TORUS_X; x++) begin : gen_col
            for(y=0; y<TORUS_Y; y++) begin : gen_row

                router_core u_router (
                    .clk(clk),
                    .rst_n(rst_n),

                    // Coordinates - hardwired per instance
                    .cx(X_BITS'(x)),
                    .cy(Y_BITS'(y)),

                    // east_out - this router drives link_h[x][y] east
                    // its consumed by router (x+1,y)'s west_in
                    .east_out(link_h[x][y].src),

                    // west_in - fed by the link western neighbor drives east
                    // Western neighbor at [(x-1+TORUS_X)%TORUS_X][y]
                    // it drives link_h[(x-1+TORUS_X)%TORUS_X][y] 
                    .west_in(link_h[(x-1+TORUS_X)%TORUS_X][y].snk),

                    // north_out - this router drives link_v[x][y] north
                    // it is consumed by router (x,y+1)'s south_in
                    .north_out(link_v[x][y].src),

                    // south_in - fed by link southern neighbor drives north
                    // south neighbor is at [x][(y-1+TORUS_Y)%TORUS_Y]
                    // it drives link_v[x][(y-1+TORUS_Y)%TORUS_Y]
                    .south_in(link_v[x][(y-1+TORUS_Y)%TORUS_Y].snk),

                    // west_out - this router drives link_hp[x][y] west
                    // its consumed by router (x-1,y)'s east_in
                    .west_out(link_hp[x][y].src),

                    // east_in - fed by link eastern neighbor drives west
                    // east neighbor is at [(x+1)%TORUS_X][y]
                    // it drives link_hp[(x+1)%TORUS_X][y]
                    .east_in(link_hp[(x+1)%TORUS_X][y].snk),

                    // south_out - this router drives link_vp[x][y] south
                    // its consumed by router (x,y-1)'s north_in
                    .south_out(link_vp[x][y].src),

                    // north_in - fed by link northern neighbor drives south
                    // north neighbor is at [x][(y+1)%TORUS_Y]
                    // it drives link_vp[x][(y+1)%TORUS_Y]
                    .north_in(link_vp[x][(y+1)%TORUS_Y].snk),

                    // PE port
                    // exposes this router's PE to top-level port
                    .pe_port(pe_ports[x][y])
                );
            end
        end
    endgenerate

endmodule


