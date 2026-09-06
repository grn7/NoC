// config params 
package noc_pkg;
    parameter int TORUS_X = 4;
    parameter int TORUS_Y = 4;
    parameter int DATA_WIDTH = 32;
    localparam int X_BITS = $clog2(TORUS_X);
    localparam int Y_BITS = $clog2(TORUS_Y);

    typedef struct packed {
        logic [X_BITS-1:0] X_dest;
        logic [Y_BITS-1:0] Y_dest;
        logic [DATA_WIDTH-1:0] data;
    } flit_t;   
    
endpackage