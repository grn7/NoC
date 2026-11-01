// config params
// TORUS_X and TORUS_Y can be overridden at compile time:
//   xvlog -sv -d TORUS_X=8 -d TORUS_Y=8 noc_pkg.sv ...
// If -d is not given, the ifndef block sets the default to 4.
`ifndef TORUS_X
  `define TORUS_X 4
`endif
`ifndef TORUS_Y
  `define TORUS_Y 4
`endif

package noc_pkg;
    parameter int TORUS_X   = `TORUS_X;
    parameter int TORUS_Y   = `TORUS_Y;
    parameter int DATA_WIDTH = 32;
    localparam int X_BITS = $clog2(TORUS_X);
    localparam int Y_BITS = $clog2(TORUS_Y);

    typedef struct packed {
        logic [X_BITS-1:0] X_dest;
        logic [Y_BITS-1:0] Y_dest;
        logic [DATA_WIDTH-1:0] data;
    } flit_t;   
    
endpackage