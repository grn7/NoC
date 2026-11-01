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
`ifndef HOLD_K
  `define HOLD_K 0
`endif
`ifndef DECAY_PERIOD
  `define DECAY_PERIOD 64
`endif

package noc_pkg;
    parameter int TORUS_X   = `TORUS_X;
    parameter int TORUS_Y   = `TORUS_Y;
    parameter int DATA_WIDTH = 32;
    parameter int HOLD_K    = `HOLD_K;
    parameter int DECAY_PERIOD = `DECAY_PERIOD;
    localparam int X_BITS = $clog2(TORUS_X);
    localparam int Y_BITS = $clog2(TORUS_Y);

    typedef struct packed {
        logic [X_BITS-1:0] X_dest;
        logic [Y_BITS-1:0] Y_dest;
        logic [DATA_WIDTH-1:0] data;
    } flit_t;   

    // Direction index constants (shared between splitter and injection_predictor)
    // 0=N, 1=S, 2=E, 3=W  (no local here - this is for network output directions only)
    localparam int DIR_N = 0;
    localparam int DIR_S = 1;
    localparam int DIR_E = 2;
    localparam int DIR_W = 3;

    // compute_direction - given this router's coords (cx,cy) and a packet's destination (dest_x,dest_y),
    // returns which of the four network output directions XY-routing would choose for that packet.
    // This is the SAME fwd/bwd wraparound tie-break arithmetic that lives in splitter.sv,
    // pulled into one shared place so both splitter and injection_predictor agree.
    // Tie-break rules (must match splitter.sv exactly):
    //   X first: fwd_x <= bwd_x => East ; else West
    //   Y (X already matched): fwd_y <= bwd_y => North ; else South
    function automatic logic [1:0] compute_direction(
        input logic [X_BITS-1:0] cx,
        input logic [Y_BITS-1:0] cy,
        input logic [X_BITS-1:0] dest_x,
        input logic [Y_BITS-1:0] dest_y
    );
        logic [X_BITS:0] fwd_x, bwd_x;
        logic [Y_BITS:0] fwd_y, bwd_y;

        // X dimension
        if (dest_x >= cx)
            fwd_x = (X_BITS+1)'(dest_x) - (X_BITS+1)'(cx);
        else
            fwd_x = (X_BITS+1)'(dest_x) + (X_BITS+1)'(TORUS_X) - (X_BITS+1)'(cx);

        bwd_x = (X_BITS+1)'(TORUS_X) - fwd_x;

        // Y dimension
        if (dest_y >= cy)
            fwd_y = (Y_BITS+1)'(dest_y) - (Y_BITS+1)'(cy);
        else
            fwd_y = (Y_BITS+1)'(dest_y) + (Y_BITS+1)'(TORUS_Y) - (Y_BITS+1)'(cy);

        bwd_y = (Y_BITS+1)'(TORUS_Y) - fwd_y;

        // Routing decision (XY routing, same as splitter)
        if (dest_x != cx) begin
            // X dimension: East if fwd_x <= bwd_x (tie breaks East)
            if (fwd_x <= bwd_x)
                compute_direction = DIR_E;
            else
                compute_direction = DIR_W;
        end else if (dest_y != cy) begin
            // Y dimension: North if fwd_y <= bwd_y (tie breaks North)
            if (fwd_y <= bwd_y)
                compute_direction = DIR_N;
            else
                compute_direction = DIR_S;
        end else begin
            // Already at destination - caller (injection_predictor) treats this as "local"
            // but the predictor should never see dest==(cx,cy) since the testbench
            // never injects same-node flits. Return N as safe default.
            compute_direction = DIR_N;
        end
    endfunction

endpackage
