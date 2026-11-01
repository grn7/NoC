// Configuration parameters for Adaptive Cluster-Shared Buffer Pool NoC
// All parameters can be overridden at compile time via xvlog -d flags:
//   xvlog -sv -d TORUS_X=4 -d TORUS_Y=4 -d POOL_ENABLE=1 -d CLUSTER_X=2 -d CLUSTER_Y=2 ...

`ifndef TORUS_X
  `define TORUS_X 4
`endif
`ifndef TORUS_Y
  `define TORUS_Y 4
`endif
`ifndef POOL_ENABLE
  `define POOL_ENABLE 1
`endif
`ifndef CLUSTER_X
  `define CLUSTER_X 2
`endif
`ifndef CLUSTER_Y
  `define CLUSTER_Y 2
`endif
`ifndef SLOTS_PER_ROUTER
  `define SLOTS_PER_ROUTER 2
`endif
`ifndef WINDOW_CYCLES
  `define WINDOW_CYCLES 64
`endif
`ifndef ALPHA_Q
  `define ALPHA_Q 8'h10 // Q4.4 format: 16 = 1.0
`endif
`ifndef BETA_Q
  `define BETA_Q 8'h10  // Q4.4 format: 16 = 1.0
`endif
`ifndef HYST_THRESH
  `define HYST_THRESH 8'h04
`endif

package noc_pkg;

    parameter int TORUS_X             = `TORUS_X;
    parameter int TORUS_Y             = `TORUS_Y;
    parameter int DATA_WIDTH          = 32;

    parameter int POOL_ENABLE         = `POOL_ENABLE;
    parameter int CLUSTER_X           = `CLUSTER_X;
    parameter int CLUSTER_Y           = `CLUSTER_Y;
    parameter int SLOTS_PER_ROUTER    = `SLOTS_PER_ROUTER;
    parameter int WINDOW_CYCLES       = `WINDOW_CYCLES;
    parameter logic [7:0] ALPHA_Q     = `ALPHA_Q;
    parameter logic [7:0] BETA_Q      = `BETA_Q;
    parameter logic [7:0] HYST_THRESH = `HYST_THRESH;

    // Derived geometry and cluster parameters
    localparam int X_BITS = $clog2(TORUS_X);
    localparam int Y_BITS = $clog2(TORUS_Y);

    localparam int CLUSTERS_X          = TORUS_X / CLUSTER_X;
    localparam int CLUSTERS_Y          = TORUS_Y / CLUSTER_Y;
    localparam int ROUTERS_PER_CLUSTER = CLUSTER_X * CLUSTER_Y;
    localparam int POOL_SIZE           = SLOTS_PER_ROUTER * ROUTERS_PER_CLUSTER;
    localparam int RPC_BITS            = (ROUTERS_PER_CLUSTER > 1) ? $clog2(ROUTERS_PER_CLUSTER) : 1;
    localparam int OCC_BITS            = $clog2(POOL_SIZE + 1);

    // Direction indices (must match splitter and deflect_resolver)
    localparam int DIR_N    = 0;
    localparam int DIR_S    = 1;
    localparam int DIR_E    = 2;
    localparam int DIR_W    = 3;
    localparam int DIR_L    = 4; // Local (ejection output)
    localparam int DIR_POOL = 5; // Cluster pool write

    // Source port indices
    localparam int SRC_N    = 0;
    localparam int SRC_S    = 1;
    localparam int SRC_E    = 2;
    localparam int SRC_W    = 3;
    localparam int SRC_PE   = 4;
    localparam int SRC_POOL = 5; // Reinjection from cluster pool

    // Basic network flit
    typedef struct packed {
        logic [X_BITS-1:0]    X_dest;
        logic [Y_BITS-1:0]    Y_dest;
        logic [DATA_WIDTH-1:0] data;
    } flit_t;

    // Entry stored inside cluster pool storage
    typedef struct packed {
        flit_t               flit;
        logic [RPC_BITS-1:0] owner_idx;  // which router in cluster this flit belongs to
        logic [15:0]         seq_or_age; // timestamp/sequence for oldest-first ordering
        logic                valid;
    } pool_entry_t;

endpackage
