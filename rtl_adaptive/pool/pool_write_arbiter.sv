// pool_write_arbiter.sv
// Cluster Pool Single-Write-Port Arbiter (Adaptive Cluster-Shared Buffer Pool)
//
// Rules (Idea 2 Plan 2 Part 3.7):
// 1. Exactly ONE write grant issued per cluster per cycle (single-write-port memory).
// 2. A router's request (req[i]) is eligible iff:
//      (a) router_occupancy[i] < target_allowance[i]
//      (b) total_occupancy < POOL_SIZE
// 3. Round-robin arbitration among eligible requesters.
// 4. rr_ptr advances only on cycles where a grant actually issued.
// 5. grant[i] is COMBINATIONAL — returned to router_core's deflect_resolver in the same cycle.

module pool_write_arbiter
    import noc_pkg::*;
#(
    parameter int ROUTERS_PER_CLUSTER = noc_pkg::ROUTERS_PER_CLUSTER,
    parameter int POOL_SIZE           = noc_pkg::POOL_SIZE,
    parameter int RPC_BITS            = noc_pkg::RPC_BITS,
    parameter int OCC_BITS            = noc_pkg::OCC_BITS
)
(
    input  logic                                clk,
    input  logic                                rst_n,

    // Requests from routers in the cluster
    input  logic [ROUTERS_PER_CLUSTER-1:0]      req,

    // Occupancy state from pool_storage
    input  logic [OCC_BITS-1:0]                 router_occupancy [ROUTERS_PER_CLUSTER],
    input  logic [OCC_BITS-1:0]                 total_occupancy,

    // Dynamic quota allowance from congestion_controller
    input  logic [OCC_BITS-1:0]                 target_allowance [ROUTERS_PER_CLUSTER],

    // Combinational grant back to each router (one-hot or all zero)
    output logic [ROUTERS_PER_CLUSTER-1:0]      grant,

    // Signals to pool_storage to perform write
    output logic                                grant_valid,
    output logic [RPC_BITS-1:0]                 grant_idx,

    // Diagnostic monitoring outputs (answers review point 2)
    output logic [ROUTERS_PER_CLUSTER-1:0]      port_bottleneck_loss,
    output logic [ROUTERS_PER_CLUSTER-1:0]      quota_exhausted_loss
);

    logic [RPC_BITS-1:0]            rr_ptr;
    logic [ROUTERS_PER_CLUSTER-1:0] eligible;

    // -------------------------------------------------------------------------
    // Eligibility filter
    // -------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < ROUTERS_PER_CLUSTER; i++) begin
            eligible[i] = req[i] &&
                          (router_occupancy[i] < target_allowance[i]) &&
                          (total_occupancy < POOL_SIZE);
        end
    end

    // -------------------------------------------------------------------------
    // Parallel Round-Robin Selection
    // -------------------------------------------------------------------------
    logic [ROUTERS_PER_CLUSTER-1:0] mask;
    logic [ROUTERS_PER_CLUSTER-1:0] req_above;
    logic [ROUTERS_PER_CLUSTER-1:0] grant_above;
    logic [ROUTERS_PER_CLUSTER-1:0] grant_below;

    always_comb begin
        grant       = '0;
        grant_valid = 1'b0;
        grant_idx   = '0;

        if (ROUTERS_PER_CLUSTER == 1) begin
            grant       = eligible;
            grant_valid = eligible[0];
            grant_idx   = '0;
        end else begin
            mask = '1 << rr_ptr;
            req_above = eligible & mask;

            grant_above = req_above & (-req_above);
            grant_below = eligible & (-eligible);

            grant = (grant_above != '0) ? grant_above : grant_below;
            grant_valid = |eligible;

            for (int i = 0; i < ROUTERS_PER_CLUSTER; i++) begin
                if (grant[i]) grant_idx = RPC_BITS'(i);
            end
        end

        // Diagnostic breakdown: why did a router lose arbitration?
        for (int i = 0; i < ROUTERS_PER_CLUSTER; i++) begin
            port_bottleneck_loss[i] = req[i] && !grant[i] && eligible[i];
            quota_exhausted_loss[i] = req[i] && !eligible[i];
        end
    end

    // -------------------------------------------------------------------------
    // State: Round-robin pointer update
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr <= '0;
        end else if (grant_valid) begin
            if (ROUTERS_PER_CLUSTER > 1)
                rr_ptr <= (grant_idx + 1'b1) % ROUTERS_PER_CLUSTER;
            else
                rr_ptr <= '0;
        end
    end

endmodule
