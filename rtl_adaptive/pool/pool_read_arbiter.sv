// pool_read_arbiter.sv
// Cluster Pool Single-Read-Port Arbiter (Adaptive Cluster-Shared Buffer Pool)
//
// Rules (Idea 2 Plan 2 Part 3.8):
// 1. Exactly ONE reinjection offer per cluster per cycle (single-read-port memory).
// 2. Independent round-robin pointer from write arbiter.
// 3. Eligible = any router in the cluster with occupancy > 0 in the pool.
// 4. Advances RR pointer across eligible routers to guarantee fair offer opportunity
//    and avoid HOL blocking if one router's desired output happens to be busy.

module pool_read_arbiter
    import noc_pkg::*;
#(
    parameter int ROUTERS_PER_CLUSTER = noc_pkg::ROUTERS_PER_CLUSTER,
    parameter int RPC_BITS            = noc_pkg::RPC_BITS,
    parameter int OCC_BITS            = noc_pkg::OCC_BITS
)
(
    input  logic                                clk,
    input  logic                                rst_n,

    // Current occupancy feedback from pool_storage
    input  logic [OCC_BITS-1:0]                 router_occupancy [ROUTERS_PER_CLUSTER],

    // Feedback: did the selected router accept the offered flit this cycle?
    input  logic                                offer_accepted,

    // Selection output to pool_storage
    output logic                                offer_sel_valid,
    output logic [RPC_BITS-1:0]                 offer_owner_idx
);

    logic [RPC_BITS-1:0]            rr_ptr;
    logic [ROUTERS_PER_CLUSTER-1:0] eligible;

    // -------------------------------------------------------------------------
    // Eligibility filter: any router with stored flits in pool
    // -------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < ROUTERS_PER_CLUSTER; i++) begin
            eligible[i] = (router_occupancy[i] > 0);
        end
    end

    // -------------------------------------------------------------------------
    // Parallel Round-Robin Selection
    // -------------------------------------------------------------------------
    logic [ROUTERS_PER_CLUSTER-1:0] mask;
    logic [ROUTERS_PER_CLUSTER-1:0] req_above;
    logic [ROUTERS_PER_CLUSTER-1:0] grant_above;
    logic [ROUTERS_PER_CLUSTER-1:0] grant_below;
    logic [ROUTERS_PER_CLUSTER-1:0] sel_grant;

    always_comb begin
        offer_sel_valid = 1'b0;
        offer_owner_idx = '0;
        sel_grant       = '0;

        if (ROUTERS_PER_CLUSTER == 1) begin
            offer_sel_valid = eligible[0];
            offer_owner_idx = '0;
        end else begin
            mask = '1 << rr_ptr;
            req_above = eligible & mask;

            grant_above = req_above & (-req_above);
            grant_below = eligible & (-eligible);

            sel_grant = (grant_above != '0) ? grant_above : grant_below;
            offer_sel_valid = |eligible;

            for (int i = 0; i < ROUTERS_PER_CLUSTER; i++) begin
                if (sel_grant[i]) offer_owner_idx = RPC_BITS'(i);
            end
        end
    end

    // -------------------------------------------------------------------------
    // State: Round-robin pointer update
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_ptr <= '0;
        end else if (offer_sel_valid) begin
            if (ROUTERS_PER_CLUSTER > 1)
                rr_ptr <= (offer_owner_idx + 1'b1) % ROUTERS_PER_CLUSTER;
            else
                rr_ptr <= '0;
        end
    end

endmodule
