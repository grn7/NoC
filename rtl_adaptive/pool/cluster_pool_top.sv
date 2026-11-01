// cluster_pool_top.sv
// Top-Level Module for One Cluster's Adaptive Shared Buffer Pool
//
// Instantiates and wires together:
// 1. pool_write_arbiter: single write grant per cluster per cycle
// 2. pool_read_arbiter: single reinjection offer per cluster per cycle
// 3. pool_storage: register/LUT-based storage (Option A) with combinational oldest-entry selection
// 4. congestion_controller: dynamic quota adjustment based on deflection demand and failure rates
//
// When POOL_ENABLE = 0:
// Generate-if gates out all submodules entirely; grant and offer_valid tied to 0.

module cluster_pool_top
    import noc_pkg::*;
#(
    parameter int POOL_ENABLE         = noc_pkg::POOL_ENABLE,
    parameter int ROUTERS_PER_CLUSTER = noc_pkg::ROUTERS_PER_CLUSTER,
    parameter int POOL_SIZE           = noc_pkg::POOL_SIZE,
    parameter int RPC_BITS            = noc_pkg::RPC_BITS,
    parameter int OCC_BITS            = noc_pkg::OCC_BITS,
    parameter int WINDOW_CYCLES       = noc_pkg::WINDOW_CYCLES,
    parameter logic [7:0] ALPHA_Q     = noc_pkg::ALPHA_Q,
    parameter logic [7:0] BETA_Q      = noc_pkg::BETA_Q,
    parameter logic [7:0] HYST_THRESH = noc_pkg::HYST_THRESH
)
(
    input  logic                                clk,
    input  logic                                rst_n,

    // Write side signals from routers in this cluster
    input  logic [ROUTERS_PER_CLUSTER-1:0]      req,
    input  flit_t                               cand_flit [ROUTERS_PER_CLUSTER],
    output logic [ROUTERS_PER_CLUSTER-1:0]      grant,

    // Read/offer side signals to routers in this cluster
    output logic [ROUTERS_PER_CLUSTER-1:0]      offer_valid,
    output flit_t                               offer_flit [ROUTERS_PER_CLUSTER],
    input  logic [ROUTERS_PER_CLUSTER-1:0]      offer_accepted,

    // Congestion event counts and pulses from routers
    input  logic [2:0]                          process_count [ROUTERS_PER_CLUSTER],
    input  logic [2:0]                          potential_deflect_count [ROUTERS_PER_CLUSTER],
    input  logic [ROUTERS_PER_CLUSTER-1:0]      did_buf_request,
    input  logic [ROUTERS_PER_CLUSTER-1:0]      did_buf_request_fail,

    // Diagnostic monitoring outputs (answers review point 2)
    output logic [ROUTERS_PER_CLUSTER-1:0]      port_bottleneck_loss,
    output logic [ROUTERS_PER_CLUSTER-1:0]      quota_exhausted_loss
);

    generate
        if (POOL_ENABLE) begin : gen_pool_active

            // -----------------------------------------------------------------
            // Internal Interconnect Signals
            // -----------------------------------------------------------------
            logic [OCC_BITS-1:0] router_occupancy [ROUTERS_PER_CLUSTER];
            logic [OCC_BITS-1:0] total_occupancy;
            logic [OCC_BITS-1:0] target_allowance [ROUTERS_PER_CLUSTER];

            logic                grant_valid;
            logic [RPC_BITS-1:0] grant_idx;
            flit_t               granted_flit;

            assign granted_flit = cand_flit[grant_idx];

            logic                offer_sel_valid;
            logic [RPC_BITS-1:0] offer_owner_idx;
            logic                selected_offer_accepted;

            assign selected_offer_accepted = offer_accepted[offer_owner_idx];

            // -----------------------------------------------------------------
            // 1. Write Arbiter
            // -----------------------------------------------------------------
            pool_write_arbiter #(
                .ROUTERS_PER_CLUSTER(ROUTERS_PER_CLUSTER),
                .POOL_SIZE          (POOL_SIZE),
                .RPC_BITS           (RPC_BITS),
                .OCC_BITS           (OCC_BITS)
            ) u_write_arb (
                .clk                 (clk),
                .rst_n               (rst_n),
                .req                 (req),
                .router_occupancy    (router_occupancy),
                .total_occupancy     (total_occupancy),
                .target_allowance    (target_allowance),
                .grant               (grant),
                .grant_valid         (grant_valid),
                .grant_idx           (grant_idx),
                .port_bottleneck_loss(port_bottleneck_loss),
                .quota_exhausted_loss(quota_exhausted_loss)
            );

            // -----------------------------------------------------------------
            // 2. Read Arbiter
            // -----------------------------------------------------------------
            pool_read_arbiter #(
                .ROUTERS_PER_CLUSTER(ROUTERS_PER_CLUSTER),
                .RPC_BITS           (RPC_BITS),
                .OCC_BITS           (OCC_BITS)
            ) u_read_arb (
                .clk             (clk),
                .rst_n           (rst_n),
                .router_occupancy(router_occupancy),
                .offer_accepted  (selected_offer_accepted),
                .offer_sel_valid (offer_sel_valid),
                .offer_owner_idx (offer_owner_idx)
            );

            // -----------------------------------------------------------------
            // 3. Pool Storage
            // -----------------------------------------------------------------
            pool_storage #(
                .POOL_SIZE          (POOL_SIZE),
                .ROUTERS_PER_CLUSTER(ROUTERS_PER_CLUSTER),
                .RPC_BITS           (RPC_BITS),
                .OCC_BITS           (OCC_BITS)
            ) u_storage (
                .clk             (clk),
                .rst_n           (rst_n),
                .write_en        (grant_valid),
                .write_flit      (granted_flit),
                .write_owner_idx (grant_idx),
                .read_sel_valid  (offer_sel_valid),
                .read_owner_idx  (offer_owner_idx),
                .offer_accepted  (selected_offer_accepted),
                .offer_valid     (offer_valid),
                .offer_flit      (offer_flit),
                .router_occupancy(router_occupancy),
                .total_occupancy (total_occupancy)
            );

            // -----------------------------------------------------------------
            // 4. Congestion Controller
            // -----------------------------------------------------------------
            congestion_controller #(
                .ROUTERS_PER_CLUSTER(ROUTERS_PER_CLUSTER),
                .POOL_SIZE          (POOL_SIZE),
                .OCC_BITS           (OCC_BITS),
                .WINDOW_CYCLES      (WINDOW_CYCLES),
                .ALPHA_Q            (ALPHA_Q),
                .BETA_Q             (BETA_Q),
                .HYST_THRESH        (HYST_THRESH)
            ) u_controller (
                .clk                     (clk),
                .rst_n                   (rst_n),
                .process_count           (process_count),
                .potential_deflect_count (potential_deflect_count),
                .did_buf_request         (did_buf_request),
                .did_buf_request_fail    (did_buf_request_fail),
                .router_occupancy        (router_occupancy),
                .target_allowance        (target_allowance)
            );

        end else begin : gen_pool_disabled

            // Pool hardware not synthesized when POOL_ENABLE=0
            assign grant                = '0;
            assign offer_valid          = '0;
            assign port_bottleneck_loss = '0;
            assign quota_exhausted_loss = '0;
            for (genvar r = 0; r < ROUTERS_PER_CLUSTER; r++) begin : gen_tieoff
                assign offer_flit[r] = '0;
            end

        end
    endgenerate

endmodule
