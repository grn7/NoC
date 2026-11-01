// torus_top.sv
// Top-Level Module for 2D Torus NoC with Adaptive Cluster-Shared Buffer Pools
//
// Features:
// 1. Instantiates TORUS_X x TORUS_Y router_core instances with toroidal wrap-around links.
// 2. Instantiates CLUSTERS_X x CLUSTERS_Y cluster_pool_top instances.
// 3. Systematically maps each router (x,y) to its parent cluster and owner_idx.
// 4. Fully parameterizable via noc_pkg parameters (CLUSTER_X, CLUSTER_Y, POOL_ENABLE, etc.).

module torus_top
    import noc_pkg::*;
(
    input  logic              clk,
    input  logic              rst_n,

    // PE interface array - one per router
    pe_port_if.router         pe_ports [TORUS_X][TORUS_Y]
);

    // -------------------------------------------------------------------------
    // Elaboration-Time Assertion Guards
    // -------------------------------------------------------------------------
    initial begin
        assert (TORUS_X % CLUSTER_X == 0)
            else $fatal(0, "CLUSTER_X (%0d) must divide TORUS_X (%0d) evenly", CLUSTER_X, TORUS_X);
        assert (TORUS_Y % CLUSTER_Y == 0)
            else $fatal(0, "CLUSTER_Y (%0d) must divide TORUS_Y (%0d) evenly", CLUSTER_Y, TORUS_Y);
    end

    // -------------------------------------------------------------------------
    // Network Links: Toroidal Mesh Interconnect
    // -------------------------------------------------------------------------
    router_link_if link_h  [TORUS_X][TORUS_Y] (); // Eastbound
    router_link_if link_hp [TORUS_X][TORUS_Y] (); // Westbound
    router_link_if link_v  [TORUS_X][TORUS_Y] (); // Northbound
    router_link_if link_vp [TORUS_X][TORUS_Y] (); // Southbound

    // -------------------------------------------------------------------------
    // Dedicated Pool Interfaces: One per Router Node
    // -------------------------------------------------------------------------
    pool_if pool_link [TORUS_X][TORUS_Y] ();

    // -------------------------------------------------------------------------
    // Cluster Pool Instances (One per cluster domain)
    // -------------------------------------------------------------------------
    generate
        genvar cx_idx, cy_idx;
        for (cx_idx = 0; cx_idx < CLUSTERS_X; cx_idx++) begin : gen_c_col
            for (cy_idx = 0; cy_idx < CLUSTERS_Y; cy_idx++) begin : gen_c_row

                // Static wiring between cluster_pool_top and member routers' pool_links
                logic [ROUTERS_PER_CLUSTER-1:0] req_vec;
                flit_t                          cand_flit_vec [ROUTERS_PER_CLUSTER];
                logic [ROUTERS_PER_CLUSTER-1:0] grant_vec;

                logic [ROUTERS_PER_CLUSTER-1:0] offer_valid_vec;
                flit_t                          offer_flit_vec [ROUTERS_PER_CLUSTER];
                logic [ROUTERS_PER_CLUSTER-1:0] offer_accepted_vec;

                logic [2:0]                     potential_deflect_count_vec [ROUTERS_PER_CLUSTER];
                logic [2:0]                     process_count_vec           [ROUTERS_PER_CLUSTER];
                logic [ROUTERS_PER_CLUSTER-1:0] did_buf_request_vec;
                logic [ROUTERS_PER_CLUSTER-1:0] did_buf_request_fail_vec;

                for (genvar rx = 0; rx < CLUSTER_X; rx++) begin : gen_rx
                    for (genvar ry = 0; ry < CLUSTER_Y; ry++) begin : gen_ry
                        localparam int GX   = cx_idx * CLUSTER_X + rx;
                        localparam int GY   = cy_idx * CLUSTER_Y + ry;
                        localparam int OIDX = rx * CLUSTER_Y + ry;

                        assign req_vec[OIDX]                  = pool_link[GX][GY].req;
                        assign cand_flit_vec[OIDX]            = pool_link[GX][GY].cand_flit;
                        assign pool_link[GX][GY].grant        = grant_vec[OIDX];

                        assign pool_link[GX][GY].offer_valid  = offer_valid_vec[OIDX];
                        assign pool_link[GX][GY].offer_flit   = offer_flit_vec[OIDX];
                        assign offer_accepted_vec[OIDX]       = pool_link[GX][GY].offer_accepted;

                        assign potential_deflect_count_vec[OIDX] = pool_link[GX][GY].potential_deflect_count;
                        assign process_count_vec[OIDX]           = pool_link[GX][GY].process_count;
                        assign did_buf_request_vec[OIDX]         = pool_link[GX][GY].did_buf_request;
                        assign did_buf_request_fail_vec[OIDX]    = pool_link[GX][GY].did_buf_request_fail;
                    end
                end

                logic [ROUTERS_PER_CLUSTER-1:0] port_bottleneck_loss;
                logic [ROUTERS_PER_CLUSTER-1:0] quota_exhausted_loss;

                cluster_pool_top #(
                    .POOL_ENABLE        (POOL_ENABLE),
                    .ROUTERS_PER_CLUSTER(ROUTERS_PER_CLUSTER),
                    .POOL_SIZE          (POOL_SIZE),
                    .RPC_BITS           (RPC_BITS),
                    .OCC_BITS           (OCC_BITS),
                    .WINDOW_CYCLES      (WINDOW_CYCLES),
                    .ALPHA_Q            (ALPHA_Q),
                    .BETA_Q             (BETA_Q),
                    .HYST_THRESH        (HYST_THRESH)
                ) u_cluster_pool (
                    .clk                     (clk),
                    .rst_n                   (rst_n),
                    .req                     (req_vec),
                    .cand_flit               (cand_flit_vec),
                    .grant                   (grant_vec),
                    .offer_valid             (offer_valid_vec),
                    .offer_flit              (offer_flit_vec),
                    .offer_accepted          (offer_accepted_vec),
                    .potential_deflect_count (potential_deflect_count_vec),
                    .process_count           (process_count_vec),
                    .did_buf_request         (did_buf_request_vec),
                    .did_buf_request_fail    (did_buf_request_fail_vec),
                    .port_bottleneck_loss    (port_bottleneck_loss),
                    .quota_exhausted_loss    (quota_exhausted_loss)
                );

            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Router Node Instantiations
    // -------------------------------------------------------------------------
    generate
        genvar x, y;
        for (x = 0; x < TORUS_X; x++) begin : gen_col
            for (y = 0; y < TORUS_Y; y++) begin : gen_row

                localparam int OWNER_IDX = (x % CLUSTER_X) * CLUSTER_Y + (y % CLUSTER_Y);

                router_core #(
                    .POOL_ENABLE(POOL_ENABLE),
                    .RPC_BITS   (RPC_BITS)
                ) u_router (
                    .clk      (clk),
                    .rst_n    (rst_n),
                    .cx       (X_BITS'(x)),
                    .cy       (Y_BITS'(y)),
                    .owner_idx(RPC_BITS'(OWNER_IDX)),

                    // East link
                    .east_out (link_h[x][y].src),
                    .east_in  (link_hp[(x + 1) % TORUS_X][y].snk),

                    // West link
                    .west_out (link_hp[x][y].src),
                    .west_in  (link_h[(x - 1 + TORUS_X) % TORUS_X][y].snk),

                    // North link
                    .north_out(link_v[x][y].src),
                    .north_in (link_vp[x][(y + 1) % TORUS_Y].snk),

                    // South link
                    .south_out(link_vp[x][y].src),
                    .south_in (link_v[x][(y - 1 + TORUS_Y) % TORUS_Y].snk),

                    // PE Port
                    .pe_port  (pe_ports[x][y]),

                    // Pool Port (connected to its node's dedicated pool link)
                    .pool_port(pool_link[x][y].router)
                );

            end
        end
    endgenerate

endmodule
