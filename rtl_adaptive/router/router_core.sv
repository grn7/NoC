// router_core.sv
// Router Core for Adaptive Cluster-Shared Buffer Pool NoC
//
// Extends baseline router_core with:
// 1. Connection to cluster pool via pool_if interface.
// 2. 6th splitter instance for pool-offer reinjection path.
// 3. Integration with 5-pass deflect_resolver.
// 4. Candidate flit and congestion event wiring to pool.

module router_core
    import noc_pkg::*;
#(
    parameter int POOL_ENABLE = noc_pkg::POOL_ENABLE,
    parameter int RPC_BITS    = noc_pkg::RPC_BITS
)
(
    input  logic                   clk,
    input  logic                   rst_n,

    // Coordinates in torus
    input  logic [X_BITS-1:0]      cx,
    input  logic [Y_BITS-1:0]      cy,

    // Index of this router within its cluster (0 .. ROUTERS_PER_CLUSTER-1)
    input  logic [RPC_BITS-1:0]    owner_idx,

    // Network links
    router_link_if.snk             north_in,
    router_link_if.src             north_out,
    router_link_if.snk             south_in,
    router_link_if.src             south_out,
    router_link_if.snk             east_in,
    router_link_if.src             east_out,
    router_link_if.snk             west_in,
    router_link_if.src             west_out,

    // Local PE interface
    pe_port_if.router              pe_port,

    // Cluster Pool interface
    pool_if.router                 pool_port
);

    localparam int NUM_SOURCES = (POOL_ENABLE) ? 6 : 5;

    // Direction indices
    localparam int DIR_N = 0;
    localparam int DIR_S = 1;
    localparam int DIR_E = 2;
    localparam int DIR_W = 3;
    localparam int DIR_L = 4;

    // Source port indices
    localparam int SRC_N    = 0;
    localparam int SRC_S    = 1;
    localparam int SRC_E    = 2;
    localparam int SRC_W    = 3;
    localparam int SRC_PE   = 4;
    localparam int SRC_POOL = 5;

    // -------------------------------------------------------------------------
    // Inputs to Splitters
    // -------------------------------------------------------------------------
    logic  valid_in [NUM_SOURCES];
    flit_t flit_in  [NUM_SOURCES];

    assign valid_in[SRC_N]  = north_in.valid;
    assign flit_in[SRC_N]   = north_in.flit;

    assign valid_in[SRC_S]  = south_in.valid;
    assign flit_in[SRC_S]   = south_in.flit;

    assign valid_in[SRC_E]  = east_in.valid;
    assign flit_in[SRC_E]   = east_in.flit;

    assign valid_in[SRC_W]  = west_in.valid;
    assign flit_in[SRC_W]   = west_in.flit;

    assign valid_in[SRC_PE] = pe_port.inj_valid;
    assign flit_in[SRC_PE]  = pe_port.inj_flit;

    generate
        if (POOL_ENABLE) begin : gen_pool_in
            assign valid_in[SRC_POOL] = pool_port.offer_valid;
            assign flit_in[SRC_POOL]  = pool_port.offer_flit;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Splitter Instances
    // -------------------------------------------------------------------------
    logic to_north [NUM_SOURCES];
    logic to_south [NUM_SOURCES];
    logic to_east  [NUM_SOURCES];
    logic to_west  [NUM_SOURCES];
    logic to_local [NUM_SOURCES];
    logic is_eject [NUM_SOURCES];
    logic [4:0] desired [NUM_SOURCES];

    generate
        genvar i;
        for (i = 0; i < NUM_SOURCES; i++) begin : gen_splitters
            splitter u_splitter (
                .cx         (cx),
                .cy         (cy),
                .valid_in   (valid_in[i]),
                .flit_in    (flit_in[i]),
                .to_north   (to_north[i]),
                .to_south   (to_south[i]),
                .to_east    (to_east[i]),
                .to_west    (to_west[i]),
                .to_local   (to_local[i]),
                .is_ejection(is_eject[i])
            );
            assign desired[i] = {to_local[i], to_west[i], to_east[i], to_south[i], to_north[i]};
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Deflect Resolver Instance
    // -------------------------------------------------------------------------
    logic  valid_out [5];
    flit_t flit_out  [5];
    logic  inj_ready_wire;

    deflect_resolver #(
        .POOL_ENABLE (POOL_ENABLE),
        .NUM_SOURCES (NUM_SOURCES)
    ) u_resolver (
        .valid_in             (valid_in),
        .flit_in              (flit_in),
        .desired              (desired),
        .is_eject             (is_eject),
        .valid_out            (valid_out),
        .flit_out             (flit_out),
        .inj_ready            (inj_ready_wire),
        .pool_req             (pool_port.req),
        .pool_cand_flit       (pool_port.cand_flit),
        .pool_grant           (pool_port.grant),
        .pool_offer_accepted     (pool_port.offer_accepted),
        .process_count           (pool_port.process_count),
        .potential_deflect_count (pool_port.potential_deflect_count),
        .actual_deflect_count    (),
        .deflect_count           (),
        .did_deflect             (),
        .did_process             (),
        .did_buf_request         (pool_port.did_buf_request),
        .did_buf_request_fail    (pool_port.did_buf_request_fail)
    );

    assign pool_port.cand_owner_idx = owner_idx;

    // -------------------------------------------------------------------------
    // Output Pipeline Registers (Stage 3)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            north_out.valid  <= 1'b0;
            north_out.flit   <= '0;
            south_out.valid  <= 1'b0;
            south_out.flit   <= '0;
            east_out.valid   <= 1'b0;
            east_out.flit    <= '0;
            west_out.valid   <= 1'b0;
            west_out.flit    <= '0;
            pe_port.ej_valid <= 1'b0;
            pe_port.ej_flit  <= '0;
        end else begin
            north_out.valid  <= valid_out[DIR_N];
            north_out.flit   <= flit_out[DIR_N];
            south_out.valid  <= valid_out[DIR_S];
            south_out.flit   <= flit_out[DIR_S];
            east_out.valid   <= valid_out[DIR_E];
            east_out.flit    <= flit_out[DIR_E];
            west_out.valid   <= valid_out[DIR_W];
            west_out.flit    <= flit_out[DIR_W];
            pe_port.ej_valid <= valid_out[DIR_L];
            pe_port.ej_flit  <= flit_out[DIR_L];
        end
    end

    // Combinational injection backpressure to PE
    assign pe_port.inj_ready = inj_ready_wire;

endmodule
