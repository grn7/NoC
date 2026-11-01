// Assembles the complete single-router datapath for one node in the torus
// Contains 5 splitters (one per input port); 1 deflect resolver and output pipeline registers
// Assumption : PE traffic generator never injects a flit with destination == (cx,cy)
// mech1 variant: injection_predictor instantiated and wired; gated_inj_valid gates Pass 4

module router_core
    import noc_pkg::*;
(
    input logic clk, 
    input logic rst_n,

    // Router's position in the torus
    // Passed in from torus_top
    input logic [X_BITS-1:0] cx,
    input logic [Y_BITS-1:0] cy,

    // Network links - one snk(incoming) and one src(outgoing) per direction
    // snk - router is consumer ; src - router is producer
    router_link_if.snk north_in,
    router_link_if.src north_out,
    router_link_if.snk south_in,
    router_link_if.src south_out,
    router_link_if.snk east_in,
    router_link_if.src east_out,
    router_link_if.snk west_in,
    router_link_if.src west_out,

    // Local PE connection - one interface , full-duplex
    pe_port_if.router pe_port
);

    // Direction index constants (must match deflect_resolver)
    localparam int DIR_N = 0;
    localparam int DIR_S = 1;
    localparam int DIR_E = 2;
    localparam int DIR_W = 3;
    localparam int DIR_L = 4;

    localparam int SRC_N = 0;
    localparam int SRC_S = 1;
    localparam int SRC_E = 2;
    localparam int SRC_W = 3;
    localparam int SRC_PE = 4;

    // Wires between splitters and deflect resolver

    // Inputs fed into each Splitter
    logic valid_in [5];
    flit_t flit_in [5];

    // Splitter outputs - individual one-hot direction bits per source ports
    logic to_north [5];
    logic to_south [5];
    logic to_east [5];
    logic to_west [5];
    logic to_local [5];
    logic is_eject [5];

    // Packed desired direction vector for each source (fed into deflect_resolver)
    logic [4:0] desired [5];

    // Deflect_resolver outputs (before register)
    logic valid_out [5];
    flit_t flit_out[5];
    logic inj_ready_wire;

    // mech1: new signal from deflect_resolver
    logic pe_took_fallback_wire;

    // mech1: predictor output and gating
    logic throttle_hint;
    logic gated_inj_valid;

    // mech1: registered feedback signals for predictor (last-cycle values)
    logic last_inj_accepted_r;
    logic was_gated_r;
    logic pe_took_fallback_r;

    // Stage 1A - Wire incoming links and PE injection to splitter inputs
    assign valid_in[SRC_N] = north_in.valid;
    assign flit_in[SRC_N] = north_in.flit;

    assign valid_in[SRC_S] = south_in.valid;
    assign flit_in[SRC_S] = south_in.flit;

    assign valid_in[SRC_E] = east_in.valid;
    assign flit_in[SRC_E] = east_in.flit;

    assign valid_in[SRC_W] = west_in.valid;
    assign flit_in[SRC_W] = west_in.flit;

    // mech1: gate the PE's injection request through the throttle hint
    // If throttle_hint is high, we do NOT present the flit to the resolver this cycle.
    // The PE sees inj_ready=0 (naturally, since Pass 4 never ran) and retries next cycle.
    assign gated_inj_valid = pe_port.inj_valid & ~throttle_hint;
    assign valid_in[SRC_PE] = gated_inj_valid;
    assign flit_in[SRC_PE] = pe_port.inj_flit;

    // Stage 1B - Instantiate the 5 splitters 
    generate
        genvar i;
        for(i=0; i<5; i++) begin:gen_splitters

            splitter u_splitter(
                .cx(cx),
                .cy(cy),
                .valid_in(valid_in[i]),
                .flit_in(flit_in[i]),
                .to_north(to_north[i]),
                .to_south(to_south[i]),
                .to_east(to_east[i]),
                .to_west(to_west[i]),
                .to_local(to_local[i]),
                .is_ejection(is_eject[i])
            );
        end

    endgenerate

    // Stage 1C - Pack splitter outputs into desired[] for deflect_resolver
    // A standard for loop can exist only inside procedural blocks like always_comb
    // and we are using assign which is not procedural stmt 
    // so use a generate block or simply write a always_comb and put for loop and assign inside it - its the same
    generate
        for(i=0; i<5; i++) begin : gen_desired
            assign desired[i] = {to_local[i], to_west[i], to_east[i], to_south[i], to_north[i]};
        end
    endgenerate

    // Stage 2 - Instantiate deflect_resolver (mech1 variant with pe_took_fallback)
    deflect_resolver u_resolver(
        .valid_in(valid_in),
        .flit_in(flit_in),
        .desired(desired),
        .is_eject(is_eject),
        .valid_out(valid_out),
        .flit_out(flit_out),
        .inj_ready(inj_ready_wire),
        .pe_took_fallback(pe_took_fallback_wire)
    );

    // mech1: Instantiate injection_predictor
    // Reads: cx, cy, destination from candidate flit, and last cycle's feedback
    // Produces: throttle_hint (combinational this cycle)
    injection_predictor u_predictor(
        .clk(clk),
        .rst_n(rst_n),
        .cx(cx),
        .cy(cy),
        .cand_dest_x(pe_port.inj_flit.X_dest),
        .cand_dest_y(pe_port.inj_flit.Y_dest),
        .pe_took_fallback(pe_took_fallback_r),
        .last_inj_accepted(last_inj_accepted_r),
        .was_gated(was_gated_r),
        .throttle_hint(throttle_hint)
    );

    // mech1: Register last cycle's feedback for the predictor
    // Needed because the predictor needs to know what happened LAST cycle
    // (this cycle's inj_ready_wire, gated_inj_valid, and pe_took_fallback_wire are for THIS cycle's attempt)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_inj_accepted_r <= 1'b0;
            was_gated_r         <= 1'b0;
            pe_took_fallback_r  <= 1'b0;
        end else begin
            last_inj_accepted_r <= inj_ready_wire;
            was_gated_r         <= gated_inj_valid;
            pe_took_fallback_r  <= pe_took_fallback_wire;
        end
    end

    // Stage 3 - Output pipeline registers 
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            north_out.valid <= 1'b0;
            north_out.flit <= '0;
            south_out.valid <= 1'b0;
            south_out.flit <= '0;
            east_out.valid <= 1'b0;
            east_out.flit <= '0;
            west_out.valid <= 1'b0;
            west_out.flit <= '0;
            pe_port.ej_valid <= 1'b0;
            pe_port.ej_flit <= '0;
        end
        else begin
            // Drive network from output links from deflect_resolver results
            north_out.valid <= valid_out[DIR_N];
            north_out.flit <= flit_out[DIR_N];
            south_out.valid <= valid_out[DIR_S];
            south_out.flit <= flit_out[DIR_S];
            east_out.valid <= valid_out[DIR_E];
            east_out.flit <= flit_out[DIR_E];
            west_out.valid <= valid_out[DIR_W];
            west_out.flit <= flit_out[DIR_W];
            pe_port.ej_valid <= valid_out[DIR_L];
            pe_port.ej_flit <= flit_out[DIR_L];
        end
    end

    // inj_ready is combinational - the PE must know this cycle whether its flit was accepted,
    // so it can hold or release next cycle.
    // Registering this would add 1 cycle of lag
    assign pe_port.inj_ready = inj_ready_wire;

endmodule
            
