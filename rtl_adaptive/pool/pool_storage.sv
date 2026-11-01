// pool_storage.sv
// Cluster Pool Storage Memory (Adaptive Cluster-Shared Buffer Pool)
//
// Register/LUT-based storage implementation (Option A):
// - Infers as distributed RAM / flip-flops in FPGA synthesis, not dedicated Block RAM.
// - Single write port: writes candidate flit on write_en from pool_write_arbiter
// - Single read/offer port: finds oldest flit for the router selected by pool_read_arbiter
// - On offer_accepted: clears the entry that was offered
// - Combinational occupancy tracking per router and total

module pool_storage
    import noc_pkg::*;
#(
    parameter int POOL_SIZE           = noc_pkg::POOL_SIZE,
    parameter int ROUTERS_PER_CLUSTER = noc_pkg::ROUTERS_PER_CLUSTER,
    parameter int RPC_BITS            = noc_pkg::RPC_BITS,
    parameter int OCC_BITS            = noc_pkg::OCC_BITS
)
(
    input  logic                                clk,
    input  logic                                rst_n,

    // Write Port
    input  logic                                write_en,
    input  flit_t                               write_flit,
    input  logic [RPC_BITS-1:0]                 write_owner_idx,

    // Read/Offer Selection Port
    input  logic                                read_sel_valid,
    input  logic [RPC_BITS-1:0]                 read_owner_idx,

    // Reinjection Feedback
    input  logic                                offer_accepted,

    // Offer Outputs to Routers
    output logic [ROUTERS_PER_CLUSTER-1:0]      offer_valid,
    output flit_t                               offer_flit [ROUTERS_PER_CLUSTER],

    // Occupancy Feedback
    output logic [OCC_BITS-1:0]                 router_occupancy [ROUTERS_PER_CLUSTER],
    output logic [OCC_BITS-1:0]                 total_occupancy
);

    localparam int ENTRY_BITS = (POOL_SIZE > 1) ? $clog2(POOL_SIZE) : 1;

    // Storage array and global sequence counter for age tracking
    pool_entry_t entries [POOL_SIZE];
    logic [15:0] global_seq;

    // -------------------------------------------------------------------------
    // Combinational Occupancy Tracking (Parallel Adder Trees)
    // -------------------------------------------------------------------------
    logic [POOL_SIZE-1:0] slot_valid;
    logic [POOL_SIZE-1:0] slot_owned_by [ROUTERS_PER_CLUSTER];

    always_comb begin
        for (int i = 0; i < POOL_SIZE; i++) begin
            slot_valid[i] = entries[i].valid;
            for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                slot_owned_by[r][i] = entries[i].valid && (entries[i].owner_idx == RPC_BITS'(r));
            end
        end

        total_occupancy = OCC_BITS'($countones(slot_valid));
        for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
            router_occupancy[r] = OCC_BITS'($countones(slot_owned_by[r]));
        end
    end

    // -------------------------------------------------------------------------
    // Find Free Slot for Write (Binary Priority Tree)
    // -------------------------------------------------------------------------
    typedef struct {
        logic                  valid;
        logic [ENTRY_BITS-1:0] idx;
    } free_cand_t;

    function automatic free_cand_t pick_first_free(free_cand_t a, free_cand_t b);
        free_cand_t res;
        if (a.valid) begin
            res = a;
        end else begin
            res = b;
        end
        return res;
    endfunction

    localparam int TREE_DEPTH  = (POOL_SIZE > 1) ? $clog2(POOL_SIZE) : 1;
    localparam int TREE_LEAVES = (POOL_SIZE > 1) ? (1 << $clog2(POOL_SIZE)) : 1;

    logic                  free_slot_found;
    logic [ENTRY_BITS-1:0] free_slot_idx;

    always_comb begin
        free_cand_t free_tree [TREE_DEPTH+1][TREE_LEAVES];

        for (int i = 0; i < TREE_LEAVES; i++) begin
            if (i < POOL_SIZE) begin
                free_tree[0][i].valid = !entries[i].valid;
                free_tree[0][i].idx   = ENTRY_BITS'(i);
            end else begin
                free_tree[0][i].valid = 1'b0;
                free_tree[0][i].idx   = '0;
            end
        end

        if (POOL_SIZE == 1) begin
            free_slot_found = free_tree[0][0].valid;
            free_slot_idx   = free_tree[0][0].idx;
        end else begin
            for (int s = 0; s < TREE_DEPTH; s++) begin
                for (int j = 0; j < (TREE_LEAVES >> (s + 1)); j++) begin
                    free_tree[s+1][j] = pick_first_free(free_tree[s][2*j], free_tree[s][2*j+1]);
                end
            end
            free_slot_found = free_tree[TREE_DEPTH][0].valid;
            free_slot_idx   = free_tree[TREE_DEPTH][0].idx;
        end
    end

    // -------------------------------------------------------------------------
    // Find Oldest Stored Flit for read_owner_idx (Balanced Comparison Tree)
    // -------------------------------------------------------------------------
    typedef struct {
        logic                  valid;
        logic [15:0]           seq;
        logic [ENTRY_BITS-1:0] idx;
    } cand_t;

    function automatic cand_t pick_older(cand_t a, cand_t b);
        cand_t res;
        if (!a.valid) begin
            res = b;
        end else if (!b.valid) begin
            res = a;
        end else if (a.seq <= b.seq) begin
            res = a;
        end else begin
            res = b;
        end
        return res;
    endfunction

    logic                  oldest_found;
    logic [ENTRY_BITS-1:0] oldest_idx;
    logic [15:0]           oldest_seq;

    always_comb begin
        cand_t cand_tree [TREE_DEPTH+1][TREE_LEAVES];

        for (int i = 0; i < TREE_LEAVES; i++) begin
            if (i < POOL_SIZE) begin
                cand_tree[0][i].valid = entries[i].valid && (entries[i].owner_idx == read_owner_idx);
                cand_tree[0][i].seq   = entries[i].seq_or_age;
                cand_tree[0][i].idx   = ENTRY_BITS'(i);
            end else begin
                cand_tree[0][i].valid = 1'b0;
                cand_tree[0][i].seq   = 16'hFFFF;
                cand_tree[0][i].idx   = '0;
            end
        end

        if (POOL_SIZE == 1) begin
            oldest_found = cand_tree[0][0].valid;
            oldest_idx   = cand_tree[0][0].idx;
            oldest_seq   = cand_tree[0][0].seq;
        end else begin
            for (int s = 0; s < TREE_DEPTH; s++) begin
                for (int j = 0; j < (TREE_LEAVES >> (s + 1)); j++) begin
                    cand_tree[s+1][j] = pick_older(cand_tree[s][2*j], cand_tree[s][2*j+1]);
                end
            end
            oldest_found = cand_tree[TREE_DEPTH][0].valid;
            oldest_idx   = cand_tree[TREE_DEPTH][0].idx;
            oldest_seq   = cand_tree[TREE_DEPTH][0].seq;
        end
    end

    // -------------------------------------------------------------------------
    // Combinational Offer Output Generation
    // -------------------------------------------------------------------------
    always_comb begin
        offer_valid = '0;
        for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
            offer_flit[r] = '0;
        end

        if (read_sel_valid && oldest_found) begin
            int r_idx;
            r_idx = int'(read_owner_idx);
            if (r_idx < ROUTERS_PER_CLUSTER) begin
                offer_valid[r_idx] = 1'b1;
                offer_flit[r_idx]  = entries[oldest_idx].flit;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Synchronous Storage Array Updates
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < POOL_SIZE; i++) begin
                entries[i].valid      <= 1'b0;
                entries[i].owner_idx  <= '0;
                entries[i].seq_or_age <= '0;
                entries[i].flit       <= '0;
            end
            global_seq <= 16'd0;
        end else begin
            // Write port: insert newly buffered flit
            if (write_en && free_slot_found) begin
                entries[free_slot_idx].valid      <= 1'b1;
                entries[free_slot_idx].flit       <= write_flit;
                entries[free_slot_idx].owner_idx  <= write_owner_idx;
                entries[free_slot_idx].seq_or_age <= global_seq;
                global_seq                        <= global_seq + 1'b1;
            end

            // Read/offer port: clear successfully reinjected entry
            if (offer_accepted && oldest_found) begin
                entries[oldest_idx].valid <= 1'b0;
            end
        end
    end

endmodule
