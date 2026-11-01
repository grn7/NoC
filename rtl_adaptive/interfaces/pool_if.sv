// Router-to-Cluster-Pool Interface
// Connects one router_core to its cluster's shared pool (cluster_pool_top).
// Operates on the common system clock domain.

interface pool_if;

    import noc_pkg::*;

    // -------------------------------------------------------------------------
    // Write side (Router -> Pool Arbiter & Storage)
    // -------------------------------------------------------------------------
    // Router asserts req when a transit flit lost contention in Pass 2 and wants buffering.
    logic                req;
    flit_t               cand_flit;
    logic [RPC_BITS-1:0] cand_owner_idx;

    // Grant from pool_write_arbiter: combinational this cycle.
    // At most ONE router in the cluster receives grant = 1 per cycle.
    logic                grant;

    // -------------------------------------------------------------------------
    // Read/Offer side (Pool Storage -> Router)
    // -------------------------------------------------------------------------
    // Pool offers a previously buffered flit back to its owner router.
    logic                offer_valid;
    flit_t               offer_flit;

    // Feedback from router to pool: asserted if the offered flit was successfully
    // reinjected onto a network port (Pass 3) or ejected locally (Pass 1).
    // Clears the corresponding entry in pool_storage.
    logic                offer_accepted;

    // -------------------------------------------------------------------------
    // Congestion Event Pulses and Counts (Router -> Congestion Controller)
    // -------------------------------------------------------------------------
    logic [2:0]          process_count;           // number of active transit flits processed this cycle (0..4)
    logic [2:0]          potential_deflect_count; // number of transit flits losing Pass 2 (raw demand, 0..4)
    logic                did_buf_request;         // router requested buffer allocation (req == 1)
    logic                did_buf_request_fail;    // requested buffer but grant was denied (req & !grant)

    // -------------------------------------------------------------------------
    // Modports
    // -------------------------------------------------------------------------
    modport router (
        output req,
        output cand_flit,
        output cand_owner_idx,
        input  grant,

        input  offer_valid,
        input  offer_flit,
        output offer_accepted,

        output process_count,
        output potential_deflect_count,
        output did_buf_request,
        output did_buf_request_fail
    );

    modport pool (
        input  req,
        input  cand_flit,
        input  cand_owner_idx,
        output grant,

        output offer_valid,
        output offer_flit,
        input  offer_accepted,

        input  process_count,
        input  potential_deflect_count,
        input  did_buf_request,
        input  did_buf_request_fail
    );

endinterface
