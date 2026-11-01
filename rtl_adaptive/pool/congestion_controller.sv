// congestion_controller.sv
// Adaptive Dynamic Quota Controller (Adaptive Cluster-Shared Buffer Pool)
//
// Rules (Idea 2 Plan 1 Part 3.9 & Plan 2 Part 3.10):
// 1. Windowed accumulation of per-router event pulses (D_i, F_i, U_i, B_i) over WINDOW_CYCLES.
// 2. Congestion score calculation:
//      C_i = ALPHA_Q * (D_i / F_i) + BETA_Q * (U_i / B_i)
//    using Q4.4 fixed-point arithmetic with division-by-zero guards.
// 3. Quota adjustment:
//      - Only if (C_max - C_min) > HYST_THRESH and max_idx != min_idx.
//      - Invariant: sum(target_allowance) == POOL_SIZE at all times.
//      - Invariant: target_allowance[donor] > router_occupancy[donor] (never force below occupancy).
//      - Invariant: target_allowance[donor] >= 1 (each router keeps at least 1 slot).

module congestion_controller
    import noc_pkg::*;
#(
    parameter int ROUTERS_PER_CLUSTER = noc_pkg::ROUTERS_PER_CLUSTER,
    parameter int POOL_SIZE           = noc_pkg::POOL_SIZE,
    parameter int OCC_BITS            = noc_pkg::OCC_BITS,
    parameter int WINDOW_CYCLES       = noc_pkg::WINDOW_CYCLES,
    parameter logic [7:0] ALPHA_Q     = noc_pkg::ALPHA_Q,
    parameter logic [7:0] BETA_Q      = noc_pkg::BETA_Q,
    parameter logic [7:0] HYST_THRESH = noc_pkg::HYST_THRESH
)
(
    input  logic                                clk,
    input  logic                                rst_n,

    // Event counts and pulses from each router in the cluster
    input  logic [2:0]                          process_count [ROUTERS_PER_CLUSTER],
    input  logic [2:0]                          potential_deflect_count [ROUTERS_PER_CLUSTER],
    input  logic [ROUTERS_PER_CLUSTER-1:0]      did_buf_request,
    input  logic [ROUTERS_PER_CLUSTER-1:0]      did_buf_request_fail,

    // Current occupancy feedback from pool_storage
    input  logic [OCC_BITS-1:0]                 router_occupancy [ROUTERS_PER_CLUSTER],

    // Output dynamic quotas to pool_write_arbiter
    output logic [OCC_BITS-1:0]                 target_allowance [ROUTERS_PER_CLUSTER]
);

    localparam int WIN_BITS = (WINDOW_CYCLES > 1) ? $clog2(WINDOW_CYCLES) : 1;
    localparam int CNT_BITS = $clog2(WINDOW_CYCLES * 4 + 1);

    // Window timer
    logic [WIN_BITS-1:0] win_timer;

    // Per-router windowed event accumulators (width derived from window size to prevent overflow)
    logic [CNT_BITS-1:0] d_cnt [ROUTERS_PER_CLUSTER];
    logic [CNT_BITS-1:0] f_cnt [ROUTERS_PER_CLUSTER];
    logic [CNT_BITS-1:0] u_cnt [ROUTERS_PER_CLUSTER];
    logic [CNT_BITS-1:0] b_cnt [ROUTERS_PER_CLUSTER];

    // Initial allowance calculation (equal distribution)
    function automatic logic [OCC_BITS-1:0] get_init_allowance(int r_idx);
        int base_share, rem;
        base_share = POOL_SIZE / ROUTERS_PER_CLUSTER;
        rem        = POOL_SIZE % ROUTERS_PER_CLUSTER;
        return OCC_BITS'(base_share + ((r_idx < rem) ? 1 : 0));
    endfunction

    // -------------------------------------------------------------------------
    // Main Sequential Process
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_timer <= '0;
            for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                d_cnt[r]            <= '0;
                f_cnt[r]            <= '0;
                u_cnt[r]            <= '0;
                b_cnt[r]            <= '0;
                target_allowance[r] <= get_init_allowance(r);
            end
        end else begin
            // Accumulate incoming counts and pulses every cycle
            for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                d_cnt[r] <= d_cnt[r] + CNT_BITS'(potential_deflect_count[r]);
                f_cnt[r] <= f_cnt[r] + CNT_BITS'(process_count[r]);
                if (did_buf_request_fail[r]) u_cnt[r] <= u_cnt[r] + 1'b1;
                if (did_buf_request[r])      b_cnt[r] <= b_cnt[r] + 1'b1;
            end

            // End-of-window evaluation
            if (win_timer == (WINDOW_CYCLES - 1)) begin
                int max_idx, min_idx;
                logic [15:0] defl_ratio, buf_fail_ratio;
                logic [15:0] score [ROUTERS_PER_CLUSTER];

                // Compute congestion scores (Q4.4 fixed-point)
                for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                    defl_ratio     = (f_cnt[r] > 0) ? logic'(16'((32'(d_cnt[r]) << 4) / 32'(f_cnt[r]))) : 16'd0;
                    buf_fail_ratio = (b_cnt[r] > 0) ? logic'(16'((32'(u_cnt[r]) << 4) / 32'(b_cnt[r]))) : 16'd0;
                    score[r]       = ((16'(ALPHA_Q) * defl_ratio) >> 4) + ((16'(BETA_Q) * buf_fail_ratio) >> 4);
                end

                // Find highest-congested recipient (unconstrained)
                max_idx = 0;
                for (int r = 1; r < ROUTERS_PER_CLUSTER; r++) begin
                    if (score[r] > score[max_idx]) max_idx = r;
                end

                // Find lowest-congested ELIGIBLE donor
                min_idx = -1;
                for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                    logic donor_eligible;
                    donor_eligible = (target_allowance[r] > router_occupancy[r]) && (target_allowance[r] > 1);
                    if (donor_eligible && (min_idx == -1 || score[r] < score[min_idx]))
                        min_idx = r;
                end

                // Quota adjustment with hysteresis and safety guards
                if ((min_idx != -1) && (max_idx != min_idx) &&
                    (score[max_idx] - score[min_idx] > 16'(HYST_THRESH))) begin
                    target_allowance[max_idx] <= target_allowance[max_idx] + 1'b1;
                    target_allowance[min_idx] <= target_allowance[min_idx] - 1'b1;
                end

                // Reset accumulators and timer
                win_timer <= '0;
                for (int r = 0; r < ROUTERS_PER_CLUSTER; r++) begin
                    d_cnt[r] <= '0;
                    f_cnt[r] <= '0;
                    u_cnt[r] <= '0;
                    b_cnt[r] <= '0;
                end
            end else begin
                win_timer <= win_timer + 1'b1;
            end
        end
    end

endmodule
