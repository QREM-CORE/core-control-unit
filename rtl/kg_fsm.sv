/*
 * Module Name: kg_fsm.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator - QREM Core
 *
 * Description
 * -----------
 * KeyGen macro-sequence FSM for QREM Core.
 *
 * Previously the CTRL_KG_* states were inlined inside core_control_unit.sv.
 * This module owns all KeyGen sequencing state including loop counters for
 * the s[], A[][col], row, and H(ek) hash passes.
 *
 * Interface contract with core_control_unit:
 *   - kg_start_i is a one-cycle pulse asserted from CTRL_START_DISPATCH
 *     in the parent; kg_fsm transitions IDLE → PRECHECK on that cycle.
 *   - kg_active_o is high from PRECHECK through HASH_EK_WAIT_DONE.
 *   - kg_done_o / kg_err_o are one-cycle pulses; on kg_err_o the parent
 *     reads kg_err_code_o and transitions to its own CTRL_ERROR state.
 *   - kg_abort_i is asserted by the parent when a subsystem error or
 *     zeroize preempts the currently running keygen; the FSM returns to
 *     KG_IDLE immediately.
 *   - All hsu_*, pau_*, and mem_phase_o outputs are only meaningful while
 *     kg_active_o is high; the parent muxes them onto the shared buses.
 */

import qrem_global_pkg::*;
import core_ctrl_pkg::*;

module kg_fsm (
    input  logic                        clk,
    input  logic                        rst,

    // ------------------------------------------------------------------
    // Handshake with core_control_unit
    // ------------------------------------------------------------------
    input  logic                        kg_start_i,        // one-cycle pulse — begin
    input  logic                        kg_abort_i,        // one-cycle pulse — abort

    // Precondition & configuration from CCU
    input  logic                        d_loaded_i,
    input  logic [1:0]                  cmd_sec_lvl_i,

    // ------------------------------------------------------------------
    // Sub-unit done strobes
    // ------------------------------------------------------------------
    input  logic                        hsu_done_i,
    input  logic                        hsu_packer_done_i,
    input  logic                        pau_done_i,

    // ------------------------------------------------------------------
    // Status back to core_control_unit
    // ------------------------------------------------------------------
    output logic                        kg_active_o,
    output logic                        kg_done_o,
    output logic                        kg_err_o,
    output logic [3:0]                  kg_err_code_o,

    // ------------------------------------------------------------------
    // HSU control (valid only while kg_active_o)
    // ------------------------------------------------------------------
    output logic                        hsu_start_o,
    output hs_mode_t                    hsu_mode_o,
    output logic [CTRL_XOF_LEN_W-1:0]  hsu_xof_len_o,
    output logic                        hsu_is_eta3_o,
    output logic [POLY_ID_WIDTH-1:0]    hsu_poly_id_o,
    output seed_id_e                    hsu_seed_id_o,
    output logic [1:0]                  hsu_input_sel_o,
    output logic                        hsu_absorb_poly_o,
    output logic                        hsu_absorb_last_o,
    output logic [7:0]                  hsu_row_o,
    output logic [7:0]                  hsu_col_o,
    output logic [7:0]                  hsu_cbd_n_o,
    output logic                        hsu_hash_ek_read_en_o,

    // ------------------------------------------------------------------
    // PAU control (valid only while kg_active_o)
    // ------------------------------------------------------------------
    output logic                        pau_start_o,
    output ctrl_pau_job_t               pau_job_o,

    // ------------------------------------------------------------------
    // Memory phase sideband (valid only while kg_active_o)
    // ------------------------------------------------------------------
    output ctrl_mem_phase_t             mem_phase_o
);

    // ------------------------------------------------------------------
    // Locked KeyGen phase mapping (controller-visible)
    // ------------------------------------------------------------------
    // This FSM is the controller-visible KeyGen macro-sequence. The state
    // names below are more granular than the locked contract, but they map
    // cleanly onto the required phases:
    //
    //   LOCKED KG_INIT:
    //     KG_PRECHECK, KG_DERIVE_ISSUE, KG_DERIVE_WAIT
    //
    //   LOCKED KG_GEN_S_PIPELINE (no overlap implemented yet; strictly sequential):
    //     KG_SAMPLE_S_*, KG_NTT_S_*, KG_NEXT_S
    //
    //   LOCKED KG_PREP_ROW_0 / KG_CWM_AND_OVERLAP (no overlap implemented yet):
    //     Per row: KG_SAMPLE_E_* -> KG_NTT_E_* -> KG_SAMPLE_A_* (k cols) -> KG_ROWMAC_*
    //
    //   LOCKED KG_HSU_HASH_EK (internal only):
    //     Absorb canonical ByteEncode12(t_hat[0..k-1]) then rho (Seed RAM), then wait done.
    //
    // Host egress (LOCKED KG_EGRESS) is intentionally not sequenced here; it is
    // a CMD_STORE responsibility in core_control_unit + host_if.

    // ------------------------------------------------------------------
    // State encoding
    // ------------------------------------------------------------------
    typedef enum logic [4:0] {
        KG_IDLE,
        KG_PRECHECK,

        KG_DERIVE_ISSUE,
        KG_DERIVE_WAIT,

        KG_SAMPLE_S_ISSUE,
        KG_SAMPLE_S_WAIT,
        KG_NTT_S_ISSUE,
        KG_NTT_S_WAIT,
        KG_NEXT_S,

        KG_SAMPLE_E_ISSUE,
        KG_SAMPLE_E_WAIT,
        KG_NTT_E_ISSUE,
        KG_NTT_E_WAIT,
        KG_SAMPLE_A_ISSUE,
        KG_SAMPLE_A_WAIT,
        KG_NEXT_A_COL,
        KG_ROWMAC_ISSUE,
        KG_ROWMAC_WAIT,
        KG_NEXT_ROW,

        KG_HASH_EK_START,
        KG_HASH_EK_ABSORB_T,
        KG_HASH_EK_WAIT_T,
        KG_HASH_EK_NEXT_T,
        KG_HASH_EK_ABSORB_RHO,
        KG_HASH_EK_WAIT_DONE,

        KG_DONE,
        KG_ERROR
    } kg_state_t;

    kg_state_t state_r, state_n;

    // Loop counters (owned here, not in the parent)
    logic [2:0] k_active_r;
    logic [2:0] s_idx_r;
    logic [2:0] row_idx_r;
    logic [2:0] col_idx_r;
    logic [2:0] hash_idx_r;

    // Next-state counter control signals
    logic init_keygen_n;
    logic inc_s_idx_n;
    logic inc_row_idx_n;
    logic inc_col_idx_n;
    logic inc_hash_idx_n;
    logic clear_col_idx_n;
    logic clear_hash_idx_n;

    logic [2:0] cmd_k_w;
    assign cmd_k_w = core_ctrl_pkg::ctrl_k_from_sec(cmd_sec_lvl_i);

    // k_active_r is derived once per accepted KeyGen operation (init_keygen_n).
    // cmd_sec_lvl_i is already latched in core_control_unit at command accept;
    // do not consult live CSRs mid-operation.

    // kg_active: everything between PRECHECK and the terminal states
    assign kg_active_o = (state_r != KG_IDLE)  &&
                         (state_r != KG_DONE)   &&
                         (state_r != KG_ERROR);

    // ------------------------------------------------------------------
    // Poly-ID helpers (identical to the ones removed from CCU)
    // ------------------------------------------------------------------
    function automatic logic [POLY_ID_WIDTH-1:0] s_poly_id (input logic [2:0] idx);
        s_poly_id = CTRL_POLY_S_BASE + POLY_ID_WIDTH'(idx);
    endfunction

    function automatic logic [POLY_ID_WIDTH-1:0] a_poly_id (input logic [2:0] idx);
        a_poly_id = CTRL_POLY_A_BASE + POLY_ID_WIDTH'(idx);
    endfunction

    function automatic logic [POLY_ID_WIDTH-1:0] t_poly_id (input logic [2:0] idx);
        t_poly_id = CTRL_POLY_T_BASE + POLY_ID_WIDTH'(idx);
    endfunction

    // ------------------------------------------------------------------
    // Combinational next-state logic
    // ------------------------------------------------------------------
    always_comb begin
        state_n = state_r;

        kg_done_o     = 1'b0;
        kg_err_o      = 1'b0;
        kg_err_code_o = CTRL_ERR_NONE;

        init_keygen_n    = 1'b0;
        inc_s_idx_n      = 1'b0;
        inc_row_idx_n    = 1'b0;
        inc_col_idx_n    = 1'b0;
        inc_hash_idx_n   = 1'b0;
        clear_col_idx_n  = 1'b0;
        clear_hash_idx_n = 1'b0;

        hsu_start_o           = 1'b0;
        hsu_mode_o            = MODE_HASH_SHA3_256;
        hsu_xof_len_o         = CTRL_XOF_LEN_UNUSED;
        hsu_is_eta3_o         = core_ctrl_pkg::ctrl_is_eta3(cmd_sec_lvl_i);
        hsu_poly_id_o         = '0;
        hsu_seed_id_o         = SEED_ID_TMP;
        hsu_input_sel_o       = HSU_IN_SEED;
        hsu_absorb_poly_o     = 1'b0;
        hsu_absorb_last_o     = 1'b0;
        hsu_row_o             = 8'h00;
        hsu_col_o             = 8'h00;
        hsu_cbd_n_o           = 8'h00;
        hsu_hash_ek_read_en_o = 1'b0;

        pau_start_o = 1'b0;
        pau_job_o   = '0;

        mem_phase_o = CTRL_MEM_PHASE_IDLE;

        // Abort is highest priority — snap back to IDLE
        if (kg_abort_i) begin
            state_n = KG_IDLE;
        end
        else begin
            unique case (state_r)

                KG_IDLE: begin
                    if (kg_start_i) begin
                        state_n = KG_PRECHECK;
                    end
                end

                KG_PRECHECK: begin
                    if (!d_loaded_i) begin
                        kg_err_o      = 1'b1;
                        kg_err_code_o = CTRL_ERR_PRECONDITION;
                        state_n       = KG_ERROR;
                    end
                    else begin
                        init_keygen_n = 1'b1;
                        state_n       = KG_DERIVE_ISSUE;
                    end
                end

                KG_DERIVE_ISSUE: begin
                    mem_phase_o     = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o     = 1'b1;
                    hsu_mode_o      = MODE_HASH_SHA3_512;
                    hsu_xof_len_o   = CTRL_XOF_LEN_64B;
                    hsu_seed_id_o   = SEED_ID_RHO;
                    hsu_input_sel_o = HSU_IN_SEED;
                    state_n         = KG_DERIVE_WAIT;
                end

                KG_DERIVE_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_HASH_SHA3_512;
                    hsu_seed_id_o = SEED_ID_RHO;
                    if (hsu_done_i) begin
                        state_n = KG_SAMPLE_S_ISSUE;
                    end
                end

                KG_SAMPLE_S_ISSUE: begin
                    mem_phase_o     = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o     = 1'b1;
                    hsu_mode_o      = MODE_SAMPLE_CBD;
                    hsu_xof_len_o   = CTRL_XOF_LEN_UNUSED;
                    hsu_seed_id_o   = SEED_ID_SIGMA;
                    hsu_poly_id_o   = s_poly_id(s_idx_r);
                    hsu_cbd_n_o     = {5'b0, s_idx_r};
                    hsu_input_sel_o = HSU_IN_SEED;
                    state_n         = KG_SAMPLE_S_WAIT;
                end

                KG_SAMPLE_S_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_CBD;
                    hsu_seed_id_o = SEED_ID_SIGMA;
                    hsu_poly_id_o = s_poly_id(s_idx_r);
                    hsu_cbd_n_o   = {5'b0, s_idx_r};
                    if (hsu_done_i) begin
                        state_n = KG_NTT_S_ISSUE;
                    end
                end

                KG_NTT_S_ISSUE: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_start_o               = 1'b1;
                    pau_job_o.opcode          = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id = s_poly_id(s_idx_r);
                    pau_job_o.k_active        = k_active_r;
                    state_n                   = KG_NTT_S_WAIT;
                end

                KG_NTT_S_WAIT: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_job_o.opcode          = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id = s_poly_id(s_idx_r);
                    pau_job_o.k_active        = k_active_r;
                    if (pau_done_i) begin
                        state_n = KG_NEXT_S;
                    end
                end

                KG_NEXT_S: begin
                    if (s_idx_r == (k_active_r - 3'd1)) begin
                        state_n = KG_SAMPLE_E_ISSUE;
                    end
                    else begin
                        inc_s_idx_n = 1'b1;
                        state_n     = KG_SAMPLE_S_ISSUE;
                    end
                end

                KG_SAMPLE_E_ISSUE: begin
                    mem_phase_o     = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o     = 1'b1;
                    hsu_mode_o      = MODE_SAMPLE_CBD;
                    hsu_seed_id_o   = SEED_ID_SIGMA;
                    hsu_poly_id_o   = CTRL_POLY_EI;
                    hsu_cbd_n_o     = {5'b0, (k_active_r + row_idx_r)};
                    hsu_input_sel_o = HSU_IN_SEED;
                    clear_col_idx_n = 1'b1;
                    state_n         = KG_SAMPLE_E_WAIT;
                end

                KG_SAMPLE_E_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_CBD;
                    hsu_seed_id_o = SEED_ID_SIGMA;
                    hsu_poly_id_o = CTRL_POLY_EI;
                    hsu_cbd_n_o   = {5'b0, (k_active_r + row_idx_r)};
                    if (hsu_done_i) begin
                        // Locked KeyGen requires e_i to be transformed into NTT
                        // form (e_hat_i) before row-mac. Keep this explicit so
                        // overwrite safety is scheduling-driven.
                        state_n = KG_NTT_E_ISSUE;
                    end
                end

                KG_NTT_E_ISSUE: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_start_o               = 1'b1;
                    pau_job_o.opcode          = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id = CTRL_POLY_EI;
                    pau_job_o.k_active        = k_active_r;
                    state_n                   = KG_NTT_E_WAIT;
                end

                KG_NTT_E_WAIT: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_job_o.opcode          = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id = CTRL_POLY_EI;
                    pau_job_o.k_active        = k_active_r;
                    if (pau_done_i) begin
                        state_n = KG_SAMPLE_A_ISSUE;
                    end
                end

                KG_SAMPLE_A_ISSUE: begin
                    mem_phase_o     = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o     = 1'b1;
                    hsu_mode_o      = MODE_SAMPLE_NTT;
                    hsu_seed_id_o   = SEED_ID_RHO;
                    hsu_poly_id_o   = a_poly_id(col_idx_r);
                    hsu_row_o       = {5'b0, row_idx_r};
                    hsu_col_o       = {5'b0, col_idx_r};
                    hsu_input_sel_o = HSU_IN_SEED;
                    state_n         = KG_SAMPLE_A_WAIT;
                end

                KG_SAMPLE_A_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_NTT;
                    hsu_seed_id_o = SEED_ID_RHO;
                    hsu_poly_id_o = a_poly_id(col_idx_r);
                    hsu_row_o     = {5'b0, row_idx_r};
                    hsu_col_o     = {5'b0, col_idx_r};
                    if (hsu_done_i) begin
                        state_n = KG_NEXT_A_COL;
                    end
                end

                KG_NEXT_A_COL: begin
                    if (col_idx_r == (k_active_r - 3'd1)) begin
                        state_n = KG_ROWMAC_ISSUE;
                    end
                    else begin
                        inc_col_idx_n = 1'b1;
                        state_n       = KG_SAMPLE_A_ISSUE;
                    end
                end

                KG_ROWMAC_ISSUE: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_start_o               = 1'b1;
                    pau_job_o.opcode          = PAU_JOB_KEYGEN_ROWMAC;
                    pau_job_o.primary_poly_id = t_poly_id(row_idx_r);
                    pau_job_o.aux_poly_id     = CTRL_POLY_EI;
                    pau_job_o.row_idx         = row_idx_r;
                    pau_job_o.k_active        = k_active_r;
                    // Blocked-by-external-repo: PAU adapter must implement a
                    // row-level MAC for KeyGen that:
                    //   - reads A_hat row buffer polys,
                    //   - multiplies by s_hat[0..k-1],
                    //   - accumulates internally,
                    //   - adds e_hat_i from CTRL_POLY_EI,
                    //   - writes the final t_hat_i to primary_poly_id.
                    state_n                   = KG_ROWMAC_WAIT;
                end

                KG_ROWMAC_WAIT: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_job_o.opcode          = PAU_JOB_KEYGEN_ROWMAC;
                    pau_job_o.primary_poly_id = t_poly_id(row_idx_r);
                    pau_job_o.aux_poly_id     = CTRL_POLY_EI;
                    pau_job_o.row_idx         = row_idx_r;
                    pau_job_o.k_active        = k_active_r;
                    if (pau_done_i) begin
                        state_n = KG_NEXT_ROW;
                    end
                end

                KG_NEXT_ROW: begin
                    if (row_idx_r == (k_active_r - 3'd1)) begin
                        clear_hash_idx_n = 1'b1;
                        state_n          = KG_HASH_EK_START;
                    end
                    else begin
                        inc_row_idx_n   = 1'b1;
                        clear_col_idx_n = 1'b1;
                        state_n         = KG_SAMPLE_E_ISSUE;
                    end
                end

                KG_HASH_EK_START: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_start_o           = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    hsu_xof_len_o         = CTRL_XOF_LEN_32B;
                    hsu_seed_id_o         = SEED_ID_HEK;
                    hsu_input_sel_o       = HSU_IN_POLY;
                    state_n               = KG_HASH_EK_ABSORB_T;
                end

                KG_HASH_EK_ABSORB_T: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    hsu_seed_id_o         = SEED_ID_HEK;
                    hsu_poly_id_o         = t_poly_id(hash_idx_r);
                    hsu_input_sel_o       = HSU_IN_POLY;
                    hsu_absorb_poly_o     = 1'b1;
                    // Locked KeyGen: absorb_last is reserved for the final rho
                    // segment, not for the last t_hat polynomial.
                    hsu_absorb_last_o     = 1'b0;
                    state_n               = KG_HASH_EK_WAIT_T;
                end

                KG_HASH_EK_WAIT_T: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    hsu_seed_id_o         = SEED_ID_HEK;
                    hsu_poly_id_o         = t_poly_id(hash_idx_r);
                    hsu_input_sel_o       = HSU_IN_POLY;
                    hsu_absorb_last_o     = 1'b0;
                    if (hsu_packer_done_i) begin
                        state_n = KG_HASH_EK_NEXT_T;
                    end
                end

                KG_HASH_EK_NEXT_T: begin
                    if (hash_idx_r == (k_active_r - 3'd1)) begin
                        state_n = KG_HASH_EK_ABSORB_RHO;
                    end
                    else begin
                        inc_hash_idx_n = 1'b1;
                        state_n        = KG_HASH_EK_ABSORB_T;
                    end
                end

                KG_HASH_EK_ABSORB_RHO: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    // Locked KeyGen: H(ek) = H( ByteEncode12(t_hat) || rho ).
                    // Controller intent: after all t_hat polys are absorbed,
                    // switch the HSU input to Seed RAM and absorb rho as the
                    // final segment.
                    //
                    // Blocked-by-external-repo: This assumes the HSU can absorb
                    // rho from Seed RAM based on {input_sel, seed_id} while in
                    // MODE_ABSORB_POLY, and that it still writes H(ek) to the
                    // correct Seed RAM slot internally.
                    hsu_seed_id_o     = SEED_ID_RHO;
                    hsu_input_sel_o   = HSU_IN_SEED;
                    hsu_absorb_poly_o = 1'b1;
                    hsu_absorb_last_o = 1'b1;
                    state_n           = KG_HASH_EK_WAIT_DONE;
                end

                KG_HASH_EK_WAIT_DONE: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    hsu_seed_id_o         = SEED_ID_RHO;
                    hsu_input_sel_o       = HSU_IN_SEED;
                    hsu_absorb_last_o     = 1'b1;
                    // Blocked-by-external-repo: HSU must assert hsu_done_o for
                    // this internal-only hash flow after squeezing H(ek) into
                    // Seed RAM.
                    if (hsu_done_i) begin
                        state_n = KG_DONE;
                    end
                end

                KG_DONE: begin
                    kg_done_o = 1'b1;
                    state_n   = KG_IDLE;
                end

                KG_ERROR: begin
                    // Remain in KG_ERROR for one cycle so parent can sample
                    // kg_err_o / kg_err_code_o, then clear.
                    state_n = KG_IDLE;
                end

                default: begin
                    state_n = KG_IDLE;
                end

            endcase
        end
    end

    // ------------------------------------------------------------------
    // Sequential state
    // ------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state_r    <= KG_IDLE;
            k_active_r <= 3'd0;
            s_idx_r    <= 3'd0;
            row_idx_r  <= 3'd0;
            col_idx_r  <= 3'd0;
            hash_idx_r <= 3'd0;
        end
        else begin
            state_r <= state_n;

            if (init_keygen_n) begin
                k_active_r <= cmd_k_w;
                s_idx_r    <= 3'd0;
                row_idx_r  <= 3'd0;
                col_idx_r  <= 3'd0;
                hash_idx_r <= 3'd0;
            end
            else begin
                if (inc_s_idx_n)    s_idx_r    <= s_idx_r    + 3'd1;
                if (inc_row_idx_n)  row_idx_r  <= row_idx_r  + 3'd1;
                if (inc_col_idx_n)  col_idx_r  <= col_idx_r  + 3'd1;
                if (inc_hash_idx_n) hash_idx_r <= hash_idx_r + 3'd1;

                if (clear_col_idx_n)  col_idx_r  <= 3'd0;
                if (clear_hash_idx_n) hash_idx_r <= 3'd0;
            end
        end
    end

endmodule
