/*
 * Module Name: core_control_unit.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator - QREM Core
 *
 * Description
 * -----------
 * Macro-sequencer for QREM Core.
 *
 * This block consumes the high-level command context already latched by
 * host_if and turns that context into ordered work for the formatting,
 * hash/sampling, arithmetic, and memory subsystems. It intentionally does not
 * own CSRs, terminate host streams, implement packing/math, or instantiate
 * datapath blocks.
 *
 * Current priority is a clean KeyGen-first sequence. Encaps and Decaps are
 * explicit skeleton paths so their future implementation plugs into the same
 * command, payload, error, and zeroize machinery.
 */

import qrem_global_pkg::*;
import core_ctrl_pkg::*;

module core_control_unit (
    input  logic                         clk,
    input  logic                         rst_n,

    // ---------------------------------------------------------------------
    // Latched high-level command input from host_if
    // ---------------------------------------------------------------------
    input  logic                         cmd_valid_i,
    output logic                         cmd_ready_o,
    input  logic [3:0]                   cmd_opcode_i,
    input  logic [3:0]                   cmd_mode_i,
    input  logic [1:0]                   cmd_sec_lvl_i,
    input  logic [4:0]                   cmd_payload_id_i,
    input  logic [15:0]                  cmd_xfer_len_i,

    // Priority abort/zeroize pulse from host_if. This bypasses the valid/ready
    // command handshake and may arrive while a macro-operation is active.
    input  logic                         cmd_zeroize_i,

    // ---------------------------------------------------------------------
    // Coarse controller status back to host_if
    // ---------------------------------------------------------------------
    output logic                         sts_busy_o,
    output logic                         sts_done_o,
    output logic [3:0]                   sts_err_code_o,

    // ---------------------------------------------------------------------
    // Transcoder control/status
    // ---------------------------------------------------------------------
    output logic                         tr_start_o,
    output tr_opcode_t                   tr_opcode_o,
    input  logic                         tr_done_i,
    input  logic [3:0]                   tr_err_i,

    // ---------------------------------------------------------------------
    // HSU control/status
    // ---------------------------------------------------------------------
    output logic                         hsu_start_o,
    output hs_mode_t                     hsu_mode_o,
    output logic [CTRL_XOF_LEN_W-1:0]    hsu_xof_len_o,
    output logic                         hsu_is_eta3_o,
    output logic [POLY_ID_WIDTH-1:0]     hsu_poly_id_o,
    output seed_id_e                     hsu_seed_id_o,
    output logic [1:0]                   hsu_input_sel_o,
    output logic                         hsu_absorb_poly_o,
    output logic                         hsu_absorb_last_o,
    input  logic                         hsu_done_i,
    input  logic                         hsu_packer_done_i,
    input  logic [3:0]                   hsu_err_i,

    // ---------------------------------------------------------------------
    // PAU control/status
    // ---------------------------------------------------------------------
    output logic                         pau_start_o,
    output ctrl_pau_job_t                pau_job_o,
    input  logic                         pau_done_i,
    input  logic [3:0]                   pau_err_i,

    // ---------------------------------------------------------------------
    // Memory control/status sideband
    // ---------------------------------------------------------------------
    // mem_zeroize_req_o maps to the memory subsystem wipe_i input.
    output logic                         mem_zeroize_req_o,
    input  logic                         mem_zeroize_done_i,
    input  logic                         mem_fault_i,
    input  logic [2:0]                   mem_fault_code_i,

    // High while the HSU/Gearbox path is authorized to read T0..T3 for H(ek).
    output logic                         hsu_hash_ek_read_en_o,

    // Coarse access phase. This is not an arbiter grant; it is a controller
    // intent/debug sideband for later top-level memory-adapter hookup.
    output ctrl_mem_phase_t              mem_phase_o
);

    // ---------------------------------------------------------------------
    // Controller state
    // ---------------------------------------------------------------------
    typedef enum logic [5:0] {
        CTRL_IDLE,
        CTRL_DECODE,

        CTRL_LOAD_ISSUE_TR,
        CTRL_LOAD_WAIT_TR,

        CTRL_STORE_ISSUE_TR,
        CTRL_STORE_WAIT_TR,
        CTRL_STORE_EK_ISSUE_RHO,
        CTRL_STORE_EK_WAIT_RHO,

        CTRL_START_DISPATCH,

        CTRL_KG_PRECHECK,
        CTRL_KG_DERIVE_ISSUE,
        CTRL_KG_DERIVE_WAIT,

        CTRL_KG_SAMPLE_S_ISSUE,
        CTRL_KG_SAMPLE_S_WAIT,
        CTRL_KG_NTT_S_ISSUE,
        CTRL_KG_NTT_S_WAIT,
        CTRL_KG_NEXT_S,

        CTRL_KG_SAMPLE_E_ISSUE,
        CTRL_KG_SAMPLE_E_WAIT,
        CTRL_KG_SAMPLE_A_ISSUE,
        CTRL_KG_SAMPLE_A_WAIT,
        CTRL_KG_NEXT_A_COL,
        CTRL_KG_ROWMAC_ISSUE,
        CTRL_KG_ROWMAC_WAIT,
        CTRL_KG_NEXT_ROW,

        CTRL_KG_HASH_EK_START,
        CTRL_KG_HASH_EK_ABSORB_T,
        CTRL_KG_HASH_EK_WAIT_T,
        CTRL_KG_HASH_EK_NEXT_T,
        CTRL_KG_HASH_EK_WAIT_DONE,
        CTRL_KG_DONE,

        CTRL_UNSUPPORTED,
        CTRL_ZEROIZE_ISSUE,
        CTRL_ZEROIZE_WAIT,
        CTRL_ERROR
    } ctrl_state_t;

    ctrl_state_t state_r, state_n;

    // Latched command context from host_if. Live CSR changes in host_if cannot
    // mutate these fields while a command is in flight.
    logic [3:0]  cmd_opcode_lat_r;
    logic [3:0]  cmd_mode_lat_r;
    logic [1:0]  cmd_sec_lvl_lat_r;
    logic [4:0]  cmd_payload_id_lat_r;
    logic [15:0] cmd_xfer_len_lat_r;

    // Current protocol progress. KeyGen is implemented first; the Encaps and
    // Decaps payload bits are still useful precondition hooks for later work.
    logic d_loaded_r;
    logic z_loaded_r;
    logic m_loaded_r;
    logic ek_loaded_r;
    logic dk_loaded_r;
    logic c_loaded_r;
    logic hek_loaded_r;

    logic ek_valid_r;
    logic dk_valid_r;
    logic hek_valid_r;
    logic ss_valid_r;

    // Loop counters for KeyGen. k_active_r is decoded from the active security
    // level and controls which subset of S/A/T slots participates.
    logic [2:0] k_active_r;
    logic [2:0] s_idx_r;
    logic [2:0] row_idx_r;
    logic [2:0] col_idx_r;
    logic [2:0] hash_idx_r;

    // Last issued transcoder opcode is latched so wait states remain stable
    // even if a new command arrives at host_if while this controller is busy.
    tr_opcode_t active_tr_opcode_r, active_tr_opcode_n;

    // One-cycle host-visible status pulses. host_if converts these to sticky
    // CSR bits; the controller deliberately does not hold them high.
    logic       done_pulse_n;
    logic [3:0] err_code_n;
    logic       done_pulse_r;
    logic [3:0] err_code_r;

    // Next-state side effects.
    logic set_d_loaded_n;
    logic set_m_loaded_n;
    logic set_z_loaded_n;
    logic set_ek_loaded_n;
    logic set_dk_loaded_n;
    logic set_c_loaded_n;
    logic set_hek_loaded_n;
    logic set_keygen_valid_n;
    logic clear_protocol_n;
    logic init_keygen_n;
    logic inc_s_idx_n;
    logic inc_row_idx_n;
    logic inc_col_idx_n;
    logic inc_hash_idx_n;
    logic clear_col_idx_n;
    logic clear_hash_idx_n;

    logic [2:0] cmd_k_w;
    assign cmd_k_w = core_ctrl_pkg::ctrl_k_from_sec(cmd_sec_lvl_lat_r);

    // ---------------------------------------------------------------------
    // Small decode helpers
    // ---------------------------------------------------------------------
    function automatic logic payload_loaded (
        input logic [4:0] payload_id
    );
        begin
            unique case (payload_id)
                PLD_D:        payload_loaded = d_loaded_r;
                PLD_Z:        payload_loaded = z_loaded_r;
                PLD_M:        payload_loaded = m_loaded_r;
                PLD_EK:       payload_loaded = ek_loaded_r;
                PLD_DK:       payload_loaded = dk_loaded_r;
                PLD_HEK:      payload_loaded = hek_loaded_r;
                PLD_C:        payload_loaded = c_loaded_r;
                PLD_SHARED_K: payload_loaded = ss_valid_r;
                default:      payload_loaded = 1'b0;
            endcase
        end
    endfunction

    function automatic logic payload_result_valid (
        input logic [4:0] payload_id
    );
        begin
            unique case (payload_id)
                PLD_EK:       payload_result_valid = ek_valid_r;
                PLD_DK:       payload_result_valid = dk_valid_r;
                PLD_HEK:      payload_result_valid = hek_valid_r;
                PLD_SHARED_K: payload_result_valid = ss_valid_r;
                default:      payload_result_valid = 1'b0;
            endcase
        end
    endfunction

    function automatic tr_opcode_t decode_load_tr_opcode (
        input logic [3:0] mode,
        input logic [4:0] payload_id
    );
        begin
            decode_load_tr_opcode = TR_OP_IDLE;

            unique case (mode)
                MODE_KEYGEN: begin
                    if (payload_id == PLD_D) begin
                        decode_load_tr_opcode = TR_OP_KG_INGEST_D;
                    end
                end

                MODE_ENCAPS: begin
                    if (payload_id == PLD_M) begin
                        decode_load_tr_opcode = TR_OP_EN_INGEST_M;
                    end
                    // TODO(Encaps): PLD_EK is a compound host payload. The
                    // transcoder currently exposes split EK ingest opcodes.
                end

                MODE_DECAPS: begin
                    unique case (payload_id)
                        PLD_DK: decode_load_tr_opcode = TR_OP_DC_INGEST_DK_PKE;
                        PLD_Z:  decode_load_tr_opcode = TR_OP_DC_INGEST_Z;
                        // TODO(Decaps): C and EK are compound host payloads
                        // that need split transcoder sequencing.
                        default: decode_load_tr_opcode = TR_OP_IDLE;
                    endcase
                end

                default: begin
                    decode_load_tr_opcode = TR_OP_IDLE;
                end
            endcase
        end
    endfunction

    function automatic tr_opcode_t decode_store_tr_opcode (
        input logic [3:0] mode,
        input logic [4:0] payload_id
    );
        begin
            decode_store_tr_opcode = TR_OP_IDLE;

            unique case (mode)
                MODE_KEYGEN: begin
                    unique case (payload_id)
                        PLD_DK:  decode_store_tr_opcode = TR_OP_KG_EXPORT_DK_PKE;
                        PLD_EK:  decode_store_tr_opcode = TR_OP_KG_EXPORT_EK_PKE_1;
                        PLD_HEK: decode_store_tr_opcode = TR_OP_KG_EXPORT_HEK;
                        default: decode_store_tr_opcode = TR_OP_IDLE;
                    endcase
                end

                MODE_ENCAPS: begin
                    unique case (payload_id)
                        PLD_C:        decode_store_tr_opcode = TR_OP_EN_EXPORT_CT_1;
                        PLD_SHARED_K: decode_store_tr_opcode = TR_OP_EN_EXPORT_K;
                        default:      decode_store_tr_opcode = TR_OP_IDLE;
                    endcase
                end

                MODE_DECAPS: begin
                    if (payload_id == PLD_SHARED_K) begin
                        decode_store_tr_opcode = TR_OP_DC_EXPORT_K;
                    end
                end

                default: begin
                    decode_store_tr_opcode = TR_OP_IDLE;
                end
            endcase
        end
    endfunction

    function automatic logic [POLY_ID_WIDTH-1:0] s_poly_id (
        input logic [2:0] idx
    );
        begin
            s_poly_id = CTRL_POLY_S_BASE + POLY_ID_WIDTH'(idx);
        end
    endfunction

    function automatic logic [POLY_ID_WIDTH-1:0] a_poly_id (
        input logic [2:0] idx
    );
        begin
            a_poly_id = CTRL_POLY_A_BASE + POLY_ID_WIDTH'(idx);
        end
    endfunction

    function automatic logic [POLY_ID_WIDTH-1:0] t_poly_id (
        input logic [2:0] idx
    );
        begin
            t_poly_id = CTRL_POLY_T_BASE + POLY_ID_WIDTH'(idx);
        end
    endfunction

    // ---------------------------------------------------------------------
    // Combinational control
    // ---------------------------------------------------------------------
    always_comb begin
        state_n            = state_r;
        active_tr_opcode_n = active_tr_opcode_r;

        done_pulse_n = 1'b0;
        err_code_n   = CTRL_ERR_NONE;

        set_d_loaded_n   = 1'b0;
        set_m_loaded_n   = 1'b0;
        set_z_loaded_n   = 1'b0;
        set_ek_loaded_n  = 1'b0;
        set_dk_loaded_n  = 1'b0;
        set_c_loaded_n   = 1'b0;
        set_hek_loaded_n = 1'b0;
        set_keygen_valid_n = 1'b0;
        clear_protocol_n = 1'b0;
        init_keygen_n    = 1'b0;
        inc_s_idx_n      = 1'b0;
        inc_row_idx_n    = 1'b0;
        inc_col_idx_n    = 1'b0;
        inc_hash_idx_n   = 1'b0;
        clear_col_idx_n  = 1'b0;
        clear_hash_idx_n = 1'b0;

        cmd_ready_o = (state_r == CTRL_IDLE);

        tr_start_o  = 1'b0;
        tr_opcode_o = active_tr_opcode_r;

        hsu_start_o            = 1'b0;
        hsu_mode_o             = MODE_HASH_SHA3_256;
        hsu_xof_len_o          = CTRL_XOF_LEN_UNUSED;
        hsu_is_eta3_o          = core_ctrl_pkg::ctrl_is_eta3(cmd_sec_lvl_lat_r);
        hsu_poly_id_o          = '0;
        hsu_seed_id_o          = SEED_ID_TMP;
        hsu_input_sel_o        = HSU_IN_SEED;
        hsu_absorb_poly_o      = 1'b0;
        hsu_absorb_last_o      = 1'b0;
        hsu_hash_ek_read_en_o  = 1'b0;

        pau_start_o = 1'b0;
        pau_job_o   = '0;

        mem_zeroize_req_o = 1'b0;
        mem_phase_o       = CTRL_MEM_PHASE_IDLE;

        // Block error inputs are sampled only while the controller owns work.
        // This prevents a sticky downstream fault from repeatedly raising a
        // fresh host_if error after the command has already been cleared.
        if ((state_r != CTRL_IDLE) && (state_r != CTRL_ERROR)) begin
            if (tr_err_i != 4'h0) begin
                state_n    = CTRL_ERROR;
                err_code_n = CTRL_ERR_TRANSCODER;
            end
            else if (hsu_err_i != 4'h0) begin
                state_n    = CTRL_ERROR;
                err_code_n = CTRL_ERR_HSU;
            end
            else if (pau_err_i != 4'h0) begin
                state_n    = CTRL_ERROR;
                err_code_n = CTRL_ERR_PAU;
            end
            else if (mem_fault_i || (mem_fault_code_i != 3'b000)) begin
                state_n    = CTRL_ERROR;
                err_code_n = CTRL_ERR_MEMORY;
            end
        end

        // ZEROIZE is a first-class priority path. It interrupts issue/wait
        // sequencing and delegates physical wipe to the memory subsystem.
        if ((err_code_n == CTRL_ERR_NONE) &&
            cmd_zeroize_i &&
            (state_r != CTRL_ZEROIZE_ISSUE) &&
            (state_r != CTRL_ZEROIZE_WAIT)) begin
            state_n = CTRL_ZEROIZE_ISSUE;
        end
        else if (err_code_n == CTRL_ERR_NONE) begin
            unique case (state_r)
                CTRL_IDLE: begin
                    if (cmd_valid_i) begin
                        state_n = CTRL_DECODE;
                    end
                end

                CTRL_DECODE: begin
                    if (!core_ctrl_pkg::ctrl_valid_sec(cmd_sec_lvl_lat_r)) begin
                        state_n    = CTRL_ERROR;
                        err_code_n = CTRL_ERR_ILLEGAL_CMD;
                    end
                    else begin
                        unique case (cmd_opcode_lat_r)
                            CMD_LOAD: begin
                                active_tr_opcode_n = decode_load_tr_opcode(
                                    cmd_mode_lat_r,
                                    cmd_payload_id_lat_r
                                );
                                if (active_tr_opcode_n == TR_OP_IDLE) begin
                                    state_n    = CTRL_ERROR;
                                    err_code_n = CTRL_ERR_UNSUPPORTED;
                                end
                                else begin
                                    state_n = CTRL_LOAD_ISSUE_TR;
                                end
                            end

                            CMD_STORE: begin
                                active_tr_opcode_n = decode_store_tr_opcode(
                                    cmd_mode_lat_r,
                                    cmd_payload_id_lat_r
                                );
                                if (active_tr_opcode_n == TR_OP_IDLE) begin
                                    state_n    = CTRL_ERROR;
                                    err_code_n = CTRL_ERR_UNSUPPORTED;
                                end
                                else if (!payload_result_valid(cmd_payload_id_lat_r)) begin
                                    state_n    = CTRL_ERROR;
                                    err_code_n = CTRL_ERR_PRECONDITION;
                                end
                                else begin
                                    state_n = CTRL_STORE_ISSUE_TR;
                                end
                            end

                            CMD_START: begin
                                state_n = CTRL_START_DISPATCH;
                            end

                            default: begin
                                state_n    = CTRL_ERROR;
                                err_code_n = CTRL_ERR_ILLEGAL_CMD;
                            end
                        endcase
                    end
                end

                CTRL_LOAD_ISSUE_TR: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_start_o  = 1'b1;
                    tr_opcode_o = active_tr_opcode_r;
                    state_n     = CTRL_LOAD_WAIT_TR;
                end

                CTRL_LOAD_WAIT_TR: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_opcode_o = active_tr_opcode_r;

                    if (tr_done_i) begin
                        unique case (cmd_payload_id_lat_r)
                            PLD_D:   set_d_loaded_n   = 1'b1;
                            PLD_M:   set_m_loaded_n   = 1'b1;
                            PLD_Z:   set_z_loaded_n   = 1'b1;
                            PLD_EK:  set_ek_loaded_n  = 1'b1;
                            PLD_DK:  set_dk_loaded_n  = 1'b1;
                            PLD_C:   set_c_loaded_n   = 1'b1;
                            PLD_HEK: set_hek_loaded_n = 1'b1;
                            default: begin end
                        endcase
                        done_pulse_n = 1'b1;
                        state_n      = CTRL_IDLE;
                    end
                end

                CTRL_STORE_ISSUE_TR: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_start_o  = 1'b1;
                    tr_opcode_o = active_tr_opcode_r;
                    state_n     = CTRL_STORE_WAIT_TR;
                end

                CTRL_STORE_WAIT_TR: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_opcode_o = active_tr_opcode_r;

                    if (tr_done_i) begin
                        if ((cmd_mode_lat_r == MODE_KEYGEN) &&
                            (cmd_payload_id_lat_r == PLD_EK) &&
                            (active_tr_opcode_r == TR_OP_KG_EXPORT_EK_PKE_1)) begin
                            // Current transcoder exposes EK as t_hat and rho
                            // sub-ops. This keeps the controller architecture
                            // correct, but a future transcoder opcode should
                            // suppress the intermediate TLAST for a single
                            // host PLD_EK store transaction.
                            active_tr_opcode_n = TR_OP_KG_EXPORT_EK_PKE_2;
                            state_n            = CTRL_STORE_EK_ISSUE_RHO;
                        end
                        else begin
                            done_pulse_n = 1'b1;
                            state_n      = CTRL_IDLE;
                        end
                    end
                end

                CTRL_STORE_EK_ISSUE_RHO: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_start_o  = 1'b1;
                    tr_opcode_o = active_tr_opcode_r;
                    state_n     = CTRL_STORE_EK_WAIT_RHO;
                end

                CTRL_STORE_EK_WAIT_RHO: begin
                    mem_phase_o = CTRL_MEM_PHASE_TRANSCODER;
                    tr_opcode_o = active_tr_opcode_r;

                    if (tr_done_i) begin
                        done_pulse_n = 1'b1;
                        state_n      = CTRL_IDLE;
                    end
                end

                CTRL_START_DISPATCH: begin
                    unique case (cmd_mode_lat_r)
                        MODE_KEYGEN: state_n = CTRL_KG_PRECHECK;
                        MODE_ENCAPS,
                        MODE_DECAPS: state_n = CTRL_UNSUPPORTED;
                        default: begin
                            state_n    = CTRL_ERROR;
                            err_code_n = CTRL_ERR_ILLEGAL_CMD;
                        end
                    endcase
                end

                CTRL_KG_PRECHECK: begin
                    if (!d_loaded_r) begin
                        state_n    = CTRL_ERROR;
                        err_code_n = CTRL_ERR_PRECONDITION;
                    end
                    else begin
                        init_keygen_n = 1'b1;
                        state_n       = CTRL_KG_DERIVE_ISSUE;
                    end
                end

                CTRL_KG_DERIVE_ISSUE: begin
                    // G(d) / seed expansion hook. The HSU has SHA3-512 bypass,
                    // but the seed-store split into rho/sigma still needs a
                    // small top-level bridge. Until then, this state marks the
                    // intended operation and uses RHO as the primary target.
                    mem_phase_o      = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o      = 1'b1;
                    hsu_mode_o       = MODE_HASH_SHA3_512;
                    hsu_xof_len_o    = CTRL_XOF_LEN_64B;
                    hsu_seed_id_o    = SEED_ID_RHO;
                    hsu_input_sel_o  = HSU_IN_SEED;
                    state_n          = CTRL_KG_DERIVE_WAIT;
                end

                CTRL_KG_DERIVE_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_HASH_SHA3_512;
                    hsu_seed_id_o = SEED_ID_RHO;

                    if (hsu_done_i) begin
                        state_n = CTRL_KG_SAMPLE_S_ISSUE;
                    end
                end

                CTRL_KG_SAMPLE_S_ISSUE: begin
                    mem_phase_o      = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o      = 1'b1;
                    hsu_mode_o       = MODE_SAMPLE_CBD;
                    hsu_xof_len_o    = CTRL_XOF_LEN_UNUSED;
                    hsu_seed_id_o    = SEED_ID_SIGMA;
                    hsu_poly_id_o    = s_poly_id(s_idx_r);
                    hsu_input_sel_o  = HSU_IN_SEED;
                    state_n          = CTRL_KG_SAMPLE_S_WAIT;
                end

                CTRL_KG_SAMPLE_S_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_CBD;
                    hsu_seed_id_o = SEED_ID_SIGMA;
                    hsu_poly_id_o = s_poly_id(s_idx_r);

                    if (hsu_done_i) begin
                        state_n = CTRL_KG_NTT_S_ISSUE;
                    end
                end

                CTRL_KG_NTT_S_ISSUE: begin
                    mem_phase_o                   = CTRL_MEM_PHASE_PAU;
                    pau_start_o                   = 1'b1;
                    pau_job_o.opcode              = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id     = s_poly_id(s_idx_r);
                    pau_job_o.k_active            = k_active_r;
                    state_n                       = CTRL_KG_NTT_S_WAIT;
                end

                CTRL_KG_NTT_S_WAIT: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_job_o.opcode          = PAU_JOB_NTT_IN_PLACE;
                    pau_job_o.primary_poly_id = s_poly_id(s_idx_r);
                    pau_job_o.k_active        = k_active_r;

                    if (pau_done_i) begin
                        state_n = CTRL_KG_NEXT_S;
                    end
                end

                CTRL_KG_NEXT_S: begin
                    if (s_idx_r == (k_active_r - 3'd1)) begin
                        state_n = CTRL_KG_SAMPLE_E_ISSUE;
                    end
                    else begin
                        inc_s_idx_n = 1'b1;
                        state_n     = CTRL_KG_SAMPLE_S_ISSUE;
                    end
                end

                CTRL_KG_SAMPLE_E_ISSUE: begin
                    mem_phase_o      = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o      = 1'b1;
                    hsu_mode_o       = MODE_SAMPLE_CBD;
                    hsu_seed_id_o    = SEED_ID_SIGMA;
                    hsu_poly_id_o    = CTRL_POLY_EI;
                    hsu_input_sel_o  = HSU_IN_SEED;
                    clear_col_idx_n  = 1'b1;
                    state_n          = CTRL_KG_SAMPLE_E_WAIT;
                end

                CTRL_KG_SAMPLE_E_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_CBD;
                    hsu_seed_id_o = SEED_ID_SIGMA;
                    hsu_poly_id_o = CTRL_POLY_EI;

                    if (hsu_done_i) begin
                        state_n = CTRL_KG_SAMPLE_A_ISSUE;
                    end
                end

                CTRL_KG_SAMPLE_A_ISSUE: begin
                    mem_phase_o      = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_start_o      = 1'b1;
                    hsu_mode_o       = MODE_SAMPLE_NTT;
                    hsu_seed_id_o    = SEED_ID_RHO;
                    hsu_poly_id_o    = a_poly_id(col_idx_r);
                    hsu_input_sel_o  = HSU_IN_SEED;
                    state_n          = CTRL_KG_SAMPLE_A_WAIT;
                end

                CTRL_KG_SAMPLE_A_WAIT: begin
                    mem_phase_o   = CTRL_MEM_PHASE_HSU_SAMPLE;
                    hsu_mode_o    = MODE_SAMPLE_NTT;
                    hsu_seed_id_o = SEED_ID_RHO;
                    hsu_poly_id_o = a_poly_id(col_idx_r);

                    if (hsu_done_i) begin
                        state_n = CTRL_KG_NEXT_A_COL;
                    end
                end

                CTRL_KG_NEXT_A_COL: begin
                    if (col_idx_r == (k_active_r - 3'd1)) begin
                        state_n = CTRL_KG_ROWMAC_ISSUE;
                    end
                    else begin
                        inc_col_idx_n = 1'b1;
                        state_n       = CTRL_KG_SAMPLE_A_ISSUE;
                    end
                end

                CTRL_KG_ROWMAC_ISSUE: begin
                    mem_phase_o                   = CTRL_MEM_PHASE_PAU;
                    pau_start_o                   = 1'b1;
                    pau_job_o.opcode              = PAU_JOB_KEYGEN_ROWMAC;
                    pau_job_o.primary_poly_id     = t_poly_id(row_idx_r);
                    pau_job_o.aux_poly_id         = CTRL_POLY_EI;
                    pau_job_o.row_idx             = row_idx_r;
                    pau_job_o.k_active            = k_active_r;
                    state_n                       = CTRL_KG_ROWMAC_WAIT;
                end

                CTRL_KG_ROWMAC_WAIT: begin
                    mem_phase_o               = CTRL_MEM_PHASE_PAU;
                    pau_job_o.opcode          = PAU_JOB_KEYGEN_ROWMAC;
                    pau_job_o.primary_poly_id = t_poly_id(row_idx_r);
                    pau_job_o.aux_poly_id     = CTRL_POLY_EI;
                    pau_job_o.row_idx         = row_idx_r;
                    pau_job_o.k_active        = k_active_r;

                    if (pau_done_i) begin
                        state_n = CTRL_KG_NEXT_ROW;
                    end
                end

                CTRL_KG_NEXT_ROW: begin
                    if (row_idx_r == (k_active_r - 3'd1)) begin
                        clear_hash_idx_n = 1'b1;
                        state_n          = CTRL_KG_HASH_EK_START;
                    end
                    else begin
                        inc_row_idx_n   = 1'b1;
                        clear_col_idx_n = 1'b1;
                        state_n         = CTRL_KG_SAMPLE_E_ISSUE;
                    end
                end

                CTRL_KG_HASH_EK_START: begin
                    mem_phase_o              = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o    = 1'b1;
                    hsu_start_o              = 1'b1;
                    hsu_mode_o               = MODE_ABSORB_POLY;
                    hsu_xof_len_o            = CTRL_XOF_LEN_32B;
                    hsu_seed_id_o            = SEED_ID_HEK;
                    hsu_input_sel_o          = HSU_IN_POLY;
                    state_n                  = CTRL_KG_HASH_EK_ABSORB_T;
                end

                CTRL_KG_HASH_EK_ABSORB_T: begin
                    mem_phase_o              = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o    = 1'b1;
                    hsu_mode_o               = MODE_ABSORB_POLY;
                    hsu_seed_id_o            = SEED_ID_HEK;
                    hsu_poly_id_o            = t_poly_id(hash_idx_r);
                    hsu_input_sel_o          = HSU_IN_POLY;
                    hsu_absorb_poly_o        = 1'b1;
                    hsu_absorb_last_o        = (hash_idx_r == (k_active_r - 3'd1));
                    state_n                  = CTRL_KG_HASH_EK_WAIT_T;
                end

                CTRL_KG_HASH_EK_WAIT_T: begin
                    mem_phase_o              = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o    = 1'b1;
                    hsu_mode_o               = MODE_ABSORB_POLY;
                    hsu_seed_id_o            = SEED_ID_HEK;
                    hsu_poly_id_o            = t_poly_id(hash_idx_r);
                    hsu_input_sel_o          = HSU_IN_POLY;
                    hsu_absorb_last_o        = (hash_idx_r == (k_active_r - 3'd1));

                    if (hsu_packer_done_i) begin
                        state_n = CTRL_KG_HASH_EK_NEXT_T;
                    end
                end

                CTRL_KG_HASH_EK_NEXT_T: begin
                    if (hash_idx_r == (k_active_r - 3'd1)) begin
                        state_n = CTRL_KG_HASH_EK_WAIT_DONE;
                    end
                    else begin
                        inc_hash_idx_n = 1'b1;
                        state_n        = CTRL_KG_HASH_EK_ABSORB_T;
                    end
                end

                CTRL_KG_HASH_EK_WAIT_DONE: begin
                    mem_phase_o           = CTRL_MEM_PHASE_HSU_HASH_EK;
                    hsu_hash_ek_read_en_o = 1'b1;
                    hsu_mode_o            = MODE_ABSORB_POLY;
                    hsu_seed_id_o         = SEED_ID_HEK;
                    // TODO(KeyGen): append rho to H(ek) once the HSU seed
                    // input request path is exposed cleanly to the controller.
                    if (hsu_done_i) begin
                        state_n = CTRL_KG_DONE;
                    end
                end

                CTRL_KG_DONE: begin
                    set_keygen_valid_n = 1'b1;
                    done_pulse_n       = 1'b1;
                    state_n            = CTRL_IDLE;
                end

                CTRL_UNSUPPORTED: begin
                    state_n    = CTRL_ERROR;
                    err_code_n = CTRL_ERR_UNSUPPORTED;
                end

                CTRL_ZEROIZE_ISSUE: begin
                    mem_phase_o         = CTRL_MEM_PHASE_ZEROIZE;
                    mem_zeroize_req_o   = 1'b1;
                    clear_protocol_n    = 1'b1;
                    state_n             = CTRL_ZEROIZE_WAIT;
                end

                CTRL_ZEROIZE_WAIT: begin
                    mem_phase_o       = CTRL_MEM_PHASE_ZEROIZE;
                    mem_zeroize_req_o = 1'b1;

                    if (mem_zeroize_done_i) begin
                        done_pulse_n = 1'b1;
                        state_n      = CTRL_IDLE;
                    end
                end

                CTRL_ERROR: begin
                    state_n = CTRL_IDLE;
                end

                default: begin
                    state_n    = CTRL_ERROR;
                    err_code_n = CTRL_ERR_ILLEGAL_CMD;
                end
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Sequential state
    // ---------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r <= CTRL_IDLE;

            cmd_opcode_lat_r     <= CMD_NOP;
            cmd_mode_lat_r       <= MODE_NONE;
            cmd_sec_lvl_lat_r    <= SEC_512;
            cmd_payload_id_lat_r <= PLD_NONE;
            cmd_xfer_len_lat_r   <= 16'd0;

            d_loaded_r   <= 1'b0;
            z_loaded_r   <= 1'b0;
            m_loaded_r   <= 1'b0;
            ek_loaded_r  <= 1'b0;
            dk_loaded_r  <= 1'b0;
            c_loaded_r   <= 1'b0;
            hek_loaded_r <= 1'b0;

            ek_valid_r  <= 1'b0;
            dk_valid_r  <= 1'b0;
            hek_valid_r <= 1'b0;
            ss_valid_r  <= 1'b0;

            k_active_r <= 3'd0;
            s_idx_r    <= 3'd0;
            row_idx_r  <= 3'd0;
            col_idx_r  <= 3'd0;
            hash_idx_r <= 3'd0;

            active_tr_opcode_r <= TR_OP_IDLE;

            done_pulse_r <= 1'b0;
            err_code_r   <= CTRL_ERR_NONE;
        end
        else begin
            state_r            <= state_n;
            active_tr_opcode_r <= active_tr_opcode_n;

            done_pulse_r <= done_pulse_n;
            err_code_r   <= err_code_n;

            if ((state_r == CTRL_IDLE) && cmd_valid_i && cmd_ready_o && !cmd_zeroize_i) begin
                cmd_opcode_lat_r     <= cmd_opcode_i;
                cmd_mode_lat_r       <= cmd_mode_i;
                cmd_sec_lvl_lat_r    <= cmd_sec_lvl_i;
                cmd_payload_id_lat_r <= cmd_payload_id_i;
                cmd_xfer_len_lat_r   <= cmd_xfer_len_i;
            end

            if (clear_protocol_n) begin
                d_loaded_r   <= 1'b0;
                z_loaded_r   <= 1'b0;
                m_loaded_r   <= 1'b0;
                ek_loaded_r  <= 1'b0;
                dk_loaded_r  <= 1'b0;
                c_loaded_r   <= 1'b0;
                hek_loaded_r <= 1'b0;
                ek_valid_r   <= 1'b0;
                dk_valid_r   <= 1'b0;
                hek_valid_r  <= 1'b0;
                ss_valid_r   <= 1'b0;
            end
            else begin
                if (set_d_loaded_n)   d_loaded_r   <= 1'b1;
                if (set_m_loaded_n)   m_loaded_r   <= 1'b1;
                if (set_z_loaded_n)   z_loaded_r   <= 1'b1;
                if (set_ek_loaded_n)  ek_loaded_r  <= 1'b1;
                if (set_dk_loaded_n)  dk_loaded_r  <= 1'b1;
                if (set_c_loaded_n)   c_loaded_r   <= 1'b1;
                if (set_hek_loaded_n) hek_loaded_r <= 1'b1;

                if (set_keygen_valid_n) begin
                    ek_valid_r  <= 1'b1;
                    dk_valid_r  <= 1'b1;
                    hek_valid_r <= 1'b1;
                    // KeyGen does not produce a shared secret.
                    ss_valid_r  <= 1'b0;
                end
            end

            if (init_keygen_n) begin
                k_active_r <= cmd_k_w;
                s_idx_r    <= 3'd0;
                row_idx_r  <= 3'd0;
                col_idx_r  <= 3'd0;
                hash_idx_r <= 3'd0;
            end
            else begin
                if (inc_s_idx_n)    s_idx_r    <= s_idx_r + 3'd1;
                if (inc_row_idx_n)  row_idx_r  <= row_idx_r + 3'd1;
                if (inc_col_idx_n)  col_idx_r  <= col_idx_r + 3'd1;
                if (inc_hash_idx_n) hash_idx_r <= hash_idx_r + 3'd1;

                if (clear_col_idx_n)  col_idx_r  <= 3'd0;
                if (clear_hash_idx_n) hash_idx_r <= 3'd0;
            end
        end
    end

    assign sts_busy_o     = (state_r != CTRL_IDLE);
    assign sts_done_o     = done_pulse_r;
    assign sts_err_code_o = err_code_r;

endmodule
