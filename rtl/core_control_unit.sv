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
    input  logic                         rst,

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
    typedef enum logic [3:0] {
        CTRL_IDLE,
        CTRL_DECODE,

        CTRL_LOAD_ISSUE_TR,
        CTRL_LOAD_WAIT_TR,

        CTRL_STORE_ISSUE_TR,
        CTRL_STORE_WAIT_TR,
        CTRL_STORE_EK_ISSUE_RHO,
        CTRL_STORE_EK_WAIT_RHO,

        CTRL_START_DISPATCH,

        // KeyGen sequencing is fully delegated to kg_fsm.
        // This state holds while kg_fsm.kg_active_o is high.
        CTRL_KEYGEN,

        CTRL_UNSUPPORTED,
        CTRL_ZEROIZE_ISSUE,
        CTRL_ZEROIZE_WAIT,
        CTRL_ERROR
    } ctrl_state_t;

    ctrl_state_t state_r, state_n;

    // Latched command context from host_if. Live CSR changes in host_if cannot
    // mutate these fields while a command is in flight.
    //
    // Security-level must be accepted and latched once per operation start.
    // All KeyGen sequencing derives k from this latched value (via kg_fsm),
    // never from a live CSR that could change mid-operation.
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

    // Transcoder opcode latch — keeps wait states stable while controller is busy.
    tr_opcode_t active_tr_opcode_r, active_tr_opcode_n;

    // One-cycle host-visible status pulses.
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

    // ------------------------------------------------------------------
    // kg_fsm wiring
    // ------------------------------------------------------------------
    logic        kg_start_w;        // one-cycle pulse into kg_fsm
    logic        kg_abort_w;        // asserted when CCU aborts an in-flight keygen
    logic        kg_active_w;       // high while kg_fsm owns the datapaths
    logic        kg_done_w;         // one-cycle success pulse from kg_fsm
    logic        kg_err_w;          // one-cycle error pulse from kg_fsm
    logic [3:0]  kg_err_code_w;

    // kg_fsm-driven datapath signals (muxed onto CCU outputs below)
    logic                      kg_hsu_start_w;
    hs_mode_t                  kg_hsu_mode_w;
    logic [CTRL_XOF_LEN_W-1:0] kg_hsu_xof_len_w;
    logic                      kg_hsu_is_eta3_w;
    logic [POLY_ID_WIDTH-1:0]  kg_hsu_poly_id_w;
    seed_id_e                  kg_hsu_seed_id_w;
    logic [1:0]                kg_hsu_input_sel_w;
    logic                      kg_hsu_absorb_poly_w;
    logic                      kg_hsu_absorb_last_w;
    logic                      kg_hsu_hash_ek_read_en_w;
    logic                      kg_pau_start_w;
    ctrl_pau_job_t             kg_pau_job_w;
    ctrl_mem_phase_t           kg_mem_phase_w;

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
                    // Blocked-by-external-repo: KeyGen export/ingest opcodes
                    // rely on the transcoder's internal view of poly/seed
                    // locations matching the controller's slot map in
                    // core_ctrl_pkg (S slots, T slots, rho/hek seed IDs).
                    // The controller enforces k_active by scheduling; the
                    // transcoder must not infer k implicitly.
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

    // s/a/t poly-id helpers removed — now live in kg_fsm.

    // kg_start is a one-cycle pulse generated combinatorially from CTRL_START_DISPATCH.
    assign kg_start_w = (state_r == CTRL_START_DISPATCH) &&
                        (cmd_mode_lat_r == MODE_KEYGEN);

    // Abort kg_fsm only on a zeroize pulse while KeyGen is running.
    // Subsystem errors (tr/hsu/pau/mem) are caught by the parent's global error
    // check which immediately overrides state_n to CTRL_ERROR, bypassing the
    // CTRL_KEYGEN case entirely — no additional abort signal is needed for those.
    assign kg_abort_w = (state_r == CTRL_KEYGEN) && cmd_zeroize_i;

    // ---------------------------------------------------------------------
    // Combinational control
    // ---------------------------------------------------------------------
    always_comb begin
        state_n            = state_r;
        active_tr_opcode_n = active_tr_opcode_r;

        done_pulse_n = 1'b0;
        err_code_n   = CTRL_ERR_NONE;

        set_d_loaded_n     = 1'b0;
        set_m_loaded_n     = 1'b0;
        set_z_loaded_n     = 1'b0;
        set_ek_loaded_n    = 1'b0;
        set_dk_loaded_n    = 1'b0;
        set_c_loaded_n     = 1'b0;
        set_hek_loaded_n   = 1'b0;
        set_keygen_valid_n = 1'b0;
        clear_protocol_n   = 1'b0;

        cmd_ready_o = (state_r == CTRL_IDLE);

        tr_start_o  = 1'b0;
        tr_opcode_o = active_tr_opcode_r;

        // Default HSU/PAU/mem outputs — overridden by kg_fsm mux below.
        hsu_start_o           = 1'b0;
        hsu_mode_o            = MODE_HASH_SHA3_256;
        hsu_xof_len_o         = CTRL_XOF_LEN_UNUSED;
        hsu_is_eta3_o         = core_ctrl_pkg::ctrl_is_eta3(cmd_sec_lvl_lat_r);
        hsu_poly_id_o         = '0;
        hsu_seed_id_o         = SEED_ID_TMP;
        hsu_input_sel_o       = HSU_IN_SEED;
        hsu_absorb_poly_o     = 1'b0;
        hsu_absorb_last_o     = 1'b0;
        hsu_hash_ek_read_en_o = 1'b0;

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
                            (cmd_payload_id_lat_r == PLD_DK)) begin
                            // Locked KeyGen: PLD_DK is dk_partial (no z):
                            //   dk_partial = ByteEncode12(s_hat[0..k-1]) || ek || H(ek)
                            // with ek = ByteEncode12(t_hat[0..k-1]) || rho.
                            //
                            // The controller sequences these as transcoder sub-ops
                            // within one host-visible CMD_STORE transaction.
                            // host_if is responsible for suppressing intermediate
                            // TLAST beats so software observes one continuous frame.
                            unique case (active_tr_opcode_r)
                                TR_OP_KG_EXPORT_DK_PKE: begin
                                    active_tr_opcode_n = TR_OP_KG_EXPORT_EK_PKE_1;
                                    state_n            = CTRL_STORE_ISSUE_TR;
                                end
                                TR_OP_KG_EXPORT_EK_PKE_1: begin
                                    active_tr_opcode_n = TR_OP_KG_EXPORT_EK_PKE_2;
                                    state_n            = CTRL_STORE_ISSUE_TR;
                                end
                                TR_OP_KG_EXPORT_EK_PKE_2: begin
                                    active_tr_opcode_n = TR_OP_KG_EXPORT_HEK;
                                    state_n            = CTRL_STORE_ISSUE_TR;
                                end
                                default: begin
                                    done_pulse_n = 1'b1;
                                    state_n      = CTRL_IDLE;
                                end
                            endcase
                        end
                        else if ((cmd_mode_lat_r == MODE_KEYGEN) &&
                                 (cmd_payload_id_lat_r == PLD_EK) &&
                                 (active_tr_opcode_r == TR_OP_KG_EXPORT_EK_PKE_1)) begin
                            // Current transcoder exposes EK as t_hat and rho sub-ops.
                            // The controller keeps the split sequencing explicit; host_if
                            // bridges the intermediate TLAST so software sees one
                            // continuous PLD_EK frame until a unified transcoder opcode
                            // exists.
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
                        MODE_KEYGEN: state_n = CTRL_KEYGEN;  // kg_start_w pulses this cycle
                        MODE_ENCAPS,
                        MODE_DECAPS: state_n = CTRL_UNSUPPORTED;
                        default: begin
                            state_n    = CTRL_ERROR;
                            err_code_n = CTRL_ERR_ILLEGAL_CMD;
                        end
                    endcase
                end

                CTRL_KEYGEN: begin
                    // Mux kg_fsm outputs onto the shared buses while it is active.
                    hsu_start_o           = kg_hsu_start_w;
                    hsu_mode_o            = kg_hsu_mode_w;
                    hsu_xof_len_o         = kg_hsu_xof_len_w;
                    hsu_is_eta3_o         = kg_hsu_is_eta3_w;
                    hsu_poly_id_o         = kg_hsu_poly_id_w;
                    hsu_seed_id_o         = kg_hsu_seed_id_w;
                    hsu_input_sel_o       = kg_hsu_input_sel_w;
                    hsu_absorb_poly_o     = kg_hsu_absorb_poly_w;
                    hsu_absorb_last_o     = kg_hsu_absorb_last_w;
                    hsu_hash_ek_read_en_o = kg_hsu_hash_ek_read_en_w;
                    pau_start_o           = kg_pau_start_w;
                    pau_job_o             = kg_pau_job_w;
                    mem_phase_o           = kg_mem_phase_w;

                    if (kg_done_w) begin
                        set_keygen_valid_n = 1'b1;
                        done_pulse_n       = 1'b1;
                        state_n            = CTRL_IDLE;
                    end
                    else if (kg_err_w) begin
                        err_code_n = kg_err_code_w;
                        state_n    = CTRL_ERROR;
                    end
                end



                CTRL_UNSUPPORTED: begin
                    state_n    = CTRL_ERROR;
                    err_code_n = CTRL_ERR_UNSUPPORTED;
                end

                CTRL_ZEROIZE_ISSUE: begin
                    mem_phase_o         = CTRL_MEM_PHASE_ZEROIZE;
                    mem_zeroize_req_o   = 1'b1;
                    clear_protocol_n    = 1'b1;
                    if (mem_zeroize_done_i) begin
                        // Memory wipe completed immediately (e.g. single-cycle stub).
                        done_pulse_n = 1'b1;
                        state_n      = CTRL_IDLE;
                    end
                    else begin
                        state_n = CTRL_ZEROIZE_WAIT;
                    end
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
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
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
                // Command acceptance point:
                // Capture a stable snapshot of {opcode,mode,sec_lvl,payload,len}.
                // The controller and kg_fsm must use this latched security-level
                // to derive k_active and loop bounds for the entire operation.
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
                    // KeyGen result validity:
                    //   - ek is exportable
                    //   - dk_valid indicates locked dk_partial (no z; host appends z later)
                    //   - H(ek) is present in Seed RAM and exportable via PLD_HEK
                    ek_valid_r  <= 1'b1;
                    dk_valid_r  <= 1'b1;
                    hek_valid_r <= 1'b1;
                    // KeyGen does not produce a shared secret.
                    ss_valid_r  <= 1'b0;
                end
            end

            // Loop counters are now owned by kg_fsm.
        end
    end

    assign sts_busy_o     = (state_r != CTRL_IDLE);
    assign sts_done_o     = done_pulse_r;
    assign sts_err_code_o = err_code_r;

    // ------------------------------------------------------------------
    // kg_fsm instantiation
    // ------------------------------------------------------------------
    kg_fsm u_kg_fsm (
        .clk                    (clk),
        .rst                    (rst),
        // Control
        .kg_start_i             (kg_start_w),
        .kg_abort_i             (kg_abort_w),
        .d_loaded_i             (d_loaded_r),
        .cmd_sec_lvl_i          (cmd_sec_lvl_lat_r),
        // Done strobes from sub-units
        .hsu_done_i             (hsu_done_i),
        .hsu_packer_done_i      (hsu_packer_done_i),
        .pau_done_i             (pau_done_i),
        // Status
        .kg_active_o            (kg_active_w),
        .kg_done_o              (kg_done_w),
        .kg_err_o               (kg_err_w),
        .kg_err_code_o          (kg_err_code_w),
        // HSU outputs
        .hsu_start_o            (kg_hsu_start_w),
        .hsu_mode_o             (kg_hsu_mode_w),
        .hsu_xof_len_o          (kg_hsu_xof_len_w),
        .hsu_is_eta3_o          (kg_hsu_is_eta3_w),
        .hsu_poly_id_o          (kg_hsu_poly_id_w),
        .hsu_seed_id_o          (kg_hsu_seed_id_w),
        .hsu_input_sel_o        (kg_hsu_input_sel_w),
        .hsu_absorb_poly_o      (kg_hsu_absorb_poly_w),
        .hsu_absorb_last_o      (kg_hsu_absorb_last_w),
        .hsu_hash_ek_read_en_o  (kg_hsu_hash_ek_read_en_w),
        // PAU outputs
        .pau_start_o            (kg_pau_start_w),
        .pau_job_o              (kg_pau_job_w),
        // Memory phase
        .mem_phase_o            (kg_mem_phase_w)
    );

endmodule
