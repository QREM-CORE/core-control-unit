package core_ctrl_pkg;
    import qrem_global_pkg::*;

    // ---------------------------------------------------------------------
    // Host-facing command classes.
    //
    // host_if owns the CSR bank and validates software-visible requests.
    // These constants are repeated here so the controller can decode the
    // already-latched command context without importing host_if internals.
    // ---------------------------------------------------------------------
    localparam logic [3:0] CMD_NOP   = 4'd0;
    localparam logic [3:0] CMD_LOAD  = 4'd1;
    localparam logic [3:0] CMD_STORE = 4'd2;
    localparam logic [3:0] CMD_START = 4'd3;

    localparam logic [3:0] MODE_NONE   = 4'd0;
    localparam logic [3:0] MODE_KEYGEN = 4'd1;
    localparam logic [3:0] MODE_ENCAPS = 4'd2;
    localparam logic [3:0] MODE_DECAPS = 4'd3;

    localparam logic [1:0] SEC_512  = 2'd0;
    localparam logic [1:0] SEC_768  = 2'd1;
    localparam logic [1:0] SEC_1024 = 2'd2;

    localparam logic [4:0] PLD_NONE     = 5'd0;
    localparam logic [4:0] PLD_D        = 5'd1;
    localparam logic [4:0] PLD_Z        = 5'd2;
    localparam logic [4:0] PLD_M        = 5'd3;
    localparam logic [4:0] PLD_EK       = 5'd4;
    localparam logic [4:0] PLD_DK       = 5'd5;
    localparam logic [4:0] PLD_HEK      = 5'd6;
    localparam logic [4:0] PLD_C        = 5'd7;
    localparam logic [4:0] PLD_SHARED_K = 5'd8;

    // Controller-reported errors. Values 1..8 are currently used by host_if
    // local protocol errors, so the controller uses the upper half of the
    // 4-bit host-visible error namespace.
    localparam logic [3:0] CTRL_ERR_NONE         = 4'h0;
    localparam logic [3:0] CTRL_ERR_UNSUPPORTED  = 4'h9;
    localparam logic [3:0] CTRL_ERR_ILLEGAL_CMD  = 4'hA;
    localparam logic [3:0] CTRL_ERR_PRECONDITION = 4'hB;
    localparam logic [3:0] CTRL_ERR_TRANSCODER   = 4'hC;
    localparam logic [3:0] CTRL_ERR_HSU          = 4'hD;
    localparam logic [3:0] CTRL_ERR_PAU          = 4'hE;
    localparam logic [3:0] CTRL_ERR_MEMORY       = 4'hF;

    // HSU input selector values used by hash_sampler_unit.
    localparam logic [1:0] HSU_IN_SEED = 2'd0;
    localparam logic [1:0] HSU_IN_POLY = 2'd1;
    localparam logic [1:0] HSU_IN_AXIS = 2'd2;

    // ------------------------------------------------------------------
    // Controller-visible polynomial slots
    // ------------------------------------------------------------------
    // Locked KeyGen intent (memory safety enforced by scheduling, not RAM HW):
    //   Poly 0..3  : s_j / s_hat_j (only 0..k-1 used; unused slots idle)
    //   Poly 4     : active error scratchpad e_i / e_hat_i
    //   Poly 5..8  : active A_hat row buffer (A_hat(i,0..k-1))
    //   Poly 9..12 : final t_hat_i (t_hat(0..k-1))
    //
    // The controller must explicitly limit loops to the first k entries.
    // No formatter/transcoder/memory module should infer k implicitly.
    //
    // These IDs mirror the current memory subsystem map and let the controller
    // express ownership without importing the full memory repository.
    localparam logic [POLY_ID_WIDTH-1:0] CTRL_POLY_S_BASE = POLY_ID_WIDTH'(0);
    localparam logic [POLY_ID_WIDTH-1:0] CTRL_POLY_EI     = POLY_ID_WIDTH'(4);
    localparam logic [POLY_ID_WIDTH-1:0] CTRL_POLY_A_BASE = POLY_ID_WIDTH'(5);
    localparam logic [POLY_ID_WIDTH-1:0] CTRL_POLY_T_BASE = POLY_ID_WIDTH'(9);

    // Placeholder PAU job opcodes. The current PAU public package is not a
    // dependency of the Control repo yet, so these are controller-local hooks
    // for the eventual PAU adapter.
    typedef enum logic [7:0] {
        PAU_JOB_NONE          = 8'd0,
        PAU_JOB_NTT_IN_PLACE  = 8'd1,
        PAU_JOB_KEYGEN_ROWMAC = 8'd2
    } ctrl_pau_opcode_t;

    typedef struct packed {
        ctrl_pau_opcode_t              opcode;
        logic [POLY_ID_WIDTH-1:0]      primary_poly_id;
        logic [POLY_ID_WIDTH-1:0]      aux_poly_id;
        logic [2:0]                    row_idx;
        logic [2:0]                    col_idx;
        logic [2:0]                    k_active;
    } ctrl_pau_job_t;

    typedef enum logic [2:0] {
        CTRL_MEM_PHASE_IDLE        = 3'd0,
        CTRL_MEM_PHASE_TRANSCODER  = 3'd1,
        CTRL_MEM_PHASE_HSU_SAMPLE  = 3'd2,
        CTRL_MEM_PHASE_PAU         = 3'd3,
        CTRL_MEM_PHASE_HSU_HASH_EK = 3'd4,
        CTRL_MEM_PHASE_ZEROIZE     = 3'd5
    } ctrl_mem_phase_t;

    // XOF length width is a temporary Control-side hook. The HSU ultimately
    // expects keccak_pkg::XOF_LEN_WIDTH; top-level glue may narrow or widen
    // this field when the Keccak package becomes a Control dependency.
    localparam int CTRL_XOF_LEN_W = 16;
    localparam logic [CTRL_XOF_LEN_W-1:0] CTRL_XOF_LEN_UNUSED = '0;
    localparam logic [CTRL_XOF_LEN_W-1:0] CTRL_XOF_LEN_32B    = 16'd32;
    localparam logic [CTRL_XOF_LEN_W-1:0] CTRL_XOF_LEN_64B    = 16'd64;

    function automatic logic [2:0] ctrl_k_from_sec (
        input logic [1:0] sec_lvl
    );
        begin
            unique case (sec_lvl)
                SEC_512:  ctrl_k_from_sec = 3'd2;
                SEC_768:  ctrl_k_from_sec = 3'd3;
                SEC_1024: ctrl_k_from_sec = 3'd4;
                default:  ctrl_k_from_sec = 3'd0;
            endcase
        end
    endfunction

    function automatic logic ctrl_valid_sec (
        input logic [1:0] sec_lvl
    );
        begin
            ctrl_valid_sec = (ctrl_k_from_sec(sec_lvl) != 3'd0);
        end
    endfunction

    function automatic logic ctrl_is_eta3 (
        input logic [1:0] sec_lvl
    );
        begin
            // Matches the current HSU contract: eta3 is selected for the
            // 768/1024 profiles and deasserted for 512.
            ctrl_is_eta3 = (sec_lvl == SEC_768) || (sec_lvl == SEC_1024);
        end
    endfunction
endpackage
