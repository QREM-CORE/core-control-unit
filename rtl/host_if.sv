/*
 * Module Name: host_if.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator - QREM Core
 *
 * Description
 * -----------
 * Thin SoC-facing wrapper for QREM Core.
 *
 * This block intentionally stays at the host/protocol boundary:
 *   - AXI4-Lite is used only for control, status, and configuration CSRs.
 *   - AXI4-Stream RX/TX is used only for payload movement.
 *   - host_if emits high-level command context to the controller.
 *   - host_if does not know transcoder internal micro-opcodes.
 *
 * The controller remains the macro-sequencer. The transcoder remains the
 * byte-domain formatting engine. This wrapper only validates coarse host
 * requests, latches command context, gates streams, and reports status.
 */

module host_if (
    input  logic        clk,
    input  logic        rst_n,

    // ============================================================
    // AXI4-Lite Slave Interface (Control / Status / Config)
    // ============================================================
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,

    output logic        irq_o,

    // ============================================================
    // AXI4-Stream Interface (External SoC Side)
    // ============================================================
    // RX (CMD_LOAD: Host -> QREM)
    input  logic [63:0] s_axis_rx_tdata,
    input  logic        s_axis_rx_tvalid,
    output logic        s_axis_rx_tready,
    input  logic        s_axis_rx_tlast,

    // TX (CMD_STORE: QREM -> Host)
    output logic [63:0] m_axis_tx_tdata,
    output logic        m_axis_tx_tvalid,
    input  logic        m_axis_tx_tready,
    output logic        m_axis_tx_tlast,

    // ============================================================
    // Deprecated Seed Outputs
    // ============================================================
    // D, Z, and M are payloads, not CSRs. They now move over AXI4-Stream
    // through CMD_LOAD transactions. These outputs are kept tied off for
    // compatibility with older integration stubs until they are removed.
    output logic [255:0] seed_d_o,
    output logic [255:0] seed_z_o,
    output logic [255:0] seed_m_o,

    // ============================================================
    // Internal: Core Command Interface (Valid/Ready Handshake)
    // ============================================================
    output logic         cmd_valid_o,
    input  logic         cmd_ready_i,

    // Latched high-level command context for the controller.
    output logic [3:0]   cmd_opcode_o,     // CMD_LOAD / CMD_STORE / CMD_START
    output logic [3:0]   cmd_mode_o,       // MODE_KEYGEN / MODE_ENCAPS / MODE_DECAPS
    output logic [1:0]   cmd_sec_lvl_o,    // SEC_512 / SEC_768 / SEC_1024
    output logic [4:0]   cmd_payload_id_o, // PLD_D / PLD_EK / PLD_C / PLD_SHARED_K / ...
    output logic [15:0]  cmd_xfer_len_o,   // Expected payload byte count

    // Priority Control
    output logic         cmd_zeroize_o,    // Direct one-cycle pulse, bypasses handshake

    // ============================================================
    // Internal: Core Status Interface
    // ============================================================
    input  logic         sts_busy_i,
    input  logic         sts_done_i,
    input  logic [3:0]   sts_err_code_i,

    // ============================================================
    // Internal: Transcoder Stream Bridge (Gated Passthrough)
    // ============================================================
    // Path to Transcoder (CMD_LOAD)
    output logic [63:0]  tc_rx_tdata_o,
    output logic         tc_rx_tvalid_o,
    input  logic         tc_rx_tready_i,
    output logic         tc_rx_tlast_o,

    // Path from Transcoder (CMD_STORE)
    input  logic [63:0]  tc_tx_tdata_i,
    input  logic         tc_tx_tvalid_i,
    output logic         tc_tx_tready_o,
    input  logic         tc_tx_tlast_i
);

    // ============================================================
    // High-Level Host Command Constants
    // ============================================================
    // These are controller-facing command classes, not transcoder micro-ops.
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

    // Coarse host-visible payload IDs. A single payload ID can require
    // several transcoder micro-ops later, but that is controller policy.
    localparam logic [4:0] PLD_NONE     = 5'd0;
    localparam logic [4:0] PLD_D        = 5'd1;
    localparam logic [4:0] PLD_Z        = 5'd2;
    localparam logic [4:0] PLD_M        = 5'd3;
    localparam logic [4:0] PLD_EK       = 5'd4;
    localparam logic [4:0] PLD_DK       = 5'd5;
    localparam logic [4:0] PLD_HEK      = 5'd6;
    localparam logic [4:0] PLD_C        = 5'd7;
    localparam logic [4:0] PLD_SHARED_K = 5'd8;

    // ============================================================
    // CSR Map
    // ============================================================
    // CONTROL  (0x00):
    //   [0] START        write-one pulse. Latches COMMAND into active context.
    //   [1] ZEROIZE      write-one pulse. Aborts local state and requests purge.
    //   [8] IRQ_DONE_EN  interrupt enable for STATUS.done.
    //   [9] IRQ_ERR_EN   interrupt enable for STATUS.error.
    //
    // COMMAND  (0x04):
    //   [3:0]   opcode
    //   [7:4]   mode
    //   [9:8]   security level
    //   [14:10] payload ID
    //
    // XFER_LEN (0x08): read-only decoded live COMMAND length.
    //
    // STATUS   (0x0C):
    //   [0] busy
    //   [1] done sticky, W1C
    //   [2] error sticky, W1C
    //   [5:3] local phase
    //
    // ERR_CODE (0x10): first sticky error code, cleared by STATUS.error W1C.
    //
    // W1C means "Write 1 to Clear": software clears a sticky bit by writing
    // a 1 to that bit position. Writing 0 leaves the sticky bit unchanged.
    localparam logic [7:0] ADDR_CONTROL  = 8'h00;
    localparam logic [7:0] ADDR_COMMAND  = 8'h04;
    localparam logic [7:0] ADDR_XFER_LEN = 8'h08;
    localparam logic [7:0] ADDR_STATUS   = 8'h0C;
    localparam logic [7:0] ADDR_ERR_CODE = 8'h10;

    localparam int CTRL_START_BIT       = 0;
    localparam int CTRL_ZEROIZE_BIT     = 1;
    localparam int CTRL_IRQ_DONE_EN_BIT = 8;
    localparam int CTRL_IRQ_ERR_EN_BIT  = 9;

    localparam int STS_BUSY_BIT  = 0;
    localparam int STS_DONE_BIT  = 1;
    localparam int STS_ERR_BIT   = 2;
    localparam int STS_PHASE_LSB = 3;
    localparam int STS_PHASE_MSB = 5;

    // Local host_if error codes. Controller-reported errors pass through
    // using sts_err_code_i when nonzero.
    localparam logic [3:0] ERR_NONE             = 4'h0;
    localparam logic [3:0] ERR_START_WHILE_BUSY = 4'h1;
    localparam logic [3:0] ERR_ILLEGAL_CMD      = 4'h2;
    localparam logic [3:0] ERR_RX_EARLY_TLAST   = 4'h3;
    localparam logic [3:0] ERR_RX_OVERRUN       = 4'h4;
    localparam logic [3:0] ERR_RX_MISSING_TLAST = 4'h5;
    localparam logic [3:0] ERR_TX_EARLY_TLAST   = 4'h6;
    localparam logic [3:0] ERR_TX_OVERRUN       = 4'h7;
    localparam logic [3:0] ERR_TX_MISSING_TLAST = 4'h8;

    typedef enum logic [2:0] {
        PH_IDLE        = 3'd0,
        PH_WAIT_ACCEPT = 3'd1,
        PH_RX          = 3'd2,
        PH_TX          = 3'd3,
        PH_WAIT_DONE   = 3'd4
    } phase_t;

    // ============================================================
    // CSR Storage
    // ============================================================
    // These registers are the live software-programmed view. They are allowed
    // to change while a command is in flight.
    logic [3:0]  csr_opcode_r;
    logic [3:0]  csr_mode_r;
    logic [1:0]  csr_sec_lvl_r;
    logic [4:0]  csr_payload_id_r;

    logic        irq_done_en_r;
    logic        irq_err_en_r;

    // Sticky status bits remain set until software clears them with W1C.
    logic        done_sticky_r;
    logic        err_sticky_r;
    logic [3:0]  err_code_r;

    // Active command context. This is latched on CONTROL.START so live CSR
    // writes cannot mutate an in-flight command while the controller and stream
    // gates are still acting on it.
    logic [3:0]  cmd_opcode_lat_r;
    logic [3:0]  cmd_mode_lat_r;
    logic [1:0]  cmd_sec_lvl_lat_r;
    logic [4:0]  cmd_payload_id_lat_r;
    logic [15:0] cmd_xfer_len_lat_r;

    logic        cmd_valid_r;
    logic        zeroize_pulse_r;
    phase_t      phase_r;

    logic [15:0] rx_bytes_r;
    logic [15:0] tx_bytes_r;

    // One-deep AXI4-Lite write channel buffers. AXI4-Lite has independent AW
    // and W channels, so this accepts either half first, then commits when both
    // halves are present.
    logic        aw_buf_valid_r;
    logic [7:0]  aw_buf_addr_r;
    logic        w_buf_valid_r;
    logic [31:0] w_buf_data_r;
    logic [3:0]  w_buf_strb_r;

    logic [15:0] live_decoded_xfer_len_w;
    logic        busy_w;
    logic        rx_hs_w;
    logic        tx_hs_w;

    assign seed_d_o = 256'h0;
    assign seed_z_o = 256'h0;
    assign seed_m_o = 256'h0;

    assign cmd_valid_o      = cmd_valid_r;
    assign cmd_opcode_o     = cmd_opcode_lat_r;
    assign cmd_mode_o       = cmd_mode_lat_r;
    assign cmd_sec_lvl_o    = cmd_sec_lvl_lat_r;
    assign cmd_payload_id_o = cmd_payload_id_lat_r;
    assign cmd_xfer_len_o   = cmd_xfer_len_lat_r;
    assign cmd_zeroize_o    = zeroize_pulse_r;

    assign busy_w = (phase_r != PH_IDLE) || cmd_valid_r || sts_busy_i;

    // IRQ is level-based from sticky status. It remains asserted until
    // software clears the enabled sticky bit with STATUS W1C.
    assign irq_o = (irq_done_en_r && done_sticky_r) ||
                   (irq_err_en_r  && err_sticky_r);

    // ============================================================
    // Stream Gating
    // ============================================================
    // RX opens only for an accepted CMD_LOAD. Backpressure comes directly from
    // the transcoder, but no payload can pass while host_if is outside PH_RX.
    assign s_axis_rx_tready = (phase_r == PH_RX) ? tc_rx_tready_i : 1'b0;
    assign tc_rx_tdata_o    = s_axis_rx_tdata;
    assign tc_rx_tvalid_o   = (phase_r == PH_RX) ? s_axis_rx_tvalid : 1'b0;
    assign tc_rx_tlast_o    = (phase_r == PH_RX) ? s_axis_rx_tlast  : 1'b0;

    // TX opens only for an accepted CMD_STORE. The host sees valid data only
    // while PH_TX is active, and host backpressure is forwarded to transcoder.
    assign m_axis_tx_tdata  = (phase_r == PH_TX) ? tc_tx_tdata_i  : 64'h0;
    assign m_axis_tx_tvalid = (phase_r == PH_TX) ? tc_tx_tvalid_i : 1'b0;
    assign m_axis_tx_tlast  = (phase_r == PH_TX) ? tc_tx_tlast_i  : 1'b0;
    assign tc_tx_tready_o   = (phase_r == PH_TX) ? m_axis_tx_tready : 1'b0;

    assign rx_hs_w = s_axis_rx_tvalid && s_axis_rx_tready;
    assign tx_hs_w = tc_tx_tvalid_i   && tc_tx_tready_o;

    // Simple AXI4-Lite single-beat access. This wrapper does not implement
    // bursts or multiple outstanding responses.
    assign s_axi_awready = !aw_buf_valid_r && !s_axi_bvalid;
    assign s_axi_wready  = !w_buf_valid_r  && !s_axi_bvalid;
    assign s_axi_arready = !s_axi_rvalid;

    // ============================================================
    // Helper Functions
    // ============================================================
    function automatic logic [31:0] apply_wstrb (
        input logic [31:0] old_data,
        input logic [31:0] new_data,
        input logic [3:0]  wstrb
    );
        logic [31:0] tmp;
        begin
            tmp = old_data;
            if (wstrb[0]) tmp[7:0]   = new_data[7:0];
            if (wstrb[1]) tmp[15:8]  = new_data[15:8];
            if (wstrb[2]) tmp[23:16] = new_data[23:16];
            if (wstrb[3]) tmp[31:24] = new_data[31:24];
            apply_wstrb = tmp;
        end
    endfunction

    function automatic logic valid_sec_lvl (
        input logic [1:0] sec_lvl
    );
        begin
            valid_sec_lvl = (sec_lvl == SEC_512)  ||
                            (sec_lvl == SEC_768)  ||
                            (sec_lvl == SEC_1024);
        end
    endfunction

    function automatic logic [15:0] decode_xfer_len (
        input logic [1:0] sec_lvl,
        input logic [4:0] payload_id
    );
        begin
            unique case (payload_id)
                PLD_D, PLD_Z, PLD_M, PLD_HEK, PLD_SHARED_K: begin
                    decode_xfer_len = 16'd32;
                end

                PLD_EK: begin
                    unique case (sec_lvl)
                        SEC_512:  decode_xfer_len = 16'd800;
                        SEC_768:  decode_xfer_len = 16'd1184;
                        SEC_1024: decode_xfer_len = 16'd1568;
                        default:  decode_xfer_len = 16'd0;
                    endcase
                end

                PLD_DK: begin
                    unique case (sec_lvl)
                        SEC_512:  decode_xfer_len = 16'd1632;
                        SEC_768:  decode_xfer_len = 16'd2400;
                        SEC_1024: decode_xfer_len = 16'd3168;
                        default:  decode_xfer_len = 16'd0;
                    endcase
                end

                PLD_C: begin
                    unique case (sec_lvl)
                        SEC_512:  decode_xfer_len = 16'd768;
                        SEC_768:  decode_xfer_len = 16'd1088;
                        SEC_1024: decode_xfer_len = 16'd1568;
                        default:  decode_xfer_len = 16'd0;
                    endcase
                end

                default: begin
                    decode_xfer_len = 16'd0;
                end
            endcase
        end
    endfunction

    // This is intentionally a coarse host-command legality check. It knows
    // whether a payload is allowed for a mode and direction, but it does not
    // know the internal sequence of transcoder formatting operations needed to
    // implement that payload.
    function automatic logic legal_cmd (
        input logic [3:0] opcode,
        input logic [3:0] mode,
        input logic [1:0] sec_lvl,
        input logic [4:0] payload_id
    );
        begin
            legal_cmd = valid_sec_lvl(sec_lvl);

            if (legal_cmd) begin
                unique case (opcode)
                    CMD_LOAD: begin
                        unique case (mode)
                            MODE_KEYGEN: legal_cmd = (payload_id == PLD_D);
                            MODE_ENCAPS: legal_cmd = (payload_id == PLD_M)  ||
                                                     (payload_id == PLD_EK);
                            MODE_DECAPS: legal_cmd = (payload_id == PLD_DK) ||
                                                     (payload_id == PLD_EK)  ||
                                                     (payload_id == PLD_HEK) ||
                                                     (payload_id == PLD_C)   ||
                                                     (payload_id == PLD_Z);
                            default:     legal_cmd = 1'b0;
                        endcase
                    end

                    CMD_STORE: begin
                        unique case (mode)
                            MODE_KEYGEN: legal_cmd = (payload_id == PLD_EK) ||
                                                     (payload_id == PLD_DK) ||
                                                     (payload_id == PLD_HEK);
                            MODE_ENCAPS: legal_cmd = (payload_id == PLD_C)  ||
                                                     (payload_id == PLD_SHARED_K);
                            MODE_DECAPS: legal_cmd = (payload_id == PLD_SHARED_K);
                            default:     legal_cmd = 1'b0;
                        endcase
                    end

                    CMD_START: begin
                        legal_cmd = (mode == MODE_KEYGEN) ||
                                    (mode == MODE_ENCAPS) ||
                                    (mode == MODE_DECAPS);
                    end

                    default: begin
                        legal_cmd = 1'b0;
                    end
                endcase
            end
        end
    endfunction

    assign live_decoded_xfer_len_w = decode_xfer_len(csr_sec_lvl_r, csr_payload_id_r);

    function automatic logic [31:0] csr_read_data (
        input logic [7:0] addr
    );
        logic [31:0] rd;
        begin
            rd = 32'h0000_0000;

            unique case (addr)
                ADDR_CONTROL: begin
                    rd[CTRL_IRQ_DONE_EN_BIT] = irq_done_en_r;
                    rd[CTRL_IRQ_ERR_EN_BIT]  = irq_err_en_r;
                end

                ADDR_COMMAND: begin
                    rd[3:0]   = csr_opcode_r;
                    rd[7:4]   = csr_mode_r;
                    rd[9:8]   = csr_sec_lvl_r;
                    rd[14:10] = csr_payload_id_r;
                end

                ADDR_XFER_LEN: begin
                    rd[15:0] = live_decoded_xfer_len_w;
                end

                ADDR_STATUS: begin
                    rd[STS_BUSY_BIT]                 = busy_w;
                    rd[STS_DONE_BIT]                 = done_sticky_r;
                    rd[STS_ERR_BIT]                  = err_sticky_r;
                    rd[STS_PHASE_MSB:STS_PHASE_LSB]  = phase_r;
                end

                ADDR_ERR_CODE: begin
                    rd[3:0] = err_code_r;
                end

                default: begin
                    rd = 32'h0000_0000;
                end
            endcase

            csr_read_data = rd;
        end
    endfunction

    // ============================================================
    // Helper Tasks
    // ============================================================
    task automatic clear_active_command;
        begin
            phase_r              <= PH_IDLE;
            cmd_valid_r          <= 1'b0;
            cmd_opcode_lat_r     <= CMD_NOP;
            cmd_mode_lat_r       <= MODE_NONE;
            cmd_sec_lvl_lat_r    <= SEC_512;
            cmd_payload_id_lat_r <= PLD_NONE;
            cmd_xfer_len_lat_r   <= 16'd0;
            rx_bytes_r           <= 16'd0;
            tx_bytes_r           <= 16'd0;
        end
    endtask

    task automatic raise_error (
        input logic [3:0] code,
        input logic       overwrite_code
    );
        begin
            clear_active_command();

            // First error wins until software clears STATUS.error with W1C.
            // If the old sticky error is W1C-cleared in this same clock, allow
            // a newly observed error to become the new first sticky code.
            if (!err_sticky_r || overwrite_code) begin
                err_code_r <= code;
            end
            err_sticky_r <= 1'b1;
        end
    endtask

    task automatic launch_command (
        input logic overwrite_error_code
    );
        logic [15:0] dec_len;
        begin
            if (busy_w) begin
                raise_error(ERR_START_WHILE_BUSY, overwrite_error_code);
            end
            else if (!legal_cmd(csr_opcode_r, csr_mode_r, csr_sec_lvl_r, csr_payload_id_r)) begin
                raise_error(ERR_ILLEGAL_CMD, overwrite_error_code);
            end
            else begin
                dec_len = 16'd0;
                if (csr_opcode_r != CMD_START) begin
                    dec_len = decode_xfer_len(csr_sec_lvl_r, csr_payload_id_r);
                end

                if (((csr_opcode_r == CMD_LOAD) || (csr_opcode_r == CMD_STORE)) &&
                    (dec_len == 16'd0)) begin
                    raise_error(ERR_ILLEGAL_CMD, overwrite_error_code);
                end
                else begin
                    cmd_opcode_lat_r     <= csr_opcode_r;
                    cmd_mode_lat_r       <= csr_mode_r;
                    cmd_sec_lvl_lat_r    <= csr_sec_lvl_r;
                    cmd_payload_id_lat_r <= (csr_opcode_r == CMD_START) ? PLD_NONE : csr_payload_id_r;
                    cmd_xfer_len_lat_r   <= dec_len;

                    rx_bytes_r  <= 16'd0;
                    tx_bytes_r  <= 16'd0;
                    cmd_valid_r <= 1'b1;
                    phase_r     <= PH_WAIT_ACCEPT;
                end
            end
        end
    endtask

    // ============================================================
    // AXI4-Lite, Command, and Transfer Sequencing
    // ============================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_bresp  <= 2'b00;
            s_axi_bvalid <= 1'b0;
            s_axi_rdata  <= 32'h0000_0000;
            s_axi_rresp  <= 2'b00;
            s_axi_rvalid <= 1'b0;

            aw_buf_valid_r <= 1'b0;
            aw_buf_addr_r  <= 8'h00;
            w_buf_valid_r  <= 1'b0;
            w_buf_data_r   <= 32'h0000_0000;
            w_buf_strb_r   <= 4'h0;

            csr_opcode_r     <= CMD_NOP;
            csr_mode_r       <= MODE_NONE;
            csr_sec_lvl_r    <= SEC_512;
            csr_payload_id_r <= PLD_NONE;

            irq_done_en_r <= 1'b0;
            irq_err_en_r  <= 1'b0;

            done_sticky_r <= 1'b0;
            err_sticky_r  <= 1'b0;
            err_code_r    <= ERR_NONE;

            cmd_opcode_lat_r     <= CMD_NOP;
            cmd_mode_lat_r       <= MODE_NONE;
            cmd_sec_lvl_lat_r    <= SEC_512;
            cmd_payload_id_lat_r <= PLD_NONE;
            cmd_xfer_len_lat_r   <= 16'd0;

            cmd_valid_r     <= 1'b0;
            zeroize_pulse_r <= 1'b0;
            phase_r         <= PH_IDLE;
            rx_bytes_r      <= 16'd0;
            tx_bytes_r      <= 16'd0;
        end
        else begin
            logic        skip_phase_update;
            logic        err_clear_this_cycle;
            logic [31:0] reg_img;
            logic [15:0] next_bytes;

            skip_phase_update = 1'b0;
            err_clear_this_cycle = 1'b0;
            reg_img           = 32'h0000_0000;
            next_bytes        = 16'd0;

            // Default one-cycle pulse behavior.
            zeroize_pulse_r <= 1'b0;

            // ----------------------------------------------------
            // AXI4-Lite Write Address/Data Buffering
            // ----------------------------------------------------
            if (s_axi_awready && s_axi_awvalid) begin
                aw_buf_valid_r <= 1'b1;
                aw_buf_addr_r  <= s_axi_awaddr;
            end

            if (s_axi_wready && s_axi_wvalid) begin
                w_buf_valid_r <= 1'b1;
                w_buf_data_r  <= s_axi_wdata;
                w_buf_strb_r  <= s_axi_wstrb;
            end

            // Commit when both buffered halves of the write are available.
            if (aw_buf_valid_r && w_buf_valid_r && !s_axi_bvalid) begin
                unique case (aw_buf_addr_r)
                    ADDR_CONTROL: begin
                        reg_img = 32'h0000_0000;
                        reg_img[CTRL_IRQ_DONE_EN_BIT] = irq_done_en_r;
                        reg_img[CTRL_IRQ_ERR_EN_BIT]  = irq_err_en_r;
                        reg_img = apply_wstrb(reg_img, w_buf_data_r, w_buf_strb_r);

                        irq_done_en_r <= reg_img[CTRL_IRQ_DONE_EN_BIT];
                        irq_err_en_r  <= reg_img[CTRL_IRQ_ERR_EN_BIT];

                        // ZEROIZE has priority over START. It aborts only
                        // local host_if state and emits a pulse requesting the
                        // rest of the system to purge sensitive state.
                        if (w_buf_strb_r[0] && w_buf_data_r[CTRL_ZEROIZE_BIT]) begin
                            clear_active_command();
                            zeroize_pulse_r   <= 1'b1;
                            skip_phase_update = 1'b1;
                        end
                        else if (w_buf_strb_r[0] && w_buf_data_r[CTRL_START_BIT]) begin
                            launch_command(1'b0);
                            skip_phase_update = 1'b1;
                        end
                    end

                    ADDR_COMMAND: begin
                        reg_img = 32'h0000_0000;
                        reg_img[3:0]   = csr_opcode_r;
                        reg_img[7:4]   = csr_mode_r;
                        reg_img[9:8]   = csr_sec_lvl_r;
                        reg_img[14:10] = csr_payload_id_r;
                        reg_img = apply_wstrb(reg_img, w_buf_data_r, w_buf_strb_r);

                        csr_opcode_r     <= reg_img[3:0];
                        csr_mode_r       <= reg_img[7:4];
                        csr_sec_lvl_r    <= reg_img[9:8];
                        csr_payload_id_r <= reg_img[14:10];
                    end

                    ADDR_STATUS: begin
                        // W1C: writing 1 clears the selected sticky bit.
                        if (w_buf_strb_r[0] && w_buf_data_r[STS_DONE_BIT]) begin
                            done_sticky_r <= 1'b0;
                        end
                        if (w_buf_strb_r[0] && w_buf_data_r[STS_ERR_BIT]) begin
                            err_sticky_r <= 1'b0;
                            err_code_r   <= ERR_NONE;
                            err_clear_this_cycle = 1'b1;
                        end
                    end

                    default: begin
                        // Unknown and read-only CSRs ignore writes and return OKAY.
                    end
                endcase

                s_axi_bvalid   <= 1'b1;
                s_axi_bresp    <= 2'b00;
                aw_buf_valid_r <= 1'b0;
                w_buf_valid_r  <= 1'b0;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            // ----------------------------------------------------
            // AXI4-Lite Read
            // ----------------------------------------------------
            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                s_axi_rdata  <= csr_read_data(s_axi_araddr);
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end

            // ----------------------------------------------------
            // Controller Error and Local Phase Control
            // ----------------------------------------------------
            if (!skip_phase_update) begin
                if (sts_err_code_i != ERR_NONE) begin
                    raise_error(sts_err_code_i, err_clear_this_cycle);
                end
                else begin
                    unique case (phase_r)
                        PH_IDLE: begin
                            // No active local transfer.
                        end

                        PH_WAIT_ACCEPT: begin
                            if (cmd_valid_r && cmd_ready_i) begin
                                cmd_valid_r <= 1'b0;

                                unique case (cmd_opcode_lat_r)
                                    CMD_LOAD:  phase_r <= PH_RX;
                                    CMD_STORE: phase_r <= PH_TX;
                                    CMD_START: phase_r <= PH_WAIT_DONE;
                                    default:   raise_error(ERR_ILLEGAL_CMD, err_clear_this_cycle);
                                endcase
                            end
                        end

                        PH_RX: begin
                            if (rx_hs_w) begin
                                next_bytes = rx_bytes_r + 16'd8;

                                // Since there is no TKEEP, every accepted
                                // stream beat is exactly 8 bytes.
                                if (next_bytes > cmd_xfer_len_lat_r) begin
                                    raise_error(ERR_RX_OVERRUN, err_clear_this_cycle);
                                end
                                else if (s_axis_rx_tlast && (next_bytes != cmd_xfer_len_lat_r)) begin
                                    raise_error(ERR_RX_EARLY_TLAST, err_clear_this_cycle);
                                end
                                else if (!s_axis_rx_tlast && (next_bytes == cmd_xfer_len_lat_r)) begin
                                    raise_error(ERR_RX_MISSING_TLAST, err_clear_this_cycle);
                                end
                                else begin
                                    rx_bytes_r <= next_bytes;

                                    // LOAD completes locally when the expected
                                    // byte count arrives and the final beat has
                                    // TLAST. The controller may then use the
                                    // loaded payload in later macro-sequences.
                                    if (s_axis_rx_tlast && (next_bytes == cmd_xfer_len_lat_r)) begin
                                        clear_active_command();
                                        done_sticky_r <= 1'b1;
                                    end
                                end
                            end
                        end

                        PH_TX: begin
                            if (tx_hs_w) begin
                                next_bytes = tx_bytes_r + 16'd8;

                                if (next_bytes > cmd_xfer_len_lat_r) begin
                                    raise_error(ERR_TX_OVERRUN, err_clear_this_cycle);
                                end
                                else if (tc_tx_tlast_i && (next_bytes != cmd_xfer_len_lat_r)) begin
                                    raise_error(ERR_TX_EARLY_TLAST, err_clear_this_cycle);
                                end
                                else if (!tc_tx_tlast_i && (next_bytes == cmd_xfer_len_lat_r)) begin
                                    raise_error(ERR_TX_MISSING_TLAST, err_clear_this_cycle);
                                end
                                else begin
                                    tx_bytes_r <= next_bytes;

                                    // STORE completes locally when host_if has
                                    // forwarded the expected number of bytes
                                    // and the transcoder marks the final beat.
                                    if (tc_tx_tlast_i && (next_bytes == cmd_xfer_len_lat_r)) begin
                                        clear_active_command();
                                        done_sticky_r <= 1'b1;
                                    end
                                end
                            end
                        end

                        PH_WAIT_DONE: begin
                            // CMD_START is compute/control work owned by the
                            // controller. host_if only waits for controller
                            // completion; it does not sequence sub-operations.
                            if (sts_done_i) begin
                                clear_active_command();
                                done_sticky_r <= 1'b1;
                            end
                        end

                        default: begin
                            clear_active_command();
                        end
                    endcase
                end
            end
        end
    end

endmodule
