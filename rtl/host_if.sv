/*
 * Module Name: host_if.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator
 *
 * Description:
 * SoC-facing host interface wrapper for QREM.
 * Bridges AXI interfaces to a generic internal Command/Status/Stream protocol.
 */

module host_if (
    input  logic        clk,
    input  logic        rst_n,

    // ============================================================
    // AXI4-Lite Slave Interface (Control / Status / Seeds)
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

    output logic        irq_o, // Interrupt on cmd_done

    // ============================================================
    // AXI4-Stream Interface (External SoC Side)
    // ============================================================
    // RX (LOAD: Host -> QREM)
    input  logic [63:0] s_axis_rx_tdata,
    input  logic        s_axis_rx_tvalid,
    output logic        s_axis_rx_tready,
    input  logic        s_axis_rx_tlast,

    // TX (STORE: QREM -> Host)
    output logic [63:0] m_axis_tx_tdata,
    output logic        m_axis_tx_tvalid,
    input  logic        m_axis_tx_tready,
    output logic        m_axis_tx_tlast,

    // ============================================================
    // Internal: Seed / Static Parameter Outputs
    // ============================================================
    output logic [255:0] seed_d_o,
    output logic [255:0] seed_z_o,
    output logic [255:0] seed_m_o,

    // ============================================================
    // Internal: Core Command Interface (Valid/Ready Handshake)
    // ============================================================
    output logic         cmd_valid_o,
    input  logic         cmd_ready_i,

    output logic [3:0]   cmd_alg_op_o,   // KEYGEN, ENCAPS, DECAPS
    output logic [3:0]   cmd_xfer_op_o,  // LD_EK, ST_C, etc.
    output logic [1:0]   cmd_sec_lvl_o,  // 00, 01, 10
    output logic [15:0]  cmd_xfer_len_o, // Byte count

    // Priority Control
    output logic         cmd_zeroize_o,  // Direct pulse, bypasses handshake

    // ============================================================
    // Internal: Core Status Interface
    // ============================================================
    input  logic         sts_busy_i,
    input  logic         sts_done_i,     // Pulse indicating completion
    input  logic [3:0]   sts_err_code_i,

    // ============================================================
    // Internal: Transcoder Stream Bridge
    // ============================================================
    // Path to Transcoder (LOAD ops)
    output logic [63:0]  tc_rx_tdata_o,
    output logic         tc_rx_tvalid_o,
    input  logic         tc_rx_tready_i,
    output logic         tc_rx_tlast_o,

    // Path from Transcoder (STORE ops)
    input  logic [63:0]  tc_tx_tdata_i,
    input  logic         tc_tx_tvalid_i,
    output logic         tc_tx_tready_o,
    input  logic         tc_tx_tlast_i
);

    // TODO: Register file logic, Pulse generation for cmd_valid_o,
    // and Stream gating logic based on cmd_xfer_op_o.

endmodule
