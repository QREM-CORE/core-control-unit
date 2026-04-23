`timescale 1ns / 1ps

import core_ctrl_pkg::*;

// Focused host_if regression: composite KeyGen TX framing.
//
// Key signals to inspect in waves:
//   - dut.phase_r, dut.tx_bytes_r, dut.cmd_*_lat_r
//   - tc_tx_tvalid_i / tc_tx_tready_o / tc_tx_tlast_i (from "transcoder")
//   - m_axis_tx_tvalid / m_axis_tx_tready / m_axis_tx_tlast (to "host")
//   - dut.err_sticky_r / dut.err_code_r
module host_if_tb;
    logic clk;
    logic rst_n;

    // AXI4-Lite (minimal single-beat driver)
    logic [7:0]  s_axi_awaddr;
    logic        s_axi_awvalid;
    logic        s_axi_awready;
    logic [31:0] s_axi_wdata;
    logic [3:0]  s_axi_wstrb;
    logic        s_axi_wvalid;
    logic        s_axi_wready;
    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready;
    logic [7:0]  s_axi_araddr;
    logic        s_axi_arvalid;
    logic        s_axi_arready;
    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready;

    logic irq_o;

    // External AXI-stream RX/TX (host side)
    logic [63:0] s_axis_rx_tdata;
    logic        s_axis_rx_tvalid;
    logic        s_axis_rx_tready;
    logic        s_axis_rx_tlast;

    logic [63:0] m_axis_tx_tdata;
    logic        m_axis_tx_tvalid;
    logic        m_axis_tx_tready;
    logic        m_axis_tx_tlast;

    logic [255:0] seed_d_o;
    logic [255:0] seed_z_o;
    logic [255:0] seed_m_o;

    // Command/status handshake to controller (stubbed)
    logic        cmd_valid_o;
    logic        cmd_ready_i;
    logic [3:0]  cmd_opcode_o;
    logic [3:0]  cmd_mode_o;
    logic [1:0]  cmd_sec_lvl_o;
    logic [4:0]  cmd_payload_id_o;
    logic [15:0] cmd_xfer_len_o;
    logic        cmd_zeroize_o;

    logic        sts_busy_i;
    logic        sts_done_i;
    logic [3:0]  sts_err_code_i;

    // Transcoder stream bridge (we act as the "transcoder" for TX)
    logic [63:0] tc_rx_tdata_o;
    logic        tc_rx_tvalid_o;
    logic        tc_rx_tready_i;
    logic        tc_rx_tlast_o;

    logic [63:0] tc_tx_tdata_i;
    logic        tc_tx_tvalid_i;
    logic        tc_tx_tready_o;
    logic        tc_tx_tlast_i;

    int errors;

    host_if dut (.*);

    always #5 clk = ~clk;

`ifdef VERILATOR
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("host_if_tb.vcd");
            $dumpvars(0, host_if_tb);
            $dumpvars(0, dut);
        end
    end
`elsif __ICARUS__
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("host_if_tb.vcd");
            $dumpvars(0, host_if_tb);
            $dumpvars(0, dut);
        end
    end
`endif

    task automatic check(input logic cond, input string msg);
        begin
            if (!cond) begin
                $error("%s", msg);
                errors++;
            end
        end
    endtask

    task automatic tick(input int n);
        begin
            repeat (n) @(posedge clk);
            #1;
        end
    endtask

    // Minimal AXI4-Lite single write (no bursts, no reordering).
    task automatic axi_write(
        input logic [7:0]  addr,
        input logic [31:0] data,
        input logic [3:0]  strb
    );
        begin
            s_axi_awaddr  = addr;
            s_axi_awvalid = 1'b1;
            s_axi_wdata   = data;
            s_axi_wstrb   = strb;
            s_axi_wvalid  = 1'b1;

            // Wait for both channels to accept.
            while (!(s_axi_awready && s_axi_wready)) tick(1);
            tick(1);
            s_axi_awvalid = 1'b0;
            s_axi_wvalid  = 1'b0;

            // Wait for response.
            while (!s_axi_bvalid) tick(1);
            tick(1);
        end
    endtask

    task automatic launch_cmd(
        input logic [3:0] opcode,
        input logic [3:0] mode,
        input logic [1:0] sec_lvl,
        input logic [4:0] payload_id
    );
        logic [31:0] cmd_img;
        begin
            // command_reg_image format: [3:0]=opcode, [7:4]=mode, [9:8]=sec, [14:10]=payload
            cmd_img = 32'h0;
            cmd_img[3:0]   = opcode;
            cmd_img[7:4]   = mode;
            cmd_img[9:8]   = sec_lvl;
            cmd_img[14:10] = payload_id;

            axi_write(8'h04, cmd_img, 4'hF); // ADDR_COMMAND
            axi_write(8'h00, 32'h0000_0001, 4'h1); // ADDR_CONTROL, START bit

            // Wait until host_if has entered TX (tc_tx_tready_o mirrors host ready in PH_TX).
            while (!tc_tx_tready_o) tick(1);
        end
    endtask

    // Drive a fixed number of TX beats from the transcoder side, with TLAST
    // asserted on up to four selected beat indices (1-based).
    task automatic drive_tx_stream(
        input int total_beats,
        input int tlast_a,
        input int tlast_b,
        input int tlast_c,
        input int tlast_d
    );
        int beat;
        begin
            tc_tx_tvalid_i = 1'b1;
            tc_tx_tdata_i  = 64'h0;
            tc_tx_tlast_i  = 1'b0;

            for (beat = 1; beat <= total_beats; beat++) begin
                tc_tx_tdata_i = 64'(beat);
                tc_tx_tlast_i = (beat == tlast_a) ||
                                (beat == tlast_b) ||
                                (beat == tlast_c) ||
                                (beat == tlast_d);

                // Wait for handshake (host ready is always 1 here).
                while (!tc_tx_tready_o) tick(1);

                // Composite framing expectation: host TLAST only at final beat.
                // Sample before the next posedge updates host_if counters.
                #1;
                if (beat != total_beats) begin
                    check(!m_axis_tx_tlast, "host TLAST should be suppressed before final beat");
                end
                else begin
                    check(m_axis_tx_tlast, "host TLAST should assert on final beat");
                end

                tick(1);
            end

            tc_tx_tvalid_i = 1'b0;
            tc_tx_tlast_i  = 1'b0;
            tick(2);
        end
    endtask

    task automatic reset_dut;
        begin
            clk = 1'b0;
            rst_n = 1'b0;

            s_axi_awaddr  = 8'h00;
            s_axi_awvalid = 1'b0;
            s_axi_wdata   = 32'h0;
            s_axi_wstrb   = 4'h0;
            s_axi_wvalid  = 1'b0;
            s_axi_bready  = 1'b1;
            s_axi_araddr  = 8'h00;
            s_axi_arvalid = 1'b0;
            s_axi_rready  = 1'b1;

            s_axis_rx_tdata  = 64'h0;
            s_axis_rx_tvalid = 1'b0;
            s_axis_rx_tlast  = 1'b0;

            m_axis_tx_tready = 1'b1;

            cmd_ready_i   = 1'b1;
            sts_busy_i    = 1'b0;
            sts_done_i    = 1'b0;
            sts_err_code_i = 4'h0;

            tc_rx_tready_i = 1'b1;

            tc_tx_tdata_i  = 64'h0;
            tc_tx_tvalid_i = 1'b0;
            tc_tx_tlast_i  = 1'b0;

            tick(4);
            rst_n = 1'b1;
            tick(2);
        end
    endtask

    initial begin
        errors = 0;
        reset_dut();

        // ------------------------------------------------------------
        // Test 1: KeyGen EK store (k=2) composite TLAST suppression
        //   expected_len = 800B = 100 beats
        //   allow intermediate TLAST at 768B = 96 beats, final at 100
        // ------------------------------------------------------------
        launch_cmd(CMD_STORE, MODE_KEYGEN, SEC_512, PLD_EK);
        check(cmd_xfer_len_o == 16'd800, "EK expected length should be 800B for SEC_512");
        drive_tx_stream(100, 96, 100, 0, 0);
        check(!dut.err_sticky_r, "EK composite TX should not raise an error");

        // ------------------------------------------------------------
        // Test 2: KeyGen DK store (dk_partial, k=2) composite TLAST suppression
        //   expected_len = 1600B = 200 beats
        //   allow intermediate TLAST at:
        //     768B (96), 1536B (192), 1568B (196), final 1600B (200)
        // ------------------------------------------------------------
        launch_cmd(CMD_STORE, MODE_KEYGEN, SEC_512, PLD_DK);
        check(cmd_xfer_len_o == 16'd1600, "DK_PARTIAL expected length should be 1600B for SEC_512");
        drive_tx_stream(200, 96, 192, 196, 200);
        check(!dut.err_sticky_r, "DK composite TX should not raise an error");

        if (errors == 0) begin
            $display("host_if_tb PASSED");
        end
        else begin
            $fatal(1, "host_if_tb FAILED with %0d errors", errors);
        end
        $finish;
    end
endmodule
