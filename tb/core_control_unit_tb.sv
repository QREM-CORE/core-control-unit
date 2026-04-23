`timescale 1ns / 1ps

import qrem_global_pkg::*;
import core_ctrl_pkg::*;

module core_control_unit_tb;
    logic clk;
    logic rst_n;

    logic        cmd_valid_i;
    logic        cmd_ready_o;
    logic [3:0]  cmd_opcode_i;
    logic [3:0]  cmd_mode_i;
    logic [1:0]  cmd_sec_lvl_i;
    logic [4:0]  cmd_payload_id_i;
    logic [15:0] cmd_xfer_len_i;
    logic        cmd_zeroize_i;

    logic       sts_busy_o;
    logic       sts_done_o;
    logic [3:0] sts_err_code_o;

    logic       tr_start_o;
    tr_opcode_t tr_opcode_o;
    logic       tr_done_i;
    logic [3:0] tr_err_i;

    logic                      hsu_start_o;
    hs_mode_t                  hsu_mode_o;
    logic [CTRL_XOF_LEN_W-1:0] hsu_xof_len_o;
    logic                      hsu_is_eta3_o;
    logic [POLY_ID_WIDTH-1:0]  hsu_poly_id_o;
    seed_id_e                  hsu_seed_id_o;
    logic [1:0]                hsu_input_sel_o;
    logic                      hsu_absorb_poly_o;
    logic                      hsu_absorb_last_o;
    logic                      hsu_done_i;
    logic                      hsu_packer_done_i;
    logic [3:0]                hsu_err_i;

    logic          pau_start_o;
    ctrl_pau_job_t pau_job_o;
    logic          pau_done_i;
    logic [3:0]    pau_err_i;

    logic            mem_zeroize_req_o;
    logic            mem_zeroize_done_i;
    logic            mem_fault_i;
    logic [2:0]      mem_fault_code_i;
    logic            hsu_hash_ek_read_en_o;
    ctrl_mem_phase_t mem_phase_o;

    int errors;

    core_control_unit dut (.*);

    always #5 clk = ~clk;

    // Optional VCD dumping for simulators that honor $dumpvars.
    // Questasim/vsim runs already emit a VCD via build-tools/common.mk.
    initial begin
`ifdef VERILATOR
        if ($test$plusargs("vcd")) begin
            $dumpfile("core_control_unit_tb.vcd");
            $dumpvars(0, core_control_unit_tb);
            $dumpvars(0, dut);
            $dumpvars(0, dut.u_kg_fsm);
        end
`elsif __ICARUS__
        if ($test$plusargs("vcd")) begin
            $dumpfile("core_control_unit_tb.vcd");
            $dumpvars(0, core_control_unit_tb);
            $dumpvars(0, dut);
            $dumpvars(0, dut.u_kg_fsm);
        end
`endif
    end

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

    task automatic reset_dut();
        begin
            clk = 1'b0;
            rst_n = 1'b0;
            cmd_valid_i = 1'b0;
            cmd_opcode_i = CMD_NOP;
            cmd_mode_i = MODE_NONE;
            cmd_sec_lvl_i = SEC_512;
            cmd_payload_id_i = PLD_NONE;
            cmd_xfer_len_i = 16'd0;
            cmd_zeroize_i = 1'b0;
            tr_done_i = 1'b0;
            tr_err_i = 4'h0;
            hsu_done_i = 1'b0;
            hsu_packer_done_i = 1'b0;
            hsu_err_i = 4'h0;
            pau_done_i = 1'b0;
            pau_err_i = 4'h0;
            mem_zeroize_done_i = 1'b0;
            mem_fault_i = 1'b0;
            mem_fault_code_i = 3'b000;
            tick(3);
            rst_n = 1'b1;
            tick(2);
            check(cmd_ready_o, "controller should be ready after reset");
        end
    endtask

    task automatic send_cmd(
        input logic [3:0] opcode,
        input logic [3:0] mode,
        input logic [1:0] sec_lvl,
        input logic [4:0] payload
    );
        begin
            while (!cmd_ready_o) tick(1);
            cmd_opcode_i = opcode;
            cmd_mode_i = mode;
            cmd_sec_lvl_i = sec_lvl;
            cmd_payload_id_i = payload;
            cmd_valid_i = 1'b1;
            tick(1);
            cmd_valid_i = 1'b0;
            cmd_opcode_i = CMD_NOP;
            cmd_mode_i = MODE_NONE;
            cmd_payload_id_i = PLD_NONE;
        end
    endtask

    task automatic expect_error(input logic [3:0] code, input string msg);
        int timeout;
        begin
            timeout = 0;
            while ((sts_err_code_o == CTRL_ERR_NONE) && (timeout < 20)) begin
                tick(1);
                timeout++;
            end
            check(sts_err_code_o == code, msg);
            tick(1);
        end
    endtask

    task automatic expect_done(input string msg);
        int timeout;
        begin
            timeout = 0;
            while (!sts_done_o && (timeout < 20)) begin
                tick(1);
                timeout++;
            end
            check(sts_done_o, msg);
            tick(1);
        end
    endtask

    task automatic expect_tr_start(input tr_opcode_t opcode);
        int timeout;
        begin
            timeout = 0;
            while (!tr_start_o && (timeout < 20)) begin
                tick(1);
                timeout++;
            end
            check(tr_start_o, "expected transcoder start");
            check(tr_opcode_o == opcode, "unexpected transcoder opcode");
        end
    endtask

    task automatic finish_tr();
        begin
            tick(1);
            tr_done_i = 1'b1;
            tick(1);
            tr_done_i = 1'b0;
        end
    endtask

    task automatic expect_hsu_start(
        input hs_mode_t mode,
        input logic [POLY_ID_WIDTH-1:0] poly_id,
        input seed_id_e seed_id
    );
        int timeout;
        begin
            timeout = 0;
            while (!hsu_start_o && (timeout < 40)) begin
                tick(1);
                timeout++;
            end
            check(hsu_start_o, "expected HSU start");
            check(hsu_mode_o == mode, "unexpected HSU mode");
            check(hsu_poly_id_o == poly_id, "unexpected HSU poly id");
            check(hsu_seed_id_o == seed_id, "unexpected HSU seed id");
        end
    endtask

    task automatic finish_hsu();
        begin
            tick(1);
            hsu_done_i = 1'b1;
            tick(1);
            hsu_done_i = 1'b0;
        end
    endtask

    task automatic expect_pau_start(
        input ctrl_pau_opcode_t opcode,
        input logic [POLY_ID_WIDTH-1:0] primary_poly,
        input logic [2:0] row
    );
        int timeout;
        begin
            timeout = 0;
            while (!pau_start_o && (timeout < 40)) begin
                tick(1);
                timeout++;
            end
            check(pau_start_o, "expected PAU start");
            check(pau_job_o.opcode == opcode, "unexpected PAU job opcode");
            check(pau_job_o.primary_poly_id == primary_poly, "unexpected PAU primary poly");
            check(pau_job_o.row_idx == row, "unexpected PAU row");
        end
    endtask

    task automatic finish_pau();
        begin
            tick(1);
            pau_done_i = 1'b1;
            tick(1);
            pau_done_i = 1'b0;
        end
    endtask

    task automatic expect_absorb_t(
        input logic [POLY_ID_WIDTH-1:0] poly_id,
        input logic is_last
    );
        int timeout;
        begin
            timeout = 0;
            while (!hsu_absorb_poly_o && (timeout < 40)) begin
                tick(1);
                timeout++;
            end
            check(hsu_hash_ek_read_en_o, "H(ek) read enable should be asserted");
            check(hsu_mode_o == MODE_ABSORB_POLY, "H(ek) should use MODE_ABSORB_POLY");
            check(hsu_poly_id_o == poly_id, "unexpected H(ek) T poly id");
            check(hsu_absorb_last_o == is_last, "unexpected H(ek) absorb_last");
            tick(1);
            hsu_packer_done_i = 1'b1;
            tick(1);
            hsu_packer_done_i = 1'b0;
        end
    endtask

    task automatic expect_absorb_rho();
        int timeout;
        begin
            timeout = 0;
            while (!hsu_absorb_poly_o && (timeout < 40)) begin
                tick(1);
                timeout++;
            end
            check(hsu_hash_ek_read_en_o, "H(ek) read enable should be asserted");
            check(hsu_mode_o == MODE_ABSORB_POLY, "H(ek) should use MODE_ABSORB_POLY");
            check(hsu_input_sel_o == HSU_IN_SEED, "rho absorb should use Seed RAM input");
            check(hsu_seed_id_o == SEED_ID_RHO, "rho absorb should select SEED_ID_RHO");
            check(hsu_absorb_last_o, "rho absorb should assert absorb_last");
            tick(1);
        end
    endtask

    task automatic run_keygen_k2();
        begin
            send_cmd(CMD_START, MODE_KEYGEN, SEC_512, PLD_NONE);

            expect_hsu_start(MODE_HASH_SHA3_512, '0, SEED_ID_RHO);
            finish_hsu();

            expect_hsu_start(MODE_SAMPLE_CBD, CTRL_POLY_S_BASE + POLY_ID_WIDTH'(0), SEED_ID_SIGMA);
            finish_hsu();
            expect_pau_start(PAU_JOB_NTT_IN_PLACE, CTRL_POLY_S_BASE + POLY_ID_WIDTH'(0), 3'd0);
            finish_pau();

            expect_hsu_start(MODE_SAMPLE_CBD, CTRL_POLY_S_BASE + POLY_ID_WIDTH'(1), SEED_ID_SIGMA);
            finish_hsu();
            expect_pau_start(PAU_JOB_NTT_IN_PLACE, CTRL_POLY_S_BASE + POLY_ID_WIDTH'(1), 3'd0);
            finish_pau();

            expect_hsu_start(MODE_SAMPLE_CBD, CTRL_POLY_EI, SEED_ID_SIGMA);
            finish_hsu();
            expect_pau_start(PAU_JOB_NTT_IN_PLACE, CTRL_POLY_EI, 3'd0);
            finish_pau();
            expect_hsu_start(MODE_SAMPLE_NTT, CTRL_POLY_A_BASE + POLY_ID_WIDTH'(0), SEED_ID_RHO);
            finish_hsu();
            expect_hsu_start(MODE_SAMPLE_NTT, CTRL_POLY_A_BASE + POLY_ID_WIDTH'(1), SEED_ID_RHO);
            finish_hsu();
            expect_pau_start(PAU_JOB_KEYGEN_ROWMAC, CTRL_POLY_T_BASE + POLY_ID_WIDTH'(0), 3'd0);
            finish_pau();

            expect_hsu_start(MODE_SAMPLE_CBD, CTRL_POLY_EI, SEED_ID_SIGMA);
            finish_hsu();
            expect_pau_start(PAU_JOB_NTT_IN_PLACE, CTRL_POLY_EI, 3'd0);
            finish_pau();
            expect_hsu_start(MODE_SAMPLE_NTT, CTRL_POLY_A_BASE + POLY_ID_WIDTH'(0), SEED_ID_RHO);
            finish_hsu();
            expect_hsu_start(MODE_SAMPLE_NTT, CTRL_POLY_A_BASE + POLY_ID_WIDTH'(1), SEED_ID_RHO);
            finish_hsu();
            expect_pau_start(PAU_JOB_KEYGEN_ROWMAC, CTRL_POLY_T_BASE + POLY_ID_WIDTH'(1), 3'd1);
            finish_pau();

            expect_hsu_start(MODE_ABSORB_POLY, '0, SEED_ID_HEK);
            expect_absorb_t(CTRL_POLY_T_BASE + POLY_ID_WIDTH'(0), 1'b0);
            expect_absorb_t(CTRL_POLY_T_BASE + POLY_ID_WIDTH'(1), 1'b0);
            expect_absorb_rho();
            finish_hsu();
            expect_done("KeyGen should finish after H(ek)");
        end
    endtask

    initial begin
        errors = 0;
        reset_dut();

        send_cmd(CMD_START, MODE_KEYGEN, SEC_512, PLD_NONE);
        expect_error(CTRL_ERR_PRECONDITION, "KeyGen without d should raise precondition error");

        send_cmd(CMD_LOAD, MODE_KEYGEN, SEC_512, PLD_D);
        expect_tr_start(TR_OP_KG_INGEST_D);
        finish_tr();
        expect_done("LOAD d should complete");

        run_keygen_k2();

        send_cmd(CMD_STORE, MODE_KEYGEN, SEC_512, PLD_HEK);
        expect_tr_start(TR_OP_KG_EXPORT_HEK);
        finish_tr();
        expect_done("STORE H(ek) should complete");

        send_cmd(CMD_LOAD, MODE_KEYGEN, SEC_512, PLD_D);
        expect_tr_start(TR_OP_KG_INGEST_D);
        tick(1);
        tr_err_i = 4'h1;
        expect_error(CTRL_ERR_TRANSCODER, "transcoder error should propagate");
        tr_err_i = 4'h0;

        send_cmd(CMD_LOAD, MODE_KEYGEN, SEC_512, PLD_D);
        expect_tr_start(TR_OP_KG_INGEST_D);
        cmd_zeroize_i = 1'b1;
        tick(1);
        cmd_zeroize_i = 1'b0;
        check(mem_zeroize_req_o, "zeroize should request memory wipe");
        mem_zeroize_done_i = 1'b1;
        tick(1);
        mem_zeroize_done_i = 1'b0;
        expect_done("zeroize should complete when memory wipe completes");

        send_cmd(CMD_START, MODE_KEYGEN, SEC_512, PLD_NONE);
        expect_error(CTRL_ERR_PRECONDITION, "zeroize should clear loaded d state");

        send_cmd(CMD_START, MODE_ENCAPS, SEC_512, PLD_NONE);
        expect_error(CTRL_ERR_UNSUPPORTED, "Encaps skeleton should report unsupported");

        if (errors == 0) begin
            $display("core_control_unit_tb PASSED");
        end
        else begin
            $fatal(1, "core_control_unit_tb FAILED with %0d errors", errors);
        end
        $finish;
    end
endmodule
