/*
 * Module Name: main_fsm.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator
 *
 * Reference: QREM Architecture Specification
 *
 * Description:
 * Legacy placeholder. The macro-sequencer now lives directly in
 * core_control_unit.sv. Reintroduce this module only if there is a real,
 * documented split between an outer controller and an inner protocol FSM.
 */

import core_ctrl_pkg::*;

module main_fsm (
    input logic clk,
    input logic rst_n
);
    // Intentionally empty.
    logic unused_clk;
    logic unused_rst_n;

    assign unused_clk   = clk;
    assign unused_rst_n = rst_n;
endmodule
