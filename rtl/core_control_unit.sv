/*
 * Module Name: core_control_unit.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator
 *
 * Reference: QREM Architecture Specification
 *
 * Description: Top-level wrapper for the control subsystem.
 * Instantiates the host interface (CSRs) and the main protocol FSM.
 * Acts as the primary orchestrator for the Hash, PAU, Transcoder, and Memory.
 */

import qrem_global_pkg::*;
import core_ctrl_pkg::*;

module core_control_unit (
    input  logic clk,
    input  logic rst_n

    // Host Interface (e.g., AXI-Lite or Custom)
    // TODO: Define host interface ports

    // Subsystem Command Interfaces (Ready/Valid)
    // TODO: PAU interface
    // TODO: Hash/Sampler interface
    // TODO: Memory access interface
    // TODO: Transcoder interface
);

    // Internal signals between Host IF and Main FSM
    // logic [1:0] op_mode;      // e.g., 00: Idle, 01: KeyGen, 10: Encaps, 11: Decaps
    // logic [1:0] security_lvl; // e.g., k=2, k=3, k=4
    // logic       start_cmd;
    // logic       done_status;

    // Instantiate Host Interface
    host_if u_host_if (
        .clk(clk),
        .rst_n(rst_n)
        // ... port maps ...
    );

    // Instantiate Main FSM
    main_fsm u_main_fsm (
        .clk(clk),
        .rst_n(rst_n)
        // ... port maps ...
    );

endmodule