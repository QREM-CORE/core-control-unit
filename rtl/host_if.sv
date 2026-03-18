/*
 * Module Name: host_if.sv
 * Author(s): Quardin Lyttle
 * Target: FIPS 203 (ML-KEM) Hardware Accelerator
 *
 * Reference: QREM Architecture Specification
 *
 * Description: Top-level wrapper for the control subsystem. 
 * Instantiates the host interface (CSRs) and the main protocol FSM.
 * Acts as the primary orchestrator for the Hash, PAU, Transcoder, and Memory.
 */

module host_if (
    input logic clk,
    input logic rst_n
    // TODO: Add actual AXI/APB ports later
);
endmodule