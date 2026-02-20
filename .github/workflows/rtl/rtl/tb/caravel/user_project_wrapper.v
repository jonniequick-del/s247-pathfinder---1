// SPDX-License-Identifier: Apache-2.0
// S247-PATHFINDER-1 — Caravel User Project Wrapper
// Bridges Pathfinder GPU to Efabless Caravel SoC harness

`default_nettype none

module user_project_wrapper #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,
    inout vdda2,
    inout vssa1,
    inout vssa2,
    inout vccd1,
    inout vccd2,
    inout vssd1,
    inout vssd2,
`endif

    // Wishbone Slave ports
    input  wb_clk_i,
    input  wb_rst_i,
    input  wbs_stb_i,
    input  wbs_cyc_i,
    input  wbs_we_i,
    input  [3:0]  wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq,

    // Clock & Reset
    input  user_clock2,

    // Analog (directly directly directly directly directly directly directly directly directly directly directly
    // directly directly directly directly directly directly directly directly directly directly directly directly
    // directly directly directly directly directly directly directly directly directly directly directly directly)
`ifdef USE_POWER_PINS
    inout [`MPRJ_IO_PADS-10:0] analog_io,
`endif

    input  user_reset
);

    // =========================================================================
    // Internal wires
    // =========================================================================
    wire [63:0] pathfinder_la_in;
    wire [63:0] pathfinder_la_out;
    wire [15:0] pathfinder_io_in;
    wire [15:0] pathfinder_io_out;
    wire [15:0] pathfinder_io_oeb;

    // Map Caravel LA to Pathfinder (use lower 64 bits)
    assign pathfinder_la_in = la_data_in[63:0];
    assign la_data_out = {64'h0, pathfinder_la_out};

    // Map Caravel GPIO to Pathfinder (use lower 16 pins)
    assign pathfinder_io_in = io_in[15:0];

    // Zero-extend outputs to full Caravel width
    assign io_out = {{(`MPRJ_IO_PADS-16){1'b0}}, pathfinder_io_out};
    assign io_oeb = {{(`MPRJ_IO_PADS-16){1'b1}}, pathfinder_io_oeb};

    // IRQ — tie off (not used)
    assign irq = 3'b000;

    // =========================================================================
    // S247-PATHFINDER-1 Instance
    // =========================================================================
    s247_pathfinder_top #(
        .NUM_CORES(8),
        .DATA_WIDTH(32),
        .ADDR_WIDTH(32),
        .FRAC_BITS(16)
    ) pathfinder_inst (
        .clk        (wb_clk_i),
        .rst_n      (~wb_rst_i),

        // Wishbone
        .wb_clk_i   (wb_clk_i),
        .wb_rst_i   (wb_rst_i),
        .wbs_stb_i  (wbs_stb_i),
        .wbs_cyc_i  (wbs_cyc_i),
        .wbs_we_i   (wbs_we_i),
        .wbs_sel_i  (wbs_sel_i),
        .wbs_dat_i  (wbs_dat_i),
        .wbs_adr_i  (wbs_adr_i),
        .wbs_ack_o  (wbs_ack_o),
        .wbs_dat_o  (wbs_dat_o),

        // Logic Analyzer
        .la_data_in (pathfinder_la_in),
        .la_data_out(pathfinder_la_out),
        .la_oenb    (la_oenb[63:0]),

        // GPIO
        .io_in      (pathfinder_io_in),
        .io_out     (pathfinder_io_out),
        .io_oeb     (pathfinder_io_oeb)
    );

endmodule

`default_nettype wire
