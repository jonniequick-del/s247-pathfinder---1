// SPDX-License-Identifier: Apache-2.0
// S247-PATHFINDER-1 — Testbench
// Verifies geofence check and emergency halt

`timescale 1ns / 1ps

module tb_pathfinder;

    // =========================================================================
    // Signals
    // =========================================================================
    reg         clk;
    reg         rst_n;
    reg         wb_clk_i;
    reg         wb_rst_i;
    reg         wbs_stb_i;
    reg         wbs_cyc_i;
    reg         wbs_we_i;
    reg  [3:0]  wbs_sel_i;
    reg  [31:0] wbs_dat_i;
    reg  [31:0] wbs_adr_i;
    wire        wbs_ack_o;
    wire [31:0] wbs_dat_o;

    wire [63:0] la_data_out;
    reg  [63:0] la_data_in;
    reg  [63:0] la_oenb;

    wire [15:0] io_out;
    wire [15:0] io_oeb;
    reg  [15:0] io_in;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    s247_pathfinder_top #(
        .NUM_CORES(8),
        .DATA_WIDTH(32),
        .ADDR_WIDTH(32),
        .FRAC_BITS(16)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
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
        .la_data_in (la_data_in),
        .la_data_out(la_data_out),
        .la_oenb    (la_oenb),
        .io_in      (io_in),
        .io_out     (io_out),
        .io_oeb     (io_oeb)
    );

    // =========================================================================
    // Clock Generation — 50 MHz (20ns period)
    // =========================================================================
    initial clk = 0;
    always #10 clk = ~clk;

    initial wb_clk_i = 0;
    always #10 wb_clk_i = ~wb_clk_i;

    // =========================================================================
    // Wishbone Write Task
    // =========================================================================
    task wb_write(input [31:0] addr, input [31:0] data);
        begin
            @(posedge wb_clk_i);
            wbs_stb_i <= 1'b1;
            wbs_cyc_i <= 1'b1;
            wbs_we_i  <= 1'b1;
            wbs_sel_i <= 4'hF;
            wbs_adr_i <= addr;
            wbs_dat_i <= data;
            @(posedge wb_clk_i);
            wait(wbs_ack_o);
            @(posedge wb_clk_i);
            wbs_stb_i <= 1'b0;
            wbs_cyc_i <= 1'b0;
            wbs_we_i  <= 1'b0;
        end
    endtask

    // =========================================================================
    // Wishbone Read Task
    // =========================================================================
    task wb_read(input [31:0] addr, output [31:0] data);
        begin
            @(posedge wb_clk_i);
            wbs_stb_i <= 1'b1;
            wbs_cyc_i <= 1'b1;
            wbs_we_i  <= 1'b0;
            wbs_sel_i <= 4'hF;
            wbs_adr_i <= addr;
            @(posedge wb_clk_i);
            wait(wbs_ack_o);
            data = wbs_dat_o;
            @(posedge wb_clk_i);
            wbs_stb_i <= 1'b0;
            wbs_cyc_i <= 1'b0;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================
    reg [31:0] read_data;

    initial begin
        $dumpfile("tb_pathfinder.vcd");
        $dumpvars(0, tb_pathfinder);

        // Initialize
        rst_n     = 0;
        wb_rst_i  = 1;
        wbs_stb_i = 0;
        wbs_cyc_i = 0;
        wbs_we_i  = 0;
        wbs_sel_i = 0;
        wbs_dat_i = 0;
        wbs_adr_i = 0;
        la_data_in = 64'h0;
        la_oenb    = 64'h0;
        io_in      = 16'h0;

        // Release reset
        #100;
        rst_n    = 1;
        wb_rst_i = 0;
        #40;

        $display("========================================");
        $display("S247-PATHFINDER-1 TESTBENCH START");
        $display("========================================");

        // ----- TEST 1: Inside Geofence (should NOT halt) -----
        $display("\n[TEST 1] GPS inside geofence — expect NO halt");

        // GPS position: lat=10.0, lon=20.0 (Q16.16)
        wb_write(32'h3000_0008, 32'h000A_0000);  // lat = 10.0
        wb_write(32'h3000_000C, 32'h0014_0000);  // lon = 20.0

        // Geofence center: lat=10.0, lon=20.0, radius=5.0
        wb_write(32'h3000_0010, 32'h000A_0000);  // fence_lat = 10.0
        wb_write(32'h3000_0014, 32'h0014_0000);  // fence_lon = 20.0
        wb_write(32'h3000_0018, 32'h0005_0000);  // fence_rad = 5.0

        // Enable core 0
        wb_write(32'h3000_0000, 32'h0000_0001);

        // Wait for processing
        #500;

        // Read status
        wb_read(32'h3000_0004, read_data);
        $display("  Status register: 0x%08h", read_data);
        if (read_data[8] == 1'b0)
            $display("  PASS — No halt triggered");
        else
            $display("  FAIL — Unexpected halt!");

        // ----- TEST 2: Outside Geofence (SHOULD halt) -----
        $display("\n[TEST 2] GPS outside geofence — expect HALT");

        // Reset
        rst_n = 0; #40; rst_n = 1; #40;

        // GPS position: lat=50.0, lon=60.0
        wb_write(32'h3000_0008, 32'h0032_0000);  // lat = 50.0
        wb_write(32'h3000_000C, 32'h003C_0000);  // lon = 60.0

        // Geofence center: lat=10.0, lon=20.0, radius=5.0
        wb_write(32'h3000_0010, 32'h000A_0000);
        wb_write(32'h3000_0014, 32'h0014_0000);
        wb_write(32'h3000_0018, 32'h0005_0000);

        // Enable core 0
        wb_write(32'h3000_0000, 32'h0000_0001);

        #500;

        wb_read(32'h3000_0004, read_data);
        $display("  Status register: 0x%08h", read_data);
        if (read_data[8] == 1'b1)
            $display("  PASS — Halt correctly triggered");
        else
            $display("  FAIL — Halt NOT triggered!");

        // ----- TEST 3: Read result register -----
        $display("\n[TEST 3] Read result register");
        wb_read(32'h3000_001C, read_data);
        $display("  Result: 0x%08h", read_data);

        $display("\n========================================");
        $display("S247-PATHFINDER-1 TESTBENCH COMPLETE");
        $display("========================================");

        #100;
        $finish;
    end

endmodule
