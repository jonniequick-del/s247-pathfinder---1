// SPDX-License-Identifier: Apache-2.0
// S247-PATHFINDER-1 — Top-level sovereign GPU
// 8-core parallel architecture with Wishbone slave interface

`default_nettype none

module s247_pathfinder_top #(
    parameter NUM_CORES   = 8,
    parameter DATA_WIDTH  = 32,
    parameter ADDR_WIDTH  = 32,
    parameter FRAC_BITS   = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Wishbone Slave Interface
    input  wire                  wb_clk_i,
    input  wire                  wb_rst_i,
    input  wire                  wbs_stb_i,
    input  wire                  wbs_cyc_i,
    input  wire                  wbs_we_i,
    input  wire [3:0]            wbs_sel_i,
    input  wire [DATA_WIDTH-1:0] wbs_dat_i,
    input  wire [ADDR_WIDTH-1:0] wbs_adr_i,
    output reg                   wbs_ack_o,
    output reg  [DATA_WIDTH-1:0] wbs_dat_o,

    // Logic Analyzer
    input  wire [63:0]           la_data_in,
    output wire [63:0]           la_data_out,
    input  wire [63:0]           la_oenb,

    // GPIO
    input  wire [15:0]           io_in,
    output wire [15:0]           io_out,
    output wire [15:0]           io_oeb
);

    // =========================================================================
    // Address Map (active base = 0x3000_0000)
    // =========================================================================
    localparam BASE_ADDR      = 32'h3000_0000;
    localparam ADDR_CTRL      = BASE_ADDR + 32'h00;  // Control register
    localparam ADDR_STATUS    = BASE_ADDR + 32'h04;  // Status register
    localparam ADDR_GPS_LAT   = BASE_ADDR + 32'h08;  // GPS latitude
    localparam ADDR_GPS_LON   = BASE_ADDR + 32'h0C;  // GPS longitude
    localparam ADDR_FENCE_LAT = BASE_ADDR + 32'h10;  // Geofence center lat
    localparam ADDR_FENCE_LON = BASE_ADDR + 32'h14;  // Geofence center lon
    localparam ADDR_FENCE_RAD = BASE_ADDR + 32'h18;  // Geofence radius
    localparam ADDR_RESULT    = BASE_ADDR + 32'h1C;  // Result readback

    // =========================================================================
    // Registers
    // =========================================================================
    reg [DATA_WIDTH-1:0] reg_ctrl;
    reg [DATA_WIDTH-1:0] reg_gps_lat;
    reg [DATA_WIDTH-1:0] reg_gps_lon;
    reg [DATA_WIDTH-1:0] reg_fence_lat;
    reg [DATA_WIDTH-1:0] reg_fence_lon;
    reg [DATA_WIDTH-1:0] reg_fence_rad;

    // Core interface signals
    wire [DATA_WIDTH-1:0] core_result [0:NUM_CORES-1];
    wire [NUM_CORES-1:0]  core_done;
    wire [NUM_CORES-1:0]  core_halt;
    reg  [NUM_CORES-1:0]  core_enable;

    // Emergency halt — OR of all core halts
    wire emergency_halt = |core_halt;

    // =========================================================================
    // Wishbone Slave — Register Read/Write
    // =========================================================================
    wire wb_valid = wbs_stb_i && wbs_cyc_i;

    always @(posedge wb_clk_i or posedge wb_rst_i) begin
        if (wb_rst_i) begin
            wbs_ack_o    <= 1'b0;
            wbs_dat_o    <= 32'h0;
            reg_ctrl     <= 32'h0;
            reg_gps_lat  <= 32'h0;
            reg_gps_lon  <= 32'h0;
            reg_fence_lat <= 32'h0;
            reg_fence_lon <= 32'h0;
            reg_fence_rad <= 32'h0;
            core_enable  <= {NUM_CORES{1'b0}};
        end else begin
            wbs_ack_o <= 1'b0;

            if (wb_valid && !wbs_ack_o) begin
                wbs_ack_o <= 1'b1;

                if (wbs_we_i) begin
                    // Write
                    case (wbs_adr_i)
                        ADDR_CTRL:      begin
                            reg_ctrl <= wbs_dat_i;
                            core_enable <= wbs_dat_i[NUM_CORES-1:0];
                        end
                        ADDR_GPS_LAT:   reg_gps_lat   <= wbs_dat_i;
                        ADDR_GPS_LON:   reg_gps_lon   <= wbs_dat_i;
                        ADDR_FENCE_LAT: reg_fence_lat <= wbs_dat_i;
                        ADDR_FENCE_LON: reg_fence_lon <= wbs_dat_i;
                        ADDR_FENCE_RAD: reg_fence_rad <= wbs_dat_i;
                        default: ;
                    endcase
                end else begin
                    // Read
                    case (wbs_adr_i)
                        ADDR_CTRL:    wbs_dat_o <= reg_ctrl;
                        ADDR_STATUS:  wbs_dat_o <= {16'h0, core_halt, core_done};
                        ADDR_GPS_LAT: wbs_dat_o <= reg_gps_lat;
                        ADDR_GPS_LON: wbs_dat_o <= reg_gps_lon;
                        ADDR_RESULT:  wbs_dat_o <= core_result[0];
                        default:      wbs_dat_o <= 32'hDEAD_BEEF;
                    endcase
                end
            end

            // Emergency halt override — disable all cores
            if (emergency_halt) begin
                core_enable <= {NUM_CORES{1'b0}};
            end
        end
    end

    // =========================================================================
    // Logic Analyzer — expose core status
    // =========================================================================
    assign la_data_out = {32'h0, 16'h0, core_halt, core_done};

    // =========================================================================
    // GPIO — directly active active active directly directly directly directly directly directly directly directly
    // active directly directly directly directly pass-through directly directly directly directly directly directly
    // =========================================================================
    assign io_out = {8'h0, emergency_halt, 7'h0};
    assign io_oeb = 16'h00FF; // lower 8 = input, upper 8 = output

    // =========================================================================
    // Compute Core Instances
    // =========================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i = i + 1) begin : core_gen
            s247_compute_core #(
                .CORE_ID(i),
                .DATA_WIDTH(DATA_WIDTH),
                .FRAC_BITS(FRAC_BITS)
            ) core_inst (
                .clk       (clk),
                .rst_n     (rst_n),
                .enable    (core_enable[i]),
                .gps_lat   (reg_gps_lat),
                .gps_lon   (reg_gps_lon),
                .fence_lat (reg_fence_lat),
                .fence_lon (reg_fence_lon),
                .fence_rad (reg_fence_rad),
                .result    (core_result[i]),
                .done      (core_done[i]),
                .halt      (core_halt[i])
            );
        end
    endgenerate

endmodule

`default_nettype wire
