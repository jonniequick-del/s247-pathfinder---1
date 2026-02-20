// SPDX-License-Identifier: Apache-2.0
// S247-PATHFINDER-1 — Single Compute Core
// Fixed-point GPS arithmetic with sovereign ISA

`default_nettype none

module s247_compute_core #(
    parameter CORE_ID    = 0,
    parameter DATA_WIDTH = 32,
    parameter FRAC_BITS  = 16
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  enable,

    // GPS & Geofence inputs
    input  wire [DATA_WIDTH-1:0] gps_lat,
    input  wire [DATA_WIDTH-1:0] gps_lon,
    input  wire [DATA_WIDTH-1:0] fence_lat,
    input  wire [DATA_WIDTH-1:0] fence_lon,
    input  wire [DATA_WIDTH-1:0] fence_rad,

    // Outputs
    output reg  [DATA_WIDTH-1:0] result,
    output reg                   done,
    output reg                   halt
);

    // =========================================================================
    // ISA Opcodes
    // =========================================================================
    localparam [3:0] OP_NOP      = 4'h0;
    localparam [3:0] OP_PATHEVAL = 4'h1;  // A* f-cost
    localparam [3:0] OP_GEOCHECK = 4'h2;  // Geofence check
    localparam [3:0] OP_AESENC   = 4'h3;  // AES-256 (stub)
    localparam [3:0] OP_HALT     = 4'hF;  // Emergency freeze

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_FETCH    = 3'd1;
    localparam [2:0] S_DECODE   = 3'd2;
    localparam [2:0] S_EXECUTE  = 3'd3;
    localparam [2:0] S_WRITEBACK = 3'd4;
    localparam [2:0] S_HALTED   = 3'd5;

    reg [2:0] state;
    reg [3:0] opcode;
    reg [7:0] pc;  // Program counter

    // Register file — 8 general-purpose registers
    reg [DATA_WIDTH-1:0] regfile [0:7];

    // ALU intermediates
    reg signed [DATA_WIDTH-1:0] alu_a, alu_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result;
    reg [DATA_WIDTH-1:0] alu_out;

    // Geofence intermediates
    reg signed [DATA_WIDTH-1:0] delta_lat, delta_lon;
    reg [DATA_WIDTH-1:0] dist_sq, rad_sq;

    integer j;

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            pc       <= 8'h0;
            opcode   <= OP_NOP;
            result   <= 32'h0;
            done     <= 1'b0;
            halt     <= 1'b0;
            alu_out  <= 32'h0;
            delta_lat <= 32'h0;
            delta_lon <= 32'h0;
            dist_sq  <= 32'h0;
            rad_sq   <= 32'h0;
            mul_result <= 64'h0;
            alu_a    <= 32'h0;
            alu_b    <= 32'h0;
            for (j = 0; j < 8; j = j + 1)
                regfile[j] <= 32'h0;
        end else begin
            case (state)

                S_IDLE: begin
                    done <= 1'b0;
                    if (enable && !halt) begin
                        state <= S_FETCH;
                    end
                end

                S_FETCH: begin
                    // Load GPS data into register file
                    regfile[0] <= gps_lat;
                    regfile[1] <= gps_lon;
                    regfile[2] <= fence_lat;
                    regfile[3] <= fence_lon;
                    regfile[4] <= fence_rad;
                    opcode     <= OP_GEOCHECK; // Default operation
                    state      <= S_DECODE;
                end

                S_DECODE: begin
                    case (opcode)
                        OP_GEOCHECK: begin
                            alu_a <= $signed(regfile[0]) - $signed(regfile[2]);
                            alu_b <= $signed(regfile[1]) - $signed(regfile[3]);
                        end
                        OP_PATHEVAL: begin
                            alu_a <= $signed(regfile[0]);
                            alu_b <= $signed(regfile[1]);
                        end
                        default: begin
                            alu_a <= 32'h0;
                            alu_b <= 32'h0;
                        end
                    endcase
                    state <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    case (opcode)
                        OP_GEOCHECK: begin
                            // Distance² = Δlat² + Δlon² (fixed-point)
                            delta_lat <= alu_a;
                            delta_lon <= alu_b;
                            mul_result <= $signed(alu_a) * $signed(alu_a);
                            dist_sq <= (($signed(alu_a) * $signed(alu_a)) >>> FRAC_BITS) +
                                       (($signed(alu_b) * $signed(alu_b)) >>> FRAC_BITS);
                            mul_result <= $signed(regfile[4]) * $signed(regfile[4]);
                            rad_sq  <= ($signed(regfile[4]) * $signed(regfile[4])) >>> FRAC_BITS;
                            // Compare: if dist² > rad² → HALT
                            alu_out <= dist_sq; // store for writeback
                        end

                        OP_PATHEVAL: begin
                            // f(n) = g(n) + h(n) — simplified A*
                            mul_result <= $signed(alu_a) + $signed(alu_b);
                            alu_out <= alu_a + alu_b;
                        end

                        OP_HALT: begin
                            halt  <= 1'b1;
                            state <= S_HALTED;
                        end

                        default: begin
                            alu_out <= 32'h0;
                        end
                    endcase

                    if (opcode != OP_HALT)
                        state <= S_WRITEBACK;
                end

                S_WRITEBACK: begin
                    result <= alu_out;
                    regfile[5] <= alu_out;

                    // Geofence violation check
                    if (opcode == OP_GEOCHECK && dist_sq > rad_sq) begin
                        halt  <= 1'b1;
                        state <= S_HALTED;
                    end else begin
                        done  <= 1'b1;
                        pc    <= pc + 8'd1;
                        state <= S_IDLE;
                    end
                end

                S_HALTED: begin
                    // Non-overrideable freeze — only hard reset exits
                    halt   <= 1'b1;
                    done   <= 1'b0;
                    result <= 32'hDEAD_HALT;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule

`default_nettype wire
