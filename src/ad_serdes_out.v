// +FHEADER-------------------------------------------------------------------------------
// Copyright 2014 - 2017 (c) Analog Devices, Inc. All rights reserved.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <https://www.gnu.org/licenses/>.
// ---------------------------------------------------------------------------------------
// Author        :
// Module Name   : ad_serdes_out
// ---------------------------------------------------------------------------------------
// Revision      : 1.0
// Description   : File Created
// ---------------------------------------------------------------------------------------
// Synthesizable : Yes
// Clock Domains : clk
// Reset Strategy: sync reset
// -FHEADER-------------------------------------------------------------------------------

// verilog_format: off
`resetall
`timescale 1ns / 1ps
`default_nettype none
// verilog_format: on

module ad_serdes_out #(
    parameter FPGA_TECHNOLOGY = 0,
    parameter DDR_OR_SDR_N    = 1,
    parameter SERDES_FACTOR   = 8,
    parameter DATA_WIDTH      = 16
) (
    // reset and clocks
    input  wire                    rst,
    input  wire                    clk,
    input  wire                    div_clk,
    // data interface
    input  wire [(DATA_WIDTH-1):0] data_s0,
    input  wire [(DATA_WIDTH-1):0] data_s1,
    input  wire [(DATA_WIDTH-1):0] data_s2,
    input  wire [(DATA_WIDTH-1):0] data_s3,
    input  wire [(DATA_WIDTH-1):0] data_s4,
    input  wire [(DATA_WIDTH-1):0] data_s5,
    input  wire [(DATA_WIDTH-1):0] data_s6,
    input  wire [(DATA_WIDTH-1):0] data_s7,
    output wire [(DATA_WIDTH-1):0] data_out_se,
    output wire [(DATA_WIDTH-1):0] data_out_p,
    output wire [(DATA_WIDTH-1):0] data_out_n
);

    localparam SEVEN_SERIES = 1;
    localparam ULTRASCALE = 2;
    localparam ULTRASCALE_PLUS = 3;
    localparam DR_OQ_DDR = DDR_OR_SDR_N == 1'b1 ? "DDR" : "SDR";

    // internal signals

    wire [(DATA_WIDTH-1):0] data_out_s;
    wire [(DATA_WIDTH-1):0] serdes_shift1_s;
    wire [(DATA_WIDTH-1):0] serdes_shift2_s;

    assign data_out_se = data_out_s;

    // instantiations
    genvar l_inst;
    generate
        for (l_inst = 0; l_inst <= (DATA_WIDTH - 1); l_inst = l_inst + 1) begin : g_data

            if (FPGA_TECHNOLOGY == SEVEN_SERIES) begin
                OSERDESE2 #(
                    .DATA_RATE_OQ  (DR_OQ_DDR),
                    .DATA_RATE_TQ  ("SDR"),
                    .DATA_WIDTH    (SERDES_FACTOR),
                    .TRISTATE_WIDTH(1),
                    .SERDES_MODE   ("MASTER")
                ) i_serdes (
                    .D1       (data_s0[l_inst]),
                    .D2       (data_s1[l_inst]),
                    .D3       (data_s2[l_inst]),
                    .D4       (data_s3[l_inst]),
                    .D5       (data_s4[l_inst]),
                    .D6       (data_s5[l_inst]),
                    .D7       (data_s6[l_inst]),
                    .D8       (data_s7[l_inst]),
                    .T1       (1'b0),
                    .T2       (1'b0),
                    .T3       (1'b0),
                    .T4       (1'b0),
                    .SHIFTIN1 (1'b0),
                    .SHIFTIN2 (1'b0),
                    .SHIFTOUT1(),
                    .SHIFTOUT2(),
                    .OCE      (1'b1),
                    .CLK      (clk),
                    .CLKDIV   (div_clk),
                    .OQ       (data_out_s[l_inst]),
                    .TQ       (),
                    .OFB      (),
                    .TFB      (),
                    .TBYTEIN  (1'b0),
                    .TBYTEOUT (),
                    .TCE      (1'b0),
                    .RST      (rst)
                );
            end

            OBUFDS i_obuf (
                .I (data_out_s[l_inst]),
                .O (data_out_p[l_inst]),
                .OB(data_out_n[l_inst])
            );

        end
    endgenerate

endmodule

// verilog_format: off
`resetall
// verilog_format: on
