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
// Module Name   : ad9739_if
// ---------------------------------------------------------------------------------------
// Revision      : 1.0
// Description   : File Created
// ---------------------------------------------------------------------------------------
// Reuse Issue   :
// Synthesizable : Yes
// Instantiations: add dependencies here
// Clock Domains : clk
// Reset Strategy: sync reset
// Other :
// -FHEADER-------------------------------------------------------------------------------

// verilog_format: off
`resetall
`timescale 1ns / 1ps
`default_nettype none
// verilog_format: on

module ad9739_if #(
    parameter DATA_WIDTH      = 14,
    parameter FPGA_TECHNOLOGY = 1,
    parameter PARALLEL_NUMS   = 16,
    parameter HLOD_LAST       = 0
) (
    output wire dac_clk,
    input  wire dac_rst,  // sync with dac_clk

    // delay control
    input  wire       dly_load,  // sync with dac_clk
    input  wire [9:0] dly_cin,   // sync with dac_clk
    output wire [9:0] dly_cout,  // sync with dac_clk

    // axis slave interface
    input wire [(16*PARALLEL_NUMS)-1:0] dac_s_tdata,  // sync with dac_clk
    input wire                          dac_s_tvalid, // sync with dac_clk

    // dac lvds interface
    input  wire                    dac_dco_p,
    input  wire                    dac_dco_n,
    output wire                    dac_dci_p,
    output wire                    dac_dci_n,
    output wire [(DATA_WIDTH-1):0] dac_db0_p,
    output wire [(DATA_WIDTH-1):0] dac_db0_n,
    output wire [(DATA_WIDTH-1):0] dac_db1_p,
    output wire [(DATA_WIDTH-1):0] dac_db1_n
);

    wire        dac_clk_in;
    wire        dac_clk_in_s;
    wire        dac_div_clk;
    wire        dac_div_clk_s;

    wire [15:0] dac_data      [0:15];
    reg  [15:0] dac_data_i    [0:15];

    assign dac_clk = dac_div_clk;

    genvar ii;
    generate
        for (ii = 0; ii < 16; ii = ii + 1) begin
            always @(posedge dac_clk) begin
                if (dac_rst) begin
                    dac_data_i[ii] <= 16'h8000;
                end else begin
                    if (dac_s_tvalid) begin
                        dac_data_i[ii] <= dac_s_tdata[ii*16+:16];
                    end else begin
                        if (HLOD_LAST) begin
                            dac_data_i[ii] <= dac_data_i[ii];
                        end else begin
                            dac_data_i[ii] <= 16'h8000;
                        end
                    end
                end
            end
        end
    endgenerate

    IBUFGDS i_dac_clk_in_ibuf (
        .I (dac_dco_p),
        .IB(dac_dco_n),
        .O (dac_clk_in_s)
    );

    BUFG i_dac_clk_in_gbuf (
        .I(dac_clk_in_s),
        .O(dac_clk_in)
    );

    BUFR #(
        .BUFR_DIVIDE("4")
    ) i_dac_div_clk_rbuf (
        .CLR(1'b0),
        .CE (1'b1),
        .I  (dac_clk_in_s),
        .O  (dac_div_clk_s)
    );

    BUFG i_dac_div_clk_gbuf (
        .I(dac_div_clk_s),
        .O(dac_div_clk)
    );

    ad_serdes_out_dly #(
        .DDR_OR_SDR_N   (1),
        .DATA_WIDTH     (1),
        .SERDES_FACTOR  (8),
        .FPGA_TECHNOLOGY(FPGA_TECHNOLOGY)
    ) i_serdes_out_clk (
        .rst        (dac_rst),
        .clk        (dac_clk_in),
        .div_clk    (dac_div_clk),
        .dly_load   (dly_load),
        .dly_cin    (5'b00000),
        .dly_cout   (),
        .data_s0    (1'b1),
        .data_s1    (1'b0),
        .data_s2    (1'b1),
        .data_s3    (1'b0),
        .data_s4    (1'b1),
        .data_s5    (1'b0),
        .data_s6    (1'b1),
        .data_s7    (1'b0),
        .data_out_se(),
        .data_out_p (dac_dci_p),
        .data_out_n (dac_dci_n)
    );

    ad_serdes_out_dly #(
        .DDR_OR_SDR_N   (1),
        .DATA_WIDTH     (14),
        .SERDES_FACTOR  (8),
        .FPGA_TECHNOLOGY(FPGA_TECHNOLOGY)
    ) i_serdes_out_data_a (
        .rst        (dac_rst),
        .clk        (dac_clk_in),
        .div_clk    (dac_div_clk),
        .dly_load   (dly_load),
        .dly_cin    (dly_cin[0*5+:5]),
        .dly_cout   (dly_cout[0*5+:5]),
        .data_s0    (dac_data_i[0][15:2]),
        .data_s1    (dac_data_i[2][15:2]),
        .data_s2    (dac_data_i[4][15:2]),
        .data_s3    (dac_data_i[6][15:2]),
        .data_s4    (dac_data_i[8][15:2]),
        .data_s5    (dac_data_i[10][15:2]),
        .data_s6    (dac_data_i[12][15:2]),
        .data_s7    (dac_data_i[14][15:2]),
        .data_out_se(),
        .data_out_p (dac_db0_p),
        .data_out_n (dac_db0_n)
    );

    ad_serdes_out_dly #(
        .DDR_OR_SDR_N   (1),
        .DATA_WIDTH     (14),
        .SERDES_FACTOR  (8),
        .FPGA_TECHNOLOGY(FPGA_TECHNOLOGY)
    ) i_serdes_out_data_b (
        .rst        (dac_rst),
        .clk        (dac_clk_in),
        .div_clk    (dac_div_clk),
        .dly_load   (dly_load),
        .dly_cin    (dly_cin[1*5+:5]),
        .dly_cout   (dly_cout[1*5+:5]),
        .data_s0    (dac_data_i[1][15:2]),
        .data_s1    (dac_data_i[3][15:2]),
        .data_s2    (dac_data_i[5][15:2]),
        .data_s3    (dac_data_i[7][15:2]),
        .data_s4    (dac_data_i[9][15:2]),
        .data_s5    (dac_data_i[11][15:2]),
        .data_s6    (dac_data_i[13][15:2]),
        .data_s7    (dac_data_i[15][15:2]),
        .data_out_se(),
        .data_out_p (dac_db1_p),
        .data_out_n (dac_db1_n)
    );

endmodule

// verilog_format: off
`resetall
// verilog_format: on
