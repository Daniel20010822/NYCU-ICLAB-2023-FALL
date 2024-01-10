//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Lab04 Exercise		: Siamese Neural Network
//   Author     		: Hsien-Chi Peng (jhpeng2012@gmail.com)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : SNN.v
//   Module Name : SNN
//   Release version : V1.0 (Release Date: 2023-10)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

// synopsys translate_off
`ifdef RTL
	`include "GATED_OR.v"
`else
	`include "Netlist/GATED_OR_SYN.v"
`endif
// synopsys translate_on


module SNN(
    //Input Port
    clk,
    rst_n,
    cg_en,
    in_valid,
    Img,
    Kernel,
	Weight,
    Opt,

    //Output Port
    out_valid,
    out
    );


//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------

// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

// Constants
parameter DEPTH = 3;
parameter IMG_WIDTH     = 4;
parameter KERNEL_WIDTH  = 3;
parameter WEIGHT_WIDTH  = 2;
parameter ONE = {1'd0, 8'd127, 23'd0};
parameter NEG_ONE = {1'd1, 8'd127, 23'd0};

// TIMING
parameter EQUALIZER_LATENCY = 6;

//---------------------------------------------------------------------
//   INTEGER
//---------------------------------------------------------------------
integer i, j, k;
genvar ii, jj, kk;

//---------------------------------------------------------------------
//   INPUTS & OUTPUTS
//---------------------------------------------------------------------
input rst_n, clk, in_valid;
input cg_en;
input [inst_sig_width+inst_exp_width:0] Img, Kernel, Weight;
input [1:0] Opt;

output reg	out_valid;
output reg [inst_sig_width+inst_exp_width:0] out;

//---------------------------------------------------------------------
//   REG & WIRE DECLARATIONS
//---------------------------------------------------------------------

// MDAs
reg  [31:0] img_r             [0:3][0:3];
reg  [31:0] img_pad           [0:5][0:5];
reg  [31:0] img_box           [0:2][0:2];
reg  [31:0] kernel_r          [0:2][0:2][0:2];
reg  [31:0] kernel_box        [0:2][0:2];
reg  [31:0] weight_r          [0:1][0:1];
reg  [31:0] feature_map_r     [0:1][0:3][0:3];
reg  [31:0] feature_pad       [0:1][0:5][0:5];
reg  [31:0] equalize_r        [0:1][0:3][0:3];
reg  [31:0] max_pool_r        [0:1][0:1][0:1];
reg  [31:0] flatten_r         [0:1][0:3];

reg   [10:0] count_r, next_count_r;
reg   [3:0] center_row, center_col;
reg   [3:0] feature_row, feature_col;
reg   [3:0] equalize_page, equalize_row, equalize_col;
reg   [1:0] opt_r;
wire        FINISH;

// module fp_dp3_sum4
reg   [1:0] fp_dp3_sum4_mode;
reg  [31:0] dp0_a, dp0_b, dp0_c, dp0_d, dp0_e, dp0_f;
reg  [31:0] dp1_a, dp1_b, dp1_c, dp1_d, dp1_e, dp1_f;
reg  [31:0] dp2_a, dp2_b, dp2_c, dp2_d, dp2_e, dp2_f;
reg  [31:0] sum_a, sum_b, sum_c, sum_d;
wire [31:0] dp0_out, dp1_out, dp2_out, sum4;
reg  [31:0] feature_map_orig;

// module fp_sort4
reg  [31:0] fp_sort0_in_a, fp_sort0_in_b, fp_sort0_in_c, fp_sort0_in_d;
reg  [31:0] fp_sort1_in_a, fp_sort1_in_b, fp_sort1_in_c, fp_sort1_in_d;
wire [31:0] fp_sort0_out_min, fp_sort0_out_max;
wire [31:0] fp_sort1_out_min, fp_sort1_out_max;

// module sub_div_sub
reg  [31:0] flatten_min0_r, flatten_max0_r, flatten_min1_r, flatten_max1_r;
reg   [1:0] sds0_mode;
reg  [31:0] sds0_flatten, sds0_flatten_min, sds0_flatten_max, sds0_sub_den_1, sds0_exp_in;
wire [31:0] sds0_norm, sds0_activation;
wire [31:0] sds0_sub_num_out, sds0_sub_den_out;
reg   [1:0] sds1_mode;
reg  [31:0] sds1_flatten, sds1_flatten_min, sds1_flatten_max, sds1_sub_den_1, sds1_exp_in;
wire [31:0] sds1_norm, sds1_activation;
wire [31:0] sds1_sub_num_out, sds1_sub_den_out;

// module Equalizer
reg  [31:0] equ_in00, equ_in01, equ_in02;
reg  [31:0] equ_in10, equ_in11, equ_in12;
reg  [31:0] equ_in20, equ_in21, equ_in22;
wire [31:0] equ_out;

// Output stage register
reg  [31:0] out_r;


//---------------------------------------------------------------------
//   GATED CLOCK
//---------------------------------------------------------------------
// Gated clocks
wire clk_img  [0:3][0:3];
wire clk_ker  [0:2][0:2][0:2];
wire clk_wei  [0:1][0:1];
wire clk_opt;
wire clk_feat [0:1][0:3][0:3];
wire clk_equ  [0:1][0:3][0:3];
wire clk_max  [0:1][0:1][0:1];
wire clk_flat [0:1][0:3];
wire clk_sort [0:3];
wire clk_out;

// Sleep flags
wire img_sleep, ker_sleep, wei_sleep, opt_sleep;
wire feat_sleep, equ_sleep, max_sleep, flat_sleep;
wire sort_sleep, out_sleep;

assign img_sleep = ~(0 <= count_r && count_r <= 96);
assign ker_sleep = ~(0 <= count_r && count_r <= 27);
assign wei_sleep = ~(0 <= count_r && count_r <= 4);
assign opt_sleep = ~(0 == count_r);
assign feat_sleep = ~(0 == count_r || (10 <= count_r && count_r <= 57) || (58 <= count_r && count_r <= 105));
assign equ_sleep  = ~(0 == count_r || (48 <= count_r && count_r <= 63) || (96 <= count_r && count_r <= 111));
assign max_sleep  = ~(0 == count_r || count_r == 54 || count_r == 56 || count_r == 62 || count_r == 64 || count_r == 102 || count_r == 104 || count_r == 110 || count_r == 112);
assign flat_sleep = ~(0 == count_r || (112 <= count_r && count_r <= 124));
assign sort_sleep = ~(0 == count_r || count_r == 115);
assign out_sleep  = ~(0 == count_r || count_r == 125);

//---------------------------------------------------------------------
//   STORAGE - IMG, KERNEL, WEIGHT, OPT
//---------------------------------------------------------------------
/* img_r */
generate
    for (ii = 0; ii < 4; ii = ii + 1) begin
        for (jj = 0; jj < 4; jj = jj + 1) begin

            GATED_OR GATED_IMG ( .CLOCK(clk), .SLEEP_CTRL(cg_en && img_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_img[ii][jj]) );
            always @(posedge clk_img[ii][jj] or negedge rst_n) begin
                if (!rst_n) img_r[ii][jj] <= 0;
                else begin
                    if (in_valid) begin
                        case(count_r)
                            4*ii + jj +  0: img_r[ii][jj] <= Img;
                            4*ii + jj + 16: img_r[ii][jj] <= Img;
                            4*ii + jj + 32: img_r[ii][jj] <= Img;
                            4*ii + jj + 48: img_r[ii][jj] <= Img;
                            4*ii + jj + 64: img_r[ii][jj] <= Img;
                            4*ii + jj + 80: img_r[ii][jj] <= Img;
                        endcase
                    end
                    else if (count_r == 0) begin
                        img_r[ii][jj] <= 0;
                    end
                end
            end

        end
    end
endgenerate

/* kernel_r */
generate
    for (ii = 0; ii < 3; ii = ii + 1) begin
        for (jj = 0; jj < 3; jj = jj + 1) begin
            for (kk = 0; kk < 3; kk = kk + 1) begin

                GATED_OR GATED_KER ( .CLOCK(clk), .SLEEP_CTRL(cg_en && ker_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_ker[ii][jj][kk]) );
                always @(posedge clk_ker[ii][jj][kk] or negedge rst_n) begin
                    if (!rst_n) kernel_r[ii][jj][kk] <= 0;
                    else begin
                        if (in_valid && count_r == (9*ii + 3*jj + kk))
                            kernel_r[ii][jj][kk] <= Kernel;
                        else if (count_r == 0)
                            kernel_r[ii][jj][kk] <= 0;
                    end
                end

            end
        end
    end
endgenerate

/* weight_r */
generate
    for (ii = 0; ii < 2; ii = ii + 1) begin
        for (jj = 0; jj < 2; jj = jj + 1) begin

            GATED_OR GATED_WEI ( .CLOCK(clk), .SLEEP_CTRL(cg_en && wei_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_wei[ii][jj]) );
            always @(posedge clk_wei[ii][jj] or negedge rst_n) begin
                if (!rst_n) weight_r[ii][jj] <= 0;
                else begin
                    if (in_valid && count_r == (2*ii + jj))
                        weight_r[ii][jj] <= Weight;
                    else if (count_r == 0)
                        weight_r[ii][jj] <= 0;
                end
            end


        end
    end
endgenerate

/* opt_r */
GATED_OR GATED_OPT ( .CLOCK(clk), .SLEEP_CTRL(cg_en && opt_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_opt) );
always @(posedge clk_opt or negedge rst_n) begin
    if (!rst_n)    opt_r <= 2'd0;
    else begin
        if (in_valid && count_r == 0)
            opt_r <= Opt;
        else if (count_r == 0)
            opt_r <= 2'd0;
    end
end

//---------------------------------------------------------------------
//   PADDING
//---------------------------------------------------------------------
/* img_pad */
always @(*) begin
    if (opt_r[0]) begin
        img_pad[0][0] = 0;              img_pad[0][1] = 0;              img_pad[0][2] = 0;              img_pad[0][3] = 0;              img_pad[0][4] = 0;              img_pad[0][5] = 0;
        img_pad[1][0] = 0;              img_pad[1][1] = img_r[0][0];    img_pad[1][2] = img_r[0][1];    img_pad[1][3] = img_r[0][2];    img_pad[1][4] = img_r[0][3];    img_pad[1][5] = 0;
        img_pad[2][0] = 0;              img_pad[2][1] = img_r[1][0];    img_pad[2][2] = img_r[1][1];    img_pad[2][3] = img_r[1][2];    img_pad[2][4] = img_r[1][3];    img_pad[2][5] = 0;
        img_pad[3][0] = 0;              img_pad[3][1] = img_r[2][0];    img_pad[3][2] = img_r[2][1];    img_pad[3][3] = img_r[2][2];    img_pad[3][4] = img_r[2][3];    img_pad[3][5] = 0;
        img_pad[4][0] = 0;              img_pad[4][1] = img_r[3][0];    img_pad[4][2] = img_r[3][1];    img_pad[4][3] = img_r[3][2];    img_pad[4][4] = img_r[3][3];    img_pad[4][5] = 0;
        img_pad[5][0] = 0;              img_pad[5][1] = 0;              img_pad[5][2] = 0;              img_pad[5][3] = 0;              img_pad[5][4] = 0;              img_pad[5][5] = 0;
    end
    else begin
        img_pad[0][0] = img_r[0][0];    img_pad[0][1] = img_r[0][0];    img_pad[0][2] = img_r[0][1];    img_pad[0][3] = img_r[0][2];    img_pad[0][4] = img_r[0][3];    img_pad[0][5] = img_r[0][3];
        img_pad[1][0] = img_r[0][0];    img_pad[1][1] = img_r[0][0];    img_pad[1][2] = img_r[0][1];    img_pad[1][3] = img_r[0][2];    img_pad[1][4] = img_r[0][3];    img_pad[1][5] = img_r[0][3];
        img_pad[2][0] = img_r[1][0];    img_pad[2][1] = img_r[1][0];    img_pad[2][2] = img_r[1][1];    img_pad[2][3] = img_r[1][2];    img_pad[2][4] = img_r[1][3];    img_pad[2][5] = img_r[1][3];
        img_pad[3][0] = img_r[2][0];    img_pad[3][1] = img_r[2][0];    img_pad[3][2] = img_r[2][1];    img_pad[3][3] = img_r[2][2];    img_pad[3][4] = img_r[2][3];    img_pad[3][5] = img_r[2][3];
        img_pad[4][0] = img_r[3][0];    img_pad[4][1] = img_r[3][0];    img_pad[4][2] = img_r[3][1];    img_pad[4][3] = img_r[3][2];    img_pad[4][4] = img_r[3][3];    img_pad[4][5] = img_r[3][3];
        img_pad[5][0] = img_r[3][0];    img_pad[5][1] = img_r[3][0];    img_pad[5][2] = img_r[3][1];    img_pad[5][3] = img_r[3][2];    img_pad[5][4] = img_r[3][3];    img_pad[5][5] = img_r[3][3];
    end
end

/* feature_pad */
always @(*) begin
    if (opt_r[0]) begin
        feature_pad[0][0][0] = 0;                         feature_pad[0][0][1] = 0;                         feature_pad[0][0][2] = 0;                         feature_pad[0][0][3] = 0;                         feature_pad[0][0][4] = 0;                         feature_pad[0][0][5] = 0;
        feature_pad[0][1][0] = 0;                         feature_pad[0][1][1] = feature_map_r[0][0][0];    feature_pad[0][1][2] = feature_map_r[0][0][1];    feature_pad[0][1][3] = feature_map_r[0][0][2];    feature_pad[0][1][4] = feature_map_r[0][0][3];    feature_pad[0][1][5] = 0;
        feature_pad[0][2][0] = 0;                         feature_pad[0][2][1] = feature_map_r[0][1][0];    feature_pad[0][2][2] = feature_map_r[0][1][1];    feature_pad[0][2][3] = feature_map_r[0][1][2];    feature_pad[0][2][4] = feature_map_r[0][1][3];    feature_pad[0][2][5] = 0;
        feature_pad[0][3][0] = 0;                         feature_pad[0][3][1] = feature_map_r[0][2][0];    feature_pad[0][3][2] = feature_map_r[0][2][1];    feature_pad[0][3][3] = feature_map_r[0][2][2];    feature_pad[0][3][4] = feature_map_r[0][2][3];    feature_pad[0][3][5] = 0;
        feature_pad[0][4][0] = 0;                         feature_pad[0][4][1] = feature_map_r[0][3][0];    feature_pad[0][4][2] = feature_map_r[0][3][1];    feature_pad[0][4][3] = feature_map_r[0][3][2];    feature_pad[0][4][4] = feature_map_r[0][3][3];    feature_pad[0][4][5] = 0;
        feature_pad[0][5][0] = 0;                         feature_pad[0][5][1] = 0;                         feature_pad[0][5][2] = 0;                         feature_pad[0][5][3] = 0;                         feature_pad[0][5][4] = 0;                         feature_pad[0][5][5] = 0;

        feature_pad[1][0][0] = 0;                         feature_pad[1][0][1] = 0;                         feature_pad[1][0][2] = 0;                         feature_pad[1][0][3] = 0;                         feature_pad[1][0][4] = 0;                         feature_pad[1][0][5] = 0;
        feature_pad[1][1][0] = 0;                         feature_pad[1][1][1] = feature_map_r[1][0][0];    feature_pad[1][1][2] = feature_map_r[1][0][1];    feature_pad[1][1][3] = feature_map_r[1][0][2];    feature_pad[1][1][4] = feature_map_r[1][0][3];    feature_pad[1][1][5] = 0;
        feature_pad[1][2][0] = 0;                         feature_pad[1][2][1] = feature_map_r[1][1][0];    feature_pad[1][2][2] = feature_map_r[1][1][1];    feature_pad[1][2][3] = feature_map_r[1][1][2];    feature_pad[1][2][4] = feature_map_r[1][1][3];    feature_pad[1][2][5] = 0;
        feature_pad[1][3][0] = 0;                         feature_pad[1][3][1] = feature_map_r[1][2][0];    feature_pad[1][3][2] = feature_map_r[1][2][1];    feature_pad[1][3][3] = feature_map_r[1][2][2];    feature_pad[1][3][4] = feature_map_r[1][2][3];    feature_pad[1][3][5] = 0;
        feature_pad[1][4][0] = 0;                         feature_pad[1][4][1] = feature_map_r[1][3][0];    feature_pad[1][4][2] = feature_map_r[1][3][1];    feature_pad[1][4][3] = feature_map_r[1][3][2];    feature_pad[1][4][4] = feature_map_r[1][3][3];    feature_pad[1][4][5] = 0;
        feature_pad[1][5][0] = 0;                         feature_pad[1][5][1] = 0;                         feature_pad[1][5][2] = 0;                         feature_pad[1][5][3] = 0;                         feature_pad[1][5][4] = 0;                         feature_pad[1][5][5] = 0;
    end
    else begin
        feature_pad[0][0][0] = feature_map_r[0][0][0];    feature_pad[0][0][1] = feature_map_r[0][0][0];    feature_pad[0][0][2] = feature_map_r[0][0][1];    feature_pad[0][0][3] = feature_map_r[0][0][2];    feature_pad[0][0][4] = feature_map_r[0][0][3];    feature_pad[0][0][5] = feature_map_r[0][0][3];
        feature_pad[0][1][0] = feature_map_r[0][0][0];    feature_pad[0][1][1] = feature_map_r[0][0][0];    feature_pad[0][1][2] = feature_map_r[0][0][1];    feature_pad[0][1][3] = feature_map_r[0][0][2];    feature_pad[0][1][4] = feature_map_r[0][0][3];    feature_pad[0][1][5] = feature_map_r[0][0][3];
        feature_pad[0][2][0] = feature_map_r[0][1][0];    feature_pad[0][2][1] = feature_map_r[0][1][0];    feature_pad[0][2][2] = feature_map_r[0][1][1];    feature_pad[0][2][3] = feature_map_r[0][1][2];    feature_pad[0][2][4] = feature_map_r[0][1][3];    feature_pad[0][2][5] = feature_map_r[0][1][3];
        feature_pad[0][3][0] = feature_map_r[0][2][0];    feature_pad[0][3][1] = feature_map_r[0][2][0];    feature_pad[0][3][2] = feature_map_r[0][2][1];    feature_pad[0][3][3] = feature_map_r[0][2][2];    feature_pad[0][3][4] = feature_map_r[0][2][3];    feature_pad[0][3][5] = feature_map_r[0][2][3];
        feature_pad[0][4][0] = feature_map_r[0][3][0];    feature_pad[0][4][1] = feature_map_r[0][3][0];    feature_pad[0][4][2] = feature_map_r[0][3][1];    feature_pad[0][4][3] = feature_map_r[0][3][2];    feature_pad[0][4][4] = feature_map_r[0][3][3];    feature_pad[0][4][5] = feature_map_r[0][3][3];
        feature_pad[0][5][0] = feature_map_r[0][3][0];    feature_pad[0][5][1] = feature_map_r[0][3][0];    feature_pad[0][5][2] = feature_map_r[0][3][1];    feature_pad[0][5][3] = feature_map_r[0][3][2];    feature_pad[0][5][4] = feature_map_r[0][3][3];    feature_pad[0][5][5] = feature_map_r[0][3][3];

        feature_pad[1][0][0] = feature_map_r[1][0][0];    feature_pad[1][0][1] = feature_map_r[1][0][0];    feature_pad[1][0][2] = feature_map_r[1][0][1];    feature_pad[1][0][3] = feature_map_r[1][0][2];    feature_pad[1][0][4] = feature_map_r[1][0][3];    feature_pad[1][0][5] = feature_map_r[1][0][3];
        feature_pad[1][1][0] = feature_map_r[1][0][0];    feature_pad[1][1][1] = feature_map_r[1][0][0];    feature_pad[1][1][2] = feature_map_r[1][0][1];    feature_pad[1][1][3] = feature_map_r[1][0][2];    feature_pad[1][1][4] = feature_map_r[1][0][3];    feature_pad[1][1][5] = feature_map_r[1][0][3];
        feature_pad[1][2][0] = feature_map_r[1][1][0];    feature_pad[1][2][1] = feature_map_r[1][1][0];    feature_pad[1][2][2] = feature_map_r[1][1][1];    feature_pad[1][2][3] = feature_map_r[1][1][2];    feature_pad[1][2][4] = feature_map_r[1][1][3];    feature_pad[1][2][5] = feature_map_r[1][1][3];
        feature_pad[1][3][0] = feature_map_r[1][2][0];    feature_pad[1][3][1] = feature_map_r[1][2][0];    feature_pad[1][3][2] = feature_map_r[1][2][1];    feature_pad[1][3][3] = feature_map_r[1][2][2];    feature_pad[1][3][4] = feature_map_r[1][2][3];    feature_pad[1][3][5] = feature_map_r[1][2][3];
        feature_pad[1][4][0] = feature_map_r[1][3][0];    feature_pad[1][4][1] = feature_map_r[1][3][0];    feature_pad[1][4][2] = feature_map_r[1][3][1];    feature_pad[1][4][3] = feature_map_r[1][3][2];    feature_pad[1][4][4] = feature_map_r[1][3][3];    feature_pad[1][4][5] = feature_map_r[1][3][3];
        feature_pad[1][5][0] = feature_map_r[1][3][0];    feature_pad[1][5][1] = feature_map_r[1][3][0];    feature_pad[1][5][2] = feature_map_r[1][3][1];    feature_pad[1][5][3] = feature_map_r[1][3][2];    feature_pad[1][5][4] = feature_map_r[1][3][3];    feature_pad[1][5][5] = feature_map_r[1][3][3];
    end
end

//---------------------------------------------------------------------
//   BOX FOR CONV
//---------------------------------------------------------------------
/* img_pad selector */
always @(*) begin
    case(count_r)
         9, 25, 41, 57, 73,  89: begin center_row = 1; center_col = 1; end
        10, 26, 42, 58, 74,  90: begin center_row = 1; center_col = 2; end
        11, 27, 43, 59, 75,  91: begin center_row = 1; center_col = 3; end
        12, 28, 44, 60, 76,  92: begin center_row = 1; center_col = 4; end
        13, 29, 45, 61, 77,  93: begin center_row = 2; center_col = 1; end
        14, 30, 46, 62, 78,  94: begin center_row = 2; center_col = 2; end
        15, 31, 47, 63, 79,  95: begin center_row = 2; center_col = 3; end
        16, 32, 48, 64, 80,  96: begin center_row = 2; center_col = 4; end
        17, 33, 49, 65, 81,  97: begin center_row = 3; center_col = 1; end
        18, 34, 50, 66, 82,  98: begin center_row = 3; center_col = 2; end
        19, 35, 51, 67, 83,  99: begin center_row = 3; center_col = 3; end
        20, 36, 52, 68, 84, 100: begin center_row = 3; center_col = 4; end
        21, 37, 53, 69, 85, 101: begin center_row = 4; center_col = 1; end
        22, 38, 54, 70, 86, 102: begin center_row = 4; center_col = 2; end
        23, 39, 55, 71, 87, 103: begin center_row = 4; center_col = 3; end
        24, 40, 56, 72, 88, 104: begin center_row = 4; center_col = 4; end
        default: begin center_row = 0; center_col = 0; end
    endcase
end

/* Img Box */
always @(*) begin
    if (count_r >= 9) begin
        img_box [0][0] = img_pad[center_row-1][center_col-1];    img_box [0][1] = img_pad[center_row-1][center_col];    img_box [0][2] = img_pad[center_row-1][center_col+1];
        img_box [1][0] = img_pad[center_row  ][center_col-1];    img_box [1][1] = img_pad[center_row  ][center_col];    img_box [1][2] = img_pad[center_row  ][center_col+1];
        img_box [2][0] = img_pad[center_row+1][center_col-1];    img_box [2][1] = img_pad[center_row+1][center_col];    img_box [2][2] = img_pad[center_row+1][center_col+1];
    end
    else begin
        img_box [0][0] = 0;    img_box [0][1] = 0;    img_box [0][2] = 0;
        img_box [1][0] = 0;    img_box [1][1] = 0;    img_box [1][2] = 0;
        img_box [2][0] = 0;    img_box [2][1] = 0;    img_box [2][2] = 0;
    end
end

/* Kernel Box */
always @(*) begin
    if ((9 <= count_r && count_r <= 24) || (57 <= count_r && count_r <= 72)) begin
        kernel_box[0][0] = kernel_r[0][0][0];    kernel_box[0][1] = kernel_r[0][0][1];    kernel_box[0][2] = kernel_r[0][0][2];
        kernel_box[1][0] = kernel_r[0][1][0];    kernel_box[1][1] = kernel_r[0][1][1];    kernel_box[1][2] = kernel_r[0][1][2];
        kernel_box[2][0] = kernel_r[0][2][0];    kernel_box[2][1] = kernel_r[0][2][1];    kernel_box[2][2] = kernel_r[0][2][2];
    end
    else if ((25 <= count_r && count_r <= 40) || (73 <= count_r && count_r <= 88)) begin
        kernel_box[0][0] = kernel_r[1][0][0];    kernel_box[0][1] = kernel_r[1][0][1];    kernel_box[0][2] = kernel_r[1][0][2];
        kernel_box[1][0] = kernel_r[1][1][0];    kernel_box[1][1] = kernel_r[1][1][1];    kernel_box[1][2] = kernel_r[1][1][2];
        kernel_box[2][0] = kernel_r[1][2][0];    kernel_box[2][1] = kernel_r[1][2][1];    kernel_box[2][2] = kernel_r[1][2][2];
    end
    else if ((41 <= count_r && count_r <= 56) || (89 <= count_r && count_r <= 104)) begin
        kernel_box[0][0] = kernel_r[2][0][0];    kernel_box[0][1] = kernel_r[2][0][1];    kernel_box[0][2] = kernel_r[2][0][2];
        kernel_box[1][0] = kernel_r[2][1][0];    kernel_box[1][1] = kernel_r[2][1][1];    kernel_box[1][2] = kernel_r[2][1][2];
        kernel_box[2][0] = kernel_r[2][2][0];    kernel_box[2][1] = kernel_r[2][2][1];    kernel_box[2][2] = kernel_r[2][2][2];
    end
    else begin
        kernel_box[0][0] = 0;    kernel_box[0][1] = 0;    kernel_box[0][2] = 0;
        kernel_box[1][0] = 0;    kernel_box[1][1] = 0;    kernel_box[1][2] = 0;
        kernel_box[2][0] = 0;    kernel_box[2][1] = 0;    kernel_box[2][2] = 0;
    end
end

//---------------------------------------------------------------------
//   FEATURE MAP
//---------------------------------------------------------------------
/* feature_map_r */
generate
    for (ii = 0; ii < 2; ii = ii + 1) begin
        for (jj = 0; jj < 4; jj = jj + 1) begin
            for (kk = 0; kk < 4; kk = kk + 1) begin
            //-----------------generate_begin-----------------//
                GATED_OR GATED_FEAT ( .CLOCK(clk), .SLEEP_CTRL(cg_en && feat_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_feat[ii][jj][kk]) );
                always @(posedge clk_feat[ii][jj][kk] or negedge rst_n) begin
                    if (!rst_n) feature_map_r[ii][jj][kk] <= 0;
                    else begin
                        case(count_r)
                            0:                      feature_map_r[ii][jj][kk] <= 0;
                            48*ii + 4*jj + kk + 10: feature_map_r[ii][jj][kk] <= sum4;
                            48*ii + 4*jj + kk + 26: feature_map_r[ii][jj][kk] <= sum4;
                            48*ii + 4*jj + kk + 42: feature_map_r[ii][jj][kk] <= sum4;
                        endcase
                    end
                end
            //-----------------generate_end--------------------//
            end
        end
    end
endgenerate

always @(*) begin
    if      (9+1  <= count_r && count_r <= 56+1)  feature_map_orig = feature_map_r[0][feature_row][feature_col];
    else if (57+1 <= count_r && count_r <= 104+1) feature_map_orig = feature_map_r[1][feature_row][feature_col];
    else                                          feature_map_orig = 0;
end

/* feature_row_col selector (delay one cycle) */
always @(*) begin
    case(count_r - 1)
         9, 25, 41, 57, 73,  89: begin feature_row = 0; feature_col = 0; end
        10, 26, 42, 58, 74,  90: begin feature_row = 0; feature_col = 1; end
        11, 27, 43, 59, 75,  91: begin feature_row = 0; feature_col = 2; end
        12, 28, 44, 60, 76,  92: begin feature_row = 0; feature_col = 3; end
        13, 29, 45, 61, 77,  93: begin feature_row = 1; feature_col = 0; end
        14, 30, 46, 62, 78,  94: begin feature_row = 1; feature_col = 1; end
        15, 31, 47, 63, 79,  95: begin feature_row = 1; feature_col = 2; end
        16, 32, 48, 64, 80,  96: begin feature_row = 1; feature_col = 3; end
        17, 33, 49, 65, 81,  97: begin feature_row = 2; feature_col = 0; end
        18, 34, 50, 66, 82,  98: begin feature_row = 2; feature_col = 1; end
        19, 35, 51, 67, 83,  99: begin feature_row = 2; feature_col = 2; end
        20, 36, 52, 68, 84, 100: begin feature_row = 2; feature_col = 3; end
        21, 37, 53, 69, 85, 101: begin feature_row = 3; feature_col = 0; end
        22, 38, 54, 70, 86, 102: begin feature_row = 3; feature_col = 1; end
        23, 39, 55, 71, 87, 103: begin feature_row = 3; feature_col = 2; end
        24, 40, 56, 72, 88, 104: begin feature_row = 3; feature_col = 3; end
        default:                 begin feature_row = 0; feature_col = 0; end
    endcase
end

//---------------------------------------------------------------------
//   EQUALIZER MAP
//---------------------------------------------------------------------
/* equalize_r */
generate
    for (ii = 0; ii < 2; ii = ii + 1) begin
        for (jj = 0; jj < 4; jj = jj + 1) begin
            for (kk = 0; kk < 4; kk = kk + 1) begin
            //-----------------generate_begin-----------------//
                GATED_OR GATED_EQU ( .CLOCK(clk), .SLEEP_CTRL(cg_en && equ_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_equ[ii][jj][kk]) );
                always @(posedge clk_equ[ii][jj][kk] or negedge rst_n) begin
                    if (!rst_n)  equalize_r[ii][jj][kk] <= 0;
                    else begin
                        case(count_r)
                            0:                      equalize_r[ii][jj][kk] <= 0;
                            48*ii + 4*jj + kk + 48: equalize_r[ii][jj][kk] <= equ_out;
                        endcase
                    end
                end
            //-----------------generate_end--------------------//
            end
        end
    end
endgenerate


Equalizer EQU (
    .in00(feature_pad[equalize_page][equalize_row  ][equalize_col  ]), .in01(feature_pad[equalize_page][equalize_row  ][equalize_col+1]), .in02(feature_pad[equalize_page][equalize_row  ][equalize_col+2]),
    .in10(feature_pad[equalize_page][equalize_row+1][equalize_col  ]), .in11(feature_pad[equalize_page][equalize_row+1][equalize_col+1]), .in12(feature_pad[equalize_page][equalize_row+1][equalize_col+2]),
    .in20(feature_pad[equalize_page][equalize_row+2][equalize_col  ]), .in21(feature_pad[equalize_page][equalize_row+2][equalize_col+1]), .in22(feature_pad[equalize_page][equalize_row+2][equalize_col+2]),
    .out(equ_out) );

always @(*) begin
    case(count_r)
        48: begin equalize_page = 0; equalize_row = 0; equalize_col = 0; end         96: begin equalize_page = 1; equalize_row = 0; equalize_col = 0; end
        49: begin equalize_page = 0; equalize_row = 0; equalize_col = 1; end         97: begin equalize_page = 1; equalize_row = 0; equalize_col = 1; end
        50: begin equalize_page = 0; equalize_row = 0; equalize_col = 2; end         98: begin equalize_page = 1; equalize_row = 0; equalize_col = 2; end
        51: begin equalize_page = 0; equalize_row = 0; equalize_col = 3; end         99: begin equalize_page = 1; equalize_row = 0; equalize_col = 3; end
        52: begin equalize_page = 0; equalize_row = 1; equalize_col = 0; end        100: begin equalize_page = 1; equalize_row = 1; equalize_col = 0; end
        53: begin equalize_page = 0; equalize_row = 1; equalize_col = 1; end        101: begin equalize_page = 1; equalize_row = 1; equalize_col = 1; end
        54: begin equalize_page = 0; equalize_row = 1; equalize_col = 2; end        102: begin equalize_page = 1; equalize_row = 1; equalize_col = 2; end
        55: begin equalize_page = 0; equalize_row = 1; equalize_col = 3; end        103: begin equalize_page = 1; equalize_row = 1; equalize_col = 3; end
        56: begin equalize_page = 0; equalize_row = 2; equalize_col = 0; end        104: begin equalize_page = 1; equalize_row = 2; equalize_col = 0; end
        57: begin equalize_page = 0; equalize_row = 2; equalize_col = 1; end        105: begin equalize_page = 1; equalize_row = 2; equalize_col = 1; end
        58: begin equalize_page = 0; equalize_row = 2; equalize_col = 2; end        106: begin equalize_page = 1; equalize_row = 2; equalize_col = 2; end
        59: begin equalize_page = 0; equalize_row = 2; equalize_col = 3; end        107: begin equalize_page = 1; equalize_row = 2; equalize_col = 3; end
        60: begin equalize_page = 0; equalize_row = 3; equalize_col = 0; end        108: begin equalize_page = 1; equalize_row = 3; equalize_col = 0; end
        61: begin equalize_page = 0; equalize_row = 3; equalize_col = 1; end        109: begin equalize_page = 1; equalize_row = 3; equalize_col = 1; end
        62: begin equalize_page = 0; equalize_row = 3; equalize_col = 2; end        110: begin equalize_page = 1; equalize_row = 3; equalize_col = 2; end
        63: begin equalize_page = 0; equalize_row = 3; equalize_col = 3; end        111: begin equalize_page = 1; equalize_row = 3; equalize_col = 3; end
        default: begin equalize_page = 0; equalize_row = 0; equalize_col = 0; end
    endcase
end

//---------------------------------------------------------------------
//   MAX POOLING
//---------------------------------------------------------------------
generate
    for (ii = 0; ii < 2; ii = ii + 1) begin
        for (jj = 0; jj < 2; jj = jj + 1) begin
            for (kk = 0; kk < 2; kk = kk + 1) begin
            //-----------------generate_begin-----------------//
                GATED_OR GATED_MAX ( .CLOCK(clk), .SLEEP_CTRL(cg_en && max_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_max[ii][jj][kk]) );
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n) max_pool_r[ii][jj][kk] <= 0;
                    else begin
                        if (count_r == 0)
                            max_pool_r[ii][jj][kk] <= 0;
                        else if (count_r == 48*ii + 8*jj + 2*kk + 54)
                            max_pool_r[ii][jj][kk] <= fp_sort0_out_max;
                    end
                end
            //-----------------generate_end--------------------//
            end
        end
    end
endgenerate

//---------------------------------------------------------------------
//   FLATTEN
//---------------------------------------------------------------------
generate
    for (ii = 0; ii < 2; ii = ii + 1) begin
        for (jj = 0; jj < 4; jj = jj + 1) begin
            // -----------------generate_begin-----------------//
            GATED_OR GATED_FLAT ( .CLOCK(clk), .SLEEP_CTRL(cg_en && flat_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_flat[ii][jj]));
            // -----------------generate_end--------------------//
        end
    end
endgenerate
always @(posedge clk_flat[0][0] or negedge rst_n) begin
    if (!rst_n)  flatten_r[0][0] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[0][0] <= 0;
            EQUALIZER_LATENCY + 106: flatten_r[0][0] <= dp0_out;
            EQUALIZER_LATENCY + 110: flatten_r[0][0] <= sds0_norm;
            EQUALIZER_LATENCY + 114: flatten_r[0][0] <= sds0_activation;
            EQUALIZER_LATENCY + 118: flatten_r[0][0] <= {1'd0, sds0_sub_num_out[30:0]};
        endcase
    end
end
always @(posedge clk_flat[0][1] or negedge rst_n) begin
    if (!rst_n)  flatten_r[0][1] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[0][1] <= 0;
            EQUALIZER_LATENCY + 106: flatten_r[0][1] <= dp1_out;
            EQUALIZER_LATENCY + 111: flatten_r[0][1] <= sds0_norm;
            EQUALIZER_LATENCY + 115: flatten_r[0][1] <= sds0_activation;
            EQUALIZER_LATENCY + 118: flatten_r[0][1] <= {1'd0, sds0_sub_den_out[30:0]};
        endcase
    end
end
always @(posedge clk_flat[0][2] or negedge rst_n) begin
    if (!rst_n)  flatten_r[0][2] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[0][2] <= 0;
            EQUALIZER_LATENCY + 106: flatten_r[0][2] <= dp2_out;
            EQUALIZER_LATENCY + 112: flatten_r[0][2] <= sds0_norm;
            EQUALIZER_LATENCY + 116: flatten_r[0][2] <= sds0_activation;
            EQUALIZER_LATENCY + 118: flatten_r[0][2] <= {1'd0, sds1_sub_num_out[30:0]};
        endcase
    end
end
always @(posedge clk_flat[0][3] or negedge rst_n) begin
    if (!rst_n)  flatten_r[0][3] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[0][3] <= 0;
            EQUALIZER_LATENCY + 107: flatten_r[0][3] <= dp0_out;
            EQUALIZER_LATENCY + 113: flatten_r[0][3] <= sds0_norm;
            EQUALIZER_LATENCY + 117: flatten_r[0][3] <= sds0_activation;
            EQUALIZER_LATENCY + 118: flatten_r[0][3] <= {1'd0, sds1_sub_den_out[30:0]};
        endcase
    end
end
always @(posedge clk_flat[1][0] or negedge rst_n) begin
    if (!rst_n)  flatten_r[1][0] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[1][0] <= 0;
            EQUALIZER_LATENCY + 107: flatten_r[1][0] <= dp1_out;
            EQUALIZER_LATENCY + 110: flatten_r[1][0] <= sds1_norm;
            EQUALIZER_LATENCY + 114: flatten_r[1][0] <= sds1_activation;
        endcase
    end
end
always @(posedge clk_flat[1][1] or negedge rst_n) begin
    if (!rst_n)  flatten_r[1][1] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[1][1] <= 0;
            EQUALIZER_LATENCY + 107: flatten_r[1][1] <= dp2_out;
            EQUALIZER_LATENCY + 111: flatten_r[1][1] <= sds1_norm;
            EQUALIZER_LATENCY + 115: flatten_r[1][1] <= sds1_activation;
        endcase
    end
end
always @(posedge clk_flat[1][2] or negedge rst_n) begin
    if (!rst_n)  flatten_r[1][2] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[1][2] <= 0;
            EQUALIZER_LATENCY + 108: flatten_r[1][2] <= dp0_out;
            EQUALIZER_LATENCY + 112: flatten_r[1][2] <= sds1_norm;
            EQUALIZER_LATENCY + 116: flatten_r[1][2] <= sds1_activation;
        endcase
    end
end
always @(posedge clk_flat[1][3] or negedge rst_n) begin
    if (!rst_n)  flatten_r[1][3] <= 0;
    else begin
        case(count_r)
                                  0: flatten_r[1][3] <= 0;
            EQUALIZER_LATENCY + 108: flatten_r[1][3] <= dp1_out;
            EQUALIZER_LATENCY + 113: flatten_r[1][3] <= sds1_norm;
            EQUALIZER_LATENCY + 117: flatten_r[1][3] <= sds1_activation;
        endcase
    end
end


generate
    for (ii = 0; ii < 4; ii = ii + 1) begin
        GATED_OR GATED_SORT ( .CLOCK(clk), .SLEEP_CTRL(cg_en && sort_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_sort[ii]));
    end
endgenerate
always @(posedge clk_sort[0] or negedge rst_n) begin
    if (!rst_n)  flatten_min0_r <= 0;
    else begin
        if (count_r == 0)
            flatten_min0_r <= 0;
        else if ((count_r == EQUALIZER_LATENCY + 109))
            flatten_min0_r <= fp_sort0_out_min;
    end
end
always @(posedge clk_sort[1] or negedge rst_n) begin
    if (!rst_n)  flatten_max0_r <= 0;
    else begin
        if (count_r == 0)
            flatten_max0_r <= 0;
        else if ((count_r == EQUALIZER_LATENCY + 109))
            flatten_max0_r <= fp_sort0_out_max;
    end
end
always @(posedge clk_sort[2] or negedge rst_n) begin
    if (!rst_n)  flatten_min1_r <= 0;
    else begin
        if (count_r == 0)
            flatten_min1_r <= 0;
        else if ((count_r == EQUALIZER_LATENCY + 109))
            flatten_min1_r <= fp_sort1_out_min;
    end
end
always @(posedge clk_sort[3] or negedge rst_n) begin
    if (!rst_n)  flatten_max1_r <= 0;
    else begin
        if (count_r == 0)
            flatten_max1_r <= 0;
        else if ((count_r == EQUALIZER_LATENCY + 109))
            flatten_max1_r <= fp_sort1_out_max;
    end
end

//---------------------------------------------------------------------
//   FP_DP3_SUM4
//---------------------------------------------------------------------
fp_dp3_sum4 DOT_PRODUCT_ADDER (
    // System inputs
    .clk(clk), .rst_n(rst_n), .mode(fp_dp3_sum4_mode),
    // Dot product inputs
    .dp0_a(dp0_a), .dp0_b(dp0_b), .dp0_c(dp0_c), .dp0_d(dp0_d), .dp0_e(dp0_e), .dp0_f(dp0_f),
    .dp1_a(dp1_a), .dp1_b(dp1_b), .dp1_c(dp1_c), .dp1_d(dp1_d), .dp1_e(dp1_e), .dp1_f(dp1_f),
    .dp2_a(dp2_a), .dp2_b(dp2_b), .dp2_c(dp2_c), .dp2_d(dp2_d), .dp2_e(dp2_e), .dp2_f(dp2_f),
    // Sum4 inputs
    .sum_a(sum_a), .sum_b(sum_b), .sum_c(sum_c), .sum_d(sum_d),
    // Output
    .sum4_out(sum4), .dp0_out(dp0_out), .dp1_out(dp1_out), .dp2_out(dp2_out)
);

always @(*) begin
    if (9 <= count_r && count_r <= 105) begin
        // Used in CONV and updating feature_map_r
        fp_dp3_sum4_mode = 2'd0;
        dp0_a = img_box[0][0]; dp0_b = kernel_box[0][0]; dp0_c = img_box[0][1]; dp0_d = kernel_box[0][1]; dp0_e = img_box[0][2]; dp0_f = kernel_box[0][2];
        dp1_a = img_box[1][0]; dp1_b = kernel_box[1][0]; dp1_c = img_box[1][1]; dp1_d = kernel_box[1][1]; dp1_e = img_box[1][2]; dp1_f = kernel_box[1][2];
        dp2_a = img_box[2][0]; dp2_b = kernel_box[2][0]; dp2_c = img_box[2][1]; dp2_d = kernel_box[2][1]; dp2_e = img_box[2][2]; dp2_f = kernel_box[2][2];
        sum_a = 0; sum_b = 0; sum_c = 0; sum_d = feature_map_orig;
    end
    else begin
        // Fully Connect
        case(count_r)
        EQUALIZER_LATENCY + 106: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[0][0][0]; dp0_b = weight_r[0][0]; dp0_c = max_pool_r[0][0][1]; dp0_d = weight_r[1][0]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[0][0][0]; dp1_b = weight_r[0][1]; dp1_c = max_pool_r[0][0][1]; dp1_d = weight_r[1][1]; dp1_e = 0; dp1_f = 0;
            dp2_a = max_pool_r[0][1][0]; dp2_b = weight_r[0][0]; dp2_c = max_pool_r[0][1][1]; dp2_d = weight_r[1][0]; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end
        EQUALIZER_LATENCY + 107: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[0][1][0]; dp0_b = weight_r[0][1]; dp0_c = max_pool_r[0][1][1]; dp0_d = weight_r[1][1]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[1][0][0]; dp1_b = weight_r[0][0]; dp1_c = max_pool_r[1][0][1]; dp1_d = weight_r[1][0]; dp1_e = 0; dp1_f = 0;
            dp2_a = max_pool_r[1][0][0]; dp2_b = weight_r[0][1]; dp2_c = max_pool_r[1][0][1]; dp2_d = weight_r[1][1]; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end
        EQUALIZER_LATENCY + 108: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[1][1][0]; dp0_b = weight_r[0][0]; dp0_c = max_pool_r[1][1][1]; dp0_d = weight_r[1][0]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[1][1][0]; dp1_b = weight_r[0][1]; dp1_c = max_pool_r[1][1][1]; dp1_d = weight_r[1][1]; dp1_e = 0; dp1_f = 0;
            dp2_a = 0; dp2_b = 0; dp2_c = 0; dp2_d = 0; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end

        // Distance Sum
        EQUALIZER_LATENCY + 119: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = 0; dp0_b = 0; dp0_c = 0; dp0_d = 0; dp0_e = 0; dp0_f = 0;
            dp1_a = 0; dp1_b = 0; dp1_c = 0; dp1_d = 0; dp1_e = 0; dp1_f = 0;
            dp2_a = 0; dp2_b = 0; dp2_c = 0; dp2_d = 0; dp2_e = 0; dp2_f = 0;
            sum_a = flatten_r[0][0]; sum_b = flatten_r[0][1]; sum_c = flatten_r[0][2]; sum_d = flatten_r[0][3];
        end
        default: begin
            fp_dp3_sum4_mode = 2'd0;
            dp0_a = 0; dp0_b = 0; dp0_c = 0; dp0_d = 0; dp0_e = 0; dp0_f = 0;
            dp1_a = 0; dp1_b = 0; dp1_c = 0; dp1_d = 0; dp1_e = 0; dp1_f = 0;
            dp2_a = 0; dp2_b = 0; dp2_c = 0; dp2_d = 0; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end
        endcase
    end
end

//---------------------------------------------------------------------
//   4-INPUT MIN MAX
//---------------------------------------------------------------------

fp_sort4 SORT0 (
    .a(fp_sort0_in_a),
    .b(fp_sort0_in_b),
    .c(fp_sort0_in_c),
    .d(fp_sort0_in_d),
    .out_min(fp_sort0_out_min),
    .out_max(fp_sort0_out_max) );

fp_sort4 SORT1 (
    .a(fp_sort1_in_a),
    .b(fp_sort1_in_b),
    .c(fp_sort1_in_c),
    .d(fp_sort1_in_d),
    .out_min(fp_sort1_out_min),
    .out_max(fp_sort1_out_max) );

always @(*) begin
    case(count_r)
        //! Changed from feature_map to equalize
        // Max pooling 1st page
        EQUALIZER_LATENCY + 48: begin
            fp_sort0_in_a = equalize_r[0][0][0];
            fp_sort0_in_b = equalize_r[0][0][1];
            fp_sort0_in_c = equalize_r[0][1][0];
            fp_sort0_in_d = equalize_r[0][1][1];
        end
        EQUALIZER_LATENCY + 50: begin
            fp_sort0_in_a = equalize_r[0][0][2];
            fp_sort0_in_b = equalize_r[0][0][3];
            fp_sort0_in_c = equalize_r[0][1][2];
            fp_sort0_in_d = equalize_r[0][1][3];
        end
        EQUALIZER_LATENCY + 56: begin
            fp_sort0_in_a = equalize_r[0][2][0];
            fp_sort0_in_b = equalize_r[0][2][1];
            fp_sort0_in_c = equalize_r[0][3][0];
            fp_sort0_in_d = equalize_r[0][3][1];
        end
        EQUALIZER_LATENCY + 58: begin
            fp_sort0_in_a = equalize_r[0][2][2];
            fp_sort0_in_b = equalize_r[0][2][3];
            fp_sort0_in_c = equalize_r[0][3][2];
            fp_sort0_in_d = equalize_r[0][3][3];
        end
        // Max pooling 2nd page
        EQUALIZER_LATENCY + 96: begin
            fp_sort0_in_a = equalize_r[1][0][0];
            fp_sort0_in_b = equalize_r[1][0][1];
            fp_sort0_in_c = equalize_r[1][1][0];
            fp_sort0_in_d = equalize_r[1][1][1];
        end
        EQUALIZER_LATENCY + 98: begin
            fp_sort0_in_a = equalize_r[1][0][2];
            fp_sort0_in_b = equalize_r[1][0][3];
            fp_sort0_in_c = equalize_r[1][1][2];
            fp_sort0_in_d = equalize_r[1][1][3];
        end
        EQUALIZER_LATENCY + 104: begin
            fp_sort0_in_a = equalize_r[1][2][0];
            fp_sort0_in_b = equalize_r[1][2][1];
            fp_sort0_in_c = equalize_r[1][3][0];
            fp_sort0_in_d = equalize_r[1][3][1];
        end
        EQUALIZER_LATENCY + 106: begin
            fp_sort0_in_a = equalize_r[1][2][2];
            fp_sort0_in_b = equalize_r[1][2][3];
            fp_sort0_in_c = equalize_r[1][3][2];
            fp_sort0_in_d = equalize_r[1][3][3];
        end
        EQUALIZER_LATENCY + 109: begin
            fp_sort0_in_a = flatten_r[0][0];
            fp_sort0_in_b = flatten_r[0][1];
            fp_sort0_in_c = flatten_r[0][2];
            fp_sort0_in_d = flatten_r[0][3];
        end
        default: begin
            fp_sort0_in_a = 0;
            fp_sort0_in_b = 0;
            fp_sort0_in_c = 0;
            fp_sort0_in_d = 0;
        end
    endcase
end

always @(*) begin
    case(count_r)
        EQUALIZER_LATENCY + 109: begin
            fp_sort1_in_a = flatten_r[1][0];
            fp_sort1_in_b = flatten_r[1][1];
            fp_sort1_in_c = flatten_r[1][2];
            fp_sort1_in_d = flatten_r[1][3];
        end
        default: begin
            fp_sort1_in_a = 0;
            fp_sort1_in_b = 0;
            fp_sort1_in_c = 0;
            fp_sort1_in_d = 0;
        end
    endcase
end

//---------------------------------------------------------------------
//   NORMALIZE & ACTIVATION FUNC
//---------------------------------------------------------------------
sub_div_sub SDS0(
    // System inputs
    .clk(clk), .rst_n(rst_n), .mode(sds0_mode),
    // Normalize inputs
    .flatten(sds0_flatten), .flatten_min(sds0_flatten_min), .flatten_max(sds0_flatten_max), .sub_den_1(sds0_sub_den_1),
    // Sigmoid or Tanh input
    .exp_in(sds0_exp_in),
    // Outputs
    .sub_num_out(sds0_sub_num_out), .sub_den_out(sds0_sub_den_out),
    .norm(sds0_norm), .activation(sds0_activation)
);

sub_div_sub SDS1(
    // System inputs
    .clk(clk), .rst_n(rst_n), .mode(sds1_mode),
    // Normalize inputs
    .flatten(sds1_flatten), .flatten_min(sds1_flatten_min), .flatten_max(sds1_flatten_max), .sub_den_1(sds1_sub_den_1),
    // Sigmoid or Tanh input
    .exp_in(sds1_exp_in),
    // Outputs
    .sub_num_out(sds1_sub_num_out), .sub_den_out(sds1_sub_den_out),
    .norm(sds1_norm), .activation(sds1_activation)
);

always @(*) begin
    case(count_r)
        // Normalize
        EQUALIZER_LATENCY + 109: begin // feed the min_max from combinational first
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][0];    sds1_flatten     = flatten_r[1][0];
            sds0_flatten_min = fp_sort0_out_min;   sds1_flatten_min = fp_sort1_out_min;
            sds0_flatten_max = fp_sort0_out_max;   sds1_flatten_max = fp_sort1_out_max;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        EQUALIZER_LATENCY + 110: begin // then feed the min_max from DFF
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][1];    sds1_flatten     = flatten_r[1][1];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        EQUALIZER_LATENCY + 111: begin
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][2];    sds1_flatten     = flatten_r[1][2];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        EQUALIZER_LATENCY + 112: begin
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][3];    sds1_flatten     = flatten_r[1][3];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        // Activation function
        EQUALIZER_LATENCY + 113: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][0];         sds1_exp_in = flatten_r[1][0];
        end
        EQUALIZER_LATENCY + 114: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][1];         sds1_exp_in = flatten_r[1][1];
        end
        EQUALIZER_LATENCY + 115: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][2];         sds1_exp_in = flatten_r[1][2];
        end
        EQUALIZER_LATENCY + 116: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][3];         sds1_exp_in = flatten_r[1][3];
        end
        // Distance
        EQUALIZER_LATENCY + 118: begin
            sds0_mode = 2'd0;                      sds1_mode = 2'd0;
            sds0_flatten     = flatten_r[0][0];    sds1_flatten     = flatten_r[0][2];
            sds0_flatten_min = flatten_r[1][0];    sds1_flatten_min = flatten_r[1][2];
            sds0_flatten_max = flatten_r[0][1];    sds1_flatten_max = flatten_r[0][3];
            sds0_sub_den_1   = flatten_r[1][1];    sds1_sub_den_1   = flatten_r[1][3];
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        default: begin
            sds0_mode = 2'd0;                      sds1_mode = 2'd0;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
    endcase
end


//---------------------------------------------------------------------
//   SYSTEM COUNTER
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  count_r <= 0;
    else         count_r <= next_count_r;
end
always @(*) begin
    if (FINISH)
        next_count_r = 0;
    else if (in_valid || count_r != 0)
        next_count_r = count_r + 1'd1;
    else
        next_count_r = count_r;
end

//---------------------------------------------------------------------
//   OUTPUTS
//---------------------------------------------------------------------
GATED_OR GATED_OUT ( .CLOCK(clk), .SLEEP_CTRL(cg_en && out_sleep), .RST_N(rst_n), .CLOCK_GATED(clk_out) );
always @(posedge clk_out or negedge rst_n) begin
    if (!rst_n) out_r <= 0;
    else begin
        if (count_r == 0)
            out_r <= 0;
        else if (count_r == EQUALIZER_LATENCY + 119)
            out_r <= sum4;
    end
end

// assign FINISH = (count_r == EQUALIZER_LATENCY + 120);
assign FINISH = (count_r == 1095);
always @(*) begin
    if (FINISH) begin out_valid = 1;   out = out_r; end
    else        begin out_valid = 0;   out = 0;     end
end
endmodule








//!=====================================================================
//!   SUBMODULES
//!=====================================================================

module fp_sort4 (
    a, b,
    c, d,
    out_min, out_max
);
// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

input  [31:0] a, b, c, d;
output [31:0] out_min, out_max;
wire   [31:0] ab_min, cd_min, ab_max, cd_max;

// Unused pins
wire         aeqb0, altb0, agtb0, aeqb1, altb1, agtb1, aeqb2, altb2, agtb2, aeqb3, altb3, agtb3;
wire         unordered0, unordered1, unordered2, unordered3;
wire  [7:0]  status00, status10, status01, status11, status02, status12, status03, status13;
wire [31:0]  z1_2, z0_3;

// ztrl = 0 , z0 -> min; z1 -> max
DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
CMP_0 (
    .a(a),
    .b(b),
    .zctr(1'b0),
    .z0(ab_min),
    .z1(ab_max),
    // Unused pins
    .aeqb(aeqb0),
    .altb(altb0),
    .agtb(agtb0),
    .unordered(unordered0),
    .status0(status00),
    .status1(status10) );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
CMP_1 (
    .a(c),
    .b(d),
    .zctr(1'b0),
    .z0(cd_min),
    .z1(cd_max),
    // Unused pins
    .aeqb(aeqb1),
    .altb(altb1),
    .agtb(agtb1),
    .unordered(unordered1),
    .status0(status01),
    .status1(status11) );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
CMP_2 (
    .a(ab_min),
    .b(cd_min),
    .zctr(1'b0),
    .z0(out_min),
    // Unused pins
    .z1(z1_2),
    .aeqb(aeqb2),
    .altb(altb2),
    .agtb(agtb2),
    .unordered(unordered2),
    .status0(status02),
    .status1(status12) );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
CMP_3 (
    .a(ab_max),
    .b(cd_max),
    .zctr(1'b0),
    .z1(out_max),
    // Unused pins
    .z0(z0_3),
    .aeqb(aeqb3),
    .altb(altb3),
    .agtb(agtb3),
    .unordered(unordered3),
    .status0(status03),
    .status1(status13) );


endmodule

module fp_dp3_sum4 (
    // System inputs
    clk, rst_n, mode, /*count_r,*/
    // Dot product inputs
    dp0_a, dp0_b, dp0_c, dp0_d, dp0_e, dp0_f,
    dp1_a, dp1_b, dp1_c, dp1_d, dp1_e, dp1_f,
    dp2_a, dp2_b, dp2_c, dp2_d, dp2_e, dp2_f,
    // Sum4 inputs
    sum_a, sum_b, sum_c, sum_d,
    // Outputs
    sum4_out, dp0_out, dp1_out, dp2_out
);
// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

input         clk, rst_n;
input   [1:0] mode;
input  [31:0] dp0_a, dp0_b, dp0_c, dp0_d, dp0_e, dp0_f;
input  [31:0] dp1_a, dp1_b, dp1_c, dp1_d, dp1_e, dp1_f;
input  [31:0] dp2_a, dp2_b, dp2_c, dp2_d, dp2_e, dp2_f;
input  [31:0] sum_a, sum_b, sum_c, sum_d;
output reg [31:0] sum4_out, dp0_out, dp1_out, dp2_out;

reg     [1:0] mode_r;
reg    [31:0] dp0_sum_r, dp1_sum_r, dp2_sum_r;
reg    [31:0] sum4_in_a, sum4_in_b, sum4_in_c, sum4_in_d;
wire   [31:0] sum4;
wire   [31:0] dp0_sum, dp1_sum, dp2_sum;
wire    [7:0] status_dp0, status_dp1, status_dp2, status_add4;


// TODO: Check whether using dp3 or 3xfp_mul will have better timing performance
DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
DP0 (
    .a(dp0_a),
    .b(dp0_b),
    .c(dp0_c),
    .d(dp0_d),
    .e(dp0_e),
    .f(dp0_f),
    .rnd(3'b000),
    .z(dp0_sum),
    .status(status_dp0) );
DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
DP1 (
    .a(dp1_a),
    .b(dp1_b),
    .c(dp1_c),
    .d(dp1_d),
    .e(dp1_e),
    .f(dp1_f),
    .rnd(3'b000),
    .z(dp1_sum),
    .status(status_dp1) );
DW_fp_dp3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
DP2 (
    .a(dp2_a),
    .b(dp2_b),
    .c(dp2_c),
    .d(dp2_d),
    .e(dp2_e),
    .f(dp2_f),
    .rnd(3'b000),
    .z(dp2_sum),
    .status(status_dp2) );

always @(*) begin
    case(mode)
        0: begin
            dp0_out = 0;
            dp1_out = 0;
            dp2_out = 0;
        end
        1: begin
            dp0_out = dp0_sum;
            dp1_out = dp1_sum;
            dp2_out = dp2_sum;
        end
        default: begin
            dp0_out = 0;
            dp1_out = 0;
            dp2_out = 0;
        end
    endcase
end

//--------------------------------------------------
// Add a stage between dp3 and sum4
//--------------------------------------------------

// All the inputs of sum4 need to be stored in DFF
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode_r <= 0;
        dp0_sum_r <= 0;
        dp1_sum_r <= 0;
        dp2_sum_r <= 0;
    end
    else begin
        mode_r <= mode;
        dp0_sum_r <= dp0_sum;
        dp1_sum_r <= dp1_sum;
        dp2_sum_r <= dp2_sum;
    end
end

always @(*) begin
    case(mode)
        0: begin
            sum4_in_a = dp0_sum_r;
            sum4_in_b = dp1_sum_r;
            sum4_in_c = dp2_sum_r;
            sum4_in_d = sum_d;
        end
        1: begin
            sum4_in_a = sum_a;
            sum4_in_b = sum_b;
            sum4_in_c = sum_c;
            sum4_in_d = sum_d;
        end
        default: begin
            sum4_in_a = 0;
            sum4_in_b = 0;
            sum4_in_c = 0;
            sum4_in_d = 0;
        end
    endcase
end


DW_fp_sum4 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
FP_ADDER4 (
    .a(sum4_in_a),
    .b(sum4_in_b),
    .c(sum4_in_c),
    .d(sum4_in_d),
    .rnd(3'b000),
    .z(sum4),
    .status(status_add4) );

always @(*) begin
    case(mode)
        0: sum4_out  = sum4;
        1: sum4_out  = sum4;
        default: sum4_out  = 0;
    endcase
end



endmodule


module sub_div_sub (
    // System inputs
    clk, rst_n, mode,
    // Normalize inputs
    flatten, flatten_min, flatten_max, sub_den_1,
    // Sigmoid or Tanh input
    exp_in,
    // Outputs
    sub_num_out, sub_den_out,
    norm, activation
);

// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

input          clk, rst_n;
input   [1:0]  mode;
input  [31:0]  flatten, flatten_min, flatten_max, sub_den_1;
input  [31:0]  exp_in;
output [31:0]  sub_num_out, sub_den_out;
output reg  [31:0]  norm, activation;

reg    [31:0]  sub_num_in0, sub_num_in1, sub_den_in0, sub_den_in1;
reg    [31:0]  sub_num_r, sub_den_r;
reg     [1:0]  mode_r;
wire   [31:0]  sub_num_out, sub_den_out, div_out;
wire   [31:0]  exp_posz, exp_negz;
wire    [7:0]  status_exp, status_div, status_sub_num, status_sub_den, status_exp_posz, status_exp_negz;

parameter ONE = {1'b0, 8'd127, 23'd0};

// Mode: 0 -> Distance,
//       1 -> Normalize,
//       2 -> Sigmoid,
//       3 -> Tanh
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch)
EXP_POSZ (
    .a(exp_in),
    .z(exp_posz),
    .status(status_exp_posz) );

DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch)
EXP_NEGZ (
    .a({~exp_in[31], exp_in[30:0]}),
    .z(exp_negz),
    .status(status_exp_negz) );

always @(*) begin
    case(mode)
        0: begin
            sub_num_in0 = flatten;
            sub_num_in1 = flatten_min;
            sub_den_in0 = flatten_max;
            sub_den_in1 = sub_den_1;
        end
        1: begin
            sub_num_in0 = flatten;
            sub_num_in1 = flatten_min;
            sub_den_in0 = flatten_max;
            sub_den_in1 = flatten_min;
        end
        2: begin
            sub_num_in0 = ONE;
            sub_num_in1 = 0;
            sub_den_in0 = ONE;
            sub_den_in1 = {~exp_negz[31], exp_negz[30:0]};
        end
        3: begin
            sub_num_in0 = exp_posz;
            sub_num_in1 = exp_negz;
            sub_den_in0 = exp_posz;
            sub_den_in1 = {~exp_negz[31], exp_negz[30:0]};
        end
        default: begin
            sub_num_in0 = 0;
            sub_num_in1 = 0;
            sub_den_in0 = 0;
            sub_den_in1 = 0;
        end
    endcase
end

DW_fp_sub #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
SUB_NUM (
    .a(sub_num_in0),
    .b(sub_num_in1),
    .rnd(3'b000),
    .z(sub_num_out),
    .status(status_sub_num) );

DW_fp_sub #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
SUB_DEN (
    .a(sub_den_in0),
    .b(sub_den_in1),
    .rnd(3'b000),
    .z(sub_den_out),
    .status(status_sub_den) );

//-------------------------------------------------------
// Add a stage between sub and div
//-------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode_r <= 2'd0;
        sub_num_r <= 0;
        sub_den_r <= 0;
    end
    else begin
        mode_r <= mode;
        sub_num_r <= sub_num_out;
        sub_den_r <= sub_den_out;
    end
end

DW_fp_div #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_faithful_round)
DIV(
    .a(sub_num_r),
    .b(sub_den_r),
    .rnd(3'b000),
    .z(div_out),
    .status(status_div) );

// Note: Use mode_r so that output has a delay-one-cycle effect.
always @(*) begin
    case(mode_r)
        1: norm = div_out;
        2,3: norm = 0;
        default: norm = 0;
    endcase
end

always @(*) begin
    case(mode_r)
        1: activation = 0;
        2,3: activation = div_out;
        default: activation = 0;
    endcase
end

endmodule




module Equalizer (
    in00, in01, in02,
    in10, in11, in12,
    in20, in21, in22,
    out );

input  [31:0] in00, in01, in02;
input  [31:0] in10, in11, in12;
input  [31:0] in20, in21, in22;
output [31:0] out;

wire [31:0] sum_r0, sum_r1, sum_r2, sum_all;
// Unused wires
wire [7:0] status_inst_r0, status_inst_r1, status_inst_r2, status_inst_all, status_div;

// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

DW_fp_sum3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
SUM_R0 (
    .a(in00),
    .b(in01),
    .c(in02),
    .rnd(3'b000),
    .z(sum_r0),
    .status(status_inst_r0) );

DW_fp_sum3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
SUM_R1 (
    .a(in10),
    .b(in11),
    .c(in12),
    .rnd(3'b000),
    .z(sum_r1),
    .status(status_inst_r1) );

DW_fp_sum3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
SUM_R2 (
    .a(in20),
    .b(in21),
    .c(in22),
    .rnd(3'b000),
    .z(sum_r2),
    .status(status_inst_r2) );

DW_fp_sum3 #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch_type)
SUM_ALL (
    .a(sum_r0),
    .b(sum_r1),
    .c(sum_r2),
    .rnd(3'b000),
    .z(sum_all),
    .status(status_inst_all) );



DW_fp_div #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_faithful_round)
DIV(
    .a(sum_all),
    .b(32'h41100000),  // Number: 9
    .rnd(3'b000),
    .z(out),
    .status(status_div) );

endmodule