//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Lab04 Exercise		: Siamese Neural Network
//   Author     		: Jia-Yu Lee (maggie8905121@gmail.com)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : SNN.v
//   Module Name : SNN
//   Release version : V1.0 (Release Date: 2023-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

`define STATE_BITS 4

module SNN(
    //Input Port
    clk,
    rst_n,
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

// // Time Stamps
// parameter CONV_FIRST_DATA = 9;
// parameter CONV_FIRST_DATA_OUTPUT = 11; // Note: img_box dp kernel_box @9 -> sum4 @10 -> feature_map_r will update @11

// parameter CONV_LAST_DATA = 104;
// parameter CONV_LAST_DATA_OUTPUT = 106; // Note: img_box dp kernel_box @104 -> sum4 @105 -> feature_map_r will update @106

// parameter MAXPOOL0_00 = 48;
// parameter MAXPOOL0_01 = 50;
// parameter MAXPOOL0_10 = 56;
// parameter MAXPOOL0_11 = 58;
// parameter MAXPOOL1_00 = 96;
// parameter MAXPOOL1_01 = 98;
// parameter MAXPOOL1_10 = 104;
// parameter MAXPOOL1_11 = 106;


//---------------------------------------------------------------------
//   INTEGER
//---------------------------------------------------------------------
integer i, j, k;

//---------------------------------------------------------------------
//   INPUTS & OUTPUTS
//---------------------------------------------------------------------
input rst_n, clk, in_valid;
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
reg  [31:0] max_pool_r        [0:1][0:1][0:1];
reg  [31:0] flatten_r         [0:1][0:3];

reg   [7:0] count_r;
reg   [3:0] center_row, center_col;
reg   [3:0] feature_row, feature_col;
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

// Output stage register
reg  [31:0] out_r;



//---------------------------------------------------------------------
//   STORAGE - IMG
//---------------------------------------------------------------------
/* img_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_r[0][0] <= 0;    img_r[0][1] <= 0;    img_r[0][2] <= 0;    img_r[0][3] <= 0;
        img_r[1][0] <= 0;    img_r[1][1] <= 0;    img_r[1][2] <= 0;    img_r[1][3] <= 0;
        img_r[2][0] <= 0;    img_r[2][1] <= 0;    img_r[2][2] <= 0;    img_r[2][3] <= 0;
        img_r[3][0] <= 0;    img_r[3][1] <= 0;    img_r[3][2] <= 0;    img_r[3][3] <= 0;
    end
    else if (FINISH) begin
        img_r[0][0] <= 0;    img_r[0][1] <= 0;    img_r[0][2] <= 0;    img_r[0][3] <= 0;
        img_r[1][0] <= 0;    img_r[1][1] <= 0;    img_r[1][2] <= 0;    img_r[1][3] <= 0;
        img_r[2][0] <= 0;    img_r[2][1] <= 0;    img_r[2][2] <= 0;    img_r[2][3] <= 0;
        img_r[3][0] <= 0;    img_r[3][1] <= 0;    img_r[3][2] <= 0;    img_r[3][3] <= 0;
    end
    else if (in_valid) begin
        case (count_r)
            // Row = 0
            0,  16, 32, 48, 64, 80: img_r[0][0] <= Img;
            1,  17, 33, 49, 65, 81: img_r[0][1] <= Img;
            2,  18, 34, 50, 66, 82: img_r[0][2] <= Img;
            3,  19, 35, 51, 67, 83: img_r[0][3] <= Img;
            // Row = 1
            4,  20, 36, 52, 68, 84: img_r[1][0] <= Img;
            5,  21, 37, 53, 69, 85: img_r[1][1] <= Img;
            6,  22, 38, 54, 70, 86: img_r[1][2] <= Img;
            7,  23, 39, 55, 71, 87: img_r[1][3] <= Img;
            // Row = 2
            8,  24, 40, 56, 72, 88: img_r[2][0] <= Img;
            9,  25, 41, 57, 73, 89: img_r[2][1] <= Img;
            10, 26, 42, 58, 74, 90: img_r[2][2] <= Img;
            11, 27, 43, 59, 75, 91: img_r[2][3] <= Img;
            // Row = 3
            12, 28, 44, 60, 76, 92: img_r[3][0] <= Img;
            13, 29, 45, 61, 77, 93: img_r[3][1] <= Img;
            14, 30, 46, 62, 78, 94: img_r[3][2] <= Img;
            15, 31, 47, 63, 79, 95: img_r[3][3] <= Img;
            default: begin
                img_r[0][0] <= img_r[0][0];    img_r[0][1] <= img_r[0][1];    img_r[0][2] <= img_r[0][2];    img_r[0][3] <= img_r[0][3];
                img_r[1][0] <= img_r[1][0];    img_r[1][1] <= img_r[1][1];    img_r[1][2] <= img_r[1][2];    img_r[1][3] <= img_r[1][3];
                img_r[2][0] <= img_r[2][0];    img_r[2][1] <= img_r[2][1];    img_r[2][2] <= img_r[2][2];    img_r[2][3] <= img_r[2][3];
                img_r[3][0] <= img_r[3][0];    img_r[3][1] <= img_r[3][1];    img_r[3][2] <= img_r[3][2];    img_r[3][3] <= img_r[3][3];
            end
        endcase
    end
    else begin
        img_r[0][0] <= img_r[0][0];    img_r[0][1] <= img_r[0][1];    img_r[0][2] <= img_r[0][2];    img_r[0][3] <= img_r[0][3];
        img_r[1][0] <= img_r[1][0];    img_r[1][1] <= img_r[1][1];    img_r[1][2] <= img_r[1][2];    img_r[1][3] <= img_r[1][3];
        img_r[2][0] <= img_r[2][0];    img_r[2][1] <= img_r[2][1];    img_r[2][2] <= img_r[2][2];    img_r[2][3] <= img_r[2][3];
        img_r[3][0] <= img_r[3][0];    img_r[3][1] <= img_r[3][1];    img_r[3][2] <= img_r[3][2];    img_r[3][3] <= img_r[3][3];
    end
end

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

//---------------------------------------------------------------------
//   STORAGE - KERNEL, WEIGHT, OPT
//---------------------------------------------------------------------
/* kernel_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 3; i = i + 1) begin
            for (j = 0; j < 3; j = j + 1) begin
                for (k = 0; k < 3; k = k + 1) begin
                    kernel_r[i][j][k] <= 0;
                end
            end
        end
    end
    else if (FINISH) begin
        for (i = 0; i < 3; i = i + 1) begin
            for (j = 0; j < 3; j = j + 1) begin
                for (k = 0; k < 3; k = k + 1) begin
                    kernel_r[i][j][k] <= 0;
                end
            end
        end
    end
    else if (in_valid) begin
        case(count_r)
            0: kernel_r[0][0][0] <= Kernel;     9: kernel_r[1][0][0] <= Kernel;    18: kernel_r[2][0][0] <= Kernel;
            1: kernel_r[0][0][1] <= Kernel;    10: kernel_r[1][0][1] <= Kernel;    19: kernel_r[2][0][1] <= Kernel;
            2: kernel_r[0][0][2] <= Kernel;    11: kernel_r[1][0][2] <= Kernel;    20: kernel_r[2][0][2] <= Kernel;
            3: kernel_r[0][1][0] <= Kernel;    12: kernel_r[1][1][0] <= Kernel;    21: kernel_r[2][1][0] <= Kernel;
            4: kernel_r[0][1][1] <= Kernel;    13: kernel_r[1][1][1] <= Kernel;    22: kernel_r[2][1][1] <= Kernel;
            5: kernel_r[0][1][2] <= Kernel;    14: kernel_r[1][1][2] <= Kernel;    23: kernel_r[2][1][2] <= Kernel;
            6: kernel_r[0][2][0] <= Kernel;    15: kernel_r[1][2][0] <= Kernel;    24: kernel_r[2][2][0] <= Kernel;
            7: kernel_r[0][2][1] <= Kernel;    16: kernel_r[1][2][1] <= Kernel;    25: kernel_r[2][2][1] <= Kernel;
            8: kernel_r[0][2][2] <= Kernel;    17: kernel_r[1][2][2] <= Kernel;    26: kernel_r[2][2][2] <= Kernel;
            default: begin
                for (i = 0; i < 3; i = i + 1) begin
                    for (j = 0; j < 3; j = j + 1) begin
                        for (k = 0; k < 3; k = k + 1) begin
                            kernel_r[i][j][k] <= kernel_r[i][j][k];
                        end
                    end
                end
            end
        endcase
    end
    else begin
        for (i = 0; i < 3; i = i + 1) begin
            for (j = 0; j < 3; j = j + 1) begin
                for (k = 0; k < 3; k = k + 1) begin
                    kernel_r[i][j][k] <= kernel_r[i][j][k];
                end
            end
        end
    end
end

/* weight_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        weight_r[0][0] <= 0;    weight_r[0][1] <= 0;
        weight_r[1][0] <= 0;    weight_r[1][1] <= 0;
    end
    else if (FINISH) begin
        weight_r[0][0] <= 0;    weight_r[0][1] <= 0;
        weight_r[1][0] <= 0;    weight_r[1][1] <= 0;
    end
    else if (in_valid) begin
        case(count_r)
            0: weight_r[0][0] <= Weight;
            1: weight_r[0][1] <= Weight;
            2: weight_r[1][0] <= Weight;
            3: weight_r[1][1] <= Weight;
            default: begin
                weight_r[0][0] <= weight_r[0][0];    weight_r[0][1] <= weight_r[0][1];
                weight_r[1][0] <= weight_r[1][0];    weight_r[1][1] <= weight_r[1][1];
            end
        endcase
    end
    else begin
        weight_r[0][0] <= weight_r[0][0];    weight_r[0][1] <= weight_r[0][1];
        weight_r[1][0] <= weight_r[1][0];    weight_r[1][1] <= weight_r[1][1];
    end
end

/* opt_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        opt_r <= 2'd0;
    else if (FINISH)
        opt_r <= 2'd0;
    else if (in_valid && count_r == 0)
        opt_r <= Opt;
    else
        opt_r <= opt_r;
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
        default: begin center_row = 'dx; center_col = 'dx; end
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
        case(count_r)
        106: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[0][0][0]; dp0_b = weight_r[0][0]; dp0_c = max_pool_r[0][0][1]; dp0_d = weight_r[1][0]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[0][0][0]; dp1_b = weight_r[0][1]; dp1_c = max_pool_r[0][0][1]; dp1_d = weight_r[1][1]; dp1_e = 0; dp1_f = 0;
            dp2_a = max_pool_r[0][1][0]; dp2_b = weight_r[0][0]; dp2_c = max_pool_r[0][1][1]; dp2_d = weight_r[1][0]; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end
        107: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[0][1][0]; dp0_b = weight_r[0][1]; dp0_c = max_pool_r[0][1][1]; dp0_d = weight_r[1][1]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[1][0][0]; dp1_b = weight_r[0][0]; dp1_c = max_pool_r[1][0][1]; dp1_d = weight_r[1][0]; dp1_e = 0; dp1_f = 0;
            dp2_a = max_pool_r[1][0][0]; dp2_b = weight_r[0][1]; dp2_c = max_pool_r[1][0][1]; dp2_d = weight_r[1][1]; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end
        108: begin
            fp_dp3_sum4_mode = 2'd1;
            dp0_a = max_pool_r[1][1][0]; dp0_b = weight_r[0][0]; dp0_c = max_pool_r[1][1][1]; dp0_d = weight_r[1][0]; dp0_e = 0; dp0_f = 0;
            dp1_a = max_pool_r[1][1][0]; dp1_b = weight_r[0][1]; dp1_c = max_pool_r[1][1][1]; dp1_d = weight_r[1][1]; dp1_e = 0; dp1_f = 0;
            dp2_a = 0; dp2_b = 0; dp2_c = 0; dp2_d = 0; dp2_e = 0; dp2_f = 0;
            sum_a = 0; sum_b = 0; sum_c = 0; sum_d = 0;
        end

        // Distance Sum
        119: begin
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
//   FEATURE MAP
//---------------------------------------------------------------------
/* feature_map_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 2; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                for (k = 0; k < 4; k = k + 1) begin
                    feature_map_r[i][j][k] <= 0;
                end
            end
        end
    end
    else if (FINISH) begin
        for (i = 0; i < 2; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                for (k = 0; k < 4; k = k + 1) begin
                    feature_map_r[i][j][k] <= 0;
                end
            end
        end
    end
    else if (9+1 <= count_r && count_r <= 56+1) begin
        feature_map_r[0][feature_row][feature_col] <= sum4;
    end
    else if (57+1 <= count_r && count_r <= 104+1) begin
        feature_map_r[1][feature_row][feature_col] <= sum4;
    end
    else begin
        for (i = 0; i < 2; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                for (k = 0; k < 4; k = k + 1) begin
                    feature_map_r[i][j][k] <= feature_map_r[i][j][k];
                end
            end
        end
    end
end
always @(*) begin
    if      (9+1  <= count_r && count_r <= 56+1)  feature_map_orig = feature_map_r[0][feature_row][feature_col];
    else if (57+1 <= count_r && count_r <= 104+1) feature_map_orig = feature_map_r[1][feature_row][feature_col];
    else feature_map_orig = 0;
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
        default: begin feature_row = 'dx; feature_col = 'dx; end
    endcase
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
        // Max pooling 1st page
        48: begin
            fp_sort0_in_a = feature_map_r[0][0][0];
            fp_sort0_in_b = feature_map_r[0][0][1];
            fp_sort0_in_c = feature_map_r[0][1][0];
            fp_sort0_in_d = feature_map_r[0][1][1];
        end
        50: begin
            fp_sort0_in_a = feature_map_r[0][0][2];
            fp_sort0_in_b = feature_map_r[0][0][3];
            fp_sort0_in_c = feature_map_r[0][1][2];
            fp_sort0_in_d = feature_map_r[0][1][3];
        end
        56: begin
            fp_sort0_in_a = feature_map_r[0][2][0];
            fp_sort0_in_b = feature_map_r[0][2][1];
            fp_sort0_in_c = feature_map_r[0][3][0];
            fp_sort0_in_d = feature_map_r[0][3][1];
        end
        58: begin
            fp_sort0_in_a = feature_map_r[0][2][2];
            fp_sort0_in_b = feature_map_r[0][2][3];
            fp_sort0_in_c = feature_map_r[0][3][2];
            fp_sort0_in_d = feature_map_r[0][3][3];
        end
        // Max pooling 2nd page
        96: begin
            fp_sort0_in_a = feature_map_r[1][0][0];
            fp_sort0_in_b = feature_map_r[1][0][1];
            fp_sort0_in_c = feature_map_r[1][1][0];
            fp_sort0_in_d = feature_map_r[1][1][1];
        end
        98: begin
            fp_sort0_in_a = feature_map_r[1][0][2];
            fp_sort0_in_b = feature_map_r[1][0][3];
            fp_sort0_in_c = feature_map_r[1][1][2];
            fp_sort0_in_d = feature_map_r[1][1][3];
        end
        104: begin
            fp_sort0_in_a = feature_map_r[1][2][0];
            fp_sort0_in_b = feature_map_r[1][2][1];
            fp_sort0_in_c = feature_map_r[1][3][0];
            fp_sort0_in_d = feature_map_r[1][3][1];
        end
        106: begin
            fp_sort0_in_a = feature_map_r[1][2][2];
            fp_sort0_in_b = feature_map_r[1][2][3];
            fp_sort0_in_c = feature_map_r[1][3][2];
            fp_sort0_in_d = feature_map_r[1][3][3];
        end
        109: begin
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
        109: begin
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
//   MAX POOLING
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        max_pool_r[0][0][0] <= 0;  max_pool_r[0][0][1] <= 0;
        max_pool_r[0][1][0] <= 0;  max_pool_r[0][1][1] <= 0;
        max_pool_r[1][0][0] <= 0;  max_pool_r[1][0][1] <= 0;
        max_pool_r[1][1][0] <= 0;  max_pool_r[1][1][1] <= 0;
    end
    else if (FINISH) begin
        max_pool_r[0][0][0] <= 0;  max_pool_r[0][0][1] <= 0;
        max_pool_r[0][1][0] <= 0;  max_pool_r[0][1][1] <= 0;
        max_pool_r[1][0][0] <= 0;  max_pool_r[1][0][1] <= 0;
        max_pool_r[1][1][0] <= 0;  max_pool_r[1][1][1] <= 0;
    end
    else begin
        case(count_r)
            48:  max_pool_r[0][0][0] <= fp_sort0_out_max;
            50:  max_pool_r[0][0][1] <= fp_sort0_out_max;
            56:  max_pool_r[0][1][0] <= fp_sort0_out_max;
            58:  max_pool_r[0][1][1] <= fp_sort0_out_max;
            96:  max_pool_r[1][0][0] <= fp_sort0_out_max;
            98:  max_pool_r[1][0][1] <= fp_sort0_out_max;
            104: max_pool_r[1][1][0] <= fp_sort0_out_max;
            106: max_pool_r[1][1][1] <= fp_sort0_out_max;
            default: begin
                max_pool_r[0][0][0] <= max_pool_r[0][0][0];  max_pool_r[0][0][1] <= max_pool_r[0][0][1];
                max_pool_r[0][1][0] <= max_pool_r[0][1][0];  max_pool_r[0][1][1] <= max_pool_r[0][1][1];
                max_pool_r[1][0][0] <= max_pool_r[1][0][0];  max_pool_r[1][0][1] <= max_pool_r[1][0][1];
                max_pool_r[1][1][0] <= max_pool_r[1][1][0];  max_pool_r[1][1][1] <= max_pool_r[1][1][1];
            end
        endcase
    end
end

//---------------------------------------------------------------------
//   FLATTEN
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        flatten_r[0][0] <= 0;    flatten_r[1][0] <= 0;
        flatten_r[0][1] <= 0;    flatten_r[1][1] <= 0;
        flatten_r[0][2] <= 0;    flatten_r[1][2] <= 0;
        flatten_r[0][3] <= 0;    flatten_r[1][3] <= 0;
    end
    else if (FINISH) begin
        flatten_r[0][0] <= 0;    flatten_r[1][0] <= 0;
        flatten_r[0][1] <= 0;    flatten_r[1][1] <= 0;
        flatten_r[0][2] <= 0;    flatten_r[1][2] <= 0;
        flatten_r[0][3] <= 0;    flatten_r[1][3] <= 0;
    end
    else begin
        case(count_r)
            // Flattening
            106: begin
                flatten_r[0][0] <= dp0_out;
                flatten_r[0][1] <= dp1_out;
                flatten_r[0][2] <= dp2_out;
            end
            107: begin
                flatten_r[0][3] <= dp0_out;
                flatten_r[1][0] <= dp1_out;
                flatten_r[1][1] <= dp2_out;
            end
            108: begin
                flatten_r[1][2] <= dp0_out;
                flatten_r[1][3] <= dp1_out;
            end
            // //109: store min and maxs
            // 109: wait one cycle
            // Normalizing
            110: begin flatten_r[0][0] <= sds0_norm;    flatten_r[1][0] <= sds1_norm; end
            111: begin flatten_r[0][1] <= sds0_norm;    flatten_r[1][1] <= sds1_norm; end
            112: begin flatten_r[0][2] <= sds0_norm;    flatten_r[1][2] <= sds1_norm; end
            113: begin flatten_r[0][3] <= sds0_norm;    flatten_r[1][3] <= sds1_norm; end
            // Activation Function
            114: begin flatten_r[0][0] <= sds0_activation;    flatten_r[1][0] <= sds1_activation; end
            115: begin flatten_r[0][1] <= sds0_activation;    flatten_r[1][1] <= sds1_activation; end
            116: begin flatten_r[0][2] <= sds0_activation;    flatten_r[1][2] <= sds1_activation; end
            117: begin flatten_r[0][3] <= sds0_activation;    flatten_r[1][3] <= sds1_activation; end
            // Distance: Absolute value => {0, exp, sig}
            118: begin
                flatten_r[0][0] <= {1'd0, sds0_sub_num_out[30:0]};
                flatten_r[0][1] <= {1'd0, sds0_sub_den_out[30:0]};
                flatten_r[0][2] <= {1'd0, sds1_sub_num_out[30:0]};
                flatten_r[0][3] <= {1'd0, sds1_sub_den_out[30:0]};
            end
            default: begin
                flatten_r[0][0] <= flatten_r[0][0];    flatten_r[1][0] <= flatten_r[1][0];
                flatten_r[0][1] <= flatten_r[0][1];    flatten_r[1][1] <= flatten_r[1][1];
                flatten_r[0][2] <= flatten_r[0][2];    flatten_r[1][2] <= flatten_r[1][2];
                flatten_r[0][3] <= flatten_r[0][3];    flatten_r[1][3] <= flatten_r[1][3];
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        flatten_min0_r <= 0;
        flatten_max0_r <= 0;
        flatten_min1_r <= 0;
        flatten_max1_r <= 0;
    end
    else if (FINISH) begin
        flatten_min0_r <= 0;
        flatten_max0_r <= 0;
        flatten_min1_r <= 0;
        flatten_max1_r <= 0;
    end
    else if (count_r == 109) begin
        flatten_min0_r <= fp_sort0_out_min;
        flatten_max0_r <= fp_sort0_out_max;
        flatten_min1_r <= fp_sort1_out_min;
        flatten_max1_r <= fp_sort1_out_max;
    end
    else begin
        flatten_min0_r <= flatten_min0_r;
        flatten_max0_r <= flatten_max0_r;
        flatten_min1_r <= flatten_min1_r;
        flatten_max1_r <= flatten_max1_r;
    end
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
        109: begin // feed the min_max from combinational first
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][0];    sds1_flatten     = flatten_r[1][0];
            sds0_flatten_min = fp_sort0_out_min;     sds1_flatten_min = fp_sort1_out_min;
            sds0_flatten_max = fp_sort0_out_max;     sds1_flatten_max = fp_sort1_out_max;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        110: begin // then feed the min_max from DFF
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][1];    sds1_flatten     = flatten_r[1][1];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        111: begin
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][2];    sds1_flatten     = flatten_r[1][2];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        112: begin
            sds0_mode = 2'd1;                      sds1_mode = 2'd1;
            sds0_flatten     = flatten_r[0][3];    sds1_flatten     = flatten_r[1][3];
            sds0_flatten_min = flatten_min0_r;     sds1_flatten_min = flatten_min1_r;
            sds0_flatten_max = flatten_max0_r;     sds1_flatten_max = flatten_max1_r;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = 0;                       sds1_exp_in = 0;
        end
        // Activation function
        113: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][0];         sds1_exp_in = flatten_r[1][0];
        end
        114: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][1];         sds1_exp_in = flatten_r[1][1];
        end
        115: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][2];         sds1_exp_in = flatten_r[1][2];
        end
        116: begin
            sds0_mode = (opt_r[1]) ? 2'd3 : 2'd2;  sds1_mode = (opt_r[1]) ? 2'd3 : 2'd2;
            sds0_flatten     = 0;                  sds1_flatten     = 0;
            sds0_flatten_min = 0;                  sds1_flatten_min = 0;
            sds0_flatten_max = 0;                  sds1_flatten_max = 0;
            sds0_sub_den_1   = 0;                  sds1_sub_den_1   = 0;
            sds0_exp_in = flatten_r[0][3];         sds1_exp_in = flatten_r[1][3];
        end
        // Distance
        118: begin
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
    if (!rst_n) begin
        count_r <= 0;
    end
    else if (FINISH) begin
        count_r <= 0;
    end
    else if (in_valid || count_r != 0) begin
        count_r <= count_r + 1'd1;
    end
    else begin
        count_r <= count_r;
    end
end

//---------------------------------------------------------------------
//   OUTPUTS
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) out_r <= 0;
    else out_r <= sum4;
end

assign FINISH = (count_r == 120);
always @(*) begin
    if (FINISH) begin
        out_valid = 1;
        out = out_r;
    end
    else begin
        out_valid = 0;
        out = 0;
    end
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