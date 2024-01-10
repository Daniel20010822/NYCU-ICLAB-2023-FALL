//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Lab05 Exercise		: CAD
//   Author     		: Cheng-Tsang Wu
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : CAD.v
//   Module Name : CAD
//   Release version : V1.0 (Release Date: 2023-10)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module CAD(
    clk,
    rst_n,
    in_valid,
    in_valid2,
    matrix_size,
    matrix,
    matrix_idx,
    mode,
    out_valid,
    out_value
    );

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------
// FSM
`define STATE_BITS 6
parameter IDLE                 = `STATE_BITS'd0;
parameter FETCH_IMG08          = `STATE_BITS'd1;
parameter FETCH_IMG16          = `STATE_BITS'd2;
parameter FETCH_IMG32          = `STATE_BITS'd3;
parameter FETCH_KERNEL         = `STATE_BITS'd4;
parameter INVALID2_0           = `STATE_BITS'd5;
parameter INVALID2_1           = `STATE_BITS'd6;
parameter CONV08               = `STATE_BITS'd7;
parameter CONV16               = `STATE_BITS'd8;
parameter CONV32               = `STATE_BITS'd9;
parameter DECONV08             = `STATE_BITS'd10;
parameter DECONV16             = `STATE_BITS'd11;
parameter DECONV32             = `STATE_BITS'd12;


// CONSTANTS
parameter READ  = 3'b111; // {CS, WEB, OE}
parameter WRITE = 3'b101; // {CS, WEB, OE}
parameter IMG_SIZE08 =  1024;  //  8* 8*16
parameter IMG_SIZE16 =  4096;  // 16*16*16
parameter IMG_SIZE32 = 16384;  // 32*32*16
parameter KERNEL_SIZE =  400;  //  5* 5*16
parameter IMG_LAST_WORD08 =  127;  //  8* 8*16 / 8 - 1
parameter IMG_LAST_WORD16 =  511;  // 16*16*16 / 8 - 1
parameter IMG_LAST_WORD32 = 2047;  // 32*32*16 / 8 - 1
parameter KERNEL_LAST_WORD =  79;  //  5* 5*16 / 5 - 1

parameter GOLDEN_PATNUM_CONV08 = 3;
parameter GOLDEN_PATNUM_CONV16 = 35;
parameter GOLDEN_PATNUM_CONV32 = 195;
parameter GOLDEN_PATNUM_DECONV08 = 143;
parameter GOLDEN_PATNUM_DECONV16 = 399;
parameter GOLDEN_PATNUM_DECONV32 = 1295;
integer i, j;

//---------------------------------------------------------------------
//   INPUTS & OUTPUTS
//---------------------------------------------------------------------
input         clk, rst_n, in_valid, in_valid2;
input  [1:0]  matrix_size;
input  [7:0]  matrix;
input  [3:0]  matrix_idx;
input         mode;
output reg    out_valid;
output reg    out_value;

//---------------------------------------------------------------------
//   REG & WIRE DECLARATIONS
//---------------------------------------------------------------------
// FSM
reg         [`STATE_BITS-1:0]  cs, ns;

// STORE INPUT INFO.
reg          [ 1:0]  matrix_size_r;
reg          [ 3:0]  img_idx_r, ker_idx_r;
reg                  mode_r;

// CONV
reg  signed  [ 7:0]  img_6X8_r          [0:5][0:7];
reg  signed  [ 7:0]  img_6X8_next_r     [0:5][0:7];
reg  signed  [ 7:0]  img_6X16_r         [0:5][0:15];
reg  signed  [ 7:0]  img_6X16_next_r    [0:5][0:15];
reg  signed  [ 7:0]  ker_5X5_r          [0:4][0:4];
reg  signed  [20:0]  feat_2X2_r         [0:1][0:1];
wire signed  [20:0]  feat_add;
wire signed  [20:0]  maxpool;
reg  signed  [20:0]  maxpool_r;
reg  signed  [ 7:0]  img_dp5_in         [0:4];
reg  signed  [ 7:0]  ker_dp5_in         [0:4];
wire signed  [20:0]  candidate0, candidate1;

// DECONV
reg  signed  [ 7:0]  img_5X32_r         [0:4][0:31];
reg  signed  [ 7:0]  img_5X40_pad       [0:4][0:39];
reg  signed  [ 7:0]  img_5X5_r          [0:4][0:4];
reg  signed  [20:0]  sum_r;
reg  signed  [20:0]  deconv_result_r;


// COUNTERS
reg          [13:0]  conv_cnt_r;
reg          [ 3:0]  cnt_10_r;
reg          [ 4:0]  cnt_20_r;
reg          [ 4:0]  shift_col_cnt_r;
reg          [ 4:0]  change_row_cnt_r;
reg          [ 4:0]  out_cnt_20_r;
reg          [12:0]  set_cnt_r;
reg          [ 3:0]  pat_cnt_r;
reg          [15:0]  deconv_cnt_r;
reg          [ 5:0]  deconv_col_r;
reg          [ 5:0]  deconv_row_r;

// ADDRESS CONTROL
reg          [10:0]  IMG_START_ADDR_r;
reg          [ 6:0]  KER_START_ADDR_r;
reg          [10:0]  img_waddr_r, img_raddr;
reg          [ 6:0]  ker_waddr_r, ker_raddr;
reg          [ 7:0]  sram_buffer  [0:7];
reg          [ 2:0]  round_r;

// SRAMs
reg          [10:0]  ADDR_IMG;
reg          [ 6:0]  ADDR_KER;
reg                  WEB_IMG, OE_IMG, CS_IMG;
reg                  WEB_KER, OE_KER, CS_KER;
reg          [63:0]  DI_IMG;
reg          [39:0]  DI_KER;
wire         [63:0]  DO_IMG;
wire         [39:0]  DO_KER;

// OUTPUT REG
reg                  out_valid_temp;
reg                  out_value_temp;

// BOOL EXPRESSIONS
reg                 IMAGE_MEM_WRITE, KERNEL_MEM_WRITE;


//---------------------------------------------------------------------
//   FSM - MAIN
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs <= IDLE;
    else        cs <= ns;
end
always @(*) begin
    case(cs)
        // IDLE
        IDLE: begin
            if (in_valid) begin
                case(matrix_size)
                    2'd0: ns = FETCH_IMG08;
                    2'd1: ns = FETCH_IMG16;
                    2'd2: ns = FETCH_IMG32;
                    default: ns = IDLE;
                endcase
            end
            else begin
                ns = IDLE;
            end
        end

        // Fetch input
        FETCH_IMG08:
            ns = (img_waddr_r == IMG_LAST_WORD08 && (&round_r)) ? FETCH_KERNEL : FETCH_IMG08;
        FETCH_IMG16:
            ns = (img_waddr_r == IMG_LAST_WORD16 && (&round_r)) ? FETCH_KERNEL : FETCH_IMG16;
        FETCH_IMG32:
            ns = (img_waddr_r == IMG_LAST_WORD32 && (&round_r)) ? FETCH_KERNEL : FETCH_IMG32;
        FETCH_KERNEL:
            ns = (ker_waddr_r == KERNEL_LAST_WORD && (round_r == 4)) ? INVALID2_0 : FETCH_KERNEL;

        // Fetch options
        INVALID2_0:
            ns = (in_valid2) ? INVALID2_1 : INVALID2_0;
        INVALID2_1: begin
            case(matrix_size_r)
                2'd0: ns = (mode_r) ? DECONV08 : CONV08;
                2'd1: ns = (mode_r) ? DECONV16 : CONV16;
                2'd2: ns = (mode_r) ? DECONV32 : CONV32;
                default: ns = IDLE;
            endcase
        end

        CONV08:
            ns = (set_cnt_r == GOLDEN_PATNUM_CONV08 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : CONV08;
        CONV16:
            ns = (set_cnt_r == GOLDEN_PATNUM_CONV16 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : CONV16;
        CONV32:
            ns = (set_cnt_r == GOLDEN_PATNUM_CONV32 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : CONV32;

        DECONV08:
            ns = (set_cnt_r == GOLDEN_PATNUM_DECONV08 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : DECONV08;
        DECONV16:
            ns = (set_cnt_r == GOLDEN_PATNUM_DECONV16 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : DECONV16;
        DECONV32:
            ns = (set_cnt_r == GOLDEN_PATNUM_DECONV32 && out_cnt_20_r == 19) ?
                 ((&pat_cnt_r) ? IDLE : INVALID2_0 ) : DECONV32;



        default: ns = IDLE;
    endcase
end


//---------------------------------------------------------------------
//   OPTIONs
//---------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        matrix_size_r <= 2'd0;
    else if (in_valid && cs == IDLE)
        matrix_size_r <= matrix_size;
    else
        matrix_size_r <= matrix_size_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        mode_r <= 0;
    else if (in_valid2 && cs == INVALID2_0)
        mode_r <= mode;
    else
        mode_r <= mode_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_idx_r <= 0;    ker_idx_r <= 0;
    end
    else if (in_valid2) begin
        img_idx_r <= ker_idx_r;    ker_idx_r <= matrix_idx;
    end
    else begin
        img_idx_r <= img_idx_r;    ker_idx_r <= ker_idx_r;
    end
end

//---------------------------------------------------------------------
//   CONV
//---------------------------------------------------------------------
/* img_6X8_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                img_6X8_r[i][j] <= 0;
            end
        end
    end
    else if (cs == CONV08) begin
        if (conv_cnt_r <= 6) begin
            case(conv_cnt_r)
                1: {img_6X8_r[0][0], img_6X8_r[0][1], img_6X8_r[0][2], img_6X8_r[0][3], img_6X8_r[0][4], img_6X8_r[0][5], img_6X8_r[0][6], img_6X8_r[0][7]} <= DO_IMG;
                2: {img_6X8_r[1][0], img_6X8_r[1][1], img_6X8_r[1][2], img_6X8_r[1][3], img_6X8_r[1][4], img_6X8_r[1][5], img_6X8_r[1][6], img_6X8_r[1][7]} <= DO_IMG;
                3: {img_6X8_r[2][0], img_6X8_r[2][1], img_6X8_r[2][2], img_6X8_r[2][3], img_6X8_r[2][4], img_6X8_r[2][5], img_6X8_r[2][6], img_6X8_r[2][7]} <= DO_IMG;
                4: {img_6X8_r[3][0], img_6X8_r[3][1], img_6X8_r[3][2], img_6X8_r[3][3], img_6X8_r[3][4], img_6X8_r[3][5], img_6X8_r[3][6], img_6X8_r[3][7]} <= DO_IMG;
                5: {img_6X8_r[4][0], img_6X8_r[4][1], img_6X8_r[4][2], img_6X8_r[4][3], img_6X8_r[4][4], img_6X8_r[4][5], img_6X8_r[4][6], img_6X8_r[4][7]} <= DO_IMG;
                6: {img_6X8_r[5][0], img_6X8_r[5][1], img_6X8_r[5][2], img_6X8_r[5][3], img_6X8_r[5][4], img_6X8_r[5][5], img_6X8_r[5][6], img_6X8_r[5][7]} <= DO_IMG;
            endcase
        end
        else if (cnt_10_r == 9 && shift_col_cnt_r == 3) begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    img_6X8_r[i][j] <= img_6X8_next_r[i][j];
                end
            end
        end
        else if (cnt_10_r == 9) begin
            case(shift_col_cnt_r)
                0,1,2: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                end
            endcase
        end
        else begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    img_6X8_r[i][j] <= img_6X8_r[i][j];
                end
            end
        end
    end
    else if (cs == CONV16) begin
        if (conv_cnt_r <= 6) begin
            case(conv_cnt_r)
                1: {img_6X8_r[0][0], img_6X8_r[0][1], img_6X8_r[0][2], img_6X8_r[0][3], img_6X8_r[0][4], img_6X8_r[0][5], img_6X8_r[0][6], img_6X8_r[0][7]} <= DO_IMG;
                2: {img_6X8_r[1][0], img_6X8_r[1][1], img_6X8_r[1][2], img_6X8_r[1][3], img_6X8_r[1][4], img_6X8_r[1][5], img_6X8_r[1][6], img_6X8_r[1][7]} <= DO_IMG;
                3: {img_6X8_r[2][0], img_6X8_r[2][1], img_6X8_r[2][2], img_6X8_r[2][3], img_6X8_r[2][4], img_6X8_r[2][5], img_6X8_r[2][6], img_6X8_r[2][7]} <= DO_IMG;
                4: {img_6X8_r[3][0], img_6X8_r[3][1], img_6X8_r[3][2], img_6X8_r[3][3], img_6X8_r[3][4], img_6X8_r[3][5], img_6X8_r[3][6], img_6X8_r[3][7]} <= DO_IMG;
                5: {img_6X8_r[4][0], img_6X8_r[4][1], img_6X8_r[4][2], img_6X8_r[4][3], img_6X8_r[4][4], img_6X8_r[4][5], img_6X8_r[4][6], img_6X8_r[4][7]} <= DO_IMG;
                6: {img_6X8_r[5][0], img_6X8_r[5][1], img_6X8_r[5][2], img_6X8_r[5][3], img_6X8_r[5][4], img_6X8_r[5][5], img_6X8_r[5][6], img_6X8_r[5][7]} <= DO_IMG;
            endcase
        end
        else if (cnt_10_r == 9 && shift_col_cnt_r == 11) begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    img_6X8_r[i][j] <= img_6X8_next_r[i][j];
                end
            end
        end
        else if (cnt_10_r == 9) begin
            case(shift_col_cnt_r)
                0: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][0];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][0];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][0];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][0];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][0];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][0];
                end
                1: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][1];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][1];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][1];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][1];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][1];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][1];
                end
                2: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][2];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][2];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][2];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][2];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][2];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][2];
                end
                3: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][3];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][3];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][3];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][3];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][3];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][3];
                end
                4: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][4];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][4];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][4];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][4];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][4];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][4];
                end
                5: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][5];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][5];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][5];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][5];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][5];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][5];
                end
                6: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][6];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][6];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][6];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][6];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][6];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][6];
                end
                7: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                    img_6X8_r[0][7] <= img_6X8_next_r[0][7];
                    img_6X8_r[1][7] <= img_6X8_next_r[1][7];
                    img_6X8_r[2][7] <= img_6X8_next_r[2][7];
                    img_6X8_r[3][7] <= img_6X8_next_r[3][7];
                    img_6X8_r[4][7] <= img_6X8_next_r[4][7];
                    img_6X8_r[5][7] <= img_6X8_next_r[5][7];
                end
                8,9,10,11: begin
                    for (i = 0; i < 7; i = i + 1) begin
                        img_6X8_r[0][i] <= img_6X8_r[0][i+1];
                        img_6X8_r[1][i] <= img_6X8_r[1][i+1];
                        img_6X8_r[2][i] <= img_6X8_r[2][i+1];
                        img_6X8_r[3][i] <= img_6X8_r[3][i+1];
                        img_6X8_r[4][i] <= img_6X8_r[4][i+1];
                        img_6X8_r[5][i] <= img_6X8_r[5][i+1];
                    end
                end
            endcase
        end
        else begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    img_6X8_r[i][j] <= img_6X8_r[i][j];
                end
            end
        end
    end
    else begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                img_6X8_r[i][j] <= 0;
            end
        end
    end
end

/* img_6X8_next_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                img_6X8_next_r[i][j] <= 0;
            end
        end
    end
    else if (cs == CONV08) begin
        if (conv_cnt_r <= 12) begin
            case(conv_cnt_r)
                7 : {img_6X8_next_r[0][0], img_6X8_next_r[0][1], img_6X8_next_r[0][2], img_6X8_next_r[0][3], img_6X8_next_r[0][4], img_6X8_next_r[0][5], img_6X8_next_r[0][6], img_6X8_next_r[0][7]} <= DO_IMG;
                8 : {img_6X8_next_r[1][0], img_6X8_next_r[1][1], img_6X8_next_r[1][2], img_6X8_next_r[1][3], img_6X8_next_r[1][4], img_6X8_next_r[1][5], img_6X8_next_r[1][6], img_6X8_next_r[1][7]} <= DO_IMG;
                9 : {img_6X8_next_r[2][0], img_6X8_next_r[2][1], img_6X8_next_r[2][2], img_6X8_next_r[2][3], img_6X8_next_r[2][4], img_6X8_next_r[2][5], img_6X8_next_r[2][6], img_6X8_next_r[2][7]} <= DO_IMG;
                10: {img_6X8_next_r[3][0], img_6X8_next_r[3][1], img_6X8_next_r[3][2], img_6X8_next_r[3][3], img_6X8_next_r[3][4], img_6X8_next_r[3][5], img_6X8_next_r[3][6], img_6X8_next_r[3][7]} <= DO_IMG;
                11: {img_6X8_next_r[4][0], img_6X8_next_r[4][1], img_6X8_next_r[4][2], img_6X8_next_r[4][3], img_6X8_next_r[4][4], img_6X8_next_r[4][5], img_6X8_next_r[4][6], img_6X8_next_r[4][7]} <= DO_IMG;
                12: {img_6X8_next_r[5][0], img_6X8_next_r[5][1], img_6X8_next_r[5][2], img_6X8_next_r[5][3], img_6X8_next_r[5][4], img_6X8_next_r[5][5], img_6X8_next_r[5][6], img_6X8_next_r[5][7]} <= DO_IMG;
            endcase
        end
    end
    else if (cs == CONV16) begin
        if (conv_cnt_r <= 12) begin
            case(conv_cnt_r)
                7 : {img_6X8_next_r[0][0], img_6X8_next_r[0][1], img_6X8_next_r[0][2], img_6X8_next_r[0][3], img_6X8_next_r[0][4], img_6X8_next_r[0][5], img_6X8_next_r[0][6], img_6X8_next_r[0][7]} <= DO_IMG;
                8 : {img_6X8_next_r[1][0], img_6X8_next_r[1][1], img_6X8_next_r[1][2], img_6X8_next_r[1][3], img_6X8_next_r[1][4], img_6X8_next_r[1][5], img_6X8_next_r[1][6], img_6X8_next_r[1][7]} <= DO_IMG;
                9 : {img_6X8_next_r[2][0], img_6X8_next_r[2][1], img_6X8_next_r[2][2], img_6X8_next_r[2][3], img_6X8_next_r[2][4], img_6X8_next_r[2][5], img_6X8_next_r[2][6], img_6X8_next_r[2][7]} <= DO_IMG;
                10: {img_6X8_next_r[3][0], img_6X8_next_r[3][1], img_6X8_next_r[3][2], img_6X8_next_r[3][3], img_6X8_next_r[3][4], img_6X8_next_r[3][5], img_6X8_next_r[3][6], img_6X8_next_r[3][7]} <= DO_IMG;
                11: {img_6X8_next_r[4][0], img_6X8_next_r[4][1], img_6X8_next_r[4][2], img_6X8_next_r[4][3], img_6X8_next_r[4][4], img_6X8_next_r[4][5], img_6X8_next_r[4][6], img_6X8_next_r[4][7]} <= DO_IMG;
                12: {img_6X8_next_r[5][0], img_6X8_next_r[5][1], img_6X8_next_r[5][2], img_6X8_next_r[5][3], img_6X8_next_r[5][4], img_6X8_next_r[5][5], img_6X8_next_r[5][6], img_6X8_next_r[5][7]} <= DO_IMG;
            endcase
        end
        else if (shift_col_cnt_r == 11) begin
            case(cnt_10_r)
                1: {img_6X8_next_r[0][0], img_6X8_next_r[0][1], img_6X8_next_r[0][2], img_6X8_next_r[0][3], img_6X8_next_r[0][4], img_6X8_next_r[0][5], img_6X8_next_r[0][6], img_6X8_next_r[0][7]} <= DO_IMG;
                2: {img_6X8_next_r[1][0], img_6X8_next_r[1][1], img_6X8_next_r[1][2], img_6X8_next_r[1][3], img_6X8_next_r[1][4], img_6X8_next_r[1][5], img_6X8_next_r[1][6], img_6X8_next_r[1][7]} <= DO_IMG;
                3: {img_6X8_next_r[2][0], img_6X8_next_r[2][1], img_6X8_next_r[2][2], img_6X8_next_r[2][3], img_6X8_next_r[2][4], img_6X8_next_r[2][5], img_6X8_next_r[2][6], img_6X8_next_r[2][7]} <= DO_IMG;
                4: {img_6X8_next_r[3][0], img_6X8_next_r[3][1], img_6X8_next_r[3][2], img_6X8_next_r[3][3], img_6X8_next_r[3][4], img_6X8_next_r[3][5], img_6X8_next_r[3][6], img_6X8_next_r[3][7]} <= DO_IMG;
                5: {img_6X8_next_r[4][0], img_6X8_next_r[4][1], img_6X8_next_r[4][2], img_6X8_next_r[4][3], img_6X8_next_r[4][4], img_6X8_next_r[4][5], img_6X8_next_r[4][6], img_6X8_next_r[4][7]} <= DO_IMG;
                6: {img_6X8_next_r[5][0], img_6X8_next_r[5][1], img_6X8_next_r[5][2], img_6X8_next_r[5][3], img_6X8_next_r[5][4], img_6X8_next_r[5][5], img_6X8_next_r[5][6], img_6X8_next_r[5][7]} <= DO_IMG;
            endcase
        end
        else if (shift_col_cnt_r == 0) begin
            case(cnt_10_r)
                1: {img_6X8_next_r[0][0], img_6X8_next_r[0][1], img_6X8_next_r[0][2], img_6X8_next_r[0][3], img_6X8_next_r[0][4], img_6X8_next_r[0][5], img_6X8_next_r[0][6], img_6X8_next_r[0][7]} <= DO_IMG;
                2: {img_6X8_next_r[1][0], img_6X8_next_r[1][1], img_6X8_next_r[1][2], img_6X8_next_r[1][3], img_6X8_next_r[1][4], img_6X8_next_r[1][5], img_6X8_next_r[1][6], img_6X8_next_r[1][7]} <= DO_IMG;
                3: {img_6X8_next_r[2][0], img_6X8_next_r[2][1], img_6X8_next_r[2][2], img_6X8_next_r[2][3], img_6X8_next_r[2][4], img_6X8_next_r[2][5], img_6X8_next_r[2][6], img_6X8_next_r[2][7]} <= DO_IMG;
                4: {img_6X8_next_r[3][0], img_6X8_next_r[3][1], img_6X8_next_r[3][2], img_6X8_next_r[3][3], img_6X8_next_r[3][4], img_6X8_next_r[3][5], img_6X8_next_r[3][6], img_6X8_next_r[3][7]} <= DO_IMG;
                5: {img_6X8_next_r[4][0], img_6X8_next_r[4][1], img_6X8_next_r[4][2], img_6X8_next_r[4][3], img_6X8_next_r[4][4], img_6X8_next_r[4][5], img_6X8_next_r[4][6], img_6X8_next_r[4][7]} <= DO_IMG;
                6: {img_6X8_next_r[5][0], img_6X8_next_r[5][1], img_6X8_next_r[5][2], img_6X8_next_r[5][3], img_6X8_next_r[5][4], img_6X8_next_r[5][5], img_6X8_next_r[5][6], img_6X8_next_r[5][7]} <= DO_IMG;
            endcase
        end
        else begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    img_6X8_next_r[i][j] <= img_6X8_next_r[i][j];
                end
            end
        end
    end
    else begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                img_6X8_next_r[i][j] <= 0;
            end
        end
    end
end

/* img_6X16_r */
always @(posedge clk ) begin
    if (!rst_n) begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                img_6X16_r[i][j] <= 0;
            end
        end
    end
    else if (cs == CONV32) begin
        if (conv_cnt_r <= 12) begin
            case(conv_cnt_r)
                1: {img_6X16_r[0][0], img_6X16_r[0][1], img_6X16_r[0][2], img_6X16_r[0][3], img_6X16_r[0][4], img_6X16_r[0][5], img_6X16_r[0][6], img_6X16_r[0][7]} <= DO_IMG;
                2: {img_6X16_r[1][0], img_6X16_r[1][1], img_6X16_r[1][2], img_6X16_r[1][3], img_6X16_r[1][4], img_6X16_r[1][5], img_6X16_r[1][6], img_6X16_r[1][7]} <= DO_IMG;
                3: {img_6X16_r[2][0], img_6X16_r[2][1], img_6X16_r[2][2], img_6X16_r[2][3], img_6X16_r[2][4], img_6X16_r[2][5], img_6X16_r[2][6], img_6X16_r[2][7]} <= DO_IMG;
                4: {img_6X16_r[3][0], img_6X16_r[3][1], img_6X16_r[3][2], img_6X16_r[3][3], img_6X16_r[3][4], img_6X16_r[3][5], img_6X16_r[3][6], img_6X16_r[3][7]} <= DO_IMG;
                5: {img_6X16_r[4][0], img_6X16_r[4][1], img_6X16_r[4][2], img_6X16_r[4][3], img_6X16_r[4][4], img_6X16_r[4][5], img_6X16_r[4][6], img_6X16_r[4][7]} <= DO_IMG;
                6: {img_6X16_r[5][0], img_6X16_r[5][1], img_6X16_r[5][2], img_6X16_r[5][3], img_6X16_r[5][4], img_6X16_r[5][5], img_6X16_r[5][6], img_6X16_r[5][7]} <= DO_IMG;

                7: {img_6X16_r[0][8], img_6X16_r[0][9], img_6X16_r[0][10], img_6X16_r[0][11], img_6X16_r[0][12], img_6X16_r[0][13], img_6X16_r[0][14], img_6X16_r[0][15]} <= DO_IMG;
                8: {img_6X16_r[1][8], img_6X16_r[1][9], img_6X16_r[1][10], img_6X16_r[1][11], img_6X16_r[1][12], img_6X16_r[1][13], img_6X16_r[1][14], img_6X16_r[1][15]} <= DO_IMG;
                9: {img_6X16_r[2][8], img_6X16_r[2][9], img_6X16_r[2][10], img_6X16_r[2][11], img_6X16_r[2][12], img_6X16_r[2][13], img_6X16_r[2][14], img_6X16_r[2][15]} <= DO_IMG;
               10: {img_6X16_r[3][8], img_6X16_r[3][9], img_6X16_r[3][10], img_6X16_r[3][11], img_6X16_r[3][12], img_6X16_r[3][13], img_6X16_r[3][14], img_6X16_r[3][15]} <= DO_IMG;
               11: {img_6X16_r[4][8], img_6X16_r[4][9], img_6X16_r[4][10], img_6X16_r[4][11], img_6X16_r[4][12], img_6X16_r[4][13], img_6X16_r[4][14], img_6X16_r[4][15]} <= DO_IMG;
               12: {img_6X16_r[5][8], img_6X16_r[5][9], img_6X16_r[5][10], img_6X16_r[5][11], img_6X16_r[5][12], img_6X16_r[5][13], img_6X16_r[5][14], img_6X16_r[5][15]} <= DO_IMG;
            endcase
        end
        else if (cnt_10_r == 9 && shift_col_cnt_r == 27) begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 16; j = j + 1) begin
                    img_6X16_r[i][j] <= img_6X16_next_r[i][j];
                end
            end
        end
        else if (cnt_10_r == 9) begin
            case(shift_col_cnt_r)
                0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15: begin
                    for (i = 0; i < 15; i = i + 1) begin
                        img_6X16_r[0][i] <= img_6X16_r[0][i+1];
                        img_6X16_r[1][i] <= img_6X16_r[1][i+1];
                        img_6X16_r[2][i] <= img_6X16_r[2][i+1];
                        img_6X16_r[3][i] <= img_6X16_r[3][i+1];
                        img_6X16_r[4][i] <= img_6X16_r[4][i+1];
                        img_6X16_r[5][i] <= img_6X16_r[5][i+1];
                    end
                    img_6X16_r[0][15] <= img_6X16_next_r[0][shift_col_cnt_r[3:0]];
                    img_6X16_r[1][15] <= img_6X16_next_r[1][shift_col_cnt_r[3:0]];
                    img_6X16_r[2][15] <= img_6X16_next_r[2][shift_col_cnt_r[3:0]];
                    img_6X16_r[3][15] <= img_6X16_next_r[3][shift_col_cnt_r[3:0]];
                    img_6X16_r[4][15] <= img_6X16_next_r[4][shift_col_cnt_r[3:0]];
                    img_6X16_r[5][15] <= img_6X16_next_r[5][shift_col_cnt_r[3:0]];
                end
                16,17,18,19,20,21,22,23,24,25,26,27: begin
                    for (i = 0; i < 15; i = i + 1) begin
                        img_6X16_r[0][i] <= img_6X16_r[0][i+1];
                        img_6X16_r[1][i] <= img_6X16_r[1][i+1];
                        img_6X16_r[2][i] <= img_6X16_r[2][i+1];
                        img_6X16_r[3][i] <= img_6X16_r[3][i+1];
                        img_6X16_r[4][i] <= img_6X16_r[4][i+1];
                        img_6X16_r[5][i] <= img_6X16_r[5][i+1];
                    end
                end
            endcase
        end
        else begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 16; j = j + 1) begin
                    img_6X16_r[i][j] <= img_6X16_r[i][j];
                end
            end
        end
    end
    else begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                img_6X16_r[i][j] <= 0;
            end
        end
    end
end

/* img_6X16_next_r */
always @(posedge clk ) begin
    if (!rst_n) begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                img_6X16_next_r[i][j] <= 0;
            end
        end
    end
    else if (cs == CONV32) begin
        if (conv_cnt_r <= 24) begin
            case(conv_cnt_r)
                13: {img_6X16_next_r[0][0], img_6X16_next_r[0][1], img_6X16_next_r[0][2], img_6X16_next_r[0][3], img_6X16_next_r[0][4], img_6X16_next_r[0][5], img_6X16_next_r[0][6], img_6X16_next_r[0][7]} <= DO_IMG;
                14: {img_6X16_next_r[1][0], img_6X16_next_r[1][1], img_6X16_next_r[1][2], img_6X16_next_r[1][3], img_6X16_next_r[1][4], img_6X16_next_r[1][5], img_6X16_next_r[1][6], img_6X16_next_r[1][7]} <= DO_IMG;
                15: {img_6X16_next_r[2][0], img_6X16_next_r[2][1], img_6X16_next_r[2][2], img_6X16_next_r[2][3], img_6X16_next_r[2][4], img_6X16_next_r[2][5], img_6X16_next_r[2][6], img_6X16_next_r[2][7]} <= DO_IMG;
                16: {img_6X16_next_r[3][0], img_6X16_next_r[3][1], img_6X16_next_r[3][2], img_6X16_next_r[3][3], img_6X16_next_r[3][4], img_6X16_next_r[3][5], img_6X16_next_r[3][6], img_6X16_next_r[3][7]} <= DO_IMG;
                17: {img_6X16_next_r[4][0], img_6X16_next_r[4][1], img_6X16_next_r[4][2], img_6X16_next_r[4][3], img_6X16_next_r[4][4], img_6X16_next_r[4][5], img_6X16_next_r[4][6], img_6X16_next_r[4][7]} <= DO_IMG;
                18: {img_6X16_next_r[5][0], img_6X16_next_r[5][1], img_6X16_next_r[5][2], img_6X16_next_r[5][3], img_6X16_next_r[5][4], img_6X16_next_r[5][5], img_6X16_next_r[5][6], img_6X16_next_r[5][7]} <= DO_IMG;

                19: {img_6X16_next_r[0][8], img_6X16_next_r[0][9], img_6X16_next_r[0][10], img_6X16_next_r[0][11], img_6X16_next_r[0][12], img_6X16_next_r[0][13], img_6X16_next_r[0][14], img_6X16_next_r[0][15]} <= DO_IMG;
                20: {img_6X16_next_r[1][8], img_6X16_next_r[1][9], img_6X16_next_r[1][10], img_6X16_next_r[1][11], img_6X16_next_r[1][12], img_6X16_next_r[1][13], img_6X16_next_r[1][14], img_6X16_next_r[1][15]} <= DO_IMG;
                21: {img_6X16_next_r[2][8], img_6X16_next_r[2][9], img_6X16_next_r[2][10], img_6X16_next_r[2][11], img_6X16_next_r[2][12], img_6X16_next_r[2][13], img_6X16_next_r[2][14], img_6X16_next_r[2][15]} <= DO_IMG;
                22: {img_6X16_next_r[3][8], img_6X16_next_r[3][9], img_6X16_next_r[3][10], img_6X16_next_r[3][11], img_6X16_next_r[3][12], img_6X16_next_r[3][13], img_6X16_next_r[3][14], img_6X16_next_r[3][15]} <= DO_IMG;
                23: {img_6X16_next_r[4][8], img_6X16_next_r[4][9], img_6X16_next_r[4][10], img_6X16_next_r[4][11], img_6X16_next_r[4][12], img_6X16_next_r[4][13], img_6X16_next_r[4][14], img_6X16_next_r[4][15]} <= DO_IMG;
                24: {img_6X16_next_r[5][8], img_6X16_next_r[5][9], img_6X16_next_r[5][10], img_6X16_next_r[5][11], img_6X16_next_r[5][12], img_6X16_next_r[5][13], img_6X16_next_r[5][14], img_6X16_next_r[5][15]} <= DO_IMG;
            endcase
        end
        else if (shift_col_cnt_r == 16 || shift_col_cnt_r == 17) begin
            case(cnt_20_r)
                1: {img_6X16_next_r[0][0], img_6X16_next_r[0][1], img_6X16_next_r[0][2], img_6X16_next_r[0][3], img_6X16_next_r[0][4], img_6X16_next_r[0][5], img_6X16_next_r[0][6], img_6X16_next_r[0][7]} <= DO_IMG;
                2: {img_6X16_next_r[1][0], img_6X16_next_r[1][1], img_6X16_next_r[1][2], img_6X16_next_r[1][3], img_6X16_next_r[1][4], img_6X16_next_r[1][5], img_6X16_next_r[1][6], img_6X16_next_r[1][7]} <= DO_IMG;
                3: {img_6X16_next_r[2][0], img_6X16_next_r[2][1], img_6X16_next_r[2][2], img_6X16_next_r[2][3], img_6X16_next_r[2][4], img_6X16_next_r[2][5], img_6X16_next_r[2][6], img_6X16_next_r[2][7]} <= DO_IMG;
                4: {img_6X16_next_r[3][0], img_6X16_next_r[3][1], img_6X16_next_r[3][2], img_6X16_next_r[3][3], img_6X16_next_r[3][4], img_6X16_next_r[3][5], img_6X16_next_r[3][6], img_6X16_next_r[3][7]} <= DO_IMG;
                5: {img_6X16_next_r[4][0], img_6X16_next_r[4][1], img_6X16_next_r[4][2], img_6X16_next_r[4][3], img_6X16_next_r[4][4], img_6X16_next_r[4][5], img_6X16_next_r[4][6], img_6X16_next_r[4][7]} <= DO_IMG;
                6: {img_6X16_next_r[5][0], img_6X16_next_r[5][1], img_6X16_next_r[5][2], img_6X16_next_r[5][3], img_6X16_next_r[5][4], img_6X16_next_r[5][5], img_6X16_next_r[5][6], img_6X16_next_r[5][7]} <= DO_IMG;

                7: {img_6X16_next_r[0][8], img_6X16_next_r[0][9], img_6X16_next_r[0][10], img_6X16_next_r[0][11], img_6X16_next_r[0][12], img_6X16_next_r[0][13], img_6X16_next_r[0][14], img_6X16_next_r[0][15]} <= DO_IMG;
                8: {img_6X16_next_r[1][8], img_6X16_next_r[1][9], img_6X16_next_r[1][10], img_6X16_next_r[1][11], img_6X16_next_r[1][12], img_6X16_next_r[1][13], img_6X16_next_r[1][14], img_6X16_next_r[1][15]} <= DO_IMG;
                9: {img_6X16_next_r[2][8], img_6X16_next_r[2][9], img_6X16_next_r[2][10], img_6X16_next_r[2][11], img_6X16_next_r[2][12], img_6X16_next_r[2][13], img_6X16_next_r[2][14], img_6X16_next_r[2][15]} <= DO_IMG;
                10: {img_6X16_next_r[3][8], img_6X16_next_r[3][9], img_6X16_next_r[3][10], img_6X16_next_r[3][11], img_6X16_next_r[3][12], img_6X16_next_r[3][13], img_6X16_next_r[3][14], img_6X16_next_r[3][15]} <= DO_IMG;
                11: {img_6X16_next_r[4][8], img_6X16_next_r[4][9], img_6X16_next_r[4][10], img_6X16_next_r[4][11], img_6X16_next_r[4][12], img_6X16_next_r[4][13], img_6X16_next_r[4][14], img_6X16_next_r[4][15]} <= DO_IMG;
                12: {img_6X16_next_r[5][8], img_6X16_next_r[5][9], img_6X16_next_r[5][10], img_6X16_next_r[5][11], img_6X16_next_r[5][12], img_6X16_next_r[5][13], img_6X16_next_r[5][14], img_6X16_next_r[5][15]} <= DO_IMG;
            endcase
        end
        else if (shift_col_cnt_r == 0 || shift_col_cnt_r == 1) begin
            case(cnt_20_r)
                1: {img_6X16_next_r[0][0], img_6X16_next_r[0][1], img_6X16_next_r[0][2], img_6X16_next_r[0][3], img_6X16_next_r[0][4], img_6X16_next_r[0][5], img_6X16_next_r[0][6], img_6X16_next_r[0][7]} <= DO_IMG;
                2: {img_6X16_next_r[1][0], img_6X16_next_r[1][1], img_6X16_next_r[1][2], img_6X16_next_r[1][3], img_6X16_next_r[1][4], img_6X16_next_r[1][5], img_6X16_next_r[1][6], img_6X16_next_r[1][7]} <= DO_IMG;
                3: {img_6X16_next_r[2][0], img_6X16_next_r[2][1], img_6X16_next_r[2][2], img_6X16_next_r[2][3], img_6X16_next_r[2][4], img_6X16_next_r[2][5], img_6X16_next_r[2][6], img_6X16_next_r[2][7]} <= DO_IMG;
                4: {img_6X16_next_r[3][0], img_6X16_next_r[3][1], img_6X16_next_r[3][2], img_6X16_next_r[3][3], img_6X16_next_r[3][4], img_6X16_next_r[3][5], img_6X16_next_r[3][6], img_6X16_next_r[3][7]} <= DO_IMG;
                5: {img_6X16_next_r[4][0], img_6X16_next_r[4][1], img_6X16_next_r[4][2], img_6X16_next_r[4][3], img_6X16_next_r[4][4], img_6X16_next_r[4][5], img_6X16_next_r[4][6], img_6X16_next_r[4][7]} <= DO_IMG;
                6: {img_6X16_next_r[5][0], img_6X16_next_r[5][1], img_6X16_next_r[5][2], img_6X16_next_r[5][3], img_6X16_next_r[5][4], img_6X16_next_r[5][5], img_6X16_next_r[5][6], img_6X16_next_r[5][7]} <= DO_IMG;

                7: {img_6X16_next_r[0][8], img_6X16_next_r[0][9], img_6X16_next_r[0][10], img_6X16_next_r[0][11], img_6X16_next_r[0][12], img_6X16_next_r[0][13], img_6X16_next_r[0][14], img_6X16_next_r[0][15]} <= DO_IMG;
                8: {img_6X16_next_r[1][8], img_6X16_next_r[1][9], img_6X16_next_r[1][10], img_6X16_next_r[1][11], img_6X16_next_r[1][12], img_6X16_next_r[1][13], img_6X16_next_r[1][14], img_6X16_next_r[1][15]} <= DO_IMG;
                9: {img_6X16_next_r[2][8], img_6X16_next_r[2][9], img_6X16_next_r[2][10], img_6X16_next_r[2][11], img_6X16_next_r[2][12], img_6X16_next_r[2][13], img_6X16_next_r[2][14], img_6X16_next_r[2][15]} <= DO_IMG;
                10: {img_6X16_next_r[3][8], img_6X16_next_r[3][9], img_6X16_next_r[3][10], img_6X16_next_r[3][11], img_6X16_next_r[3][12], img_6X16_next_r[3][13], img_6X16_next_r[3][14], img_6X16_next_r[3][15]} <= DO_IMG;
                11: {img_6X16_next_r[4][8], img_6X16_next_r[4][9], img_6X16_next_r[4][10], img_6X16_next_r[4][11], img_6X16_next_r[4][12], img_6X16_next_r[4][13], img_6X16_next_r[4][14], img_6X16_next_r[4][15]} <= DO_IMG;
                12: {img_6X16_next_r[5][8], img_6X16_next_r[5][9], img_6X16_next_r[5][10], img_6X16_next_r[5][11], img_6X16_next_r[5][12], img_6X16_next_r[5][13], img_6X16_next_r[5][14], img_6X16_next_r[5][15]} <= DO_IMG;
            endcase
        end
        else begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 16; j = j + 1) begin
                    img_6X16_next_r[i][j] <= img_6X16_next_r[i][j];
                end
            end
        end

    end
    else begin
        for (i = 0; i < 6; i = i + 1) begin
            for (j = 0; j < 16; j = j + 1) begin
                img_6X16_next_r[i][j] <= 0;
            end
        end
    end
end

/* ker_5X5_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 5; i = i + 1) begin
            for (j = 0; j < 5; j = j + 1) begin
                ker_5X5_r[i][j] <= 0;
            end
        end
    end
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32) begin
        case(conv_cnt_r)
            1: {ker_5X5_r[0][0], ker_5X5_r[0][1], ker_5X5_r[0][2], ker_5X5_r[0][3], ker_5X5_r[0][4]} <= DO_KER;
            2: {ker_5X5_r[1][0], ker_5X5_r[1][1], ker_5X5_r[1][2], ker_5X5_r[1][3], ker_5X5_r[1][4]} <= DO_KER;
            3: {ker_5X5_r[2][0], ker_5X5_r[2][1], ker_5X5_r[2][2], ker_5X5_r[2][3], ker_5X5_r[2][4]} <= DO_KER;
            4: {ker_5X5_r[3][0], ker_5X5_r[3][1], ker_5X5_r[3][2], ker_5X5_r[3][3], ker_5X5_r[3][4]} <= DO_KER;
            5: {ker_5X5_r[4][0], ker_5X5_r[4][1], ker_5X5_r[4][2], ker_5X5_r[4][3], ker_5X5_r[4][4]} <= DO_KER;
        endcase
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        case(deconv_cnt_r)
            5: {ker_5X5_r[0][0], ker_5X5_r[0][1], ker_5X5_r[0][2], ker_5X5_r[0][3], ker_5X5_r[0][4]} <= DO_KER;
            4: {ker_5X5_r[1][0], ker_5X5_r[1][1], ker_5X5_r[1][2], ker_5X5_r[1][3], ker_5X5_r[1][4]} <= DO_KER;
            3: {ker_5X5_r[2][0], ker_5X5_r[2][1], ker_5X5_r[2][2], ker_5X5_r[2][3], ker_5X5_r[2][4]} <= DO_KER;
            2: {ker_5X5_r[3][0], ker_5X5_r[3][1], ker_5X5_r[3][2], ker_5X5_r[3][3], ker_5X5_r[3][4]} <= DO_KER;
            1: {ker_5X5_r[4][0], ker_5X5_r[4][1], ker_5X5_r[4][2], ker_5X5_r[4][3], ker_5X5_r[4][4]} <= DO_KER;
        endcase
    end
    else begin
        for (i = 0; i < 5; i = i + 1) begin
            for (j = 0; j < 5; j = j + 1) begin
                ker_5X5_r[i][j] <= ker_5X5_r[i][j];
            end
        end
    end
end

/* img_dp5_in, ker_dp5_in */
always @(*) begin
    if (cs == CONV08 || cs == CONV16) begin
        case(cnt_10_r)
            0: begin
                img_dp5_in[0] = img_6X8_r[0][0];    img_dp5_in[1] = img_6X8_r[0][1];    img_dp5_in[2] = img_6X8_r[0][2];    img_dp5_in[3] = img_6X8_r[0][3];    img_dp5_in[4] = img_6X8_r[0][4];
                ker_dp5_in[0] = ker_5X5_r[0][0];    ker_dp5_in[1] = ker_5X5_r[0][1];    ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][3];    ker_dp5_in[4] = ker_5X5_r[0][4];
            end
            1: begin
                img_dp5_in[0] = img_6X8_r[1][0];    img_dp5_in[1] = img_6X8_r[1][1];    img_dp5_in[2] = img_6X8_r[1][2];    img_dp5_in[3] = img_6X8_r[1][3];    img_dp5_in[4] = img_6X8_r[1][4];
                ker_dp5_in[0] = ker_5X5_r[1][0];    ker_dp5_in[1] = ker_5X5_r[1][1];    ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][3];    ker_dp5_in[4] = ker_5X5_r[1][4];
            end
            2: begin
                img_dp5_in[0] = img_6X8_r[2][0];    img_dp5_in[1] = img_6X8_r[2][1];    img_dp5_in[2] = img_6X8_r[2][2];    img_dp5_in[3] = img_6X8_r[2][3];    img_dp5_in[4] = img_6X8_r[2][4];
                ker_dp5_in[0] = ker_5X5_r[2][0];    ker_dp5_in[1] = ker_5X5_r[2][1];    ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][3];    ker_dp5_in[4] = ker_5X5_r[2][4];
            end
            3: begin
                img_dp5_in[0] = img_6X8_r[3][0];    img_dp5_in[1] = img_6X8_r[3][1];    img_dp5_in[2] = img_6X8_r[3][2];    img_dp5_in[3] = img_6X8_r[3][3];    img_dp5_in[4] = img_6X8_r[3][4];
                ker_dp5_in[0] = ker_5X5_r[3][0];    ker_dp5_in[1] = ker_5X5_r[3][1];    ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][3];    ker_dp5_in[4] = ker_5X5_r[3][4];
            end
            4: begin
                img_dp5_in[0] = img_6X8_r[4][0];    img_dp5_in[1] = img_6X8_r[4][1];    img_dp5_in[2] = img_6X8_r[4][2];    img_dp5_in[3] = img_6X8_r[4][3];    img_dp5_in[4] = img_6X8_r[4][4];
                ker_dp5_in[0] = ker_5X5_r[4][0];    ker_dp5_in[1] = ker_5X5_r[4][1];    ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][3];    ker_dp5_in[4] = ker_5X5_r[4][4];
            end

            5: begin
                img_dp5_in[0] = img_6X8_r[1][0];    img_dp5_in[1] = img_6X8_r[1][1];    img_dp5_in[2] = img_6X8_r[1][2];    img_dp5_in[3] = img_6X8_r[1][3];    img_dp5_in[4] = img_6X8_r[1][4];
                ker_dp5_in[0] = ker_5X5_r[0][0];    ker_dp5_in[1] = ker_5X5_r[0][1];    ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][3];    ker_dp5_in[4] = ker_5X5_r[0][4];
            end
            6: begin
                img_dp5_in[0] = img_6X8_r[2][0];    img_dp5_in[1] = img_6X8_r[2][1];    img_dp5_in[2] = img_6X8_r[2][2];    img_dp5_in[3] = img_6X8_r[2][3];    img_dp5_in[4] = img_6X8_r[2][4];
                ker_dp5_in[0] = ker_5X5_r[1][0];    ker_dp5_in[1] = ker_5X5_r[1][1];    ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][3];    ker_dp5_in[4] = ker_5X5_r[1][4];
            end
            7: begin
                img_dp5_in[0] = img_6X8_r[3][0];    img_dp5_in[1] = img_6X8_r[3][1];    img_dp5_in[2] = img_6X8_r[3][2];    img_dp5_in[3] = img_6X8_r[3][3];    img_dp5_in[4] = img_6X8_r[3][4];
                ker_dp5_in[0] = ker_5X5_r[2][0];    ker_dp5_in[1] = ker_5X5_r[2][1];    ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][3];    ker_dp5_in[4] = ker_5X5_r[2][4];
            end
            8: begin
                img_dp5_in[0] = img_6X8_r[4][0];    img_dp5_in[1] = img_6X8_r[4][1];    img_dp5_in[2] = img_6X8_r[4][2];    img_dp5_in[3] = img_6X8_r[4][3];    img_dp5_in[4] = img_6X8_r[4][4];
                ker_dp5_in[0] = ker_5X5_r[3][0];    ker_dp5_in[1] = ker_5X5_r[3][1];    ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][3];    ker_dp5_in[4] = ker_5X5_r[3][4];
            end
            9: begin
                img_dp5_in[0] = img_6X8_r[5][0];    img_dp5_in[1] = img_6X8_r[5][1];    img_dp5_in[2] = img_6X8_r[5][2];    img_dp5_in[3] = img_6X8_r[5][3];    img_dp5_in[4] = img_6X8_r[5][4];
                ker_dp5_in[0] = ker_5X5_r[4][0];    ker_dp5_in[1] = ker_5X5_r[4][1];    ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][3];    ker_dp5_in[4] = ker_5X5_r[4][4];
            end
            default: begin
                img_dp5_in[0] = 0;    img_dp5_in[1] = 0;    img_dp5_in[2] = 0;    img_dp5_in[3] = 0;    img_dp5_in[4] = 0;
                ker_dp5_in[0] = 0;    ker_dp5_in[1] = 0;    ker_dp5_in[2] = 0;    ker_dp5_in[3] = 0;    ker_dp5_in[4] = 0;
            end
        endcase
    end
    else if (cs == CONV32) begin
        case(cnt_10_r)
            0: begin
                img_dp5_in[0] = img_6X16_r[0][0];    img_dp5_in[1] = img_6X16_r[0][1];    img_dp5_in[2] = img_6X16_r[0][2];    img_dp5_in[3] = img_6X16_r[0][3];    img_dp5_in[4] = img_6X16_r[0][4];
                ker_dp5_in[0] = ker_5X5_r[0][0];    ker_dp5_in[1] = ker_5X5_r[0][1];    ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][3];    ker_dp5_in[4] = ker_5X5_r[0][4];
            end
            1: begin
                img_dp5_in[0] = img_6X16_r[1][0];    img_dp5_in[1] = img_6X16_r[1][1];    img_dp5_in[2] = img_6X16_r[1][2];    img_dp5_in[3] = img_6X16_r[1][3];    img_dp5_in[4] = img_6X16_r[1][4];
                ker_dp5_in[0] = ker_5X5_r[1][0];    ker_dp5_in[1] = ker_5X5_r[1][1];    ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][3];    ker_dp5_in[4] = ker_5X5_r[1][4];
            end
            2: begin
                img_dp5_in[0] = img_6X16_r[2][0];    img_dp5_in[1] = img_6X16_r[2][1];    img_dp5_in[2] = img_6X16_r[2][2];    img_dp5_in[3] = img_6X16_r[2][3];    img_dp5_in[4] = img_6X16_r[2][4];
                ker_dp5_in[0] = ker_5X5_r[2][0];    ker_dp5_in[1] = ker_5X5_r[2][1];    ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][3];    ker_dp5_in[4] = ker_5X5_r[2][4];
            end
            3: begin
                img_dp5_in[0] = img_6X16_r[3][0];    img_dp5_in[1] = img_6X16_r[3][1];    img_dp5_in[2] = img_6X16_r[3][2];    img_dp5_in[3] = img_6X16_r[3][3];    img_dp5_in[4] = img_6X16_r[3][4];
                ker_dp5_in[0] = ker_5X5_r[3][0];    ker_dp5_in[1] = ker_5X5_r[3][1];    ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][3];    ker_dp5_in[4] = ker_5X5_r[3][4];
            end
            4: begin
                img_dp5_in[0] = img_6X16_r[4][0];    img_dp5_in[1] = img_6X16_r[4][1];    img_dp5_in[2] = img_6X16_r[4][2];    img_dp5_in[3] = img_6X16_r[4][3];    img_dp5_in[4] = img_6X16_r[4][4];
                ker_dp5_in[0] = ker_5X5_r[4][0];    ker_dp5_in[1] = ker_5X5_r[4][1];    ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][3];    ker_dp5_in[4] = ker_5X5_r[4][4];
            end

            5: begin
                img_dp5_in[0] = img_6X16_r[1][0];    img_dp5_in[1] = img_6X16_r[1][1];    img_dp5_in[2] = img_6X16_r[1][2];    img_dp5_in[3] = img_6X16_r[1][3];    img_dp5_in[4] = img_6X16_r[1][4];
                ker_dp5_in[0] = ker_5X5_r[0][0];    ker_dp5_in[1] = ker_5X5_r[0][1];    ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][3];    ker_dp5_in[4] = ker_5X5_r[0][4];
            end
            6: begin
                img_dp5_in[0] = img_6X16_r[2][0];    img_dp5_in[1] = img_6X16_r[2][1];    img_dp5_in[2] = img_6X16_r[2][2];    img_dp5_in[3] = img_6X16_r[2][3];    img_dp5_in[4] = img_6X16_r[2][4];
                ker_dp5_in[0] = ker_5X5_r[1][0];    ker_dp5_in[1] = ker_5X5_r[1][1];    ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][3];    ker_dp5_in[4] = ker_5X5_r[1][4];
            end
            7: begin
                img_dp5_in[0] = img_6X16_r[3][0];    img_dp5_in[1] = img_6X16_r[3][1];    img_dp5_in[2] = img_6X16_r[3][2];    img_dp5_in[3] = img_6X16_r[3][3];    img_dp5_in[4] = img_6X16_r[3][4];
                ker_dp5_in[0] = ker_5X5_r[2][0];    ker_dp5_in[1] = ker_5X5_r[2][1];    ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][3];    ker_dp5_in[4] = ker_5X5_r[2][4];
            end
            8: begin
                img_dp5_in[0] = img_6X16_r[4][0];    img_dp5_in[1] = img_6X16_r[4][1];    img_dp5_in[2] = img_6X16_r[4][2];    img_dp5_in[3] = img_6X16_r[4][3];    img_dp5_in[4] = img_6X16_r[4][4];
                ker_dp5_in[0] = ker_5X5_r[3][0];    ker_dp5_in[1] = ker_5X5_r[3][1];    ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][3];    ker_dp5_in[4] = ker_5X5_r[3][4];
            end
            9: begin
                img_dp5_in[0] = img_6X16_r[5][0];    img_dp5_in[1] = img_6X16_r[5][1];    img_dp5_in[2] = img_6X16_r[5][2];    img_dp5_in[3] = img_6X16_r[5][3];    img_dp5_in[4] = img_6X16_r[5][4];
                ker_dp5_in[0] = ker_5X5_r[4][0];    ker_dp5_in[1] = ker_5X5_r[4][1];    ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][3];    ker_dp5_in[4] = ker_5X5_r[4][4];
            end
            default: begin
                img_dp5_in[0] = 0;    img_dp5_in[1] = 0;    img_dp5_in[2] = 0;    img_dp5_in[3] = 0;    img_dp5_in[4] = 0;
                ker_dp5_in[0] = 0;    ker_dp5_in[1] = 0;    ker_dp5_in[2] = 0;    ker_dp5_in[3] = 0;    ker_dp5_in[4] = 0;
            end
        endcase
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                2: begin
                    img_dp5_in[0] = img_5X5_r[0][0];   img_dp5_in[1] = img_5X5_r[0][1];   img_dp5_in[2] = img_5X5_r[0][2];    img_dp5_in[3] = img_5X5_r[0][3];    img_dp5_in[4] = img_5X5_r[0][4];
                    ker_dp5_in[0] = ker_5X5_r[4][4];   ker_dp5_in[1] = ker_5X5_r[4][3];   ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][1];    ker_dp5_in[4] = ker_5X5_r[4][0];
                end
                3: begin
                    img_dp5_in[0] = img_5X5_r[1][0];   img_dp5_in[1] = img_5X5_r[1][1];   img_dp5_in[2] = img_5X5_r[1][2];    img_dp5_in[3] = img_5X5_r[1][3];    img_dp5_in[4] = img_5X5_r[1][4];
                    ker_dp5_in[0] = ker_5X5_r[3][4];   ker_dp5_in[1] = ker_5X5_r[3][3];   ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][1];    ker_dp5_in[4] = ker_5X5_r[3][0];
                end
                4: begin
                    img_dp5_in[0] = img_5X5_r[2][0];   img_dp5_in[1] = img_5X5_r[2][1];   img_dp5_in[2] = img_5X5_r[2][2];    img_dp5_in[3] = img_5X5_r[2][3];    img_dp5_in[4] = img_5X5_r[2][4];
                    ker_dp5_in[0] = ker_5X5_r[2][4];   ker_dp5_in[1] = ker_5X5_r[2][3];   ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][1];    ker_dp5_in[4] = ker_5X5_r[2][0];
                end
                5: begin
                    img_dp5_in[0] = img_5X5_r[3][0];   img_dp5_in[1] = img_5X5_r[3][1];   img_dp5_in[2] = img_5X5_r[3][2];    img_dp5_in[3] = img_5X5_r[3][3];    img_dp5_in[4] = img_5X5_r[3][4];
                    ker_dp5_in[0] = ker_5X5_r[1][4];   ker_dp5_in[1] = ker_5X5_r[1][3];   ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][1];    ker_dp5_in[4] = ker_5X5_r[1][0];
                end
                6: begin
                    img_dp5_in[0] = img_5X5_r[4][0];   img_dp5_in[1] = img_5X5_r[4][1];   img_dp5_in[2] = img_5X5_r[4][2];    img_dp5_in[3] = img_5X5_r[4][3];    img_dp5_in[4] = img_5X5_r[4][4];
                    ker_dp5_in[0] = ker_5X5_r[0][4];   ker_dp5_in[1] = ker_5X5_r[0][3];   ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][1];    ker_dp5_in[4] = ker_5X5_r[0][0];
                end
                default: begin
                    img_dp5_in[0] = 0;   img_dp5_in[1] = 0;   img_dp5_in[2] = 0;    img_dp5_in[3] = 0;    img_dp5_in[4] = 0;
                    ker_dp5_in[0] = 0;   ker_dp5_in[1] = 0;   ker_dp5_in[2] = 0;    ker_dp5_in[3] = 0;    ker_dp5_in[4] = 0;
                end
            endcase
        end
        else begin
            case(out_cnt_20_r)
                14: begin
                    img_dp5_in[0] = img_5X5_r[0][0];   img_dp5_in[1] = img_5X5_r[0][1];   img_dp5_in[2] = img_5X5_r[0][2];    img_dp5_in[3] = img_5X5_r[0][3];    img_dp5_in[4] = img_5X5_r[0][4];
                    ker_dp5_in[0] = ker_5X5_r[4][4];   ker_dp5_in[1] = ker_5X5_r[4][3];   ker_dp5_in[2] = ker_5X5_r[4][2];    ker_dp5_in[3] = ker_5X5_r[4][1];    ker_dp5_in[4] = ker_5X5_r[4][0];
                end
                15: begin
                    img_dp5_in[0] = img_5X5_r[1][0];   img_dp5_in[1] = img_5X5_r[1][1];   img_dp5_in[2] = img_5X5_r[1][2];    img_dp5_in[3] = img_5X5_r[1][3];    img_dp5_in[4] = img_5X5_r[1][4];
                    ker_dp5_in[0] = ker_5X5_r[3][4];   ker_dp5_in[1] = ker_5X5_r[3][3];   ker_dp5_in[2] = ker_5X5_r[3][2];    ker_dp5_in[3] = ker_5X5_r[3][1];    ker_dp5_in[4] = ker_5X5_r[3][0];
                end
                16: begin
                    img_dp5_in[0] = img_5X5_r[2][0];   img_dp5_in[1] = img_5X5_r[2][1];   img_dp5_in[2] = img_5X5_r[2][2];    img_dp5_in[3] = img_5X5_r[2][3];    img_dp5_in[4] = img_5X5_r[2][4];
                    ker_dp5_in[0] = ker_5X5_r[2][4];   ker_dp5_in[1] = ker_5X5_r[2][3];   ker_dp5_in[2] = ker_5X5_r[2][2];    ker_dp5_in[3] = ker_5X5_r[2][1];    ker_dp5_in[4] = ker_5X5_r[2][0];
                end
                17: begin
                    img_dp5_in[0] = img_5X5_r[3][0];   img_dp5_in[1] = img_5X5_r[3][1];   img_dp5_in[2] = img_5X5_r[3][2];    img_dp5_in[3] = img_5X5_r[3][3];    img_dp5_in[4] = img_5X5_r[3][4];
                    ker_dp5_in[0] = ker_5X5_r[1][4];   ker_dp5_in[1] = ker_5X5_r[1][3];   ker_dp5_in[2] = ker_5X5_r[1][2];    ker_dp5_in[3] = ker_5X5_r[1][1];    ker_dp5_in[4] = ker_5X5_r[1][0];
                end
                18: begin
                    img_dp5_in[0] = img_5X5_r[4][0];   img_dp5_in[1] = img_5X5_r[4][1];   img_dp5_in[2] = img_5X5_r[4][2];    img_dp5_in[3] = img_5X5_r[4][3];    img_dp5_in[4] = img_5X5_r[4][4];
                    ker_dp5_in[0] = ker_5X5_r[0][4];   ker_dp5_in[1] = ker_5X5_r[0][3];   ker_dp5_in[2] = ker_5X5_r[0][2];    ker_dp5_in[3] = ker_5X5_r[0][1];    ker_dp5_in[4] = ker_5X5_r[0][0];
                end
                default: begin
                    img_dp5_in[0] = 0;   img_dp5_in[1] = 0;   img_dp5_in[2] = 0;    img_dp5_in[3] = 0;    img_dp5_in[4] = 0;
                    ker_dp5_in[0] = 0;   ker_dp5_in[1] = 0;   ker_dp5_in[2] = 0;    ker_dp5_in[3] = 0;    ker_dp5_in[4] = 0;
                end
            endcase
        end

    end
    else begin
        img_dp5_in[0] = 0;    img_dp5_in[1] = 0;    img_dp5_in[2] = 0;    img_dp5_in[3] = 0;    img_dp5_in[4] = 0;
        ker_dp5_in[0] = 0;    ker_dp5_in[1] = 0;    ker_dp5_in[2] = 0;    ker_dp5_in[3] = 0;    ker_dp5_in[4] = 0;
    end
end

/* MODULE DP5 */
DP5 DP5 (
    .A(img_dp5_in[0]), .B(ker_dp5_in[0]),
    .C(img_dp5_in[1]), .D(ker_dp5_in[1]),
    .E(img_dp5_in[2]), .F(ker_dp5_in[2]),
    .G(img_dp5_in[3]), .H(ker_dp5_in[3]),
    .I(img_dp5_in[4]), .J(ker_dp5_in[4]),
    .dp5_out(feat_add)
);

/* feat_2X2_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        feat_2X2_r[0][0] <= 0;    feat_2X2_r[0][1] <= 0;
        feat_2X2_r[1][0] <= 0;    feat_2X2_r[1][1] <= 0;
    end
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32) begin
        if (conv_cnt_r > 3) begin
            case(cnt_20_r)
                0 : feat_2X2_r[0][0] <= feat_add;
                5 : feat_2X2_r[1][0] <= feat_add;
                10: feat_2X2_r[0][1] <= feat_add;
                15: feat_2X2_r[1][1] <= feat_add;
                1,2,3,4     : feat_2X2_r[0][0] <= feat_2X2_r[0][0] + feat_add;
                6,7,8,9     : feat_2X2_r[1][0] <= feat_2X2_r[1][0] + feat_add;
                11,12,13,14 : feat_2X2_r[0][1] <= feat_2X2_r[0][1] + feat_add;
                16,17,18,19 : feat_2X2_r[1][1] <= feat_2X2_r[1][1] + feat_add;
            endcase
        end
        else begin
            feat_2X2_r[0][0] <= feat_2X2_r[0][0];    feat_2X2_r[0][1] <= feat_2X2_r[0][1];
            feat_2X2_r[1][0] <= feat_2X2_r[1][0];    feat_2X2_r[1][1] <= feat_2X2_r[1][1];
        end
    end
    else begin
        feat_2X2_r[0][0] <= 0;    feat_2X2_r[0][1] <= 0;
        feat_2X2_r[1][0] <= 0;    feat_2X2_r[1][1] <= 0;
    end
end

/* Maxpooling*/
assign candidate0 = (feat_2X2_r[0][0] > feat_2X2_r[0][1]) ? feat_2X2_r[0][0] : feat_2X2_r[0][1];
assign candidate1 = (feat_2X2_r[1][0] > feat_2X2_r[1][1]) ? feat_2X2_r[1][0] : feat_2X2_r[1][1];
assign maxpool    = (candidate0 > candidate1) ? candidate0 : candidate1;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        maxpool_r <= 0;
    else if (cs == CONV08 || cs == CONV16) begin
        if (conv_cnt_r >= 24) begin
            case(cnt_20_r)
                0: maxpool_r <= maxpool;
                default: maxpool_r <= maxpool_r;
            endcase
        end
        else begin
            maxpool_r <= maxpool_r;
        end
    end
    else if (cs == CONV32) begin
        if (conv_cnt_r >= 34) begin
            case(cnt_20_r)
                0: maxpool_r <= maxpool;
                default: maxpool_r <= maxpool_r;
            endcase
        end
        else begin
            maxpool_r <= maxpool_r;
        end
    end
    else begin
        maxpool_r <= 0;
    end
end

/* img_raddr */
always @(*) begin
    if (cs == CONV08) begin
        if (conv_cnt_r < 12) begin
            case(conv_cnt_r)
                0 : img_raddr = IMG_START_ADDR_r + 0;
                1 : img_raddr = IMG_START_ADDR_r + 1;
                2 : img_raddr = IMG_START_ADDR_r + 2;
                3 : img_raddr = IMG_START_ADDR_r + 3;
                4 : img_raddr = IMG_START_ADDR_r + 4;
                5 : img_raddr = IMG_START_ADDR_r + 5;
                6 : img_raddr = IMG_START_ADDR_r + 2;
                7 : img_raddr = IMG_START_ADDR_r + 3;
                8 : img_raddr = IMG_START_ADDR_r + 4;
                9 : img_raddr = IMG_START_ADDR_r + 5;
                10: img_raddr = IMG_START_ADDR_r + 6;
                11: img_raddr = IMG_START_ADDR_r + 7;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else if (cs == CONV16) begin
        if (conv_cnt_r < 12) begin
            case(conv_cnt_r)
                0 : img_raddr = IMG_START_ADDR_r + 0;
                1 : img_raddr = IMG_START_ADDR_r + 2;
                2 : img_raddr = IMG_START_ADDR_r + 4;
                3 : img_raddr = IMG_START_ADDR_r + 6;
                4 : img_raddr = IMG_START_ADDR_r + 8;
                5 : img_raddr = IMG_START_ADDR_r + 10;
                6 : img_raddr = IMG_START_ADDR_r + 1;
                7 : img_raddr = IMG_START_ADDR_r + 3;
                8 : img_raddr = IMG_START_ADDR_r + 5;
                9 : img_raddr = IMG_START_ADDR_r + 7;
                10: img_raddr = IMG_START_ADDR_r + 9;
                11: img_raddr = IMG_START_ADDR_r + 11;
                default: img_raddr = 0;
            endcase
        end
        else if (shift_col_cnt_r == 11) begin
            case(cnt_10_r)
                0: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                1: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                2: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                3: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                4: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                5: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + ((change_row_cnt_r + 1) << 2);
                default: img_raddr = 0;
            endcase
        end
        else if (shift_col_cnt_r == 0) begin
            case(cnt_10_r)
                0: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                1: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                2: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                3: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                4: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                5: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + (change_row_cnt_r << 2) + 1;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else if (cs == CONV32) begin
        if (conv_cnt_r < 24) begin
            case(conv_cnt_r)
                0 : img_raddr = IMG_START_ADDR_r + 0;
                1 : img_raddr = IMG_START_ADDR_r + 4;
                2 : img_raddr = IMG_START_ADDR_r + 8;
                3 : img_raddr = IMG_START_ADDR_r + 12;
                4 : img_raddr = IMG_START_ADDR_r + 16;
                5 : img_raddr = IMG_START_ADDR_r + 20;

                6 : img_raddr = IMG_START_ADDR_r + 1;
                7 : img_raddr = IMG_START_ADDR_r + 5;
                8 : img_raddr = IMG_START_ADDR_r + 9;
                9 : img_raddr = IMG_START_ADDR_r + 13;
                10: img_raddr = IMG_START_ADDR_r + 17;
                11: img_raddr = IMG_START_ADDR_r + 21;

                12: img_raddr = IMG_START_ADDR_r + 2;
                13: img_raddr = IMG_START_ADDR_r + 6;
                14: img_raddr = IMG_START_ADDR_r + 10;
                15: img_raddr = IMG_START_ADDR_r + 14;
                16: img_raddr = IMG_START_ADDR_r + 18;
                17: img_raddr = IMG_START_ADDR_r + 22;

                18: img_raddr = IMG_START_ADDR_r + 3;
                19: img_raddr = IMG_START_ADDR_r + 7;
                20: img_raddr = IMG_START_ADDR_r + 11;
                21: img_raddr = IMG_START_ADDR_r + 15;
                22: img_raddr = IMG_START_ADDR_r + 19;
                23: img_raddr = IMG_START_ADDR_r + 23;
                default: img_raddr = 0;
            endcase
        end
        else if (shift_col_cnt_r == 16 || shift_col_cnt_r == 17) begin
            case(cnt_20_r)
                0: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);
                1: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);
                2: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);
                3: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);
                4: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);
                5: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + ((change_row_cnt_r + 1) << 3);

                6: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
                7: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
                8: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
                9: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
               10: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
               11: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + ((change_row_cnt_r + 1) << 3) + 1;
                default: img_raddr = 0;
            endcase
        end
        else if (shift_col_cnt_r == 0 || shift_col_cnt_r == 1) begin
            case(cnt_20_r)
                0: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;
                1: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;
                2: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;
                3: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;
                4: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;
                5: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + ((change_row_cnt_r) << 3) + 2;

                6: img_raddr = IMG_START_ADDR_r + (0 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
                7: img_raddr = IMG_START_ADDR_r + (1 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
                8: img_raddr = IMG_START_ADDR_r + (2 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
                9: img_raddr = IMG_START_ADDR_r + (3 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
               10: img_raddr = IMG_START_ADDR_r + (4 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
               11: img_raddr = IMG_START_ADDR_r + (5 << matrix_size_r) + ((change_row_cnt_r) << 3) + 3;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else if (cs == DECONV08) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                0: img_raddr = IMG_START_ADDR_r + 0;
                1: img_raddr = 0;
                2: img_raddr = 0;
                3: img_raddr = 0;
                default: img_raddr = 0;
            endcase
        end
        else if (deconv_col_r == 0) begin
            case(out_cnt_20_r)
                0: img_raddr = IMG_START_ADDR_r + deconv_row_r;
                1: img_raddr = 0;
                2: img_raddr = 0;
                3: img_raddr = 0;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else if (cs == DECONV16) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                0: img_raddr = IMG_START_ADDR_r + 0;
                1: img_raddr = IMG_START_ADDR_r + 1;
                2: img_raddr = 0;
                3: img_raddr = 0;
                default: img_raddr = 0;
            endcase
        end
        else if (deconv_col_r == 0) begin
            case(out_cnt_20_r)
                0: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 1);
                1: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 1) + 1;
                2: img_raddr = 0;
                3: img_raddr = 0;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else if (cs == DECONV32) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                0: img_raddr = IMG_START_ADDR_r + 0;
                1: img_raddr = IMG_START_ADDR_r + 1;
                2: img_raddr = IMG_START_ADDR_r + 2;
                3: img_raddr = IMG_START_ADDR_r + 3;
                default: img_raddr = 0;
            endcase
        end
        else if (deconv_col_r == 0) begin
            case(out_cnt_20_r)
                0: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 2) + 0;
                1: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 2) + 1;
                2: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 2) + 2;
                3: img_raddr = IMG_START_ADDR_r + (deconv_row_r << 2) + 3;
                default: img_raddr = 0;
            endcase
        end
        else begin
            img_raddr = 0;
        end
    end
    else begin
        img_raddr = 0;
    end
end

/* ker_raddr */
always @(*) begin
    if (cs == CONV08 || cs == CONV16 || cs == CONV32) begin
        case(conv_cnt_r)
            0 : ker_raddr = KER_START_ADDR_r + 0;
            1 : ker_raddr = KER_START_ADDR_r + 1;
            2 : ker_raddr = KER_START_ADDR_r + 2;
            3 : ker_raddr = KER_START_ADDR_r + 3;
            4 : ker_raddr = KER_START_ADDR_r + 4;
            default: ker_raddr = 0;
        endcase
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        case(deconv_cnt_r)
            4 : ker_raddr = KER_START_ADDR_r + 0;
            3 : ker_raddr = KER_START_ADDR_r + 1;
            2 : ker_raddr = KER_START_ADDR_r + 2;
            1 : ker_raddr = KER_START_ADDR_r + 3;
            0 : ker_raddr = KER_START_ADDR_r + 4;
            default: ker_raddr = 0;
        endcase
    end
    else begin
        ker_raddr = 0;
    end
end



/* IMG_START_ADDR_r, KER_START_ADDR_r*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        IMG_START_ADDR_r <= 0;
        KER_START_ADDR_r <= 0;
    end
    else if (cs == INVALID2_0 && in_valid2) begin
        case(matrix_size_r)
            2'd0: IMG_START_ADDR_r <= 11'd8   * matrix_idx;
            2'd1: IMG_START_ADDR_r <= 11'd32  * matrix_idx;
            2'd2: IMG_START_ADDR_r <= 11'd128 * matrix_idx;
        endcase
        KER_START_ADDR_r <= KER_START_ADDR_r;
    end
    else if (cs == INVALID2_1) begin
        IMG_START_ADDR_r <= IMG_START_ADDR_r;
        KER_START_ADDR_r <= 7'd5 * matrix_idx;
    end
    else begin
        IMG_START_ADDR_r <= IMG_START_ADDR_r;
        KER_START_ADDR_r <= KER_START_ADDR_r;
    end
end

/* conv_cnt_r*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_cnt_r <= 0;
    else if (cs == CONV08)
        conv_cnt_r <= conv_cnt_r + 1'd1;
    else if (cs == CONV16)
        conv_cnt_r <= conv_cnt_r + 1'd1;
    else if (cs == CONV32)
        conv_cnt_r <= conv_cnt_r + 1'd1;
    else
        conv_cnt_r <= 0;
end

/* cnt_10_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_10_r <= 0;
    else if ((cs == CONV08 || cs == CONV16) && conv_cnt_r == 3)
        cnt_10_r <= 0;
    else if (cs == CONV32 && conv_cnt_r == 13)
        cnt_10_r <= 0;
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32)
        cnt_10_r <= (cnt_10_r == 9) ? 0 : cnt_10_r + 1'd1;
    else
        cnt_10_r <= 0;
end

/* cnt_20_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_20_r <= 0;
    else if ((cs == CONV08 || cs == CONV16) && conv_cnt_r == 3)
        cnt_20_r <= 0;
    else if (cs == CONV32 && conv_cnt_r == 13)
        cnt_20_r <= 0;
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32)
        cnt_20_r <= (cnt_20_r == 19) ? 0 : cnt_20_r + 1'd1;
    else
        cnt_20_r <= 0;
end

/* shift_col_cnt_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        shift_col_cnt_r <= 0;
    else if (cs == CONV08) begin
        if (cnt_10_r == 9)
            shift_col_cnt_r <= (shift_col_cnt_r == 3) ? 0 : shift_col_cnt_r + 1'd1;
        else
            shift_col_cnt_r <= shift_col_cnt_r;
    end
    else if (cs == CONV16) begin
        if (cnt_10_r == 9)
            shift_col_cnt_r <= (shift_col_cnt_r == 11) ? 0 : shift_col_cnt_r + 1'd1;
        else
            shift_col_cnt_r <= shift_col_cnt_r;
    end
    else if (cs == CONV32) begin
        if (cnt_10_r == 9 && conv_cnt_r > 13)
            shift_col_cnt_r <= (shift_col_cnt_r == 27) ? 0 : shift_col_cnt_r + 1'd1;
        else
            shift_col_cnt_r <= shift_col_cnt_r;
    end
    else
        shift_col_cnt_r <= 0;
end

/* change_row_cnt_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        change_row_cnt_r <= 0;
    else if (cs == CONV08) begin
        if (cnt_10_r == 9 && shift_col_cnt_r == 3)
            change_row_cnt_r <= change_row_cnt_r + 1'd1;
        else
            change_row_cnt_r <= change_row_cnt_r;
    end
    else if (cs == CONV16) begin
        if (cnt_10_r == 9 && shift_col_cnt_r == 11)
            change_row_cnt_r <= change_row_cnt_r + 1'd1;
        else
            change_row_cnt_r <= change_row_cnt_r;
    end
    else if (cs == CONV32) begin
        if (cnt_10_r == 9 && shift_col_cnt_r == 27)
            change_row_cnt_r <= change_row_cnt_r + 1'd1;
        else
            change_row_cnt_r <= change_row_cnt_r;
    end
    else
        change_row_cnt_r <= 0;
end


//---------------------------------------------------------------------
//   DECONV
//---------------------------------------------------------------------
/* img_5X32_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 5; i = i + 1) begin
            for (j = 0; j < 32; j = j + 1) begin
                img_5X32_r[i][j] <= 0;
            end
        end
    end
    else if (cs == DECONV08) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                1: {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= DO_IMG;
                2: {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= 0;
                3: {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= 0;
                4: {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= 0;
            endcase
        end
        else if (deconv_col_r == 0) begin // PUSH UP
            case(out_cnt_20_r)
                1: begin // col 1
                    img_5X32_r[0][0] <= img_5X32_r[1][0];   img_5X32_r[0][1] <= img_5X32_r[1][1];  img_5X32_r[0][2] <= img_5X32_r[1][2];  img_5X32_r[0][3] <= img_5X32_r[1][3];  img_5X32_r[0][4] <= img_5X32_r[1][4];  img_5X32_r[0][5] <= img_5X32_r[1][5];  img_5X32_r[0][6] <= img_5X32_r[1][6];  img_5X32_r[0][7] <= img_5X32_r[1][7];
                    img_5X32_r[1][0] <= img_5X32_r[2][0];   img_5X32_r[1][1] <= img_5X32_r[2][1];  img_5X32_r[1][2] <= img_5X32_r[2][2];  img_5X32_r[1][3] <= img_5X32_r[2][3];  img_5X32_r[1][4] <= img_5X32_r[2][4];  img_5X32_r[1][5] <= img_5X32_r[2][5];  img_5X32_r[1][6] <= img_5X32_r[2][6];  img_5X32_r[1][7] <= img_5X32_r[2][7];
                    img_5X32_r[2][0] <= img_5X32_r[3][0];   img_5X32_r[2][1] <= img_5X32_r[3][1];  img_5X32_r[2][2] <= img_5X32_r[3][2];  img_5X32_r[2][3] <= img_5X32_r[3][3];  img_5X32_r[2][4] <= img_5X32_r[3][4];  img_5X32_r[2][5] <= img_5X32_r[3][5];  img_5X32_r[2][6] <= img_5X32_r[3][6];  img_5X32_r[2][7] <= img_5X32_r[3][7];
                    img_5X32_r[3][0] <= img_5X32_r[4][0];   img_5X32_r[3][1] <= img_5X32_r[4][1];  img_5X32_r[3][2] <= img_5X32_r[4][2];  img_5X32_r[3][3] <= img_5X32_r[4][3];  img_5X32_r[3][4] <= img_5X32_r[4][4];  img_5X32_r[3][5] <= img_5X32_r[4][5];  img_5X32_r[3][6] <= img_5X32_r[4][6];  img_5X32_r[3][7] <= img_5X32_r[4][7];
                    {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= (deconv_row_r >= 8) ? 0 : DO_IMG;
                end
                2: begin // col 2
                    img_5X32_r[0][8] <= img_5X32_r[1][8];   img_5X32_r[0][9] <= img_5X32_r[1][9];  img_5X32_r[0][10] <= img_5X32_r[1][10];  img_5X32_r[0][11] <= img_5X32_r[1][11];  img_5X32_r[0][12] <= img_5X32_r[1][12];  img_5X32_r[0][13] <= img_5X32_r[1][13];  img_5X32_r[0][14] <= img_5X32_r[1][14];  img_5X32_r[0][15] <= img_5X32_r[1][15];
                    img_5X32_r[1][8] <= img_5X32_r[2][8];   img_5X32_r[1][9] <= img_5X32_r[2][9];  img_5X32_r[1][10] <= img_5X32_r[2][10];  img_5X32_r[1][11] <= img_5X32_r[2][11];  img_5X32_r[1][12] <= img_5X32_r[2][12];  img_5X32_r[1][13] <= img_5X32_r[2][13];  img_5X32_r[1][14] <= img_5X32_r[2][14];  img_5X32_r[1][15] <= img_5X32_r[2][15];
                    img_5X32_r[2][8] <= img_5X32_r[3][8];   img_5X32_r[2][9] <= img_5X32_r[3][9];  img_5X32_r[2][10] <= img_5X32_r[3][10];  img_5X32_r[2][11] <= img_5X32_r[3][11];  img_5X32_r[2][12] <= img_5X32_r[3][12];  img_5X32_r[2][13] <= img_5X32_r[3][13];  img_5X32_r[2][14] <= img_5X32_r[3][14];  img_5X32_r[2][15] <= img_5X32_r[3][15];
                    img_5X32_r[3][8] <= img_5X32_r[4][8];   img_5X32_r[3][9] <= img_5X32_r[4][9];  img_5X32_r[3][10] <= img_5X32_r[4][10];  img_5X32_r[3][11] <= img_5X32_r[4][11];  img_5X32_r[3][12] <= img_5X32_r[4][12];  img_5X32_r[3][13] <= img_5X32_r[4][13];  img_5X32_r[3][14] <= img_5X32_r[4][14];  img_5X32_r[3][15] <= img_5X32_r[4][15];
                    {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= 0;
                end
                3: begin // col 3
                    img_5X32_r[0][16] <= img_5X32_r[1][16];   img_5X32_r[0][17] <= img_5X32_r[1][17];  img_5X32_r[0][18] <= img_5X32_r[1][18];  img_5X32_r[0][19] <= img_5X32_r[1][19];  img_5X32_r[0][20] <= img_5X32_r[1][20];  img_5X32_r[0][21] <= img_5X32_r[1][21];  img_5X32_r[0][22] <= img_5X32_r[1][22];  img_5X32_r[0][23] <= img_5X32_r[1][23];
                    img_5X32_r[1][16] <= img_5X32_r[2][16];   img_5X32_r[1][17] <= img_5X32_r[2][17];  img_5X32_r[1][18] <= img_5X32_r[2][18];  img_5X32_r[1][19] <= img_5X32_r[2][19];  img_5X32_r[1][20] <= img_5X32_r[2][20];  img_5X32_r[1][21] <= img_5X32_r[2][21];  img_5X32_r[1][22] <= img_5X32_r[2][22];  img_5X32_r[1][23] <= img_5X32_r[2][23];
                    img_5X32_r[2][16] <= img_5X32_r[3][16];   img_5X32_r[2][17] <= img_5X32_r[3][17];  img_5X32_r[2][18] <= img_5X32_r[3][18];  img_5X32_r[2][19] <= img_5X32_r[3][19];  img_5X32_r[2][20] <= img_5X32_r[3][20];  img_5X32_r[2][21] <= img_5X32_r[3][21];  img_5X32_r[2][22] <= img_5X32_r[3][22];  img_5X32_r[2][23] <= img_5X32_r[3][23];
                    img_5X32_r[3][16] <= img_5X32_r[4][16];   img_5X32_r[3][17] <= img_5X32_r[4][17];  img_5X32_r[3][18] <= img_5X32_r[4][18];  img_5X32_r[3][19] <= img_5X32_r[4][19];  img_5X32_r[3][20] <= img_5X32_r[4][20];  img_5X32_r[3][21] <= img_5X32_r[4][21];  img_5X32_r[3][22] <= img_5X32_r[4][22];  img_5X32_r[3][23] <= img_5X32_r[4][23];
                    {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= 0;
                end
                4: begin // col 4
                    img_5X32_r[0][24] <= img_5X32_r[1][24];   img_5X32_r[0][25] <= img_5X32_r[1][25];  img_5X32_r[0][26] <= img_5X32_r[1][26];  img_5X32_r[0][27] <= img_5X32_r[1][27];  img_5X32_r[0][28] <= img_5X32_r[1][28];  img_5X32_r[0][29] <= img_5X32_r[1][29];  img_5X32_r[0][30] <= img_5X32_r[1][30];  img_5X32_r[0][31] <= img_5X32_r[1][31];
                    img_5X32_r[1][24] <= img_5X32_r[2][24];   img_5X32_r[1][25] <= img_5X32_r[2][25];  img_5X32_r[1][26] <= img_5X32_r[2][26];  img_5X32_r[1][27] <= img_5X32_r[2][27];  img_5X32_r[1][28] <= img_5X32_r[2][28];  img_5X32_r[1][29] <= img_5X32_r[2][29];  img_5X32_r[1][30] <= img_5X32_r[2][30];  img_5X32_r[1][31] <= img_5X32_r[2][31];
                    img_5X32_r[2][24] <= img_5X32_r[3][24];   img_5X32_r[2][25] <= img_5X32_r[3][25];  img_5X32_r[2][26] <= img_5X32_r[3][26];  img_5X32_r[2][27] <= img_5X32_r[3][27];  img_5X32_r[2][28] <= img_5X32_r[3][28];  img_5X32_r[2][29] <= img_5X32_r[3][29];  img_5X32_r[2][30] <= img_5X32_r[3][30];  img_5X32_r[2][31] <= img_5X32_r[3][31];
                    img_5X32_r[3][24] <= img_5X32_r[4][24];   img_5X32_r[3][25] <= img_5X32_r[4][25];  img_5X32_r[3][26] <= img_5X32_r[4][26];  img_5X32_r[3][27] <= img_5X32_r[4][27];  img_5X32_r[3][28] <= img_5X32_r[4][28];  img_5X32_r[3][29] <= img_5X32_r[4][29];  img_5X32_r[3][30] <= img_5X32_r[4][30];  img_5X32_r[3][31] <= img_5X32_r[4][31];
                    {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= 0;
                end
            endcase
        end
        else begin
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 32; j = j + 1) begin
                    img_5X32_r[i][j] <= img_5X32_r[i][j];
                end
            end
        end
    end
    else if (cs == DECONV16) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                1: {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= DO_IMG;
                2: {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= DO_IMG;
                3: {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= 0;
                4: {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= 0;
            endcase
        end
        else if (deconv_col_r == 0) begin // PUSH UP
            case(out_cnt_20_r)
                1: begin // col 1
                    img_5X32_r[0][0] <= img_5X32_r[1][0];   img_5X32_r[0][1] <= img_5X32_r[1][1];  img_5X32_r[0][2] <= img_5X32_r[1][2];  img_5X32_r[0][3] <= img_5X32_r[1][3];  img_5X32_r[0][4] <= img_5X32_r[1][4];  img_5X32_r[0][5] <= img_5X32_r[1][5];  img_5X32_r[0][6] <= img_5X32_r[1][6];  img_5X32_r[0][7] <= img_5X32_r[1][7];
                    img_5X32_r[1][0] <= img_5X32_r[2][0];   img_5X32_r[1][1] <= img_5X32_r[2][1];  img_5X32_r[1][2] <= img_5X32_r[2][2];  img_5X32_r[1][3] <= img_5X32_r[2][3];  img_5X32_r[1][4] <= img_5X32_r[2][4];  img_5X32_r[1][5] <= img_5X32_r[2][5];  img_5X32_r[1][6] <= img_5X32_r[2][6];  img_5X32_r[1][7] <= img_5X32_r[2][7];
                    img_5X32_r[2][0] <= img_5X32_r[3][0];   img_5X32_r[2][1] <= img_5X32_r[3][1];  img_5X32_r[2][2] <= img_5X32_r[3][2];  img_5X32_r[2][3] <= img_5X32_r[3][3];  img_5X32_r[2][4] <= img_5X32_r[3][4];  img_5X32_r[2][5] <= img_5X32_r[3][5];  img_5X32_r[2][6] <= img_5X32_r[3][6];  img_5X32_r[2][7] <= img_5X32_r[3][7];
                    img_5X32_r[3][0] <= img_5X32_r[4][0];   img_5X32_r[3][1] <= img_5X32_r[4][1];  img_5X32_r[3][2] <= img_5X32_r[4][2];  img_5X32_r[3][3] <= img_5X32_r[4][3];  img_5X32_r[3][4] <= img_5X32_r[4][4];  img_5X32_r[3][5] <= img_5X32_r[4][5];  img_5X32_r[3][6] <= img_5X32_r[4][6];  img_5X32_r[3][7] <= img_5X32_r[4][7];
                    {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= (deconv_row_r >= 16) ? 0 : DO_IMG;
                end
                2: begin // col 2
                    img_5X32_r[0][8] <= img_5X32_r[1][8];   img_5X32_r[0][9] <= img_5X32_r[1][9];  img_5X32_r[0][10] <= img_5X32_r[1][10];  img_5X32_r[0][11] <= img_5X32_r[1][11];  img_5X32_r[0][12] <= img_5X32_r[1][12];  img_5X32_r[0][13] <= img_5X32_r[1][13];  img_5X32_r[0][14] <= img_5X32_r[1][14];  img_5X32_r[0][15] <= img_5X32_r[1][15];
                    img_5X32_r[1][8] <= img_5X32_r[2][8];   img_5X32_r[1][9] <= img_5X32_r[2][9];  img_5X32_r[1][10] <= img_5X32_r[2][10];  img_5X32_r[1][11] <= img_5X32_r[2][11];  img_5X32_r[1][12] <= img_5X32_r[2][12];  img_5X32_r[1][13] <= img_5X32_r[2][13];  img_5X32_r[1][14] <= img_5X32_r[2][14];  img_5X32_r[1][15] <= img_5X32_r[2][15];
                    img_5X32_r[2][8] <= img_5X32_r[3][8];   img_5X32_r[2][9] <= img_5X32_r[3][9];  img_5X32_r[2][10] <= img_5X32_r[3][10];  img_5X32_r[2][11] <= img_5X32_r[3][11];  img_5X32_r[2][12] <= img_5X32_r[3][12];  img_5X32_r[2][13] <= img_5X32_r[3][13];  img_5X32_r[2][14] <= img_5X32_r[3][14];  img_5X32_r[2][15] <= img_5X32_r[3][15];
                    img_5X32_r[3][8] <= img_5X32_r[4][8];   img_5X32_r[3][9] <= img_5X32_r[4][9];  img_5X32_r[3][10] <= img_5X32_r[4][10];  img_5X32_r[3][11] <= img_5X32_r[4][11];  img_5X32_r[3][12] <= img_5X32_r[4][12];  img_5X32_r[3][13] <= img_5X32_r[4][13];  img_5X32_r[3][14] <= img_5X32_r[4][14];  img_5X32_r[3][15] <= img_5X32_r[4][15];
                    {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= (deconv_row_r >= 16) ? 0 : DO_IMG;
                end
                3: begin // col 3
                    img_5X32_r[0][16] <= img_5X32_r[1][16];   img_5X32_r[0][17] <= img_5X32_r[1][17];  img_5X32_r[0][18] <= img_5X32_r[1][18];  img_5X32_r[0][19] <= img_5X32_r[1][19];  img_5X32_r[0][20] <= img_5X32_r[1][20];  img_5X32_r[0][21] <= img_5X32_r[1][21];  img_5X32_r[0][22] <= img_5X32_r[1][22];  img_5X32_r[0][23] <= img_5X32_r[1][23];
                    img_5X32_r[1][16] <= img_5X32_r[2][16];   img_5X32_r[1][17] <= img_5X32_r[2][17];  img_5X32_r[1][18] <= img_5X32_r[2][18];  img_5X32_r[1][19] <= img_5X32_r[2][19];  img_5X32_r[1][20] <= img_5X32_r[2][20];  img_5X32_r[1][21] <= img_5X32_r[2][21];  img_5X32_r[1][22] <= img_5X32_r[2][22];  img_5X32_r[1][23] <= img_5X32_r[2][23];
                    img_5X32_r[2][16] <= img_5X32_r[3][16];   img_5X32_r[2][17] <= img_5X32_r[3][17];  img_5X32_r[2][18] <= img_5X32_r[3][18];  img_5X32_r[2][19] <= img_5X32_r[3][19];  img_5X32_r[2][20] <= img_5X32_r[3][20];  img_5X32_r[2][21] <= img_5X32_r[3][21];  img_5X32_r[2][22] <= img_5X32_r[3][22];  img_5X32_r[2][23] <= img_5X32_r[3][23];
                    img_5X32_r[3][16] <= img_5X32_r[4][16];   img_5X32_r[3][17] <= img_5X32_r[4][17];  img_5X32_r[3][18] <= img_5X32_r[4][18];  img_5X32_r[3][19] <= img_5X32_r[4][19];  img_5X32_r[3][20] <= img_5X32_r[4][20];  img_5X32_r[3][21] <= img_5X32_r[4][21];  img_5X32_r[3][22] <= img_5X32_r[4][22];  img_5X32_r[3][23] <= img_5X32_r[4][23];
                    {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= 0;
                end
                4: begin // col 4
                    img_5X32_r[0][24] <= img_5X32_r[1][24];   img_5X32_r[0][25] <= img_5X32_r[1][25];  img_5X32_r[0][26] <= img_5X32_r[1][26];  img_5X32_r[0][27] <= img_5X32_r[1][27];  img_5X32_r[0][28] <= img_5X32_r[1][28];  img_5X32_r[0][29] <= img_5X32_r[1][29];  img_5X32_r[0][30] <= img_5X32_r[1][30];  img_5X32_r[0][31] <= img_5X32_r[1][31];
                    img_5X32_r[1][24] <= img_5X32_r[2][24];   img_5X32_r[1][25] <= img_5X32_r[2][25];  img_5X32_r[1][26] <= img_5X32_r[2][26];  img_5X32_r[1][27] <= img_5X32_r[2][27];  img_5X32_r[1][28] <= img_5X32_r[2][28];  img_5X32_r[1][29] <= img_5X32_r[2][29];  img_5X32_r[1][30] <= img_5X32_r[2][30];  img_5X32_r[1][31] <= img_5X32_r[2][31];
                    img_5X32_r[2][24] <= img_5X32_r[3][24];   img_5X32_r[2][25] <= img_5X32_r[3][25];  img_5X32_r[2][26] <= img_5X32_r[3][26];  img_5X32_r[2][27] <= img_5X32_r[3][27];  img_5X32_r[2][28] <= img_5X32_r[3][28];  img_5X32_r[2][29] <= img_5X32_r[3][29];  img_5X32_r[2][30] <= img_5X32_r[3][30];  img_5X32_r[2][31] <= img_5X32_r[3][31];
                    img_5X32_r[3][24] <= img_5X32_r[4][24];   img_5X32_r[3][25] <= img_5X32_r[4][25];  img_5X32_r[3][26] <= img_5X32_r[4][26];  img_5X32_r[3][27] <= img_5X32_r[4][27];  img_5X32_r[3][28] <= img_5X32_r[4][28];  img_5X32_r[3][29] <= img_5X32_r[4][29];  img_5X32_r[3][30] <= img_5X32_r[4][30];  img_5X32_r[3][31] <= img_5X32_r[4][31];
                    {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= 0;
                end
            endcase
        end
        else begin
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 32; j = j + 1) begin
                    img_5X32_r[i][j] <= img_5X32_r[i][j];
                end
            end
        end
    end
    else if (cs == DECONV32) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                1: {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= DO_IMG;
                2: {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= DO_IMG;
                3: {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= DO_IMG;
                4: {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= DO_IMG;
            endcase
        end
        else if (deconv_col_r == 0) begin // PUSH UP
            case(out_cnt_20_r)
                1: begin // col 1
                    img_5X32_r[0][0] <= img_5X32_r[1][0];   img_5X32_r[0][1] <= img_5X32_r[1][1];  img_5X32_r[0][2] <= img_5X32_r[1][2];  img_5X32_r[0][3] <= img_5X32_r[1][3];  img_5X32_r[0][4] <= img_5X32_r[1][4];  img_5X32_r[0][5] <= img_5X32_r[1][5];  img_5X32_r[0][6] <= img_5X32_r[1][6];  img_5X32_r[0][7] <= img_5X32_r[1][7];
                    img_5X32_r[1][0] <= img_5X32_r[2][0];   img_5X32_r[1][1] <= img_5X32_r[2][1];  img_5X32_r[1][2] <= img_5X32_r[2][2];  img_5X32_r[1][3] <= img_5X32_r[2][3];  img_5X32_r[1][4] <= img_5X32_r[2][4];  img_5X32_r[1][5] <= img_5X32_r[2][5];  img_5X32_r[1][6] <= img_5X32_r[2][6];  img_5X32_r[1][7] <= img_5X32_r[2][7];
                    img_5X32_r[2][0] <= img_5X32_r[3][0];   img_5X32_r[2][1] <= img_5X32_r[3][1];  img_5X32_r[2][2] <= img_5X32_r[3][2];  img_5X32_r[2][3] <= img_5X32_r[3][3];  img_5X32_r[2][4] <= img_5X32_r[3][4];  img_5X32_r[2][5] <= img_5X32_r[3][5];  img_5X32_r[2][6] <= img_5X32_r[3][6];  img_5X32_r[2][7] <= img_5X32_r[3][7];
                    img_5X32_r[3][0] <= img_5X32_r[4][0];   img_5X32_r[3][1] <= img_5X32_r[4][1];  img_5X32_r[3][2] <= img_5X32_r[4][2];  img_5X32_r[3][3] <= img_5X32_r[4][3];  img_5X32_r[3][4] <= img_5X32_r[4][4];  img_5X32_r[3][5] <= img_5X32_r[4][5];  img_5X32_r[3][6] <= img_5X32_r[4][6];  img_5X32_r[3][7] <= img_5X32_r[4][7];
                    {img_5X32_r[4][0],  img_5X32_r[4][1],  img_5X32_r[4][2],  img_5X32_r[4][3],  img_5X32_r[4][4],  img_5X32_r[4][5],  img_5X32_r[4][6],  img_5X32_r[4][7]}  <= (deconv_row_r >= 32) ? 0 : DO_IMG;
                end
                2: begin // col 2
                    img_5X32_r[0][8] <= img_5X32_r[1][8];   img_5X32_r[0][9] <= img_5X32_r[1][9];  img_5X32_r[0][10] <= img_5X32_r[1][10];  img_5X32_r[0][11] <= img_5X32_r[1][11];  img_5X32_r[0][12] <= img_5X32_r[1][12];  img_5X32_r[0][13] <= img_5X32_r[1][13];  img_5X32_r[0][14] <= img_5X32_r[1][14];  img_5X32_r[0][15] <= img_5X32_r[1][15];
                    img_5X32_r[1][8] <= img_5X32_r[2][8];   img_5X32_r[1][9] <= img_5X32_r[2][9];  img_5X32_r[1][10] <= img_5X32_r[2][10];  img_5X32_r[1][11] <= img_5X32_r[2][11];  img_5X32_r[1][12] <= img_5X32_r[2][12];  img_5X32_r[1][13] <= img_5X32_r[2][13];  img_5X32_r[1][14] <= img_5X32_r[2][14];  img_5X32_r[1][15] <= img_5X32_r[2][15];
                    img_5X32_r[2][8] <= img_5X32_r[3][8];   img_5X32_r[2][9] <= img_5X32_r[3][9];  img_5X32_r[2][10] <= img_5X32_r[3][10];  img_5X32_r[2][11] <= img_5X32_r[3][11];  img_5X32_r[2][12] <= img_5X32_r[3][12];  img_5X32_r[2][13] <= img_5X32_r[3][13];  img_5X32_r[2][14] <= img_5X32_r[3][14];  img_5X32_r[2][15] <= img_5X32_r[3][15];
                    img_5X32_r[3][8] <= img_5X32_r[4][8];   img_5X32_r[3][9] <= img_5X32_r[4][9];  img_5X32_r[3][10] <= img_5X32_r[4][10];  img_5X32_r[3][11] <= img_5X32_r[4][11];  img_5X32_r[3][12] <= img_5X32_r[4][12];  img_5X32_r[3][13] <= img_5X32_r[4][13];  img_5X32_r[3][14] <= img_5X32_r[4][14];  img_5X32_r[3][15] <= img_5X32_r[4][15];
                    {img_5X32_r[4][8],  img_5X32_r[4][9],  img_5X32_r[4][10], img_5X32_r[4][11], img_5X32_r[4][12], img_5X32_r[4][13], img_5X32_r[4][14], img_5X32_r[4][15]} <= (deconv_row_r >= 32) ? 0 : DO_IMG;
                end
                3: begin // col 3
                    img_5X32_r[0][16] <= img_5X32_r[1][16];   img_5X32_r[0][17] <= img_5X32_r[1][17];  img_5X32_r[0][18] <= img_5X32_r[1][18];  img_5X32_r[0][19] <= img_5X32_r[1][19];  img_5X32_r[0][20] <= img_5X32_r[1][20];  img_5X32_r[0][21] <= img_5X32_r[1][21];  img_5X32_r[0][22] <= img_5X32_r[1][22];  img_5X32_r[0][23] <= img_5X32_r[1][23];
                    img_5X32_r[1][16] <= img_5X32_r[2][16];   img_5X32_r[1][17] <= img_5X32_r[2][17];  img_5X32_r[1][18] <= img_5X32_r[2][18];  img_5X32_r[1][19] <= img_5X32_r[2][19];  img_5X32_r[1][20] <= img_5X32_r[2][20];  img_5X32_r[1][21] <= img_5X32_r[2][21];  img_5X32_r[1][22] <= img_5X32_r[2][22];  img_5X32_r[1][23] <= img_5X32_r[2][23];
                    img_5X32_r[2][16] <= img_5X32_r[3][16];   img_5X32_r[2][17] <= img_5X32_r[3][17];  img_5X32_r[2][18] <= img_5X32_r[3][18];  img_5X32_r[2][19] <= img_5X32_r[3][19];  img_5X32_r[2][20] <= img_5X32_r[3][20];  img_5X32_r[2][21] <= img_5X32_r[3][21];  img_5X32_r[2][22] <= img_5X32_r[3][22];  img_5X32_r[2][23] <= img_5X32_r[3][23];
                    img_5X32_r[3][16] <= img_5X32_r[4][16];   img_5X32_r[3][17] <= img_5X32_r[4][17];  img_5X32_r[3][18] <= img_5X32_r[4][18];  img_5X32_r[3][19] <= img_5X32_r[4][19];  img_5X32_r[3][20] <= img_5X32_r[4][20];  img_5X32_r[3][21] <= img_5X32_r[4][21];  img_5X32_r[3][22] <= img_5X32_r[4][22];  img_5X32_r[3][23] <= img_5X32_r[4][23];
                    {img_5X32_r[4][16], img_5X32_r[4][17], img_5X32_r[4][18], img_5X32_r[4][19], img_5X32_r[4][20], img_5X32_r[4][21], img_5X32_r[4][22], img_5X32_r[4][23]} <= (deconv_row_r >= 32) ? 0 : DO_IMG;
                end
                4: begin // col 4
                    img_5X32_r[0][24] <= img_5X32_r[1][24];   img_5X32_r[0][25] <= img_5X32_r[1][25];  img_5X32_r[0][26] <= img_5X32_r[1][26];  img_5X32_r[0][27] <= img_5X32_r[1][27];  img_5X32_r[0][28] <= img_5X32_r[1][28];  img_5X32_r[0][29] <= img_5X32_r[1][29];  img_5X32_r[0][30] <= img_5X32_r[1][30];  img_5X32_r[0][31] <= img_5X32_r[1][31];
                    img_5X32_r[1][24] <= img_5X32_r[2][24];   img_5X32_r[1][25] <= img_5X32_r[2][25];  img_5X32_r[1][26] <= img_5X32_r[2][26];  img_5X32_r[1][27] <= img_5X32_r[2][27];  img_5X32_r[1][28] <= img_5X32_r[2][28];  img_5X32_r[1][29] <= img_5X32_r[2][29];  img_5X32_r[1][30] <= img_5X32_r[2][30];  img_5X32_r[1][31] <= img_5X32_r[2][31];
                    img_5X32_r[2][24] <= img_5X32_r[3][24];   img_5X32_r[2][25] <= img_5X32_r[3][25];  img_5X32_r[2][26] <= img_5X32_r[3][26];  img_5X32_r[2][27] <= img_5X32_r[3][27];  img_5X32_r[2][28] <= img_5X32_r[3][28];  img_5X32_r[2][29] <= img_5X32_r[3][29];  img_5X32_r[2][30] <= img_5X32_r[3][30];  img_5X32_r[2][31] <= img_5X32_r[3][31];
                    img_5X32_r[3][24] <= img_5X32_r[4][24];   img_5X32_r[3][25] <= img_5X32_r[4][25];  img_5X32_r[3][26] <= img_5X32_r[4][26];  img_5X32_r[3][27] <= img_5X32_r[4][27];  img_5X32_r[3][28] <= img_5X32_r[4][28];  img_5X32_r[3][29] <= img_5X32_r[4][29];  img_5X32_r[3][30] <= img_5X32_r[4][30];  img_5X32_r[3][31] <= img_5X32_r[4][31];
                    {img_5X32_r[4][24], img_5X32_r[4][25], img_5X32_r[4][26], img_5X32_r[4][27], img_5X32_r[4][28], img_5X32_r[4][29], img_5X32_r[4][30], img_5X32_r[4][31]} <= (deconv_row_r >= 32) ? 0 : DO_IMG;
                end
            endcase
        end
        else begin
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 32; j = j + 1) begin
                    img_5X32_r[i][j] <= img_5X32_r[i][j];
                end
            end
        end
    end
    else begin
        for (i = 0; i < 5; i = i + 1) begin
            for (j = 0; j < 32; j = j + 1) begin
                img_5X32_r[i][j] <= 0;
            end
        end
    end
end

/* img_5X40_pad */
always @(*) begin
    for (i = 0; i < 5; i = i + 1) begin
        for (j = 0; j < 4; j = j + 1) begin
            img_5X40_pad[i][j] = 0;
            img_5X40_pad[i][j+36] = 0;
        end
    end
    for (i = 0; i < 5; i = i + 1) begin
        for (j = 4; j < 36; j = j + 1) begin
            img_5X40_pad[i][j] = img_5X32_r[i][j-4];
        end
    end
end

/* img_5X5_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_5X5_r[0][0] <= 0;    img_5X5_r[0][1] <= 0;    img_5X5_r[0][2] <= 0;    img_5X5_r[0][3] <= 0;    img_5X5_r[0][4] <= 0;
        img_5X5_r[1][0] <= 0;    img_5X5_r[1][1] <= 0;    img_5X5_r[1][2] <= 0;    img_5X5_r[1][3] <= 0;    img_5X5_r[1][4] <= 0;
        img_5X5_r[2][0] <= 0;    img_5X5_r[2][1] <= 0;    img_5X5_r[2][2] <= 0;    img_5X5_r[2][3] <= 0;    img_5X5_r[2][4] <= 0;
        img_5X5_r[3][0] <= 0;    img_5X5_r[3][1] <= 0;    img_5X5_r[3][2] <= 0;    img_5X5_r[3][3] <= 0;    img_5X5_r[3][4] <= 0;
        img_5X5_r[4][0] <= 0;    img_5X5_r[4][1] <= 0;    img_5X5_r[4][2] <= 0;    img_5X5_r[4][3] <= 0;    img_5X5_r[4][4] <= 0;
    end
    else if (deconv_cnt_r == 7) begin // Col shift right (setup stage)
        // img_5X5_r[0][0] <= img_5X5_r[0][1];    img_5X5_r[0][1] <= img_5X5_r[0][2];    img_5X5_r[0][2] <= img_5X5_r[0][3];    img_5X5_r[0][3] <= img_5X5_r[0][4];    img_5X5_r[0][4] <= img_5X32_r[0][deconv_col_r];
        // img_5X5_r[1][0] <= img_5X5_r[1][1];    img_5X5_r[1][1] <= img_5X5_r[1][2];    img_5X5_r[1][2] <= img_5X5_r[1][3];    img_5X5_r[1][3] <= img_5X5_r[1][4];    img_5X5_r[1][4] <= img_5X32_r[1][deconv_col_r];
        // img_5X5_r[2][0] <= img_5X5_r[2][1];    img_5X5_r[2][1] <= img_5X5_r[2][2];    img_5X5_r[2][2] <= img_5X5_r[2][3];    img_5X5_r[2][3] <= img_5X5_r[2][4];    img_5X5_r[2][4] <= img_5X32_r[2][deconv_col_r];
        // img_5X5_r[3][0] <= img_5X5_r[3][1];    img_5X5_r[3][1] <= img_5X5_r[3][2];    img_5X5_r[3][2] <= img_5X5_r[3][3];    img_5X5_r[3][3] <= img_5X5_r[3][4];    img_5X5_r[3][4] <= img_5X32_r[3][deconv_col_r];
        // img_5X5_r[4][0] <= img_5X5_r[4][1];    img_5X5_r[4][1] <= img_5X5_r[4][2];    img_5X5_r[4][2] <= img_5X5_r[4][3];    img_5X5_r[4][3] <= img_5X5_r[4][4];    img_5X5_r[4][4] <= img_5X32_r[4][deconv_col_r];
        img_5X5_r[0][0] <= img_5X5_r[0][1];    img_5X5_r[0][1] <= img_5X5_r[0][2];    img_5X5_r[0][2] <= img_5X5_r[0][3];    img_5X5_r[0][3] <= img_5X5_r[0][4];    img_5X5_r[0][4] <= img_5X40_pad[0][deconv_col_r + 4];
        img_5X5_r[1][0] <= img_5X5_r[1][1];    img_5X5_r[1][1] <= img_5X5_r[1][2];    img_5X5_r[1][2] <= img_5X5_r[1][3];    img_5X5_r[1][3] <= img_5X5_r[1][4];    img_5X5_r[1][4] <= img_5X40_pad[1][deconv_col_r + 4];
        img_5X5_r[2][0] <= img_5X5_r[2][1];    img_5X5_r[2][1] <= img_5X5_r[2][2];    img_5X5_r[2][2] <= img_5X5_r[2][3];    img_5X5_r[2][3] <= img_5X5_r[2][4];    img_5X5_r[2][4] <= img_5X40_pad[2][deconv_col_r + 4];
        img_5X5_r[3][0] <= img_5X5_r[3][1];    img_5X5_r[3][1] <= img_5X5_r[3][2];    img_5X5_r[3][2] <= img_5X5_r[3][3];    img_5X5_r[3][3] <= img_5X5_r[3][4];    img_5X5_r[3][4] <= img_5X40_pad[3][deconv_col_r + 4];
        img_5X5_r[4][0] <= img_5X5_r[4][1];    img_5X5_r[4][1] <= img_5X5_r[4][2];    img_5X5_r[4][2] <= img_5X5_r[4][3];    img_5X5_r[4][3] <= img_5X5_r[4][4];    img_5X5_r[4][4] <= img_5X40_pad[4][deconv_col_r + 4];
    end
    else if (deconv_cnt_r > 7 && out_cnt_20_r == 19) begin // Col shift right (circular stage)
        // img_5X5_r[0][0] <= img_5X5_r[0][1];    img_5X5_r[0][1] <= img_5X5_r[0][2];    img_5X5_r[0][2] <= img_5X5_r[0][3];    img_5X5_r[0][3] <= img_5X5_r[0][4];    img_5X5_r[0][4] <= img_5X32_r[0][deconv_col_r];
        // img_5X5_r[1][0] <= img_5X5_r[1][1];    img_5X5_r[1][1] <= img_5X5_r[1][2];    img_5X5_r[1][2] <= img_5X5_r[1][3];    img_5X5_r[1][3] <= img_5X5_r[1][4];    img_5X5_r[1][4] <= img_5X32_r[1][deconv_col_r];
        // img_5X5_r[2][0] <= img_5X5_r[2][1];    img_5X5_r[2][1] <= img_5X5_r[2][2];    img_5X5_r[2][2] <= img_5X5_r[2][3];    img_5X5_r[2][3] <= img_5X5_r[2][4];    img_5X5_r[2][4] <= img_5X32_r[2][deconv_col_r];
        // img_5X5_r[3][0] <= img_5X5_r[3][1];    img_5X5_r[3][1] <= img_5X5_r[3][2];    img_5X5_r[3][2] <= img_5X5_r[3][3];    img_5X5_r[3][3] <= img_5X5_r[3][4];    img_5X5_r[3][4] <= img_5X32_r[3][deconv_col_r];
        // img_5X5_r[4][0] <= img_5X5_r[4][1];    img_5X5_r[4][1] <= img_5X5_r[4][2];    img_5X5_r[4][2] <= img_5X5_r[4][3];    img_5X5_r[4][3] <= img_5X5_r[4][4];    img_5X5_r[4][4] <= img_5X32_r[4][deconv_col_r];
        img_5X5_r[0][0] <= img_5X5_r[0][1];    img_5X5_r[0][1] <= img_5X5_r[0][2];    img_5X5_r[0][2] <= img_5X5_r[0][3];    img_5X5_r[0][3] <= img_5X5_r[0][4];    img_5X5_r[0][4] <= img_5X40_pad[0][deconv_col_r + 4];
        img_5X5_r[1][0] <= img_5X5_r[1][1];    img_5X5_r[1][1] <= img_5X5_r[1][2];    img_5X5_r[1][2] <= img_5X5_r[1][3];    img_5X5_r[1][3] <= img_5X5_r[1][4];    img_5X5_r[1][4] <= img_5X40_pad[1][deconv_col_r + 4];
        img_5X5_r[2][0] <= img_5X5_r[2][1];    img_5X5_r[2][1] <= img_5X5_r[2][2];    img_5X5_r[2][2] <= img_5X5_r[2][3];    img_5X5_r[2][3] <= img_5X5_r[2][4];    img_5X5_r[2][4] <= img_5X40_pad[2][deconv_col_r + 4];
        img_5X5_r[3][0] <= img_5X5_r[3][1];    img_5X5_r[3][1] <= img_5X5_r[3][2];    img_5X5_r[3][2] <= img_5X5_r[3][3];    img_5X5_r[3][3] <= img_5X5_r[3][4];    img_5X5_r[3][4] <= img_5X40_pad[3][deconv_col_r + 4];
        img_5X5_r[4][0] <= img_5X5_r[4][1];    img_5X5_r[4][1] <= img_5X5_r[4][2];    img_5X5_r[4][2] <= img_5X5_r[4][3];    img_5X5_r[4][3] <= img_5X5_r[4][4];    img_5X5_r[4][4] <= img_5X40_pad[4][deconv_col_r + 4];
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        img_5X5_r[0][0] <= img_5X5_r[0][0];    img_5X5_r[0][1] <= img_5X5_r[0][1];    img_5X5_r[0][2] <= img_5X5_r[0][2];    img_5X5_r[0][3] <= img_5X5_r[0][3];    img_5X5_r[0][4] <= img_5X5_r[0][4];
        img_5X5_r[1][0] <= img_5X5_r[1][0];    img_5X5_r[1][1] <= img_5X5_r[1][1];    img_5X5_r[1][2] <= img_5X5_r[1][2];    img_5X5_r[1][3] <= img_5X5_r[1][3];    img_5X5_r[1][4] <= img_5X5_r[1][4];
        img_5X5_r[2][0] <= img_5X5_r[2][0];    img_5X5_r[2][1] <= img_5X5_r[2][1];    img_5X5_r[2][2] <= img_5X5_r[2][2];    img_5X5_r[2][3] <= img_5X5_r[2][3];    img_5X5_r[2][4] <= img_5X5_r[2][4];
        img_5X5_r[3][0] <= img_5X5_r[3][0];    img_5X5_r[3][1] <= img_5X5_r[3][1];    img_5X5_r[3][2] <= img_5X5_r[3][2];    img_5X5_r[3][3] <= img_5X5_r[3][3];    img_5X5_r[3][4] <= img_5X5_r[3][4];
        img_5X5_r[4][0] <= img_5X5_r[4][0];    img_5X5_r[4][1] <= img_5X5_r[4][1];    img_5X5_r[4][2] <= img_5X5_r[4][2];    img_5X5_r[4][3] <= img_5X5_r[4][3];    img_5X5_r[4][4] <= img_5X5_r[4][4];
    end
    else begin
        img_5X5_r[0][0] <= 0;    img_5X5_r[0][1] <= 0;    img_5X5_r[0][2] <= 0;    img_5X5_r[0][3] <= 0;    img_5X5_r[0][4] <= 0;
        img_5X5_r[1][0] <= 0;    img_5X5_r[1][1] <= 0;    img_5X5_r[1][2] <= 0;    img_5X5_r[1][3] <= 0;    img_5X5_r[1][4] <= 0;
        img_5X5_r[2][0] <= 0;    img_5X5_r[2][1] <= 0;    img_5X5_r[2][2] <= 0;    img_5X5_r[2][3] <= 0;    img_5X5_r[2][4] <= 0;
        img_5X5_r[3][0] <= 0;    img_5X5_r[3][1] <= 0;    img_5X5_r[3][2] <= 0;    img_5X5_r[3][3] <= 0;    img_5X5_r[3][4] <= 0;
        img_5X5_r[4][0] <= 0;    img_5X5_r[4][1] <= 0;    img_5X5_r[4][2] <= 0;    img_5X5_r[4][3] <= 0;    img_5X5_r[4][4] <= 0;
    end
end

/* sum_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sum_r <= 0;
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        if (deconv_cnt_r <= 7) begin
            case(deconv_cnt_r)
                2: sum_r <= feat_add;
                3,4,5,6: sum_r <= sum_r + feat_add;
                default: sum_r <= sum_r;
            endcase
        end
        else begin
            case(out_cnt_20_r)
                14: sum_r <= feat_add;
                15,16,17,18: sum_r <= sum_r + feat_add;
                default: sum_r <= sum_r;
            endcase
        end
    end
    else
        sum_r <= 0;
end

/* deconv_result_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        deconv_result_r <= 0;
    else if (deconv_cnt_r == 7)
        deconv_result_r <= sum_r;
    else if (deconv_cnt_r > 7 && out_cnt_20_r == 19)
        deconv_result_r <= sum_r;
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32)
        deconv_result_r <= deconv_result_r;
    else
        deconv_result_r <= 0;
end

/* deconv_cnt_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        deconv_cnt_r <= 0;
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32)
        deconv_cnt_r <= deconv_cnt_r + 1'd1;
    else
        deconv_cnt_r <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        deconv_row_r <= 0;
    else if (cs == DECONV08 && deconv_col_r == 11 && out_cnt_20_r == 19)
        deconv_row_r <= deconv_row_r + 1;
    else if (cs == DECONV16 && deconv_col_r == 19 && out_cnt_20_r == 19)
        deconv_row_r <= deconv_row_r + 1;
    else if (cs == DECONV32 && deconv_col_r == 35 && out_cnt_20_r == 19)
        deconv_row_r <= deconv_row_r + 1;
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32)
        deconv_row_r <= deconv_row_r;
    else
        deconv_row_r <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        deconv_col_r <= 0;
    else if (deconv_cnt_r == 7)
        deconv_col_r <= deconv_col_r + 1;
    else if (cs == DECONV08) begin
        if (out_cnt_20_r == 19)
            deconv_col_r <= (deconv_col_r == 11) ? 0 : deconv_col_r + 1;
        else
            deconv_col_r <= deconv_col_r;
    end
    else if (cs == DECONV16) begin
        if (out_cnt_20_r == 19)
            deconv_col_r <= (deconv_col_r == 19) ? 0 : deconv_col_r + 1;
        else
            deconv_col_r <= deconv_col_r;
    end
    else if (cs == DECONV32) begin
        if (out_cnt_20_r == 19)
            deconv_col_r <= (deconv_col_r == 35) ? 0 : deconv_col_r + 1;
        else
            deconv_col_r <= deconv_col_r;
    end
    else
        deconv_col_r <= 0;
end


//---------------------------------------------------------------------
//   OUTPUT
//---------------------------------------------------------------------
/* pat_cnt_r*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pat_cnt_r <= 0;
    else if (cs == CONV08 && set_cnt_r == GOLDEN_PATNUM_CONV08 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else if (cs == CONV16 && set_cnt_r == GOLDEN_PATNUM_CONV16 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else if (cs == CONV32 && set_cnt_r == GOLDEN_PATNUM_CONV32 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else if (cs == DECONV08 && set_cnt_r == GOLDEN_PATNUM_DECONV08 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else if (cs == DECONV16 && set_cnt_r == GOLDEN_PATNUM_DECONV16 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else if (cs == DECONV32 && set_cnt_r == GOLDEN_PATNUM_DECONV32 && out_cnt_20_r == 19)
        pat_cnt_r <= pat_cnt_r + 1'd1;
    else
        pat_cnt_r <= pat_cnt_r;
end

/* set_cnt_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        set_cnt_r <= 0;
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32) begin
        if (out_cnt_20_r == 19)
            set_cnt_r <= set_cnt_r + 1'd1;
        else
            set_cnt_r <= set_cnt_r;
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        if (out_cnt_20_r == 19 && deconv_cnt_r > 28)
            set_cnt_r <= set_cnt_r + 1'd1;
        else
            set_cnt_r <= set_cnt_r;
    end
    else
        set_cnt_r <= 0;
end

/* out_cnt_20_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_cnt_20_r <= 0;
    else if (cs == CONV08 || cs == CONV16) begin
        if (conv_cnt_r >= 25)
            out_cnt_20_r <= (out_cnt_20_r == 19) ? 0 : out_cnt_20_r + 1'd1;
        else
            out_cnt_20_r <= 0;
    end
    else if (cs == CONV32) begin
        if (conv_cnt_r >= 35)
            out_cnt_20_r <= (out_cnt_20_r == 19) ? 0 : out_cnt_20_r + 1'd1;
        else
            out_cnt_20_r <= 0;
    end
    else if (cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        if (deconv_cnt_r >= 8)
            out_cnt_20_r <= (out_cnt_20_r == 19) ? 0 : out_cnt_20_r + 1'd1;
        else
            out_cnt_20_r <= 0;
    end
    else
        out_cnt_20_r <= 0;
end


always @(*) begin
    if ((cs == CONV08 || cs == CONV16) && conv_cnt_r >= 25) begin
        out_valid_temp = 1;
        case(out_cnt_20_r)
            0: out_value_temp = maxpool_r[0];
            1: out_value_temp = maxpool_r[1];
            2: out_value_temp = maxpool_r[2];
            3: out_value_temp = maxpool_r[3];
            4: out_value_temp = maxpool_r[4];
            5: out_value_temp = maxpool_r[5];
            6: out_value_temp = maxpool_r[6];
            7: out_value_temp = maxpool_r[7];
            8: out_value_temp = maxpool_r[8];
            9: out_value_temp = maxpool_r[9];
            10: out_value_temp = maxpool_r[10];
            11: out_value_temp = maxpool_r[11];
            12: out_value_temp = maxpool_r[12];
            13: out_value_temp = maxpool_r[13];
            14: out_value_temp = maxpool_r[14];
            15: out_value_temp = maxpool_r[15];
            16: out_value_temp = maxpool_r[16];
            17: out_value_temp = maxpool_r[17];
            18: out_value_temp = maxpool_r[18];
            19: out_value_temp = maxpool_r[19];
            default: out_value_temp = 0;
        endcase
    end
    else if (cs == CONV32 && conv_cnt_r >= 35) begin
        out_valid_temp = 1;
        case(out_cnt_20_r)
            0: out_value_temp = maxpool_r[0];
            1: out_value_temp = maxpool_r[1];
            2: out_value_temp = maxpool_r[2];
            3: out_value_temp = maxpool_r[3];
            4: out_value_temp = maxpool_r[4];
            5: out_value_temp = maxpool_r[5];
            6: out_value_temp = maxpool_r[6];
            7: out_value_temp = maxpool_r[7];
            8: out_value_temp = maxpool_r[8];
            9: out_value_temp = maxpool_r[9];
            10: out_value_temp = maxpool_r[10];
            11: out_value_temp = maxpool_r[11];
            12: out_value_temp = maxpool_r[12];
            13: out_value_temp = maxpool_r[13];
            14: out_value_temp = maxpool_r[14];
            15: out_value_temp = maxpool_r[15];
            16: out_value_temp = maxpool_r[16];
            17: out_value_temp = maxpool_r[17];
            18: out_value_temp = maxpool_r[18];
            19: out_value_temp = maxpool_r[19];
            default: out_value_temp = 0;
        endcase
    end
    else if ((cs == DECONV08 || cs == DECONV16 || cs == DECONV32) && deconv_cnt_r >= 28) begin
        out_valid_temp = 1;
        case(out_cnt_20_r)
            0: out_value_temp = deconv_result_r[0];
            1: out_value_temp = deconv_result_r[1];
            2: out_value_temp = deconv_result_r[2];
            3: out_value_temp = deconv_result_r[3];
            4: out_value_temp = deconv_result_r[4];
            5: out_value_temp = deconv_result_r[5];
            6: out_value_temp = deconv_result_r[6];
            7: out_value_temp = deconv_result_r[7];
            8: out_value_temp = deconv_result_r[8];
            9: out_value_temp = deconv_result_r[9];
            10: out_value_temp = deconv_result_r[10];
            11: out_value_temp = deconv_result_r[11];
            12: out_value_temp = deconv_result_r[12];
            13: out_value_temp = deconv_result_r[13];
            14: out_value_temp = deconv_result_r[14];
            15: out_value_temp = deconv_result_r[15];
            16: out_value_temp = deconv_result_r[16];
            17: out_value_temp = deconv_result_r[17];
            18: out_value_temp = deconv_result_r[18];
            19: out_value_temp = deconv_result_r[19];
            default: out_value_temp = 0;
        endcase
    end
    else begin
        out_valid_temp = 0;
        out_value_temp = 0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_valid <= 0;
        out_value <= 0;
    end
    else begin
        out_valid <= out_valid_temp;
        out_value <= out_value_temp;
    end
end

//---------------------------------------------------------------------
//   SRAM WRITE ADDRESS
//---------------------------------------------------------------------
/* img_waddr_r, ker_waddr_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_waddr_r <= 0;
        ker_waddr_r <= 0;
    end
    else if (cs == FETCH_IMG08) begin
        // &round <=> round == 7
        img_waddr_r <= (&round_r) ?
                      ((img_waddr_r == IMG_LAST_WORD08) ? 0 : img_waddr_r + 1'd1) : img_waddr_r;
        ker_waddr_r <= 0;
    end
    else if (cs == FETCH_IMG16) begin
        // &round <=> round == 7
        img_waddr_r <= (&round_r) ?
                      ((img_waddr_r == IMG_LAST_WORD16) ? 0 : img_waddr_r + 1'd1) : img_waddr_r;
        ker_waddr_r <= 0;
    end
    else if (cs == FETCH_IMG32) begin
        // &round <=> round == 7
        img_waddr_r <= (&round_r) ?
                      ((img_waddr_r == IMG_LAST_WORD32) ? 0 : img_waddr_r + 1'd1) : img_waddr_r;
        ker_waddr_r <= 0;
    end
    else if (cs == FETCH_KERNEL) begin
        img_waddr_r <= 0;
        ker_waddr_r <= (round_r == 4) ?
                      ((ker_waddr_r == KERNEL_LAST_WORD) ? 0 : ker_waddr_r + 1'd1) : ker_waddr_r;
    end
    else begin
        img_waddr_r <= 0;
        ker_waddr_r <= 0;
    end
end

/* round_r */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        round_r <= 0;
    end
    else if (cs == FETCH_IMG08 || cs == FETCH_IMG16 || cs == FETCH_IMG32) begin
        round_r <= round_r + 1'd1;
    end
    else if (cs == FETCH_KERNEL) begin
        round_r <= (round_r == 4) ? 0 : round_r + 1'd1;
    end
    else begin
        round_r <= 0;
    end
end


//---------------------------------------------------------------------
//   SRAM
//---------------------------------------------------------------------
/* ADDR_IMG */
always @(*) begin
    if (cs == FETCH_IMG08 || cs == FETCH_IMG16 || cs == FETCH_IMG32) begin
        ADDR_IMG = img_waddr_r;
        ADDR_KER = 0;
    end
    else if (cs == FETCH_KERNEL) begin
        ADDR_IMG = 0;
        ADDR_KER = ker_waddr_r;
    end
    else if (cs == CONV08 || cs == CONV16 || cs == CONV32 || cs == DECONV08 || cs == DECONV16 || cs == DECONV32) begin
        ADDR_IMG = img_raddr;
        ADDR_KER = ker_raddr;
    end
    else begin
        ADDR_IMG = 0;
        ADDR_KER = 0;
    end
end

/* sram_buffer */
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sram_buffer[0] <= 0;
        sram_buffer[1] <= 0;
        sram_buffer[2] <= 0;
        sram_buffer[3] <= 0;
        sram_buffer[4] <= 0;
        sram_buffer[5] <= 0;
        sram_buffer[6] <= 0;
        sram_buffer[7] <= 0;
    end
    else if (in_valid) begin
        sram_buffer[0] <= sram_buffer[1];
        sram_buffer[1] <= sram_buffer[2];
        sram_buffer[2] <= sram_buffer[3];
        sram_buffer[3] <= sram_buffer[4];
        sram_buffer[4] <= sram_buffer[5];
        sram_buffer[5] <= sram_buffer[6];
        sram_buffer[6] <= sram_buffer[7];
        sram_buffer[7] <= matrix;
    end
    else begin
        sram_buffer[0] <= 0;
        sram_buffer[1] <= 0;
        sram_buffer[2] <= 0;
        sram_buffer[3] <= 0;
        sram_buffer[4] <= 0;
        sram_buffer[5] <= 0;
        sram_buffer[6] <= 0;
        sram_buffer[7] <= 0;
    end
end

/* CS, WEB, OE */
always @(*) begin
    IMAGE_MEM_WRITE = (cs == FETCH_IMG08 | cs == FETCH_IMG16 | cs == FETCH_IMG32) & (&round_r);
    KERNEL_MEM_WRITE = (cs == FETCH_KERNEL) & (round_r == 4);

    if (IMAGE_MEM_WRITE) begin
        {CS_IMG, WEB_IMG, OE_IMG} = WRITE;
        {CS_KER, WEB_KER, OE_KER} = READ;
    end
    else if (KERNEL_MEM_WRITE) begin
        {CS_IMG, WEB_IMG, OE_IMG} = READ;
        {CS_KER, WEB_KER, OE_KER} = WRITE;
    end
    else begin
        {CS_IMG, WEB_IMG, OE_IMG} = READ;
        {CS_KER, WEB_KER, OE_KER} = READ;
    end
end

/* DI */
always @(*) begin
    DI_IMG = {sram_buffer[0],sram_buffer[1],sram_buffer[2],sram_buffer[3],sram_buffer[4],sram_buffer[5],sram_buffer[6],sram_buffer[7]};
    DI_KER = {                                             sram_buffer[3],sram_buffer[4],sram_buffer[5],sram_buffer[6],sram_buffer[7]};
end

IMAGE_MEM IMAGE_MEM (
    .CLK(clk), .WEB(WEB_IMG), .OE(OE_IMG), .CS(CS_IMG),
    .ADDR(ADDR_IMG),
    .DI(DI_IMG),
    .DO(DO_IMG)  );

KERNEL_MEM KERNEL_MEM (
    .CLK(clk), .WEB(WEB_KER), .OE(OE_KER), .CS(CS_KER),
    .ADDR(ADDR_KER),
    .DI(DI_KER),
    .DO(DO_KER)  );
endmodule


//---------------------------------------------------------------------
//   DOT PRODUCT SUBMODULE
//---------------------------------------------------------------------
module DP5 (
    A, B, C, D, E,
    F, G, H, I, J,
    dp5_out
);
input  signed   [7:0] A, B, C, D, E, F, G, H, I, J;
output signed  [20:0] dp5_out;
wire   signed  [20:0] MUL0, MUL1, MUL2, MUL3, MUL4;

assign MUL0 = A * B;
assign MUL1 = C * D;
assign MUL2 = E * F;
assign MUL3 = G * H;
assign MUL4 = I * J;
assign dp5_out = MUL0 + MUL1 + MUL2 + MUL3 + MUL4;
endmodule

//---------------------------------------------------------------------
//   SRAM SUBMODULE
//---------------------------------------------------------------------
module IMAGE_MEM (
    CLK, WEB, OE, CS,
    ADDR,
    DI,
    DO  );
input          CLK, WEB, OE, CS;
input  [63:0]  DI;
input  [10:0]  ADDR;
output [63:0]  DO;
IMG_2048X64 IMG_SRAM (
    .CK(CLK), .WEB(WEB), .OE(OE), .CS(CS),
    // ADDRs
    .A0(ADDR[0]), .A1(ADDR[1]), .A2(ADDR[2]), .A3(ADDR[3]), .A4(ADDR[4]), .A5(ADDR[5]), .A6(ADDR[6]), .A7(ADDR[7]), .A8(ADDR[8]), .A9(ADDR[9]), .A10(ADDR[10]),
    // DIs
    .DI0(DI[0]),  .DI1(DI[1]),  .DI2(DI[2]),  .DI3(DI[3]),  .DI4(DI[4]),  .DI5(DI[5]),  .DI6(DI[6]),  .DI7(DI[7]),  .DI8(DI[8]),  .DI9(DI[9]),
    .DI10(DI[10]),.DI11(DI[11]),.DI12(DI[12]),.DI13(DI[13]),.DI14(DI[14]),.DI15(DI[15]),.DI16(DI[16]),.DI17(DI[17]),.DI18(DI[18]),.DI19(DI[19]),
    .DI20(DI[20]),.DI21(DI[21]),.DI22(DI[22]),.DI23(DI[23]),.DI24(DI[24]),.DI25(DI[25]),.DI26(DI[26]),.DI27(DI[27]),.DI28(DI[28]),.DI29(DI[29]),
    .DI30(DI[30]),.DI31(DI[31]),.DI32(DI[32]),.DI33(DI[33]),.DI34(DI[34]),.DI35(DI[35]),.DI36(DI[36]),.DI37(DI[37]),.DI38(DI[38]),.DI39(DI[39]),
    .DI40(DI[40]),.DI41(DI[41]),.DI42(DI[42]),.DI43(DI[43]),.DI44(DI[44]),.DI45(DI[45]),.DI46(DI[46]),.DI47(DI[47]),.DI48(DI[48]),.DI49(DI[49]),
    .DI50(DI[50]),.DI51(DI[51]),.DI52(DI[52]),.DI53(DI[53]),.DI54(DI[54]),.DI55(DI[55]),.DI56(DI[56]),.DI57(DI[57]),.DI58(DI[58]),.DI59(DI[59]),
    .DI60(DI[60]),.DI61(DI[61]),.DI62(DI[62]),.DI63(DI[63]),
    // DOs
    .DO0(DO[0]),  .DO1(DO[1]),  .DO2(DO[2]),  .DO3(DO[3]),  .DO4(DO[4]),  .DO5(DO[5]),  .DO6(DO[6]),  .DO7(DO[7]),  .DO8(DO[8]),  .DO9(DO[9]),
    .DO10(DO[10]),.DO11(DO[11]),.DO12(DO[12]),.DO13(DO[13]),.DO14(DO[14]),.DO15(DO[15]),.DO16(DO[16]),.DO17(DO[17]),.DO18(DO[18]),.DO19(DO[19]),
    .DO20(DO[20]),.DO21(DO[21]),.DO22(DO[22]),.DO23(DO[23]),.DO24(DO[24]),.DO25(DO[25]),.DO26(DO[26]),.DO27(DO[27]),.DO28(DO[28]),.DO29(DO[29]),
    .DO30(DO[30]),.DO31(DO[31]),.DO32(DO[32]),.DO33(DO[33]),.DO34(DO[34]),.DO35(DO[35]),.DO36(DO[36]),.DO37(DO[37]),.DO38(DO[38]),.DO39(DO[39]),
    .DO40(DO[40]),.DO41(DO[41]),.DO42(DO[42]),.DO43(DO[43]),.DO44(DO[44]),.DO45(DO[45]),.DO46(DO[46]),.DO47(DO[47]),.DO48(DO[48]),.DO49(DO[49]),
    .DO50(DO[50]),.DO51(DO[51]),.DO52(DO[52]),.DO53(DO[53]),.DO54(DO[54]),.DO55(DO[55]),.DO56(DO[56]),.DO57(DO[57]),.DO58(DO[58]),.DO59(DO[59]),
    .DO60(DO[60]),.DO61(DO[61]),.DO62(DO[62]),.DO63(DO[63]) );
endmodule

module KERNEL_MEM (
    CLK, WEB, OE, CS,
    ADDR,
    DI,
    DO  );
input          CLK, WEB, OE, CS;
input  [39:0]  DI;
input  [ 6:0]  ADDR;
output [39:0]  DO;
KERNEL_80X40 KERNEL_SRAM (
    .CK(CLK), .WEB(WEB), .OE(OE), .CS(CS),
    // ADDRs
    .A0(ADDR[0]), .A1(ADDR[1]), .A2(ADDR[2]), .A3(ADDR[3]), .A4(ADDR[4]), .A5(ADDR[5]), .A6(ADDR[6]),
    // DIs
    .DI0(DI[0]),  .DI1(DI[1]),  .DI2(DI[2]),  .DI3(DI[3]),  .DI4(DI[4]),  .DI5(DI[5]),  .DI6(DI[6]),  .DI7(DI[7]),  .DI8(DI[8]),  .DI9(DI[9]),
    .DI10(DI[10]),.DI11(DI[11]),.DI12(DI[12]),.DI13(DI[13]),.DI14(DI[14]),.DI15(DI[15]),.DI16(DI[16]),.DI17(DI[17]),.DI18(DI[18]),.DI19(DI[19]),
    .DI20(DI[20]),.DI21(DI[21]),.DI22(DI[22]),.DI23(DI[23]),.DI24(DI[24]),.DI25(DI[25]),.DI26(DI[26]),.DI27(DI[27]),.DI28(DI[28]),.DI29(DI[29]),
    .DI30(DI[30]),.DI31(DI[31]),.DI32(DI[32]),.DI33(DI[33]),.DI34(DI[34]),.DI35(DI[35]),.DI36(DI[36]),.DI37(DI[37]),.DI38(DI[38]),.DI39(DI[39]),
    // DOs
    .DO0(DO[0]),  .DO1(DO[1]),  .DO2(DO[2]),  .DO3(DO[3]),  .DO4(DO[4]),  .DO5(DO[5]),  .DO6(DO[6]),  .DO7(DO[7]),  .DO8(DO[8]),  .DO9(DO[9]),
    .DO10(DO[10]),.DO11(DO[11]),.DO12(DO[12]),.DO13(DO[13]),.DO14(DO[14]),.DO15(DO[15]),.DO16(DO[16]),.DO17(DO[17]),.DO18(DO[18]),.DO19(DO[19]),
    .DO20(DO[20]),.DO21(DO[21]),.DO22(DO[22]),.DO23(DO[23]),.DO24(DO[24]),.DO25(DO[25]),.DO26(DO[26]),.DO27(DO[27]),.DO28(DO[28]),.DO29(DO[29]),
    .DO30(DO[30]),.DO31(DO[31]),.DO32(DO[32]),.DO33(DO[33]),.DO34(DO[34]),.DO35(DO[35]),.DO36(DO[36]),.DO37(DO[37]),.DO38(DO[38]),.DO39(DO[39])  );
endmodule