//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    (C) Copyright System Integration and Silicon Implementation Laboratory
//    All Right Reserved
//		Date		: 2023/10
//		Version		: v1.0
//   	File Name   : HT_TOP.v
//   	Module Name : HT_TOP
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

//synopsys translate_off
`include "SORT_IP.v"
//synopsys translate_on

module HT_TOP(
    // Input signals
    clk,
    rst_n,
    in_valid,
    in_weight,
    out_mode,
    // Output signals
    out_valid,
    out_code
);

//================================================================
// INPUT & OUTPUT DECLARATION
//================================================================
input       clk, rst_n, in_valid, out_mode;
input [2:0] in_weight;

output reg  out_valid, out_code;

//================================================================
// PARAMETER
//================================================================
// FSM
`define STATE_BITS 5
parameter IDLE   = `STATE_BITS'd0;
parameter FETCH  = `STATE_BITS'd1;
parameter MERGE6 = `STATE_BITS'd3;
parameter MERGE5 = `STATE_BITS'd4;
parameter MERGE4 = `STATE_BITS'd5;
parameter MERGE3 = `STATE_BITS'd6;
parameter MERGE2 = `STATE_BITS'd7;
parameter MERGE1 = `STATE_BITS'd8;
parameter MERGE0 = `STATE_BITS'd9;
parameter OUT_ILOVE    = `STATE_BITS'd10;
parameter OUT_ICLAB    = `STATE_BITS'd11;

// CHARACTERS
parameter MAX_WEIGHT = 5'd31;
parameter MAX = 4'd15;
parameter A = 4'd14;
parameter B = 4'd13;
parameter C = 4'd12;
parameter E = 4'd11;
parameter I = 4'd10;
parameter L = 4'd9;
parameter O = 4'd8;
parameter V = 4'd7;
parameter SUBTREE6 = 4'd6;
parameter SUBTREE5 = 4'd5;
parameter SUBTREE4 = 4'd4;
parameter SUBTREE3 = 4'd3;
parameter SUBTREE2 = 4'd2;
parameter SUBTREE1 = 4'd1;
parameter SUBTREE0 = 4'd0;
integer i, j;
//================================================================
// REG & WIRE DECLARATION
//================================================================
// FSM
reg   [`STATE_BITS-1:0]  cs, ns;

// SYSTEM
reg          out_mode_r;

// SORT_IP
reg   [3:0]  IN_character_r     [0:7];
reg   [4:0]  IN_weight_r        [0:7];
reg   [3:0]  IN_character_next  [0:7];
reg   [4:0]  IN_weight_next     [0:7];
wire [31:0]  OUT_character;

// COUNTER
reg   [2:0]  cnt_8_r;
reg   [2:0]  bit_cnt_r, alpha_cnt_r;

// TABLES
reg   [4:0]  weight_table        [15:0];
reg   [7:0]  huff_relation       [14:0];
reg   [2:0]  huff_depth          [7:0];
reg   [7:0]  huff_code           [7:0];
reg   [7:0]  new_relation;
reg   [4:0]  new_weight;

// OUTPUT
reg   [2:0]  current_depth;
reg   [7:0]  current_code;
reg   [19:0] ILOVE, ICLAB;

//================================================================
// FSM
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs <= IDLE;
    else        cs <= ns;
end
always @(*) begin
    case(cs)
        IDLE:   ns = (in_valid) ? FETCH : IDLE;
        FETCH:  ns = (&cnt_8_r) ? MERGE6 : FETCH;
        MERGE6: ns = MERGE5;
        MERGE5: ns = MERGE4;
        MERGE4: ns = MERGE3;
        MERGE3: ns = MERGE2;
        MERGE2: ns = MERGE1;
        MERGE1: ns = MERGE0;
        MERGE0: ns = (out_mode_r) ? OUT_ICLAB : OUT_ILOVE;
        OUT_ILOVE:    ns = (alpha_cnt_r == 4 && bit_cnt_r == current_depth - 1) ? IDLE : OUT_ILOVE;
        OUT_ICLAB:    ns = (alpha_cnt_r == 4 && bit_cnt_r == current_depth - 1) ? IDLE : OUT_ICLAB;
        default: ns = IDLE;
    endcase
end

//================================================================
// MAIN INFO
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_mode_r <= 0;
    else if (cs == IDLE && in_valid)
        out_mode_r <= out_mode;
    else
        out_mode_r <= out_mode_r;
end

//================================================================
// HUFFMAN TABLES
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        weight_table[MAX] <= MAX_WEIGHT;   weight_table[V]        <= 0;
        weight_table[A]   <= 0;            weight_table[SUBTREE6] <= 0;
        weight_table[B]   <= 0;            weight_table[SUBTREE5] <= 0;
        weight_table[C]   <= 0;            weight_table[SUBTREE4] <= 0;
        weight_table[E]   <= 0;            weight_table[SUBTREE3] <= 0;
        weight_table[I]   <= 0;            weight_table[SUBTREE2] <= 0;
        weight_table[L]   <= 0;            weight_table[SUBTREE1] <= 0;
        weight_table[O]   <= 0;            weight_table[SUBTREE0] <= 0;
    end
    else if (in_valid) begin
        case(cnt_8_r)
            0: weight_table[A] <= in_weight;
            1: weight_table[B] <= in_weight;
            2: weight_table[C] <= in_weight;
            3: weight_table[E] <= in_weight;
            4: weight_table[I] <= in_weight;
            5: weight_table[L] <= in_weight;
            6: weight_table[O] <= in_weight;
            7: weight_table[V] <= in_weight;
            default: begin
                weight_table[MAX] <= weight_table[MAX];    weight_table[V]        <= weight_table[V]       ;
                weight_table[A]   <= weight_table[A]  ;    weight_table[SUBTREE6] <= weight_table[SUBTREE6];
                weight_table[B]   <= weight_table[B]  ;    weight_table[SUBTREE5] <= weight_table[SUBTREE5];
                weight_table[C]   <= weight_table[C]  ;    weight_table[SUBTREE4] <= weight_table[SUBTREE4];
                weight_table[E]   <= weight_table[E]  ;    weight_table[SUBTREE3] <= weight_table[SUBTREE3];
                weight_table[I]   <= weight_table[I]  ;    weight_table[SUBTREE2] <= weight_table[SUBTREE2];
                weight_table[L]   <= weight_table[L]  ;    weight_table[SUBTREE1] <= weight_table[SUBTREE1];
                weight_table[O]   <= weight_table[O]  ;    weight_table[SUBTREE0] <= weight_table[SUBTREE0];
            end
        endcase
    end
    else if (cs == MERGE6) begin
        weight_table[SUBTREE6] <= new_weight;
    end
    else if (cs == MERGE5) begin
        weight_table[SUBTREE5] <= new_weight;
    end
    else if (cs == MERGE4) begin
        weight_table[SUBTREE4] <= new_weight;
    end
    else if (cs == MERGE3) begin
        weight_table[SUBTREE3] <= new_weight;
    end
    else if (cs == MERGE2) begin
        weight_table[SUBTREE2] <= new_weight;
    end
    else if (cs == MERGE1) begin
        weight_table[SUBTREE1] <= new_weight;
    end
    else if (cs == MERGE0) begin
        weight_table[SUBTREE0] <= new_weight;
    end
    else if (cs == IDLE) begin
        weight_table[MAX] <= MAX_WEIGHT;   weight_table[V]        <= 0;
        weight_table[A]   <= 0;            weight_table[SUBTREE6] <= 0;
        weight_table[B]   <= 0;            weight_table[SUBTREE5] <= 0;
        weight_table[C]   <= 0;            weight_table[SUBTREE4] <= 0;
        weight_table[E]   <= 0;            weight_table[SUBTREE3] <= 0;
        weight_table[I]   <= 0;            weight_table[SUBTREE2] <= 0;
        weight_table[L]   <= 0;            weight_table[SUBTREE1] <= 0;
        weight_table[O]   <= 0;            weight_table[SUBTREE0] <= 0;
    end
    else begin
        weight_table[MAX] <= weight_table[MAX];    weight_table[V]        <= weight_table[V]       ;
        weight_table[A]   <= weight_table[A]  ;    weight_table[SUBTREE6] <= weight_table[SUBTREE6];
        weight_table[B]   <= weight_table[B]  ;    weight_table[SUBTREE5] <= weight_table[SUBTREE5];
        weight_table[C]   <= weight_table[C]  ;    weight_table[SUBTREE4] <= weight_table[SUBTREE4];
        weight_table[E]   <= weight_table[E]  ;    weight_table[SUBTREE3] <= weight_table[SUBTREE3];
        weight_table[I]   <= weight_table[I]  ;    weight_table[SUBTREE2] <= weight_table[SUBTREE2];
        weight_table[L]   <= weight_table[L]  ;    weight_table[SUBTREE1] <= weight_table[SUBTREE1];
        weight_table[O]   <= weight_table[O]  ;    weight_table[SUBTREE0] <= weight_table[SUBTREE0];
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        huff_relation[A] <= 8'b10000000;    huff_relation[SUBTREE6] <= 8'd0;
        huff_relation[B] <= 8'b01000000;    huff_relation[SUBTREE5] <= 8'd0;
        huff_relation[C] <= 8'b00100000;    huff_relation[SUBTREE4] <= 8'd0;
        huff_relation[E] <= 8'b00010000;    huff_relation[SUBTREE3] <= 8'd0;
        huff_relation[I] <= 8'b00001000;    huff_relation[SUBTREE2] <= 8'd0;
        huff_relation[L] <= 8'b00000100;    huff_relation[SUBTREE1] <= 8'd0;
        huff_relation[O] <= 8'b00000010;    huff_relation[SUBTREE0] <= 8'd0;
        huff_relation[V] <= 8'b00000001;
    end
    else if (cs == MERGE6) begin
        huff_relation[SUBTREE6] <= new_relation;
    end
    else if (cs == MERGE5) begin
        huff_relation[SUBTREE5] <= new_relation;
    end
    else if (cs == MERGE4) begin
        huff_relation[SUBTREE4] <= new_relation;
    end
    else if (cs == MERGE3) begin
        huff_relation[SUBTREE3] <= new_relation;
    end
    else if (cs == MERGE2) begin
        huff_relation[SUBTREE2] <= new_relation;
    end
    else if (cs == MERGE1) begin
        huff_relation[SUBTREE1] <= new_relation;
    end
    else if (cs == MERGE0) begin
        huff_relation[SUBTREE0] <= new_relation;
    end
    else if (cs == IDLE) begin
        huff_relation[A] <= 8'b10000000;    huff_relation[SUBTREE6] <= 8'd0;
        huff_relation[B] <= 8'b01000000;    huff_relation[SUBTREE5] <= 8'd0;
        huff_relation[C] <= 8'b00100000;    huff_relation[SUBTREE4] <= 8'd0;
        huff_relation[E] <= 8'b00010000;    huff_relation[SUBTREE3] <= 8'd0;
        huff_relation[I] <= 8'b00001000;    huff_relation[SUBTREE2] <= 8'd0;
        huff_relation[L] <= 8'b00000100;    huff_relation[SUBTREE1] <= 8'd0;
        huff_relation[O] <= 8'b00000010;    huff_relation[SUBTREE0] <= 8'd0;
        huff_relation[V] <= 8'b00000001;
    end
    else begin
        huff_relation[A] <= huff_relation[A];    huff_relation[SUBTREE6] <= huff_relation[SUBTREE6];
        huff_relation[B] <= huff_relation[B];    huff_relation[SUBTREE5] <= huff_relation[SUBTREE5];
        huff_relation[C] <= huff_relation[C];    huff_relation[SUBTREE4] <= huff_relation[SUBTREE4];
        huff_relation[E] <= huff_relation[E];    huff_relation[SUBTREE3] <= huff_relation[SUBTREE3];
        huff_relation[I] <= huff_relation[I];    huff_relation[SUBTREE2] <= huff_relation[SUBTREE2];
        huff_relation[L] <= huff_relation[L];    huff_relation[SUBTREE1] <= huff_relation[SUBTREE1];
        huff_relation[O] <= huff_relation[O];    huff_relation[SUBTREE0] <= huff_relation[SUBTREE0];
        huff_relation[V] <= huff_relation[V];
    end
end

always @(*) begin
    // Operation of the last two characters in OUT_character
    new_weight   = weight_table [OUT_character[7:4]] + weight_table [OUT_character[3:0]];
    new_relation = huff_relation[OUT_character[7:4]] | huff_relation[OUT_character[3:0]];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        huff_depth[7] <= 0; // A
        huff_depth[6] <= 0; // B
        huff_depth[5] <= 0; // C
        huff_depth[4] <= 0; // E
        huff_depth[3] <= 0; // I
        huff_depth[2] <= 0; // L
        huff_depth[1] <= 0; // O
        huff_depth[0] <= 0; // V
    end
    // Depth plus one if last two characters are included
    else if (cs == MERGE6 || cs == MERGE5 || cs == MERGE4 || cs == MERGE3 || cs == MERGE2 || cs == MERGE1 || cs == MERGE0) begin
        huff_depth[7] <= (new_relation[7]) ? huff_depth[7] + 1 : huff_depth[7];
        huff_depth[6] <= (new_relation[6]) ? huff_depth[6] + 1 : huff_depth[6];
        huff_depth[5] <= (new_relation[5]) ? huff_depth[5] + 1 : huff_depth[5];
        huff_depth[4] <= (new_relation[4]) ? huff_depth[4] + 1 : huff_depth[4];
        huff_depth[3] <= (new_relation[3]) ? huff_depth[3] + 1 : huff_depth[3];
        huff_depth[2] <= (new_relation[2]) ? huff_depth[2] + 1 : huff_depth[2];
        huff_depth[1] <= (new_relation[1]) ? huff_depth[1] + 1 : huff_depth[1];
        huff_depth[0] <= (new_relation[0]) ? huff_depth[0] + 1 : huff_depth[0];
    end
    else if (cs == IDLE) begin
        huff_depth[7] <= 0; // A
        huff_depth[6] <= 0; // B
        huff_depth[5] <= 0; // C
        huff_depth[4] <= 0; // E
        huff_depth[3] <= 0; // I
        huff_depth[2] <= 0; // L
        huff_depth[1] <= 0; // O
        huff_depth[0] <= 0; // V
    end
    else begin
        huff_depth[7] <= huff_depth[7];
        huff_depth[6] <= huff_depth[6];
        huff_depth[5] <= huff_depth[5];
        huff_depth[4] <= huff_depth[4];
        huff_depth[3] <= huff_depth[3];
        huff_depth[2] <= huff_depth[2];
        huff_depth[1] <= huff_depth[1];
        huff_depth[0] <= huff_depth[0];
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        huff_code[7] <= 0;
        huff_code[6] <= 0;
        huff_code[5] <= 0;
        huff_code[4] <= 0;
        huff_code[3] <= 0;
        huff_code[2] <= 0;
        huff_code[1] <= 0;
        huff_code[0] <= 0;
    end
    else if (cs == MERGE6 || cs == MERGE5 || cs == MERGE4 || cs == MERGE3 || cs == MERGE2 || cs == MERGE1 || cs == MERGE0) begin
        huff_code[7] <= (huff_relation[OUT_character[7:4]][7]) ? {huff_code[7][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][7]) ? {huff_code[7][6:0], 1'b1} : huff_code[7];
        huff_code[6] <= (huff_relation[OUT_character[7:4]][6]) ? {huff_code[6][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][6]) ? {huff_code[6][6:0], 1'b1} : huff_code[6];
        huff_code[5] <= (huff_relation[OUT_character[7:4]][5]) ? {huff_code[5][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][5]) ? {huff_code[5][6:0], 1'b1} : huff_code[5];
        huff_code[4] <= (huff_relation[OUT_character[7:4]][4]) ? {huff_code[4][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][4]) ? {huff_code[4][6:0], 1'b1} : huff_code[4];
        huff_code[3] <= (huff_relation[OUT_character[7:4]][3]) ? {huff_code[3][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][3]) ? {huff_code[3][6:0], 1'b1} : huff_code[3];
        huff_code[2] <= (huff_relation[OUT_character[7:4]][2]) ? {huff_code[2][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][2]) ? {huff_code[2][6:0], 1'b1} : huff_code[2];
        huff_code[1] <= (huff_relation[OUT_character[7:4]][1]) ? {huff_code[1][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][1]) ? {huff_code[1][6:0], 1'b1} : huff_code[1];
        huff_code[0] <= (huff_relation[OUT_character[7:4]][0]) ? {huff_code[0][6:0], 1'b0} :
                        (huff_relation[OUT_character[3:0]][0]) ? {huff_code[0][6:0], 1'b1} : huff_code[0];
    end
    else if (cs == IDLE) begin
        huff_code[7] <= 0;
        huff_code[6] <= 0;
        huff_code[5] <= 0;
        huff_code[4] <= 0;
        huff_code[3] <= 0;
        huff_code[2] <= 0;
        huff_code[1] <= 0;
        huff_code[0] <= 0;
    end
    else begin
        huff_code[7] <= huff_code[7];
        huff_code[6] <= huff_code[6];
        huff_code[5] <= huff_code[5];
        huff_code[4] <= huff_code[4];
        huff_code[3] <= huff_code[3];
        huff_code[2] <= huff_code[2];
        huff_code[1] <= huff_code[1];
        huff_code[0] <= huff_code[0];
    end
end

//================================================================
// SORT_IP
//================================================================
SORT_IP #(.IP_WIDTH(8))
SORT_IP (
    .IN_character({IN_character_r[0], IN_character_r[1], IN_character_r[2], IN_character_r[3], IN_character_r[4], IN_character_r[5], IN_character_r[6], IN_character_r[7]}),
    .IN_weight({IN_weight_r[0], IN_weight_r[1], IN_weight_r[2], IN_weight_r[3], IN_weight_r[4], IN_weight_r[5], IN_weight_r[6], IN_weight_r[7]}),
    .OUT_character(OUT_character)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        IN_character_r[0] <= 0;    IN_weight_r[0] <= 0;
        IN_character_r[1] <= 0;    IN_weight_r[1] <= 0;
        IN_character_r[2] <= 0;    IN_weight_r[2] <= 0;
        IN_character_r[3] <= 0;    IN_weight_r[3] <= 0;
        IN_character_r[4] <= 0;    IN_weight_r[4] <= 0;
        IN_character_r[5] <= 0;    IN_weight_r[5] <= 0;
        IN_character_r[6] <= 0;    IN_weight_r[6] <= 0;
        IN_character_r[7] <= 0;    IN_weight_r[7] <= 0;
    end
    else if (in_valid) begin
        case(cnt_8_r)
            0: begin IN_character_r[0] <= A;    IN_weight_r[0] <= in_weight; end
            1: begin IN_character_r[1] <= B;    IN_weight_r[1] <= in_weight; end
            2: begin IN_character_r[2] <= C;    IN_weight_r[2] <= in_weight; end
            3: begin IN_character_r[3] <= E;    IN_weight_r[3] <= in_weight; end
            4: begin IN_character_r[4] <= I;    IN_weight_r[4] <= in_weight; end
            5: begin IN_character_r[5] <= L;    IN_weight_r[5] <= in_weight; end
            6: begin IN_character_r[6] <= O;    IN_weight_r[6] <= in_weight; end
            7: begin IN_character_r[7] <= V;    IN_weight_r[7] <= in_weight; end
        endcase
    end
    else if (cs == MERGE6 || cs == MERGE5 || cs == MERGE4 || cs == MERGE3 || cs == MERGE2 || cs == MERGE1 || cs == MERGE0) begin
        IN_character_r[0] <= IN_character_next[0];    IN_weight_r[0] <= IN_weight_next[0];
        IN_character_r[1] <= IN_character_next[1];    IN_weight_r[1] <= IN_weight_next[1];
        IN_character_r[2] <= IN_character_next[2];    IN_weight_r[2] <= IN_weight_next[2];
        IN_character_r[3] <= IN_character_next[3];    IN_weight_r[3] <= IN_weight_next[3];
        IN_character_r[4] <= IN_character_next[4];    IN_weight_r[4] <= IN_weight_next[4];
        IN_character_r[5] <= IN_character_next[5];    IN_weight_r[5] <= IN_weight_next[5];
        IN_character_r[6] <= IN_character_next[6];    IN_weight_r[6] <= IN_weight_next[6];
        IN_character_r[7] <= IN_character_next[7];    IN_weight_r[7] <= IN_weight_next[7];
    end
    else if (cs == IDLE) begin
        IN_character_r[0] <= 0;    IN_weight_r[0] <= 0;
        IN_character_r[1] <= 0;    IN_weight_r[1] <= 0;
        IN_character_r[2] <= 0;    IN_weight_r[2] <= 0;
        IN_character_r[3] <= 0;    IN_weight_r[3] <= 0;
        IN_character_r[4] <= 0;    IN_weight_r[4] <= 0;
        IN_character_r[5] <= 0;    IN_weight_r[5] <= 0;
        IN_character_r[6] <= 0;    IN_weight_r[6] <= 0;
        IN_character_r[7] <= 0;    IN_weight_r[7] <= 0;
    end
    else begin
        IN_character_r[0] <= IN_character_r[0];    IN_weight_r[0] <= IN_weight_r[0];
        IN_character_r[1] <= IN_character_r[1];    IN_weight_r[1] <= IN_weight_r[1];
        IN_character_r[2] <= IN_character_r[2];    IN_weight_r[2] <= IN_weight_r[2];
        IN_character_r[3] <= IN_character_r[3];    IN_weight_r[3] <= IN_weight_r[3];
        IN_character_r[4] <= IN_character_r[4];    IN_weight_r[4] <= IN_weight_r[4];
        IN_character_r[5] <= IN_character_r[5];    IN_weight_r[5] <= IN_weight_r[5];
        IN_character_r[6] <= IN_character_r[6];    IN_weight_r[6] <= IN_weight_r[6];
        IN_character_r[7] <= IN_character_r[7];    IN_weight_r[7] <= IN_weight_r[7];
    end
end

always @(*) begin
    IN_character_next[0] = MAX;
    IN_character_next[1] = OUT_character[31:28];
    IN_character_next[2] = OUT_character[27:24];
    IN_character_next[3] = OUT_character[23:20];
    IN_character_next[4] = OUT_character[19:16];
    IN_character_next[5] = OUT_character[15:12];
    IN_character_next[6] = OUT_character[11: 8];
    IN_character_next[7] = (cs == MERGE6) ? SUBTREE6 :
                           (cs == MERGE5) ? SUBTREE5 :
                           (cs == MERGE4) ? SUBTREE4 :
                           (cs == MERGE3) ? SUBTREE3 :
                           (cs == MERGE2) ? SUBTREE2 :
                           (cs == MERGE1) ? SUBTREE1 :
                           (cs == MERGE0) ? SUBTREE0 : 0;

    IN_weight_next[0] = MAX_WEIGHT;
    IN_weight_next[1] = weight_table[OUT_character[31:28]];
    IN_weight_next[2] = weight_table[OUT_character[27:24]];
    IN_weight_next[3] = weight_table[OUT_character[23:20]];
    IN_weight_next[4] = weight_table[OUT_character[19:16]];
    IN_weight_next[5] = weight_table[OUT_character[15:12]];
    IN_weight_next[6] = weight_table[OUT_character[11: 8]];
    IN_weight_next[7] = weight_table[OUT_character[ 7: 4]] + weight_table[OUT_character[ 3: 0]];
end

//================================================================
// COUNTERS
//================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_8_r <= 0;
    else if (in_valid)
        cnt_8_r <= cnt_8_r + 1;
    else
        cnt_8_r <= 0;
end



always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        alpha_cnt_r <= 0;
    else if (cs == OUT_ILOVE || cs == OUT_ICLAB)
        alpha_cnt_r <= (bit_cnt_r == current_depth - 1) ? alpha_cnt_r + 1 : alpha_cnt_r;
    else
        alpha_cnt_r <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bit_cnt_r <= 0;
    else if (cs == OUT_ILOVE || cs == OUT_ICLAB)
        bit_cnt_r <= (bit_cnt_r == current_depth - 1) ? 0 : bit_cnt_r + 1;
    else
        bit_cnt_r <= 0;
end

always @(*) begin
    if (cs == OUT_ILOVE) begin
        case(alpha_cnt_r)
            0: begin current_depth = huff_depth[3];    current_code = huff_code[3]; end // I
            1: begin current_depth = huff_depth[2];    current_code = huff_code[2]; end // L
            2: begin current_depth = huff_depth[1];    current_code = huff_code[1]; end // O
            3: begin current_depth = huff_depth[0];    current_code = huff_code[0]; end // V
            4: begin current_depth = huff_depth[4];    current_code = huff_code[4]; end // E
            default: begin current_depth = 0; current_code = 0; end
        endcase
    end
    else if (cs == OUT_ICLAB) begin
        case(alpha_cnt_r)
            0: begin current_depth = huff_depth[3];    current_code = huff_code[3]; end // I
            1: begin current_depth = huff_depth[5];    current_code = huff_code[5]; end // C
            2: begin current_depth = huff_depth[2];    current_code = huff_code[2]; end // L
            3: begin current_depth = huff_depth[7];    current_code = huff_code[7]; end // A
            4: begin current_depth = huff_depth[6];    current_code = huff_code[6]; end // B
            default: begin current_depth = 0; current_code = 0; end
        endcase
    end
    else begin
        current_depth = 0;
        current_code  = 0;
    end
end
//================================================================
// OUTPUT
//================================================================
always @(*) begin
    if (cs == OUT_ILOVE || cs == OUT_ICLAB) begin
        out_valid = 1;
        out_code  = current_code[bit_cnt_r];
    end
    else begin
        out_valid = 0;
        out_code  = 0;
    end
end

endmodule