//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Lab01 Exercise		: Supper MOSFET Calculator
//   Author     		: Lin-Hung Lai (lhlai@ieee.org)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : SMC.v
//   Module Name : SMC
//   Release version : V1.0 (Release Date: 2023-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################


module SMC(
  // Input signals
    mode,
    W_0, V_GS_0, V_DS_0,
    W_1, V_GS_1, V_DS_1,
    W_2, V_GS_2, V_DS_2,
    W_3, V_GS_3, V_DS_3,
    W_4, V_GS_4, V_DS_4,
    W_5, V_GS_5, V_DS_5,
  // Output signals
    out_n
);

//================================================================
//   INPUT AND OUTPUT DECLARATION
//================================================================
input [2:0] W_0, V_GS_0, V_DS_0;
input [2:0] W_1, V_GS_1, V_DS_1;
input [2:0] W_2, V_GS_2, V_DS_2;
input [2:0] W_3, V_GS_3, V_DS_3;
input [2:0] W_4, V_GS_4, V_DS_4;
input [2:0] W_5, V_GS_5, V_DS_5;
input [1:0] mode;
output [7:0] out_n;         					// use this if using continuous assignment for out_n  // Ex: assign out_n = XXX;
// output reg [7:0] out_n; 								// use this if using procedure assignment for out_n   // Ex: always@(*) begin out_n = XXX; end

//================================================================
//    Wire & Registers
//================================================================
wire  [6:0] I0_g0, I0_g0_stage1, I0_g0_stage2, I0_g0_stage3;
wire  [6:0] I1_g1, I1_g1_stage1, I1_g1_stage2, I1_g1_stage3, I1_g1_stage4;
wire  [6:0] I2_g2, I2_g2_stage1, I2_g2_stage2, I2_g2_stage3, I2_g2_stage4, I2_g2_stage5;
wire  [6:0] I3_g3, I3_g3_stage1, I3_g3_stage2, I3_g3_stage3, I3_g3_stage4, I3_g3_stage5;
wire  [6:0] I4_g4, I4_g4_stage1, I4_g4_stage2, I4_g4_stage3, I4_g4_stage4;
wire  [6:0] I5_g5, I5_g5_stage1, I5_g5_stage2, I5_g5_stage3;
wire  [6:0] n0, n1, n2, n3, n4, n5;
wire  [6:0] ADD_0, ADD_1, ADD_2;
wire  [7:0] sum, dividend;
wire  [9:0] weight;


//================================================================
//    DESIGN
//================================================================

// --------------------------------------------------
// write your design here
// --------------------------------------------------

/*Calculate Id or gm*/
Id_gm_calculator cal0 (.W(W_0), .V_GS(V_GS_0), .V_DS(V_DS_0), .Id_gm_sel(mode[0]), .out(I0_g0));
Id_gm_calculator cal1 (.W(W_1), .V_GS(V_GS_1), .V_DS(V_DS_1), .Id_gm_sel(mode[0]), .out(I1_g1));
Id_gm_calculator cal2 (.W(W_2), .V_GS(V_GS_2), .V_DS(V_DS_2), .Id_gm_sel(mode[0]), .out(I2_g2));
Id_gm_calculator cal3 (.W(W_3), .V_GS(V_GS_3), .V_DS(V_DS_3), .Id_gm_sel(mode[0]), .out(I3_g3));
Id_gm_calculator cal4 (.W(W_4), .V_GS(V_GS_4), .V_DS(V_DS_4), .Id_gm_sel(mode[0]), .out(I4_g4));
Id_gm_calculator cal5 (.W(W_5), .V_GS(V_GS_5), .V_DS(V_DS_5), .Id_gm_sel(mode[0]), .out(I5_g5));

/*Sort*/
// Stage 1
assign {I0_g0_stage1, I1_g1_stage1} = (I0_g0 > I1_g1) ? {I0_g0, I1_g1} : {I1_g1, I0_g0};
assign {I2_g2_stage1, I3_g3_stage1} = (I2_g2 > I3_g3) ? {I2_g2, I3_g3} : {I3_g3, I2_g2};
assign {I4_g4_stage1, I5_g5_stage1} = (I4_g4 > I5_g5) ? {I4_g4, I5_g5} : {I5_g5, I4_g4};
// Stage 2
assign {I0_g0_stage2, I1_g1_stage2} = (I0_g0_stage1 > I2_g2_stage1) ? {I0_g0_stage1, I2_g2_stage1} : {I2_g2_stage1, I0_g0_stage1};
assign {I2_g2_stage2, I3_g3_stage2} = (I1_g1_stage1 > I4_g4_stage1) ? {I1_g1_stage1, I4_g4_stage1} : {I4_g4_stage1, I1_g1_stage1};
assign {I4_g4_stage2, I5_g5_stage2} = (I3_g3_stage1 > I5_g5_stage1) ? {I3_g3_stage1, I5_g5_stage1} : {I5_g5_stage1, I3_g3_stage1};
// Stage 3
assign {I0_g0_stage3, I1_g1_stage3} = (I0_g0_stage2 > I2_g2_stage2) ? {I0_g0_stage2, I2_g2_stage2} : {I2_g2_stage2, I0_g0_stage2};
assign {I2_g2_stage3, I3_g3_stage3} = (I1_g1_stage2 > I4_g4_stage2) ? {I1_g1_stage2, I4_g4_stage2} : {I4_g4_stage2, I1_g1_stage2};
assign {I4_g4_stage3, I5_g5_stage3} = (I3_g3_stage2 > I5_g5_stage2) ? {I3_g3_stage2, I5_g5_stage2} : {I5_g5_stage2, I3_g3_stage2};
// Stage 4
assign {I1_g1_stage4, I2_g2_stage4} = (I1_g1_stage3 > I2_g2_stage3) ? {I1_g1_stage3, I2_g2_stage3} : {I2_g2_stage3, I1_g1_stage3};
assign {I3_g3_stage4, I4_g4_stage4} = (I3_g3_stage3 > I4_g4_stage3) ? {I3_g3_stage3, I4_g4_stage3} : {I4_g4_stage3, I3_g3_stage3};
// Stage 5
assign {I2_g2_stage5, I3_g3_stage5} = (I2_g2_stage4 > I3_g3_stage4) ? {I2_g2_stage4, I3_g3_stage4} : {I3_g3_stage4, I2_g2_stage4};
// Transfer to decending order
assign {n0, n1, n2, n3, n4, n5} = {I0_g0_stage3, I1_g1_stage4, I2_g2_stage5, I3_g3_stage5, I4_g4_stage4, I5_g5_stage3};

/*Select according to mode*/
assign {ADD_0, ADD_1, ADD_2} = (mode[1]) ? {n0, n1, n2} : {n3, n4, n5};
assign sum = ADD_0 + ADD_1 + ADD_2;
assign weight = (mode[0]) ? (sum << 2) - ADD_0 + ADD_2 : sum;
assign dividend = (mode[0]) ? weight >> 2 : weight;

/*Output*/
assign out_n = dividend / 2'd3;

endmodule


//================================================================
//   SUB MODULE
//================================================================

module Id_gm_calculator (
  // Input Signals
  W, V_GS, V_DS, Id_gm_sel,
  // Output Signals
  out
);
input  [2:0] W, V_GS, V_DS;
input        Id_gm_sel;
output [6:0] out;

wire   [7:0]  mul_out;
wire   [3:0]  vgs, vds;
wire          region;
reg    [3:0]  MUL_IN0, MUL_IN1;

assign vgs = {1'b0, V_GS};
assign vds = {1'b0, V_DS};
assign region = (vgs <= vds + 1'd1); // tri -> 0, sat -> 1

always @(*) begin
  case({Id_gm_sel, region})
    2'b00: begin
      MUL_IN0 = V_DS;
      MUL_IN1 = 4'd2;
    end
    2'b01: begin
      MUL_IN0 = V_GS - 1'd1;
      MUL_IN1 = 4'd2;
    end
    2'b10: begin
      MUL_IN0 = V_GS + V_GS - V_DS - 2'd2;
      MUL_IN1 = V_DS;
    end
    2'b11: begin
      MUL_IN0 = V_GS - 1'd1;
      MUL_IN1 = V_GS - 1'd1;
    end
  endcase
end

assign mul_out = W * MUL_IN0 * MUL_IN1;
assign out = mul_out / 2'd3;

endmodule
