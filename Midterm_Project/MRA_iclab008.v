//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Si2 LAB @NYCU ED430
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2023 Fall
//   Midterm Proejct            : MRA
//   Author                     : Lin-Hung, Lai
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : MRA.v
//   Module Name : MRA
//   Release version : V2.0 (Release Date: 2023-10)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module MRA(
    // CHIP IO
    clk            	,
    rst_n          	,
    in_valid       	,
    frame_id        ,
    net_id         	,
    loc_x          	,
    loc_y         	,
    cost	 		,
    busy         	,

    // AXI4 IO
         arid_m_inf,
       araddr_m_inf,
        arlen_m_inf,
       arsize_m_inf,
      arburst_m_inf,
      arvalid_m_inf,
      arready_m_inf,

          rid_m_inf,
        rdata_m_inf,
        rresp_m_inf,
        rlast_m_inf,
       rvalid_m_inf,
       rready_m_inf,

         awid_m_inf,
       awaddr_m_inf,
       awsize_m_inf,
      awburst_m_inf,
        awlen_m_inf,
      awvalid_m_inf,
      awready_m_inf,

        wdata_m_inf,
        wlast_m_inf,
       wvalid_m_inf,
       wready_m_inf,

          bid_m_inf,
        bresp_m_inf,
       bvalid_m_inf,
       bready_m_inf
);

// ===============================================================
//  					PARAMETERS
// ===============================================================
parameter ID_WIDTH   = 4;
parameter ADDR_WIDTH = 32;
parameter DATA_WIDTH = 128;
parameter READ  = 1'b1;
parameter WRITE = 1'b0;
// FSM_MAIN
`define STATE_BITS 4
parameter IDLE            = `STATE_BITS'd0;
parameter FETCH           = `STATE_BITS'd1;
parameter FILL_PATH       = `STATE_BITS'd2;
parameter PROPAGATE       = `STATE_BITS'd3;
parameter WAIT            = `STATE_BITS'd7;
parameter RETRACE         = `STATE_BITS'd4;
parameter UPDATE          = `STATE_BITS'd5;
parameter WRITE_RESULT    = `STATE_BITS'd6;
parameter PROPAGATE_FILL_SOURCE    = `STATE_BITS'd8;
parameter PROPAGATE_FILL_SINK      = `STATE_BITS'd9;
// FSM_DRAM
parameter DRAM_IDLE            = `STATE_BITS'd0;
parameter DRAM_ARVALID         = `STATE_BITS'd1;
parameter DRAM_FILL_LOCATION   = `STATE_BITS'd2;
parameter DRAM_ARVALID2        = `STATE_BITS'd3;
parameter DRAM_FILL_WEIGHT     = `STATE_BITS'd4;
parameter DRAM_WAIT            = `STATE_BITS'd5;
parameter DRAM_AWVALID         = `STATE_BITS'd6;
parameter DRAM_WRITE_RESULT    = `STATE_BITS'd7;
// PATH MAP
parameter PATH_EMPTY      = 2'b00;
parameter PATH_BLOCKED    = 2'b01;
parameter PATH_STATE0     = 2'b10;
parameter PATH_STATE1     = 2'b11;
// AXI CONSTANTS
// (1)	axi read channel
parameter ARID    = 4'd0;    // ID = 0
parameter ARLEN   = 8'd127;  // Burst Length
parameter ARSIZE  = 3'b100;  // 16 Bytes per Transfer
parameter ARBURST = 2'd1;    // INCR mode
// (2) 	axi write channel
parameter AWID    = 4'd0;
parameter AWLEN   = 8'd127;
parameter AWSIZE  = 3'b100;
parameter AWBURST = 2'd1;
// INTEGERS
integer i, j;


// ===============================================================
//  					Input / Output
// ===============================================================
// << CHIP io port with system >>
input 			  	clk, rst_n;
input 			   	in_valid;
input  [4:0] 		frame_id;
input  [3:0]       	net_id;
input  [5:0]       	loc_x;
input  [5:0]       	loc_y;
output reg [13:0] 	cost;
output reg          busy;

// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
       Your AXI-4 interface could be designed as a bridge in submodule,
       therefore I declared output of AXI as wire.
       Ex: AXI4_interface AXI4_INF(...);
*/

// ------------------------
// <<<<< AXI READ >>>>>
// ------------------------
// (1)	axi read address channel
output wire [ID_WIDTH-1:0]      arid_m_inf;
output wire [1:0]            arburst_m_inf;
output wire [2:0]             arsize_m_inf;
output wire [7:0]              arlen_m_inf;
output wire                  arvalid_m_inf;
input  wire                  arready_m_inf;
output reg  [ADDR_WIDTH-1:0]  araddr_m_inf;
// ------------------------
// (2)	axi read data channel
input  wire [ID_WIDTH-1:0]       rid_m_inf;
input  wire                   rvalid_m_inf;
output wire                   rready_m_inf;
input  wire [DATA_WIDTH-1:0]   rdata_m_inf;
input  wire                    rlast_m_inf;
input  wire [1:0]              rresp_m_inf;
// ------------------------
// <<<<< AXI WRITE >>>>>
// ------------------------
// (1) 	axi write address channel
output wire [ID_WIDTH-1:0]      awid_m_inf;
output wire [1:0]            awburst_m_inf;
output wire [2:0]             awsize_m_inf;
output wire [7:0]              awlen_m_inf;
output wire                  awvalid_m_inf;
input  wire                  awready_m_inf;
output  reg [ADDR_WIDTH-1:0]  awaddr_m_inf;
// -------------------------
// (2)	axi write data channel
output  reg                   wvalid_m_inf;
input  wire                   wready_m_inf;
output  reg [DATA_WIDTH-1:0]   wdata_m_inf;
output wire                    wlast_m_inf;
// output  reg                    wlast_m_inf;
// -------------------------
// (3)	axi write response channel
input  wire  [ID_WIDTH-1:0]      bid_m_inf;
input  wire                   bvalid_m_inf;
output wire                   bready_m_inf;
input  wire  [1:0]             bresp_m_inf;
// -----------------------------

// ===============================================================
//  					REG / WIRE
// ===============================================================
// FSM
reg [`STATE_BITS-1:0] cs, ns;
reg [`STATE_BITS-2:0] cs_dram, ns_dram;
// AXI INTERFACE
reg  [ADDR_WIDTH-1:0] addr_location_offset, addr_weight_offset;
// INPUT REGISTERS
reg    [4:0] frame_id_r;
reg    [3:0] net_id_r     [0:14];
reg    [5:0] source_x_r   [0:14];
reg    [5:0] source_y_r   [0:14];
reg    [5:0] sink_x_r     [0:14];
reg    [5:0] sink_y_r     [0:14];
reg    [3:0] cur_net_id_r, total_net_id_r;
reg          cnt2_r;
// LOCATION MAP, WEIGHT MAP
reg           WEB_LOC,  WEB_WEI;
reg  [127:0]   DI_LOC,   DI_WEI;
reg    [6:0] ADDR_LOC, ADDR_WEI;
wire [127:0]   DO_LOC,   DO_WEI;
reg    [6:0] addr_cnt_r;
reg  [127:0] replaced_data;
// PATH MAP
reg    [1:0] path_map_r     [0:63][0:63];
reg    [1:0] path_state;
reg    [1:0] cnt4_r;
reg    [5:0] retrace_x_r, retrace_y_r;
reg    [3:0] cur_cost;
// FLAGS
wire         finish_propagate_flag;
wire         finish_retrace_flag;




// ===============================================================
//  					FINITE STATE MACHINE
// ===============================================================
// FSM_MAIN
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  cs <= IDLE;
    else         cs <= ns;
end
always @(*) begin
    case(cs)
        IDLE:               ns = (in_valid)            ? FETCH     : IDLE;
        FETCH:              ns = (~in_valid)           ? FILL_PATH : FETCH;
        FILL_PATH:          ns = (rlast_m_inf)         ? PROPAGATE_FILL_SINK : FILL_PATH;
        PROPAGATE_FILL_SINK  : ns = PROPAGATE_FILL_SOURCE;
        PROPAGATE_FILL_SOURCE: ns = PROPAGATE;
        PROPAGATE: begin
            if (cur_net_id_r == 0)     ns = (finish_propagate_flag) ? WAIT    : PROPAGATE;
            else                       ns = (finish_propagate_flag) ? RETRACE : PROPAGATE;
        end
        WAIT:               ns = (rlast_m_inf)         ? RETRACE   : WAIT;
        RETRACE:            ns = (finish_retrace_flag) ? UPDATE    : RETRACE;
        UPDATE:             ns = (cur_net_id_r == total_net_id_r) ? WRITE_RESULT : PROPAGATE_FILL_SINK;
        WRITE_RESULT:       ns = (bvalid_m_inf)        ? IDLE : WRITE_RESULT;
        default:            ns = cs;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  busy <= 0;
    else begin
        busy <= ~(ns == IDLE || ns == FETCH);
    end
end

// FSM_DRAM
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  cs_dram <= DRAM_IDLE;
    else         cs_dram <= ns_dram;
end
always @(*) begin
    case(cs_dram)
        DRAM_IDLE:              ns_dram = (in_valid)                       ? DRAM_ARVALID       : DRAM_IDLE;
        DRAM_ARVALID:           ns_dram = (arready_m_inf)                  ? DRAM_FILL_LOCATION : DRAM_ARVALID;
        DRAM_FILL_LOCATION:     ns_dram = (rlast_m_inf)                    ? DRAM_ARVALID2      : DRAM_FILL_LOCATION;
        DRAM_ARVALID2:          ns_dram = (arready_m_inf)                  ? DRAM_FILL_WEIGHT   : DRAM_ARVALID2;
        DRAM_FILL_WEIGHT:       ns_dram = (rlast_m_inf)                    ? DRAM_WAIT          : DRAM_FILL_WEIGHT;
        DRAM_WAIT:              ns_dram = (cur_net_id_r == total_net_id_r) ? DRAM_AWVALID       : DRAM_WAIT;
        DRAM_AWVALID:           ns_dram = (awready_m_inf)                  ? DRAM_WRITE_RESULT  : DRAM_AWVALID;
        DRAM_WRITE_RESULT:      ns_dram = (bvalid_m_inf)                   ? DRAM_IDLE          : DRAM_WRITE_RESULT;
        default:                ns_dram = cs_dram;
    endcase
end

// ===============================================================
//  					AXI INTERFACE
// ===============================================================
// (1) 	axi read channel
assign arid_m_inf     = ARID;    // constant
assign arburst_m_inf  = ARBURST; // constant
assign arsize_m_inf   = ARSIZE;  // constant
assign arlen_m_inf    = ARLEN;   // constant
assign arvalid_m_inf  = (cs_dram == DRAM_ARVALID || cs_dram == DRAM_ARVALID2);
assign rready_m_inf   = (cs_dram == DRAM_FILL_LOCATION || cs_dram == DRAM_FILL_WEIGHT);

always @(*) begin
    if (cs_dram == DRAM_ARVALID)        araddr_m_inf = addr_location_offset;
    else if (cs_dram == DRAM_ARVALID2)  araddr_m_inf = addr_weight_offset;
    else                                araddr_m_inf = 0;
end
always @(*) begin
    if (cs_dram == DRAM_AWVALID)        awaddr_m_inf = addr_location_offset;
    else                                awaddr_m_inf = 0;
end

always @(*) addr_location_offset = {16'h0001, frame_id_r, 11'd0};
always @(*) addr_weight_offset   = {16'h0002, frame_id_r, 11'd0};

// (2) 	axi write channel
assign awid_m_inf     = AWID;    // constant
assign awburst_m_inf  = AWBURST; // constant
assign awsize_m_inf   = AWSIZE;  // constant
assign awlen_m_inf    = AWLEN;   // constant
assign awvalid_m_inf  = (cs_dram == DRAM_AWVALID);
assign wlast_m_inf    = (&addr_cnt_r && cs_dram == DRAM_WRITE_RESULT);
assign bready_m_inf   = (cs_dram == DRAM_WRITE_RESULT);

// wdata_m_inf
always @(*) begin
    if (cs_dram == DRAM_WRITE_RESULT) wdata_m_inf = DO_LOC;
    else                              wdata_m_inf = 0;
end

always @(*) wvalid_m_inf = (cs_dram == DRAM_WRITE_RESULT);

// ===============================================================
//  					LOCATION & WEIGHT MAP
// ===============================================================

MAP_128X128 LOCATION_MAP(
    .CK(clk), .WEB(WEB_LOC), .OE(1'b1), .CS(1'b1),

    .A0(ADDR_LOC[0]), .A1(ADDR_LOC[1]), .A2(ADDR_LOC[2]), .A3(ADDR_LOC[3]), .A4(ADDR_LOC[4]), .A5(ADDR_LOC[5]), .A6(ADDR_LOC[6]),

    .DI0  (DI_LOC[0]),   .DI1  (DI_LOC[1]),   .DI2 (DI_LOC[2]),   .DI3 (DI_LOC[3]),   .DI4 (DI_LOC[4]),   .DI5 (DI_LOC[5]),   .DI6 (DI_LOC[6]),   .DI7 (DI_LOC[7]),
    .DI8  (DI_LOC[8]),   .DI9  (DI_LOC[9]),   .DI10(DI_LOC[10]),  .DI11(DI_LOC[11]),  .DI12(DI_LOC[12]),  .DI13(DI_LOC[13]),  .DI14(DI_LOC[14]),  .DI15(DI_LOC[15]),
    .DI16 (DI_LOC[16]),  .DI17 (DI_LOC[17]),  .DI18(DI_LOC[18]),  .DI19(DI_LOC[19]),  .DI20(DI_LOC[20]),  .DI21(DI_LOC[21]),  .DI22(DI_LOC[22]),  .DI23(DI_LOC[23]),
    .DI24 (DI_LOC[24]),  .DI25 (DI_LOC[25]),  .DI26(DI_LOC[26]),  .DI27(DI_LOC[27]),  .DI28(DI_LOC[28]),  .DI29(DI_LOC[29]),  .DI30(DI_LOC[30]),  .DI31(DI_LOC[31]),
    .DI32 (DI_LOC[32]),  .DI33 (DI_LOC[33]),  .DI34(DI_LOC[34]),  .DI35(DI_LOC[35]),  .DI36(DI_LOC[36]),  .DI37(DI_LOC[37]),  .DI38(DI_LOC[38]),  .DI39(DI_LOC[39]),
    .DI40 (DI_LOC[40]),  .DI41 (DI_LOC[41]),  .DI42(DI_LOC[42]),  .DI43(DI_LOC[43]),  .DI44(DI_LOC[44]),  .DI45(DI_LOC[45]),  .DI46(DI_LOC[46]),  .DI47(DI_LOC[47]),
    .DI48 (DI_LOC[48]),  .DI49 (DI_LOC[49]),  .DI50(DI_LOC[50]),  .DI51(DI_LOC[51]),  .DI52(DI_LOC[52]),  .DI53(DI_LOC[53]),  .DI54(DI_LOC[54]),  .DI55(DI_LOC[55]),
    .DI56 (DI_LOC[56]),  .DI57 (DI_LOC[57]),  .DI58(DI_LOC[58]),  .DI59(DI_LOC[59]),  .DI60(DI_LOC[60]),  .DI61(DI_LOC[61]),  .DI62(DI_LOC[62]),  .DI63(DI_LOC[63]),
    .DI64 (DI_LOC[64]),  .DI65 (DI_LOC[65]),  .DI66(DI_LOC[66]),  .DI67(DI_LOC[67]),  .DI68(DI_LOC[68]),  .DI69(DI_LOC[69]),  .DI70(DI_LOC[70]),  .DI71(DI_LOC[71]),
    .DI72 (DI_LOC[72]),  .DI73 (DI_LOC[73]),  .DI74(DI_LOC[74]),  .DI75(DI_LOC[75]),  .DI76(DI_LOC[76]),  .DI77(DI_LOC[77]),  .DI78(DI_LOC[78]),  .DI79(DI_LOC[79]),
    .DI80 (DI_LOC[80]),  .DI81 (DI_LOC[81]),  .DI82(DI_LOC[82]),  .DI83(DI_LOC[83]),  .DI84(DI_LOC[84]),  .DI85(DI_LOC[85]),  .DI86(DI_LOC[86]),  .DI87(DI_LOC[87]),
    .DI88 (DI_LOC[88]),  .DI89 (DI_LOC[89]),  .DI90(DI_LOC[90]),  .DI91(DI_LOC[91]),  .DI92(DI_LOC[92]),  .DI93(DI_LOC[93]),  .DI94(DI_LOC[94]),  .DI95(DI_LOC[95]),
    .DI96 (DI_LOC[96]),  .DI97 (DI_LOC[97]),  .DI98(DI_LOC[98]),  .DI99(DI_LOC[99]), .DI100(DI_LOC[100]),.DI101(DI_LOC[101]),.DI102(DI_LOC[102]),.DI103(DI_LOC[103]),
    .DI104(DI_LOC[104]), .DI105(DI_LOC[105]),.DI106(DI_LOC[106]),.DI107(DI_LOC[107]),.DI108(DI_LOC[108]),.DI109(DI_LOC[109]),.DI110(DI_LOC[110]),.DI111(DI_LOC[111]),
    .DI112(DI_LOC[112]), .DI113(DI_LOC[113]),.DI114(DI_LOC[114]),.DI115(DI_LOC[115]),.DI116(DI_LOC[116]),.DI117(DI_LOC[117]),.DI118(DI_LOC[118]),.DI119(DI_LOC[119]),
    .DI120(DI_LOC[120]), .DI121(DI_LOC[121]),.DI122(DI_LOC[122]),.DI123(DI_LOC[123]),.DI124(DI_LOC[124]),.DI125(DI_LOC[125]),.DI126(DI_LOC[126]),.DI127(DI_LOC[127]),

    .DO0  (DO_LOC[0]),   .DO1 (DO_LOC[1]),   .DO2 (DO_LOC[2]),   .DO3 (DO_LOC[3]),   .DO4 (DO_LOC[4]),   .DO5 (DO_LOC[5]),   .DO6 (DO_LOC[6]),   .DO7 (DO_LOC[7]),
    .DO8  (DO_LOC[8]),   .DO9 (DO_LOC[9]),   .DO10(DO_LOC[10]),  .DO11(DO_LOC[11]),  .DO12(DO_LOC[12]),  .DO13(DO_LOC[13]),  .DO14(DO_LOC[14]),  .DO15(DO_LOC[15]),
    .DO16 (DO_LOC[16]),  .DO17(DO_LOC[17]),  .DO18(DO_LOC[18]),  .DO19(DO_LOC[19]),  .DO20(DO_LOC[20]),  .DO21(DO_LOC[21]),  .DO22(DO_LOC[22]),  .DO23(DO_LOC[23]),
    .DO24 (DO_LOC[24]),  .DO25(DO_LOC[25]),  .DO26(DO_LOC[26]),  .DO27(DO_LOC[27]),  .DO28(DO_LOC[28]),  .DO29(DO_LOC[29]),  .DO30(DO_LOC[30]),  .DO31(DO_LOC[31]),
    .DO32 (DO_LOC[32]),  .DO33(DO_LOC[33]),  .DO34(DO_LOC[34]),  .DO35(DO_LOC[35]),  .DO36(DO_LOC[36]),  .DO37(DO_LOC[37]),  .DO38(DO_LOC[38]),  .DO39(DO_LOC[39]),
    .DO40 (DO_LOC[40]),  .DO41(DO_LOC[41]),  .DO42(DO_LOC[42]),  .DO43(DO_LOC[43]),  .DO44(DO_LOC[44]),  .DO45(DO_LOC[45]),  .DO46(DO_LOC[46]),  .DO47(DO_LOC[47]),
    .DO48 (DO_LOC[48]),  .DO49(DO_LOC[49]),  .DO50(DO_LOC[50]),  .DO51(DO_LOC[51]),  .DO52(DO_LOC[52]),  .DO53(DO_LOC[53]),  .DO54(DO_LOC[54]),  .DO55(DO_LOC[55]),
    .DO56 (DO_LOC[56]),  .DO57(DO_LOC[57]),  .DO58(DO_LOC[58]),  .DO59(DO_LOC[59]),  .DO60(DO_LOC[60]),  .DO61(DO_LOC[61]),  .DO62(DO_LOC[62]),  .DO63(DO_LOC[63]),
    .DO64 (DO_LOC[64]),  .DO65(DO_LOC[65]),  .DO66(DO_LOC[66]),  .DO67(DO_LOC[67]),  .DO68(DO_LOC[68]),  .DO69(DO_LOC[69]),  .DO70(DO_LOC[70]),  .DO71(DO_LOC[71]),
    .DO72 (DO_LOC[72]),  .DO73(DO_LOC[73]),  .DO74(DO_LOC[74]),  .DO75(DO_LOC[75]),  .DO76(DO_LOC[76]),  .DO77(DO_LOC[77]),  .DO78(DO_LOC[78]),  .DO79(DO_LOC[79]),
    .DO80 (DO_LOC[80]),  .DO81(DO_LOC[81]),  .DO82(DO_LOC[82]),  .DO83(DO_LOC[83]),  .DO84(DO_LOC[84]),  .DO85(DO_LOC[85]),  .DO86(DO_LOC[86]),  .DO87(DO_LOC[87]),
    .DO88 (DO_LOC[88]),  .DO89(DO_LOC[89]),  .DO90(DO_LOC[90]),  .DO91(DO_LOC[91]),  .DO92(DO_LOC[92]),  .DO93(DO_LOC[93]),  .DO94(DO_LOC[94]),  .DO95(DO_LOC[95]),
    .DO96 (DO_LOC[96]),  .DO97(DO_LOC[97]),  .DO98(DO_LOC[98]),  .DO99(DO_LOC[99]), .DO100(DO_LOC[100]),.DO101(DO_LOC[101]),.DO102(DO_LOC[102]),.DO103(DO_LOC[103]),
    .DO104(DO_LOC[104]),.DO105(DO_LOC[105]),.DO106(DO_LOC[106]),.DO107(DO_LOC[107]),.DO108(DO_LOC[108]),.DO109(DO_LOC[109]),.DO110(DO_LOC[110]),.DO111(DO_LOC[111]),
    .DO112(DO_LOC[112]),.DO113(DO_LOC[113]),.DO114(DO_LOC[114]),.DO115(DO_LOC[115]),.DO116(DO_LOC[116]),.DO117(DO_LOC[117]),.DO118(DO_LOC[118]),.DO119(DO_LOC[119]),
    .DO120(DO_LOC[120]),.DO121(DO_LOC[121]),.DO122(DO_LOC[122]),.DO123(DO_LOC[123]),.DO124(DO_LOC[124]),.DO125(DO_LOC[125]),.DO126(DO_LOC[126]),.DO127(DO_LOC[127]));

MAP_128X128 WEIGHT_MAP(
    .CK(clk), .WEB(WEB_WEI), .OE(1'b1), .CS(1'b1),

    .A0(ADDR_WEI[0]), .A1(ADDR_WEI[1]), .A2(ADDR_WEI[2]), .A3(ADDR_WEI[3]), .A4(ADDR_WEI[4]), .A5(ADDR_WEI[5]), .A6(ADDR_WEI[6]),

    .DI0  (DI_WEI[0]),   .DI1  (DI_WEI[1]),   .DI2 (DI_WEI[2]),   .DI3 (DI_WEI[3]),   .DI4 (DI_WEI[4]),   .DI5 (DI_WEI[5]),   .DI6 (DI_WEI[6]),   .DI7 (DI_WEI[7]),
    .DI8  (DI_WEI[8]),   .DI9  (DI_WEI[9]),   .DI10(DI_WEI[10]),  .DI11(DI_WEI[11]),  .DI12(DI_WEI[12]),  .DI13(DI_WEI[13]),  .DI14(DI_WEI[14]),  .DI15(DI_WEI[15]),
    .DI16 (DI_WEI[16]),  .DI17 (DI_WEI[17]),  .DI18(DI_WEI[18]),  .DI19(DI_WEI[19]),  .DI20(DI_WEI[20]),  .DI21(DI_WEI[21]),  .DI22(DI_WEI[22]),  .DI23(DI_WEI[23]),
    .DI24 (DI_WEI[24]),  .DI25 (DI_WEI[25]),  .DI26(DI_WEI[26]),  .DI27(DI_WEI[27]),  .DI28(DI_WEI[28]),  .DI29(DI_WEI[29]),  .DI30(DI_WEI[30]),  .DI31(DI_WEI[31]),
    .DI32 (DI_WEI[32]),  .DI33 (DI_WEI[33]),  .DI34(DI_WEI[34]),  .DI35(DI_WEI[35]),  .DI36(DI_WEI[36]),  .DI37(DI_WEI[37]),  .DI38(DI_WEI[38]),  .DI39(DI_WEI[39]),
    .DI40 (DI_WEI[40]),  .DI41 (DI_WEI[41]),  .DI42(DI_WEI[42]),  .DI43(DI_WEI[43]),  .DI44(DI_WEI[44]),  .DI45(DI_WEI[45]),  .DI46(DI_WEI[46]),  .DI47(DI_WEI[47]),
    .DI48 (DI_WEI[48]),  .DI49 (DI_WEI[49]),  .DI50(DI_WEI[50]),  .DI51(DI_WEI[51]),  .DI52(DI_WEI[52]),  .DI53(DI_WEI[53]),  .DI54(DI_WEI[54]),  .DI55(DI_WEI[55]),
    .DI56 (DI_WEI[56]),  .DI57 (DI_WEI[57]),  .DI58(DI_WEI[58]),  .DI59(DI_WEI[59]),  .DI60(DI_WEI[60]),  .DI61(DI_WEI[61]),  .DI62(DI_WEI[62]),  .DI63(DI_WEI[63]),
    .DI64 (DI_WEI[64]),  .DI65 (DI_WEI[65]),  .DI66(DI_WEI[66]),  .DI67(DI_WEI[67]),  .DI68(DI_WEI[68]),  .DI69(DI_WEI[69]),  .DI70(DI_WEI[70]),  .DI71(DI_WEI[71]),
    .DI72 (DI_WEI[72]),  .DI73 (DI_WEI[73]),  .DI74(DI_WEI[74]),  .DI75(DI_WEI[75]),  .DI76(DI_WEI[76]),  .DI77(DI_WEI[77]),  .DI78(DI_WEI[78]),  .DI79(DI_WEI[79]),
    .DI80 (DI_WEI[80]),  .DI81 (DI_WEI[81]),  .DI82(DI_WEI[82]),  .DI83(DI_WEI[83]),  .DI84(DI_WEI[84]),  .DI85(DI_WEI[85]),  .DI86(DI_WEI[86]),  .DI87(DI_WEI[87]),
    .DI88 (DI_WEI[88]),  .DI89 (DI_WEI[89]),  .DI90(DI_WEI[90]),  .DI91(DI_WEI[91]),  .DI92(DI_WEI[92]),  .DI93(DI_WEI[93]),  .DI94(DI_WEI[94]),  .DI95(DI_WEI[95]),
    .DI96 (DI_WEI[96]),  .DI97 (DI_WEI[97]),  .DI98(DI_WEI[98]),  .DI99(DI_WEI[99]), .DI100(DI_WEI[100]),.DI101(DI_WEI[101]),.DI102(DI_WEI[102]),.DI103(DI_WEI[103]),
    .DI104(DI_WEI[104]), .DI105(DI_WEI[105]),.DI106(DI_WEI[106]),.DI107(DI_WEI[107]),.DI108(DI_WEI[108]),.DI109(DI_WEI[109]),.DI110(DI_WEI[110]),.DI111(DI_WEI[111]),
    .DI112(DI_WEI[112]), .DI113(DI_WEI[113]),.DI114(DI_WEI[114]),.DI115(DI_WEI[115]),.DI116(DI_WEI[116]),.DI117(DI_WEI[117]),.DI118(DI_WEI[118]),.DI119(DI_WEI[119]),
    .DI120(DI_WEI[120]), .DI121(DI_WEI[121]),.DI122(DI_WEI[122]),.DI123(DI_WEI[123]),.DI124(DI_WEI[124]),.DI125(DI_WEI[125]),.DI126(DI_WEI[126]),.DI127(DI_WEI[127]),

    .DO0  (DO_WEI[0]),   .DO1 (DO_WEI[1]),   .DO2 (DO_WEI[2]),   .DO3 (DO_WEI[3]),   .DO4 (DO_WEI[4]),   .DO5 (DO_WEI[5]),   .DO6 (DO_WEI[6]),   .DO7 (DO_WEI[7]),
    .DO8  (DO_WEI[8]),   .DO9 (DO_WEI[9]),   .DO10(DO_WEI[10]),  .DO11(DO_WEI[11]),  .DO12(DO_WEI[12]),  .DO13(DO_WEI[13]),  .DO14(DO_WEI[14]),  .DO15(DO_WEI[15]),
    .DO16 (DO_WEI[16]),  .DO17(DO_WEI[17]),  .DO18(DO_WEI[18]),  .DO19(DO_WEI[19]),  .DO20(DO_WEI[20]),  .DO21(DO_WEI[21]),  .DO22(DO_WEI[22]),  .DO23(DO_WEI[23]),
    .DO24 (DO_WEI[24]),  .DO25(DO_WEI[25]),  .DO26(DO_WEI[26]),  .DO27(DO_WEI[27]),  .DO28(DO_WEI[28]),  .DO29(DO_WEI[29]),  .DO30(DO_WEI[30]),  .DO31(DO_WEI[31]),
    .DO32 (DO_WEI[32]),  .DO33(DO_WEI[33]),  .DO34(DO_WEI[34]),  .DO35(DO_WEI[35]),  .DO36(DO_WEI[36]),  .DO37(DO_WEI[37]),  .DO38(DO_WEI[38]),  .DO39(DO_WEI[39]),
    .DO40 (DO_WEI[40]),  .DO41(DO_WEI[41]),  .DO42(DO_WEI[42]),  .DO43(DO_WEI[43]),  .DO44(DO_WEI[44]),  .DO45(DO_WEI[45]),  .DO46(DO_WEI[46]),  .DO47(DO_WEI[47]),
    .DO48 (DO_WEI[48]),  .DO49(DO_WEI[49]),  .DO50(DO_WEI[50]),  .DO51(DO_WEI[51]),  .DO52(DO_WEI[52]),  .DO53(DO_WEI[53]),  .DO54(DO_WEI[54]),  .DO55(DO_WEI[55]),
    .DO56 (DO_WEI[56]),  .DO57(DO_WEI[57]),  .DO58(DO_WEI[58]),  .DO59(DO_WEI[59]),  .DO60(DO_WEI[60]),  .DO61(DO_WEI[61]),  .DO62(DO_WEI[62]),  .DO63(DO_WEI[63]),
    .DO64 (DO_WEI[64]),  .DO65(DO_WEI[65]),  .DO66(DO_WEI[66]),  .DO67(DO_WEI[67]),  .DO68(DO_WEI[68]),  .DO69(DO_WEI[69]),  .DO70(DO_WEI[70]),  .DO71(DO_WEI[71]),
    .DO72 (DO_WEI[72]),  .DO73(DO_WEI[73]),  .DO74(DO_WEI[74]),  .DO75(DO_WEI[75]),  .DO76(DO_WEI[76]),  .DO77(DO_WEI[77]),  .DO78(DO_WEI[78]),  .DO79(DO_WEI[79]),
    .DO80 (DO_WEI[80]),  .DO81(DO_WEI[81]),  .DO82(DO_WEI[82]),  .DO83(DO_WEI[83]),  .DO84(DO_WEI[84]),  .DO85(DO_WEI[85]),  .DO86(DO_WEI[86]),  .DO87(DO_WEI[87]),
    .DO88 (DO_WEI[88]),  .DO89(DO_WEI[89]),  .DO90(DO_WEI[90]),  .DO91(DO_WEI[91]),  .DO92(DO_WEI[92]),  .DO93(DO_WEI[93]),  .DO94(DO_WEI[94]),  .DO95(DO_WEI[95]),
    .DO96 (DO_WEI[96]),  .DO97(DO_WEI[97]),  .DO98(DO_WEI[98]),  .DO99(DO_WEI[99]), .DO100(DO_WEI[100]),.DO101(DO_WEI[101]),.DO102(DO_WEI[102]),.DO103(DO_WEI[103]),
    .DO104(DO_WEI[104]),.DO105(DO_WEI[105]),.DO106(DO_WEI[106]),.DO107(DO_WEI[107]),.DO108(DO_WEI[108]),.DO109(DO_WEI[109]),.DO110(DO_WEI[110]),.DO111(DO_WEI[111]),
    .DO112(DO_WEI[112]),.DO113(DO_WEI[113]),.DO114(DO_WEI[114]),.DO115(DO_WEI[115]),.DO116(DO_WEI[116]),.DO117(DO_WEI[117]),.DO118(DO_WEI[118]),.DO119(DO_WEI[119]),
    .DO120(DO_WEI[120]),.DO121(DO_WEI[121]),.DO122(DO_WEI[122]),.DO123(DO_WEI[123]),.DO124(DO_WEI[124]),.DO125(DO_WEI[125]),.DO126(DO_WEI[126]),.DO127(DO_WEI[127]));

always @(*) begin
    if (cs_dram == DRAM_FILL_LOCATION && rvalid_m_inf) begin
        WEB_LOC  = WRITE;
        ADDR_LOC = addr_cnt_r;
        DI_LOC   = rdata_m_inf;
    end
    else if (cs == RETRACE) begin
        WEB_LOC  = ~cnt2_r;
        ADDR_LOC = {retrace_y_r, retrace_x_r[5]};
        DI_LOC   = replaced_data;
    end
    else if (cs == WRITE_RESULT) begin
        WEB_LOC  = READ;
        ADDR_LOC = addr_cnt_r + wready_m_inf;
        DI_LOC   = 0;
    end
    else begin
        WEB_LOC  = READ;
        ADDR_LOC = 0;
        DI_LOC   = 0;
    end
end

always @(*) begin
    if (cs_dram == DRAM_FILL_WEIGHT && rvalid_m_inf) begin
        WEB_WEI  = WRITE;
        ADDR_WEI = addr_cnt_r;
        DI_WEI   = rdata_m_inf;
    end
    else if (cs == RETRACE) begin
        WEB_WEI  = READ;
        ADDR_WEI = {retrace_y_r, retrace_x_r[5]};
        DI_WEI   = 0;
    end
    else begin
        WEB_WEI  = READ;
        ADDR_WEI = 0;
        DI_WEI   = 0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  addr_cnt_r <= 0;
    else begin
        if (cs_dram == DRAM_FILL_LOCATION)
            addr_cnt_r <= (rvalid_m_inf && rready_m_inf) ? addr_cnt_r + 1'd1 : addr_cnt_r;
        else if (cs_dram == DRAM_FILL_WEIGHT)
            addr_cnt_r <= (rvalid_m_inf && rready_m_inf) ? addr_cnt_r + 1'd1 : addr_cnt_r;
        else if (cs_dram == DRAM_WRITE_RESULT)
            addr_cnt_r <= addr_cnt_r + wready_m_inf;
        else
            addr_cnt_r <= 0;
    end
end

always @(*) begin
    replaced_data = DO_LOC;
    replaced_data[4*(retrace_x_r[4:0])+3 -: 4] = net_id_r[cur_net_id_r];
end

// ===============================================================
//  					PATH MAP
// ===============================================================
// path_mar_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for(i = 0; i < 64; i = i + 1)
            for(j = 0; j < 64; j = j + 1)
                path_map_r[i][j] <= 0;
    end
    else begin
        if (cs_dram == DRAM_FILL_LOCATION && rvalid_m_inf) begin
            if (addr_cnt_r[0]) begin // RHS
                for (i = 0; i < 32; i = i + 1) begin
                    if (rdata_m_inf[4*i+3 -: 4] != 4'd0)       path_map_r[addr_cnt_r[6:1]][i+32] <= PATH_BLOCKED;
                    else                                       path_map_r[addr_cnt_r[6:1]][i+32] <= PATH_EMPTY;
                end
            end
            else begin // LHS
                for (i = 0; i < 32; i = i + 1) begin
                    if (rdata_m_inf[4*i+3 -: 4] != 4'd0)       path_map_r[addr_cnt_r[6:1]][i] <= PATH_BLOCKED;
                    else                                       path_map_r[addr_cnt_r[6:1]][i] <= PATH_EMPTY;
                end
            end
        end
        else if (cs == PROPAGATE_FILL_SOURCE) begin
            path_map_r[source_y_r[cur_net_id_r]][source_x_r[cur_net_id_r]] <= PATH_STATE0;
        end
        else if (cs == PROPAGATE_FILL_SINK) begin
            path_map_r[sink_y_r  [cur_net_id_r]][sink_x_r  [cur_net_id_r]] <= PATH_EMPTY;
        end
        else if (cs == PROPAGATE) begin
            // Inner 62X62
            for (i = 1; i < 63; i = i + 1) begin
                for (j = 1; j < 63 ;j = j + 1) begin
                    if (path_map_r[i][j] == PATH_EMPTY && (path_map_r[i-1][j][1] | path_map_r[i+1][j][1] | path_map_r[i][j-1][1] | path_map_r[i][j+1][1]))
                        path_map_r[i][j] <= path_state;
                end
            end
            // Upper 1X62
            for (j = 1; j < 63; j = j + 1) begin
                if (path_map_r[0][j] == PATH_EMPTY && (path_map_r[1][j][1] | path_map_r[0][j-1][1] | path_map_r[0][j+1][1]))
                    path_map_r[0][j] <= path_state;
            end
            // Lower 1X62
            for (j = 1; j < 63; j = j + 1) begin
                if (path_map_r[63][j] == PATH_EMPTY && (path_map_r[62][j][1] | path_map_r[63][j-1][1] | path_map_r[63][j+1][1]))
                    path_map_r[63][j] <= path_state;
            end
            // Left 62X1
            for (i = 1; i < 63; i = i + 1) begin
                if (path_map_r[i][0] == PATH_EMPTY && (path_map_r[i][1][1] | path_map_r[i-1][0][1] | path_map_r[i+1][0][1]))
                    path_map_r[i][0] <= path_state;
            end
            // Right 62X1
            for (i = 1; i < 63; i = i + 1) begin
                if (path_map_r[i][63] == PATH_EMPTY && (path_map_r[i][62][1] | path_map_r[i-1][63][1] | path_map_r[i+1][63][1]))
                    path_map_r[i][63] <= path_state;
            end
            // Corner UL
            if (path_map_r[0][0] == PATH_EMPTY && (path_map_r[1][0][1] | path_map_r[0][1][1]))
                path_map_r[0][0] <= path_state;
            // Corner UR
            if (path_map_r[0][63] == PATH_EMPTY && (path_map_r[1][63][1] | path_map_r[0][62][1]))
                path_map_r[0][63] <= path_state;
            // Corner DL
            if (path_map_r[63][0] == PATH_EMPTY && (path_map_r[62][0][1] | path_map_r[63][1][1]))
                path_map_r[63][0] <= path_state;
            // Corner DR
            if (path_map_r[63][63] == PATH_EMPTY && (path_map_r[62][63][1] | path_map_r[63][62][1]))
                path_map_r[63][63] <= path_state;
        end
        else if (cs == RETRACE && cnt2_r) begin
            path_map_r[retrace_y_r][retrace_x_r] <= PATH_BLOCKED;
        end
        else if (cs == UPDATE) begin
            for (i = 0; i < 64; i = i + 1)
                for (j = 0; j < 64; j = j + 1)
                    if (path_map_r[i][j] != PATH_BLOCKED)
                        path_map_r[i][j] <= PATH_EMPTY;
        end
    end
end

// finish_propagate_flag
assign finish_propagate_flag = (path_map_r[sink_y_r[cur_net_id_r]][sink_x_r[cur_net_id_r]][1]);

// finish_retrace_flag
assign finish_retrace_flag = (retrace_x_r == source_x_r[cur_net_id_r] && retrace_y_r == source_y_r[cur_net_id_r]) & (cs == RETRACE && cnt2_r);


// retrace_x_r, retrace_y_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        retrace_x_r <= 0;
        retrace_y_r <= 0;
    end
    else begin
        if (cs == PROPAGATE && finish_propagate_flag) begin // Initial retrace coordinates
            retrace_x_r <= sink_x_r[cur_net_id_r];
            retrace_y_r <= sink_y_r[cur_net_id_r];
        end
        else if (cs == RETRACE && cnt2_r) begin // Retrace Priority (Adding Boundary conditions)
            if (path_map_r[retrace_y_r+1'd1][retrace_x_r] == path_state && ~(&retrace_y_r)) begin //DOWN
                retrace_x_r <= retrace_x_r;
                retrace_y_r <= retrace_y_r + 1'd1;
            end
            else if (path_map_r[retrace_y_r-1'd1][retrace_x_r] == path_state && (|retrace_y_r)) begin // UP
                retrace_x_r <= retrace_x_r;
                retrace_y_r <= retrace_y_r - 1'd1;
            end
            else if (path_map_r[retrace_y_r][retrace_x_r+1'd1] == path_state && ~(&retrace_x_r)) begin // RIGHT
                retrace_x_r <= retrace_x_r + 1'd1;
                retrace_y_r <= retrace_y_r;
            end
            else if (path_map_r[retrace_y_r][retrace_x_r-1'd1] == path_state && (|retrace_x_r)) begin // LEFT
                retrace_x_r <= retrace_x_r - 1'd1;
                retrace_y_r <= retrace_y_r;
            end
            else begin
                retrace_x_r <= retrace_x_r;
                retrace_y_r <= retrace_y_r;
            end
        end
    end
end

// path_state
always @(*) begin
    case(cnt4_r)
        0, 1:    path_state = PATH_STATE0;
        2, 3:    path_state = PATH_STATE1;
        default: path_state = PATH_BLOCKED;
    endcase
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt4_r <= 0;
    else if (cs == PROPAGATE_FILL_SOURCE || cs == PROPAGATE)
        cnt4_r <= (finish_propagate_flag) ? cnt4_r - 2'd2 : cnt4_r + 1'd1;
    else if (cs == WAIT)
        cnt4_r <= cnt4_r;
    else if (cs == RETRACE)
        cnt4_r <= (cnt2_r) ? cnt4_r - 1'd1 : cnt4_r;
    else
        cnt4_r <= 0;
end

// cur_net_id_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)           cur_net_id_r <= 0;
    else begin
        if (cs == IDLE)
            cur_net_id_r <= 0;
        else if (finish_retrace_flag)
            cur_net_id_r <= cur_net_id_r + 1'd1;
    end
end

// ===============================================================
//  					INPUT REGISTERS
// ===============================================================
// frame_id_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)           frame_id_r <= 0;
    else if (in_valid)    frame_id_r <= frame_id;
    else                  frame_id_r <= frame_id_r;
end

// net_id_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 15; i = i + 1)  net_id_r[i] <= 0;
    end
    else begin
        if (in_valid && ~cnt2_r)
            net_id_r[total_net_id_r] <= net_id;
    end
end

// source_x_r, source_y_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 15; i = i + 1) begin
            source_x_r[i] <= 0;
            source_y_r[i] <= 0;
        end
    end
    else begin
        if (in_valid && ~cnt2_r) begin
            source_x_r[total_net_id_r] <= loc_x;
            source_y_r[total_net_id_r] <= loc_y;
        end
    end
end

// sink_x_r, sink_y_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 15; i = i + 1)  begin
            sink_x_r[i] <= 0;
            sink_y_r[i] <= 0;
        end
    end
    else begin
        if (in_valid && cnt2_r) begin
            sink_x_r[total_net_id_r] <= loc_x;
            sink_y_r[total_net_id_r] <= loc_y;
        end
    end
end

// total_net_id_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)           total_net_id_r <= 0;
    else if (cs == IDLE)  total_net_id_r <= 0;
    else begin
        if (in_valid && cnt2_r)
            total_net_id_r <= total_net_id_r + 1'd1;
        else
            total_net_id_r <= total_net_id_r;
    end
end

// cnt2_r
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                          cnt2_r <= 0;
    else if (in_valid || cs == RETRACE)  cnt2_r <= ~cnt2_r;
    else                                 cnt2_r <= 0;
end



// ===============================================================
//  					OUTPUT LOGICS
// ===============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cost <= 0;
    end
    else begin
        if (cs == IDLE)
            cost <= 0;
        else if (cs == RETRACE && cnt2_r) begin
            if ((retrace_x_r == source_x_r[cur_net_id_r] && retrace_y_r == source_y_r[cur_net_id_r]) || (retrace_x_r == sink_x_r  [cur_net_id_r] && retrace_y_r == sink_y_r  [cur_net_id_r]))
                cost <= cost;
            else begin
                cost <= cost + DO_WEI[4*(retrace_x_r[4:0])+3 -: 4];
            end
        end
    end
end

endmodule