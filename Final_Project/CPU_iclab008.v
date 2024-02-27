//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2021 Final Project: Customized ISA Processor
//   Author              : Hsi-Hao Huang
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : CPU.v
//   Module Name : CPU.v
//   Release version : V1.0 (Release Date: 2021-May)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module CPU(

                clk,
              rst_n,

           IO_stall,

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
       bready_m_inf,

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
       rready_m_inf

);
// Input port
input  wire clk, rst_n;
// Output port
output reg  IO_stall;

parameter ID_WIDTH = 4 , ADDR_WIDTH = 32, DATA_WIDTH = 16, DRAM_NUMBER=2, WRIT_NUMBER=1;

// AXI Interface wire connecttion for pseudo DRAM read/write
/* Hint:
  your AXI-4 interface could be designed as convertor in submodule(which used reg for output signal),
  therefore I declared output of AXI as wire in CPU
*/



// axi write address channel
output  wire [WRIT_NUMBER * ID_WIDTH-1:0]        awid_m_inf;
output  wire [WRIT_NUMBER * ADDR_WIDTH-1:0]    awaddr_m_inf;
output  wire [WRIT_NUMBER * 3 -1:0]            awsize_m_inf;
output  wire [WRIT_NUMBER * 2 -1:0]           awburst_m_inf;
output  wire [WRIT_NUMBER * 7 -1:0]             awlen_m_inf;
output  reg  [WRIT_NUMBER-1:0]                awvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                awready_m_inf;
// axi write data channel
output  wire [WRIT_NUMBER * DATA_WIDTH-1:0]     wdata_m_inf;
output  reg  [WRIT_NUMBER-1:0]                  wlast_m_inf;
output  reg  [WRIT_NUMBER-1:0]                 wvalid_m_inf;
input   wire [WRIT_NUMBER-1:0]                 wready_m_inf;
// axi write response channel
input   wire [WRIT_NUMBER * ID_WIDTH-1:0]         bid_m_inf;
input   wire [WRIT_NUMBER * 2 -1:0]             bresp_m_inf;
input   wire [WRIT_NUMBER-1:0]             	   bvalid_m_inf;
output  reg  [WRIT_NUMBER-1:0]                 bready_m_inf;
// -----------------------------
// axi read address channel
output  wire [DRAM_NUMBER * ID_WIDTH-1:0]        arid_m_inf;
output  wire [DRAM_NUMBER * ADDR_WIDTH-1:0]    araddr_m_inf;
output  wire [DRAM_NUMBER * 7 -1:0]             arlen_m_inf;
output  wire [DRAM_NUMBER * 3 -1:0]            arsize_m_inf;
output  wire [DRAM_NUMBER * 2 -1:0]           arburst_m_inf;
output  wire [DRAM_NUMBER-1:0]                arvalid_m_inf;
input   wire [DRAM_NUMBER-1:0]                arready_m_inf;
// -----------------------------
// axi read data channel
input   wire [DRAM_NUMBER * ID_WIDTH-1:0]         rid_m_inf;
input   wire [DRAM_NUMBER * DATA_WIDTH-1:0]     rdata_m_inf;
input   wire [DRAM_NUMBER * 2 -1:0]             rresp_m_inf;
input   wire [DRAM_NUMBER-1:0]                  rlast_m_inf;
input   wire [DRAM_NUMBER-1:0]                 rvalid_m_inf;
output  wire [DRAM_NUMBER-1:0]                 rready_m_inf;
// -----------------------------

/* Register in each core:
  There are sixteen registers in your CPU. You should not change the name of those registers.
  TA will check the value in each register when your core is not busy.
  If you change the name of registers below, you must get the fail in this lab.
*/

reg signed [15:0] core_r0 , core_r1 , core_r2 , core_r3 ;
reg signed [15:0] core_r4 , core_r5 , core_r6 , core_r7 ;
reg signed [15:0] core_r8 , core_r9 , core_r10, core_r11;
reg signed [15:0] core_r12, core_r13, core_r14, core_r15;

//####################################################
//               reg / wire
//####################################################
// FSM
reg [2:0] cs, ns;
reg [2:0] cs_inst, ns_inst;
reg [2:0] cs_data, ns_data;
reg [2:0] cs_dataw, ns_dataw;

// Instruction memory
reg  signed  [15:0]  pc, pc_next;
// Instruction
reg          [15:0]  inst;
wire         [ 2:0]  opcode;
wire         [ 3:0]  rs, rt, rd;
reg  signed  [15:0]  rs_val, rt_val, rd_val;
wire                 func;
wire signed  [ 4:0]  imm;
wire         [15:0]  jump_addr;
// Data Memory
reg          [15:0]  data;
reg  signed  [15:0]  DM_addr;
// AXI inst/data (R)
/*output  */ wire [ID_WIDTH-1:0]        arid_inst   , arid_data;
/*output  */ wire [ADDR_WIDTH-1:0]      araddr_inst , araddr_data;
/*output  */ wire [6:0]                 arlen_inst  , arlen_data;
/*output  */ wire [2:0]                 arsize_inst , arsize_data;
/*output  */ wire [1:0]                 arburst_inst, arburst_data;
/*output  */ wire                       arvalid_inst, arvalid_data;
/*input   */ wire                       arready_inst, arready_data;
// -----------------------------
/*input   */ wire [ID_WIDTH-1:0]        rid_inst   , rid_data;
/*input   */ wire [DATA_WIDTH-1:0]      rdata_inst , rdata_data;
/*input   */ wire [2 -1:0]              rresp_inst , rresp_data;
/*input   */ wire                       rlast_inst , rlast_data;
/*input   */ wire                       rvalid_inst, rvalid_data;
/*output  */ wire                       rready_inst, rready_data;
// Cache
reg   [6:0] cache_inst_addr, cache_data_addr;
reg   [3:0] inst_tag, data_tag;
reg         inst_in_valid,  data_in_valid,  write_valid;
reg         inst_out_valid, data_out_valid, write_finish;
reg         inst_tag_valid, data_tag_valid;
wire [15:0] cache_inst_out, cache_data_out;
wire [15:0] cache_data;

wire [DATA_WIDTH-1:0]      rdata_inst_dummy, rdata_inst_dummy1, rdata_inst_dummy2;
// wire [DATA_WIDTH-1:0]      rdata_inst_dummy;


//####################################################
//               parameter / integer
//####################################################
parameter signed OFFST = 16'h1000;

// FSM
parameter S_IDLE   = 3'd0;
parameter S_IF     = 3'd1; // Instruction Fetch
parameter S_ID     = 3'd2; // Instruction Decode
parameter S_EX     = 3'd3; // Execute
parameter S_MEM_L  = 3'd4; // Load
parameter S_MEM_S  = 3'd5; // Store
parameter S_WB     = 3'd6; // Write Back

parameter S_INST_IDLE   = 3'd0;
parameter S_INST_ADDR   = 3'd1;
parameter S_INST_DATA   = 3'd2;
parameter S_INST_OUT    = 3'd3;
parameter S_INST_HIT    = 3'd4;
parameter S_INST_LOAD   = 3'd5;

parameter S_DATA_IDLE   = 3'd0;
parameter S_DATA_ADDR   = 3'd1;
parameter S_DATA_DATA   = 3'd2;
parameter S_DATA_OUT    = 3'd3;
parameter S_DATA_HIT    = 3'd4;
parameter S_DATA_LOAD   = 3'd5;
parameter S_DATA_STORE  = 3'd6;

parameter S_DATAW_IDLE   = 3'd0;
parameter S_DATAW_ADDR   = 3'd1;
parameter S_DATAW_DATA   = 3'd2;
parameter S_DATAW_RESP   = 3'd3;

// AXI CONSTANTS
// (1)	axi read channel
parameter ARID    = 4'd0;    // ID = 0
parameter ARLEN   = 7'd127;  // Burst Length
parameter ARSIZE  = 3'b001;  // 4 Bytes (16 bits) per Transfer
parameter ARBURST = 2'd1;    // INCR mode
// (2) 	axi write channel
parameter AWID    = 4'd0;
parameter AWLEN   = 7'd0;
parameter AWSIZE  = 3'b001;
parameter AWBURST = 2'd1;


//###########################################
//
// AXI
//
//###########################################
/*
output  wire [ID_WIDTH-1:0]        arid_inst, arid_data;
output  wire [ADDR_WIDTH-1:0]      araddr_inst, araddr_data;
output  wire [7 -1:0]              arlen_inst, arlen_data;
output  wire [3 -1:0]              arsize_inst, arsize_data;
output  wire [2 -1:0]              arburst_inst, arburst_data;
output  wire                       arvalid_inst, arvalid_data;
input   wire                       arready_inst, arready_data;
-----------------------------
input   wire [ID_WIDTH-1:0]        rid_inst, rid_data;
input   wire [DATA_WIDTH-1:0]      rdata_inst, rdata_data;
input   wire [2 -1:0]              rresp_inst, rresp_data;
input   wire                       rlast_inst, rlast_data;
input   wire                       rvalid_inst, rvalid_data;
output  wire                       rready_inst, rready_data;
*/
// Split and Merge
assign    arid_m_inf = {   arid_inst,    arid_data};
assign  araddr_m_inf = { araddr_inst,  araddr_data};
assign   arlen_m_inf = {  arlen_inst,   arlen_data};
assign  arsize_m_inf = { arsize_inst,  arsize_data};
assign arburst_m_inf = {arburst_inst, arburst_data};
assign arvalid_m_inf = {arvalid_inst, arvalid_data};
assign {arready_inst, arready_data} = {arready_m_inf[1], arready_m_inf[0]};

assign {   rid_inst,    rid_data} = {     rid_m_inf[7:4],     rid_m_inf[3:0]};
assign { rdata_inst,  rdata_data} = { rdata_m_inf[31:16],  rdata_m_inf[15:0]};
assign { rresp_inst,  rresp_data} = {   rresp_m_inf[3:2],   rresp_m_inf[1:0]};
assign { rlast_inst,  rlast_data} = {     rlast_m_inf[1],     rlast_m_inf[0]};
assign {rvalid_inst, rvalid_data} = {    rvalid_m_inf[1],    rvalid_m_inf[0]};
assign rready_m_inf = {rready_inst, rready_data};

// Apply Constants
assign arid_inst    = ARID;
assign arlen_inst   = ARLEN;
assign arsize_inst  = ARSIZE;
assign arburst_inst = ARBURST;

assign arid_data    = ARID;
assign arlen_data   = ARLEN;
assign arsize_data  = ARSIZE;
assign arburst_data = ARBURST;

assign awid_m_inf    = AWID;
assign awlen_m_inf   = AWLEN;
assign awsize_m_inf  = AWSIZE;
assign awburst_m_inf = AWBURST;

//###########################################
//
// FSM / IO_stall
//
//###########################################
// FSM
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs <= S_IDLE;
    else        cs <= ns;
end
/*----------------------------------------
ADD      : IF -> ID -> EXE -> WB
SUB      : IF -> ID -> EXE -> WB
SLT      : IF -> ID -> EXE -> WB
MULT     : IF -> ID -> EXE -> WB
LOAD     : IF -> ID -> EXE -> MEM_L
STORE    : IF -> ID -> EXE -> MEM_S
BOE      : IF -> ID -> EXE
JUMP     : IF -> ID -> EXE
----------------------------------------*/
always @(*) begin // TODO: ns
    case(cs)
        S_IDLE  : ns = S_IF;
        S_IF    : ns = (inst_out_valid) ? S_ID : cs;
        S_ID    : ns = S_EX;
        S_EX    : begin
            case(opcode)
                3'b000: ns = S_WB;
                3'b001: ns = S_WB;
                3'b010: ns = S_MEM_L;
                3'b011: ns = S_MEM_S;
                3'b100: ns = S_IF;
                3'b101: ns = S_IF;
                default: ns = cs;
            endcase
        end
        S_MEM_L : ns = (data_out_valid) ? S_IF : cs;
        S_MEM_S : ns = (write_finish)   ? S_IF : cs;
        S_WB    : ns = S_IF;
        default : ns = S_IDLE;
    endcase
end

// IO_stall
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) IO_stall <= 1;
    else begin
        if (ns == S_IF && cs != S_IF && cs != S_IDLE)
            IO_stall <= 0;
        else
            IO_stall <= 1;
    end
end

//###########################################
//
// IF
//
//###########################################
// Program counter (16-bit representation)
// 0x1000 ~ 0x1FFF (only pc[11:0] will be used)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          pc <= OFFST;
    else if (ns == S_EX) pc <= pc_next;
end
always @(*) begin
    if (opcode == 3'b100) begin // BOE
        if (rs_val == rt_val) pc_next = pc + 2 + imm*2;
        else                  pc_next = pc + 2;
    end
    else if (opcode == 3'b101) begin // JUMP
        pc_next = jump_addr;
    end
    else begin
        pc_next = pc + 2;
    end
end


// ########## DRAM_inst #############
// FSM_inst
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs_inst <= S_INST_IDLE;
    else        cs_inst <= ns_inst;
end
always @(*) begin
    case (cs_inst)
        S_INST_IDLE: begin
            if (inst_in_valid) begin
                if (inst_tag_valid && inst_tag == pc[11:8])
                    ns_inst = S_INST_HIT;
                else
                    ns_inst = S_INST_ADDR;
            end
            else ns_inst = S_INST_IDLE;
        end
        // DRAM
        S_INST_ADDR: ns_inst = (arready_inst) ? S_INST_DATA : cs_inst;
        S_INST_DATA: ns_inst = (rlast_inst)   ? S_INST_OUT  : cs_inst;
        // CACHE_HIT
        S_INST_HIT:  ns_inst = S_INST_LOAD;
        S_INST_LOAD: ns_inst = S_INST_OUT;
        // GET_INST
        S_INST_OUT:  ns_inst = S_INST_IDLE;
        default: ns_inst = cs_inst;
    endcase
end

assign araddr_inst  = {20'd1, pc[11:8], 8'd0};
assign arvalid_inst = (cs_inst == S_INST_ADDR);
assign rready_inst  = (cs_inst == S_INST_DATA);

SRAM_128X16 CACHE_inst (
    .CK(clk),  .WEB(cs_inst != S_INST_DATA), .OE(1'b1),  .CS(1'b1),
    // Address
    .A0(cache_inst_addr[0]),  .A1(cache_inst_addr[1]),  .A2(cache_inst_addr[2]),  .A3(cache_inst_addr[3]),  .A4(cache_inst_addr[4]),  .A5(cache_inst_addr[5]),  .A6(cache_inst_addr[6]),
    // DI
    .DI0(rdata_inst_dummy[0]),   .DI1(rdata_inst_dummy[1]),   .DI2(rdata_inst_dummy[2]),   .DI3(rdata_inst_dummy[3]),
    .DI4(rdata_inst_dummy[4]),   .DI5(rdata_inst_dummy[5]),   .DI6(rdata_inst_dummy[6]),   .DI7(rdata_inst_dummy[7]),
    .DI8(rdata_inst_dummy[8]),   .DI9(rdata_inst_dummy[9]),   .DI10(rdata_inst_dummy[10]), .DI11(rdata_inst_dummy[11]),
    .DI12(rdata_inst_dummy[12]), .DI13(rdata_inst_dummy[13]), .DI14(rdata_inst_dummy[14]), .DI15(rdata_inst_dummy[15]),
    // DO
    .DO0(cache_inst_out[0]),   .DO1(cache_inst_out[1]),   .DO2(cache_inst_out[2]),   .DO3(cache_inst_out[3]),
    .DO4(cache_inst_out[4]),   .DO5(cache_inst_out[5]),   .DO6(cache_inst_out[6]),   .DO7(cache_inst_out[7]),
    .DO8(cache_inst_out[8]),   .DO9(cache_inst_out[9]),   .DO10(cache_inst_out[10]), .DO11(cache_inst_out[11]),
    .DO12(cache_inst_out[12]), .DO13(cache_inst_out[13]), .DO14(cache_inst_out[14]), .DO15(cache_inst_out[15])  );

assign rdata_inst_dummy2 = (rvalid_inst) ? rdata_inst : 0;
assign rdata_inst_dummy1 = (rvalid_inst) ? rdata_inst_dummy2 : 0;
assign rdata_inst_dummy =  (rvalid_inst) ? rdata_inst_dummy1 : 0;

// { pc[11:8] , pc[7:1] } ... 1 command / 2 pc
// { 4'tag    , 7'cache_inst_addr }
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inst_tag <= 0;
    else begin
        if (cs_inst == S_INST_ADDR) inst_tag <= pc[11:8];
    end
end

// A valid signal that make sure to read DRAM at the first time
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inst_tag_valid <= 0;
    else begin
        if (cs_inst == S_INST_ADDR) inst_tag_valid <= 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cache_inst_addr <= 0;
    else begin
        if (ns_inst == S_INST_HIT)  cache_inst_addr <= pc[7:1];
        else if (rvalid_inst)       cache_inst_addr <= cache_inst_addr + 1'd1;
        else if (ns_inst == S_IDLE) cache_inst_addr <= 0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inst <= 0;
    else begin
        if (cs_inst == S_INST_DATA) begin
            if (rvalid_inst && cache_inst_addr == pc[7:1])
                inst <= rdata_inst;
        end
        else if (cs_inst == S_INST_LOAD) begin
            inst <= cache_inst_out;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inst_in_valid <= 0;
    else        inst_in_valid <= (ns == S_IF && cs != S_IF);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inst_out_valid <= 0;
    else        inst_out_valid <= (ns_inst == S_INST_OUT && cs_inst != S_INST_OUT);
end
// ###################################



//###########################################
//
// ID
//
//###########################################
assign opcode = inst[15:13];
assign rs     = inst[12:9];
assign rt     = inst[8:5];
assign rd     = inst[4:1];
assign func   = inst[0];
assign imm    = inst[4:0];
assign jump_addr = {3'd0, inst[12:0]};

// Reg File
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rs_val <= 0;
    else begin
        if (ns == S_ID) begin
            case (rs)
                0:  rs_val <= core_r0;     1:  rs_val <= core_r1;     2:  rs_val <= core_r2;     3:  rs_val <= core_r3;
                4:  rs_val <= core_r4;     5:  rs_val <= core_r5;     6:  rs_val <= core_r6;     7:  rs_val <= core_r7;
                8:  rs_val <= core_r8;     9:  rs_val <= core_r9;     10: rs_val <= core_r10;    11: rs_val <= core_r11;
                12: rs_val <= core_r12;    13: rs_val <= core_r13;    14: rs_val <= core_r14;    15: rs_val <= core_r15;
                default: rs_val <= 0;
            endcase
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rt_val <= 0;
    else begin
        if (ns == S_ID) begin
            case (rt)
                0:  rt_val <= core_r0;     1:  rt_val <= core_r1;     2:  rt_val <= core_r2;     3:  rt_val <= core_r3;
                4:  rt_val <= core_r4;     5:  rt_val <= core_r5;     6:  rt_val <= core_r6;     7:  rt_val <= core_r7;
                8:  rt_val <= core_r8;     9:  rt_val <= core_r9;     10: rt_val <= core_r10;    11: rt_val <= core_r11;
                12: rt_val <= core_r12;    13: rt_val <= core_r13;    14: rt_val <= core_r14;    15: rt_val <= core_r15;
                default: rt_val <= 0;
            endcase
        end
    end
end




//###########################################
//
// EXE
//
//###########################################
// ALU
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rd_val <= 0;
    else begin
        if (ns == S_EX) begin
            case (opcode)
                3'b000: begin
                    if (func) rd_val <= rs_val - rt_val;
                    else      rd_val <= rs_val + rt_val;
                end
                3'b001: begin
                    if (func) rd_val <= rs_val * rt_val;
                    else begin
                        if (rs_val < rt_val) rd_val <= 1;
                        else                 rd_val <= 0;
                    end
                end
            endcase
        end
    end
end

//###########################################
//
// MEM
//
//###########################################
// ###################################
// ########## DRAM_data_r ############
// ###################################
// FSM_data
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs_data <= S_DATA_IDLE;
    else        cs_data <= ns_data;
end
always @(*) begin
    case (cs_data)
        S_DATA_IDLE: begin
            if (data_in_valid) begin
                if (write_valid && data_tag == DM_addr[11:8])
                    ns_data = S_DATA_STORE;
                else if (data_tag_valid && data_tag == DM_addr[11:8])
                    ns_data = S_DATA_HIT;
                else if (!write_valid)
                    ns_data = S_DATA_ADDR;
                else
                    ns_data = S_DATA_IDLE;
            end
            else ns_data = S_DATA_IDLE;
        end
        // STORE_SRAM_SIMULTANIOUSLY
        S_DATA_STORE: ns_data = S_DATA_IDLE;
        // DRAM
        S_DATA_ADDR:  ns_data = (arready_data) ? S_DATA_DATA : cs_data;
        S_DATA_DATA:  ns_data = (rlast_data)   ? S_DATA_OUT  : cs_data;
        // CACHE_HIT
        S_DATA_HIT:   ns_data = S_DATA_LOAD;
        S_DATA_LOAD:  ns_data = S_DATA_OUT;
        // GET_DATA
        S_DATA_OUT:   ns_data = S_DATA_IDLE;
        default: ns_data = cs_data;
    endcase
end

assign DM_addr = (rs_val + imm)*2 + OFFST;
assign araddr_data  = {20'd1, DM_addr[11:8], 8'd0};
assign arvalid_data = (cs_data == S_DATA_ADDR);
assign rready_data  = (cs_data == S_DATA_DATA);
assign cache_data   = (cs_data == S_DATA_STORE) ? wdata_m_inf : rdata_data;

SRAM_128X16 CACHE_data (
    .CK(clk),  .WEB(~(cs_data == S_DATA_DATA || cs_data == S_DATA_STORE)), .OE(1'b1),  .CS(1'b1),
    // Address
    .A0(cache_data_addr[0]),  .A1(cache_data_addr[1]),  .A2(cache_data_addr[2]),  .A3(cache_data_addr[3]),  .A4(cache_data_addr[4]),  .A5(cache_data_addr[5]),  .A6(cache_data_addr[6]),
    // DI
    .DI0(cache_data[0]),   .DI1(cache_data[1]),   .DI2(cache_data[2]),   .DI3(cache_data[3]),
    .DI4(cache_data[4]),   .DI5(cache_data[5]),   .DI6(cache_data[6]),   .DI7(cache_data[7]),
    .DI8(cache_data[8]),   .DI9(cache_data[9]),   .DI10(cache_data[10]), .DI11(cache_data[11]),
    .DI12(cache_data[12]), .DI13(cache_data[13]), .DI14(cache_data[14]), .DI15(cache_data[15]),
    // DO
    .DO0(cache_data_out[0]),   .DO1(cache_data_out[1]),   .DO2(cache_data_out[2]),   .DO3(cache_data_out[3]),
    .DO4(cache_data_out[4]),   .DO5(cache_data_out[5]),   .DO6(cache_data_out[6]),   .DO7(cache_data_out[7]),
    .DO8(cache_data_out[8]),   .DO9(cache_data_out[9]),   .DO10(cache_data_out[10]), .DO11(cache_data_out[11]),
    .DO12(cache_data_out[12]), .DO13(cache_data_out[13]), .DO14(cache_data_out[14]), .DO15(cache_data_out[15])  );

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data_tag <= 0;
    else begin
        if (cs_data == S_DATA_ADDR) data_tag <= DM_addr[11:8];
    end
end

// A valid signal that make sure to read DRAM at the first time
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data_tag_valid <= 0;
    else begin
        if (cs_data == S_DATA_ADDR) data_tag_valid <= 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cache_data_addr <= 0;
    else begin
        if (ns_data == S_DATA_HIT)         cache_data_addr <= DM_addr[7:1];
        else if (ns_data == S_DATA_STORE)  cache_data_addr <= DM_addr[7:1];
        else if (rvalid_data)              cache_data_addr <= cache_data_addr + 1'd1;
        else if (ns_data == S_IDLE)        cache_data_addr <= 0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data <= 0;
    else begin
        if (cs_data == S_DATA_DATA) begin
            if (rvalid_data && cache_data_addr == DM_addr[7:1])
                data <= rdata_data;
        end
        else if (cs_data == S_DATA_LOAD) begin
            data <= cache_data_out;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data_in_valid <= 0;
    else        data_in_valid <= (ns == S_MEM_L && cs != S_MEM_L) || (ns == S_MEM_S && cs != S_MEM_S);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data_out_valid <= 0;
    else        data_out_valid <= (ns_data == S_DATA_OUT && cs_data != S_DATA_OUT);
end


// ###################################
// ########## DRAM_data_w ############
// ###################################
// FSM_dataw
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs_dataw <= S_DATAW_IDLE;
    else        cs_dataw <= ns_dataw;
end
always @(*) begin
    case(cs_dataw)
        S_DATAW_IDLE: ns_dataw = (write_valid)    ? S_DATAW_ADDR : cs_dataw;
        S_DATAW_ADDR: ns_dataw = (awready_m_inf)  ? S_DATAW_DATA : cs_dataw;
        S_DATAW_DATA: ns_dataw = (wready_m_inf && wlast_m_inf) ? S_DATAW_RESP : cs_dataw;
        S_DATAW_RESP: ns_dataw = (bvalid_m_inf)   ? S_DATAW_IDLE : cs_dataw;
        default: ns_dataw = S_DATAW_IDLE;
    endcase
end

assign awaddr_m_inf = {20'd1, DM_addr[11:1], 1'd0};
assign wdata_m_inf = rt_val;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) awvalid_m_inf <= 0;
    else        awvalid_m_inf <= (ns_dataw == S_DATAW_ADDR);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wvalid_m_inf <= 0;
    else        wvalid_m_inf <= (ns_dataw == S_DATAW_DATA);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wlast_m_inf <= 0;
    else        wlast_m_inf <= (ns_dataw == S_DATAW_DATA);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) bready_m_inf <= 0;
    else        bready_m_inf <= (ns_dataw == S_DATAW_RESP);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) write_valid <= 0;
    else        write_valid <= (ns == S_MEM_S && cs != S_MEM_S);
end

assign write_finish = bvalid_m_inf;

//###########################################
//
// WB
//
//###########################################
// core_r0 ~ 15
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        core_r0  <= 0;   core_r1  <= 0;   core_r2  <= 0;    core_r3  <= 0;
        core_r4  <= 0;   core_r5  <= 0;   core_r6  <= 0;    core_r7  <= 0;
        core_r8  <= 0;   core_r9  <= 0;   core_r10 <= 0;    core_r11 <= 0;
        core_r12 <= 0;   core_r13 <= 0;   core_r14 <= 0;    core_r15 <= 0;
    end
    else begin
        if (ns == S_WB) begin
            case (rd)
                0:  core_r0  <= rd_val;    1:  core_r1  <= rd_val;    2:  core_r2  <= rd_val;    3:  core_r3  <= rd_val;
                4:  core_r4  <= rd_val;    5:  core_r5  <= rd_val;    6:  core_r6  <= rd_val;    7:  core_r7  <= rd_val;
                8:  core_r8  <= rd_val;    9:  core_r9  <= rd_val;    10: core_r10 <= rd_val;    11: core_r11 <= rd_val;
                12: core_r12 <= rd_val;    13: core_r13 <= rd_val;    14: core_r14 <= rd_val;    15: core_r15 <= rd_val;
            endcase
        end
        else if (data_out_valid) begin
            case (rt)
                0:  core_r0  <= data;    1:  core_r1  <= data;    2:  core_r2  <= data;    3:  core_r3  <= data;
                4:  core_r4  <= data;    5:  core_r5  <= data;    6:  core_r6  <= data;    7:  core_r7  <= data;
                8:  core_r8  <= data;    9:  core_r9  <= data;    10: core_r10 <= data;    11: core_r11 <= data;
                12: core_r12 <= data;    13: core_r13 <= data;    14: core_r14 <= data;    15: core_r15 <= data;
            endcase
        end
    end
end

endmodule