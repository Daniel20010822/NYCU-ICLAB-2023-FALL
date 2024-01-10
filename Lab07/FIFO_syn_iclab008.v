module FIFO_syn #(parameter WIDTH=32, parameter WORDS=64) (
    wclk,
    rclk,
    rst_n,
    winc,
    wdata,
    wfull,
    rinc,
    rdata,
    rempty,

    clk2_fifo_flag1,
    clk2_fifo_flag2,
    clk2_fifo_flag3, // finish_write_flag
    clk2_fifo_flag4,

    fifo_clk3_flag1,
    fifo_clk3_flag2,
    fifo_clk3_flag3,
    fifo_clk3_flag4
);

input wclk, rclk;
input rst_n;
input winc;
input [WIDTH-1:0] wdata;
output reg wfull;
input rinc;
output reg [WIDTH-1:0] rdata;
output reg rempty;

// You can change the input / output of the custom flag ports
input clk2_fifo_flag1;
input clk2_fifo_flag2;
output clk2_fifo_flag3;
output clk2_fifo_flag4;

input fifo_clk3_flag1;
input fifo_clk3_flag2;
output fifo_clk3_flag3;
output fifo_clk3_flag4;

wire [WIDTH-1:0] rdata_q;

// Remember:
//   wptr and rptr should be gray coded
//   Don't modify the signal name
reg [$clog2(WORDS):0] wptr;
reg [$clog2(WORDS):0] rptr;
reg [$clog2(WORDS):0] rq2_wptr;
reg [$clog2(WORDS):0] wq2_rptr;
reg [$clog2(WORDS)+1:0] waddr_r;
reg [$clog2(WORDS)+1:0] next_waddr;
reg [$clog2(WORDS):0] raddr_r;
reg [WIDTH-1:0] wdata_sram;

reg WEAN, CSA, OEA;
reg WEBN, CSB, OEB;
reg rinc_r;
reg full_r;
wire full;
parameter WRITE = 3'b011;
parameter READ  = 3'b111;
parameter HIGHZ = 3'b000;

// rdata
//  Add one more register stage to rdata
always @(posedge rclk) begin
    if (rinc_r)
        rdata <= rdata_q;
end

always @(posedge rclk) begin
    rinc_r <= rinc;
end
//====================================
// Write Control
//====================================
always @(posedge wclk or negedge rst_n) begin
    if (!rst_n)  waddr_r <= 0;
    else begin
        // if (!wfull && winc) waddr_r <= waddr_r + 1'd1;
        // else                waddr_r <= waddr_r;
        waddr_r <= next_waddr;
    end
end

assign next_waddr = (!wfull && winc) ? waddr_r + 1'd1 : waddr_r;

// finish_write_flag
assign clk2_fifo_flag3 = (&waddr_r && ~wfull);

/*
always @(*) begin
    wptr[6] = waddr_r[6];
    wptr[5] = waddr_r[5] ^ waddr_r[6];
    wptr[4] = waddr_r[4] ^ waddr_r[5];
    wptr[3] = waddr_r[3] ^ waddr_r[4];
    wptr[2] = waddr_r[2] ^ waddr_r[3];
    wptr[1] = waddr_r[1] ^ waddr_r[2];
    wptr[0] = waddr_r[0] ^ waddr_r[1];
end

NDFF_BUS_syn #(7) sync_w2r (.D(wptr[6:0]), .Q(wq2_rptr), .clk(rclk), .rst_n(rst_n));

assign wfull = (wptr[6:5] == ~rq2_wptr[6:5] && wptr[4:0] == rq2_wptr[4:0]);
*/

always @(*) begin
    wptr[6] = next_waddr[6];
    wptr[5] = next_waddr[5] ^ next_waddr[6];
    wptr[4] = next_waddr[4] ^ next_waddr[5];
    wptr[3] = next_waddr[3] ^ next_waddr[4];
    wptr[2] = next_waddr[2] ^ next_waddr[3];
    wptr[1] = next_waddr[1] ^ next_waddr[2];
    wptr[0] = next_waddr[0] ^ next_waddr[1];
end

NDFF_BUS_syn #(7) sync_w2r (.D(wptr[6:0]), .Q(wq2_rptr), .clk(rclk), .rst_n(rst_n));

always @(posedge wclk or negedge rst_n) begin
    if (!rst_n) full_r <= 0;
    else        full_r <= full;
end

assign full = (wptr[6:5] == ~rq2_wptr[6:5] && wptr[4:0] == rq2_wptr[4:0]);
assign wfull = full_r;


always @(*) begin
    if (winc) begin
        if (wfull)  begin WEAN = 1'b1; CSA = 1'b1; OEA = 1'b1; end //{WEAN, CSA, OEA} = READ;
        else        begin WEAN = 1'b0; CSA = 1'b1; OEA = 1'b1; end //{WEAN, CSA, OEA} = WRITE;
    end
    else begin
        WEAN = 1'b0; CSA = 1'b0; OEA = 1'b0; //{WEAN, CSA, OEA} = HIGHZ;
    end
end

//====================================
// Read Control
//====================================
always @(posedge rclk or negedge rst_n) begin
    if (!rst_n)  raddr_r <= 0;
    else begin
        if (!rempty)            raddr_r <= raddr_r + 1'd1;
        else                    raddr_r <= raddr_r;
    end
end

always @(*) begin
    rptr[6] = raddr_r[6];
    rptr[5] = raddr_r[5] ^ raddr_r[6];
    rptr[4] = raddr_r[4] ^ raddr_r[5];
    rptr[3] = raddr_r[3] ^ raddr_r[4];
    rptr[2] = raddr_r[2] ^ raddr_r[3];
    rptr[1] = raddr_r[1] ^ raddr_r[2];
    rptr[0] = raddr_r[0] ^ raddr_r[1];
end

NDFF_BUS_syn #(7) sync_r2w (.D(rptr), .Q(rq2_wptr), .clk(wclk), .rst_n(rst_n));

always @(*) begin
    if (rinc | rinc_r)    begin WEBN = 1'b1; CSB = 1'b1; OEB = 1'b1; end //{WEBN, CSB, OEB} = READ;
    else                  begin WEBN = 1'b0; CSB = 1'b0; OEB = 1'b0; end //{WEBN, CSB, OEB} = HIGHZ;
end
// always @(*) begin
//     if (rinc | rinc_r) begin
//         if   (waddr_r == raddr_r) begin WEBN = 1'b0; CSB = 1'b0; OEB = 1'b0; end //{WEBN, CSB, OEB} = STANDBY;
//         else                      begin WEBN = 1'b1; CSB = 1'b1; OEB = 1'b1; end //{WEBN, CSB, OEB} = READ;
//     end
//     else      begin WEBN = 1'b0; CSB = 1'b0; OEB = 1'b0; end //{WEBN, CSB, OEB} = HIGHZ;
// end

assign rempty = (rptr == wq2_rptr);


DUAL_64X32X1BM1 u_dual_sram (
    .CKA(wclk),
    .CKB(rclk),
    .WEAN(WEAN),
    .WEBN(WEBN),
    .CSA(CSA),
    .CSB(CSB),
    .OEA(OEA),
    .OEB(OEB),
    .A0(waddr_r[0]),
    .A1(waddr_r[1]),
    .A2(waddr_r[2]),
    .A3(waddr_r[3]),
    .A4(waddr_r[4]),
    .A5(waddr_r[5]),
    .B0(raddr_r[0]),
    .B1(raddr_r[1]),
    .B2(raddr_r[2]),
    .B3(raddr_r[3]),
    .B4(raddr_r[4]),
    .B5(raddr_r[5]),
    .DIA0 (wdata[ 0]),
    .DIA1 (wdata[ 1]),
    .DIA2 (wdata[ 2]),
    .DIA3 (wdata[ 3]),
    .DIA4 (wdata[ 4]),
    .DIA5 (wdata[ 5]),
    .DIA6 (wdata[ 6]),
    .DIA7 (wdata[ 7]),
    .DIA8 (wdata[ 8]),
    .DIA9 (wdata[ 9]),
    .DIA10(wdata[10]),
    .DIA11(wdata[11]),
    .DIA12(wdata[12]),
    .DIA13(wdata[13]),
    .DIA14(wdata[14]),
    .DIA15(wdata[15]),
    .DIA16(wdata[16]),
    .DIA17(wdata[17]),
    .DIA18(wdata[18]),
    .DIA19(wdata[19]),
    .DIA20(wdata[20]),
    .DIA21(wdata[21]),
    .DIA22(wdata[22]),
    .DIA23(wdata[23]),
    .DIA24(wdata[24]),
    .DIA25(wdata[25]),
    .DIA26(wdata[26]),
    .DIA27(wdata[27]),
    .DIA28(wdata[28]),
    .DIA29(wdata[29]),
    .DIA30(wdata[30]),
    .DIA31(wdata[31]),
    .DIB0(),
    .DIB1(),
    .DIB2(),
    .DIB3(),
    .DIB4(),
    .DIB5(),
    .DIB6(),
    .DIB7(),
    .DIB8(),
    .DIB9(),
    .DIB10(),
    .DIB11(),
    .DIB12(),
    .DIB13(),
    .DIB14(),
    .DIB15(),
    .DIB16(),
    .DIB17(),
    .DIB18(),
    .DIB19(),
    .DIB20(),
    .DIB21(),
    .DIB22(),
    .DIB23(),
    .DIB24(),
    .DIB25(),
    .DIB26(),
    .DIB27(),
    .DIB28(),
    .DIB29(),
    .DIB30(),
    .DIB31(),
    .DOB0(rdata_q[0]),
    .DOB1(rdata_q[1]),
    .DOB2(rdata_q[2]),
    .DOB3(rdata_q[3]),
    .DOB4(rdata_q[4]),
    .DOB5(rdata_q[5]),
    .DOB6(rdata_q[6]),
    .DOB7(rdata_q[7]),
    .DOB8(rdata_q[8]),
    .DOB9(rdata_q[9]),
    .DOB10(rdata_q[10]),
    .DOB11(rdata_q[11]),
    .DOB12(rdata_q[12]),
    .DOB13(rdata_q[13]),
    .DOB14(rdata_q[14]),
    .DOB15(rdata_q[15]),
    .DOB16(rdata_q[16]),
    .DOB17(rdata_q[17]),
    .DOB18(rdata_q[18]),
    .DOB19(rdata_q[19]),
    .DOB20(rdata_q[20]),
    .DOB21(rdata_q[21]),
    .DOB22(rdata_q[22]),
    .DOB23(rdata_q[23]),
    .DOB24(rdata_q[24]),
    .DOB25(rdata_q[25]),
    .DOB26(rdata_q[26]),
    .DOB27(rdata_q[27]),
    .DOB28(rdata_q[28]),
    .DOB29(rdata_q[29]),
    .DOB30(rdata_q[30]),
    .DOB31(rdata_q[31])
);


endmodule
