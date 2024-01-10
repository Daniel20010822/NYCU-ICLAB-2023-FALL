module Handshake_syn #(parameter WIDTH=32) (
    sclk,
    dclk,
    rst_n,
    sready,
    din,
    dbusy,
    sidle,
    dvalid,
    dout,

    clk1_handshake_flag1,
    clk1_handshake_flag2,
    clk1_handshake_flag3,
    clk1_handshake_flag4,

    handshake_clk2_flag1,
    handshake_clk2_flag2,
    handshake_clk2_flag3,
    handshake_clk2_flag4
);

input sclk, dclk;
input rst_n;
input sready; // CLK1 out_valid
input [WIDTH-1:0] din; // CLK1
input dbusy; // CLK2
output sidle; // CLK1
output reg dvalid;
output reg [WIDTH-1:0] dout;

// You can change the input / output of the custom flag ports
input  clk1_handshake_flag1;
input  clk1_handshake_flag2;
output clk1_handshake_flag3;
output clk1_handshake_flag4;

input  handshake_clk2_flag1;
input  handshake_clk2_flag2;
output handshake_clk2_flag3;
output handshake_clk2_flag4;

// Remember:
//   Don't modify the signal name
reg  sreq;
wire dreq;
reg  dack;
wire sack;

reg [WIDTH-1:0] data;
reg [2:0] cnt_5_r;
reg [1:0] src_cs, src_ns;
parameter SRC_IDLE          = 2'd0;
parameter SRC_WAIT_SACK     = 2'd1;
parameter SRC_WAIT_SACK_LOW = 2'd2;
reg [1:0] dst_cs, dst_ns;
parameter DST_IDLE          = 2'd0;
parameter DST_DACK_HIGH     = 2'd1;
parameter DST_WAIT_5_CYCLE  = 2'd2;


//================================
// SCLK Part
//================================
always @(posedge sclk or negedge rst_n) begin
    if (!rst_n) data <= 0;
    else begin
        // if (~sreq && sready)
        if (src_cs == SRC_IDLE && sready)
            data <= din;
    end
end

always @(posedge sclk or negedge rst_n) begin
    if (!rst_n) src_cs <= SRC_IDLE;
    else        src_cs <= src_ns;
end
always @(*) begin
    case(src_cs)
        SRC_IDLE:           
            src_ns = (sready) ? SRC_WAIT_SACK : SRC_IDLE;
        SRC_WAIT_SACK:      
            src_ns = (sack)   ? SRC_WAIT_SACK_LOW : SRC_WAIT_SACK;
        SRC_WAIT_SACK_LOW:  
            src_ns = (~sack)  ? SRC_IDLE : SRC_WAIT_SACK_LOW;
        default: src_ns = src_cs;
    endcase
end

always @(posedge sclk or negedge rst_n) begin
    if (!rst_n) sreq <= 'd0;
    else begin
       if (sready && src_cs == SRC_IDLE)         sreq <= 'd1;
       else if (sack && src_cs == SRC_WAIT_SACK) sreq <= 'd0;
    end       
end


NDFF_syn dclk_NDFF_syn (.D(sreq), .Q(dreq), .clk(dclk), .rst_n(rst_n));

// assign sidle = ~(sreq | sack);
assign sidle = (src_cs == SRC_IDLE);


//================================
// DCLK Part
//================================
always @(posedge dclk or negedge rst_n) begin
    if (!rst_n) dout <= 0;
    else begin
        if (dreq && ~dack && ~dbusy)
            dout <= data;
    end
end

always @(posedge dclk or negedge rst_n) begin
    if (!rst_n) dst_cs <= DST_IDLE;
    else        dst_cs <= dst_ns;
end
always @(*) begin
    case(dst_cs)
        DST_IDLE:
            dst_ns = (dreq && !dbusy) ? DST_DACK_HIGH : DST_IDLE;
        DST_DACK_HIGH:
            dst_ns = (~dreq) ? DST_WAIT_5_CYCLE : DST_DACK_HIGH;
        DST_WAIT_5_CYCLE:
            dst_ns = (cnt_5_r == 4) ? DST_IDLE : DST_WAIT_5_CYCLE;

        default: dst_ns = dst_cs;
    endcase
end

always @(posedge dclk or negedge rst_n) begin
    if (!rst_n)  dack <= 'd0;
    else begin
        if (dreq && !dbusy && dst_cs == DST_IDLE)  
            dack <= (dreq);
        else if (~dreq && dst_cs == DST_DACK_HIGH)
            dack <= (dreq);
        
    end        
end

always @(posedge dclk or negedge rst_n) begin
    if (!rst_n) cnt_5_r <= 0;
    else begin
        if (dst_cs == DST_WAIT_5_CYCLE)  cnt_5_r <= cnt_5_r + 1'd1;
        else                             cnt_5_r <= 0;
    end
end

NDFF_syn sclk_NDFF_syn (.D(dack), .Q(sack), .clk(sclk), .rst_n(rst_n));

always @(posedge dclk or negedge rst_n) begin
    if (!rst_n)  dvalid <= 'd0;
    else begin
        if      (dreq && ~dack && ~dbusy) dvalid <= 'd1;
        else if (dvalid)                  dvalid <= 'd0;
        // if      (sreq && sack) dvalid <= 'd1;
        // else if (dvalid)       dvalid <= 'd0;
    end
end



endmodule