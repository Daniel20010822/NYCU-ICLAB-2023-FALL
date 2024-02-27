module CLK_1_MODULE (
    clk,
    rst_n,
    in_valid,
    seed_in,
    out_idle,
    out_valid,
    seed_out,

    clk1_handshake_flag1,
    clk1_handshake_flag2,
    clk1_handshake_flag3,
    clk1_handshake_flag4
);

input        clk;
input        rst_n;
input        in_valid;
input [31:0] seed_in;
input        out_idle; // maybe for reseting seed_in ?
output reg   out_valid;
output reg [31:0] seed_out;

// You can change the input / output of the custom flag ports
input  clk1_handshake_flag1;
input  clk1_handshake_flag2;
output clk1_handshake_flag3;
output clk1_handshake_flag4;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) seed_out <= 'b0;
    else begin
        if (in_valid && out_idle)
            seed_out <= seed_in;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) out_valid <= 'b0;
    else begin
        if (in_valid)
            out_valid <= 'b1;
        else
            out_valid <= 'b0;
    end
end


endmodule

module CLK_2_MODULE (
    clk,
    rst_n,
    in_valid,
    fifo_full,
    seed,
    out_valid,
    rand_num,
    busy,

    handshake_clk2_flag1,
    handshake_clk2_flag2,
    handshake_clk2_flag3,
    handshake_clk2_flag4,

    clk2_fifo_flag1,
    clk2_fifo_flag2,
    clk2_fifo_flag3,
    clk2_fifo_flag4
);

input clk;
input rst_n;
input in_valid;
input fifo_full;
input [31:0] seed;
output out_valid;
output [31:0] rand_num;
output busy;

// You can change the input / output of the custom flag ports
input handshake_clk2_flag1;
input handshake_clk2_flag2;
output handshake_clk2_flag3;
output handshake_clk2_flag4;

input clk2_fifo_flag1;
input clk2_fifo_flag2;
output clk2_fifo_flag3;
output clk2_fifo_flag4;

wire [31:0] seed_stage1, seed_stage2;
reg [31:0] cur_seed_r;
reg [7:0] cnt_256_r, cnt_256_syn;
reg cnt_en;
reg cnt_en_syn;
reg full_r;

// always @(posedge clk or negedge rst_n) begin
//     if (~rst_n) full_r <= 0;
//     else        full_r <= winc & fifo_full;
// end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cur_seed_r <= 0;
    else begin
        if (cnt_en)
        // if (cnt_en_syn)
            cur_seed_r <= (fifo_full) ? cur_seed_r : rand_num;
        else
            cur_seed_r <= seed;
    end
end

assign seed_stage1 = cur_seed_r  ^ (cur_seed_r  << 13);
assign seed_stage2 = seed_stage1 ^ (seed_stage1 >> 17);
assign rand_num    = seed_stage2 ^ (seed_stage2 <<  5);

// assign busy      = (cnt_en);
// assign out_valid = (cnt_en && ~full_r);
assign busy      = (cnt_en);
assign out_valid = (cnt_en && ~fifo_full);
// assign busy      = (cnt_en_syn);
// assign out_valid = (cnt_en_syn && ~fifo_full);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cnt_en <= 0;
    else begin
        if (in_valid) cnt_en <= 1;
        else if (&cnt_256_syn && ~fifo_full) cnt_en <= 0;
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cnt_256_r <= 0;
    else begin
        if (cnt_en && ~fifo_full)
        // if (cnt_en_syn && ~fifo_full)
            cnt_256_r <= cnt_256_r + 1'd1;
    end
end

NDFF_BUS_syn #(8) cnt_256_synchronizer (.D(cnt_256_r), .Q(cnt_256_syn), .clk(clk), .rst_n(rst_n));

// NDFF_syn cnt_en_synchronizer (.D(cnt_en), .Q(cnt_en_syn), .clk(clk), .rst_n(rst_n));

endmodule

module CLK_3_MODULE (
    clk,
    rst_n,
    fifo_empty,
    fifo_rdata,
    fifo_rinc,
    out_valid,
    rand_num,

    fifo_clk3_flag1,
    fifo_clk3_flag2,
    fifo_clk3_flag3,
    fifo_clk3_flag4
);

input clk;
input rst_n;
input fifo_empty;
input [31:0] fifo_rdata;
output reg fifo_rinc;
// output fifo_rinc;
output reg out_valid;
output reg [31:0] rand_num;
reg [8:0] out_cnt256_r;
reg [1:0] cs_clk3, ns_clk3;
parameter CLK3_IDLE     = 2'd0;
parameter CLK3_OUT256   = 2'd1;
parameter CLK3_WAITLOW  = 2'd2;


// You can change the input / output of the custom flag ports
input fifo_clk3_flag1;
input fifo_clk3_flag2;
output fifo_clk3_flag3;
output fifo_clk3_flag4;

reg out_temp;
reg out_valid_r;



always @(*) fifo_rinc = ~fifo_empty;
// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) fifo_rinc <= 'b0;
//     else        fifo_rinc <= ~fifo_empty;
// end

// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         out_temp  <= 'b0;
//         out_valid <= 'b0;
//     end
//     else begin
//         out_temp  <= fifo_rinc;
//         out_valid <= out_temp;
//     end
// end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_temp  <= 'b0;
        out_valid_r <= 'b0;
    end
    else begin
        out_temp  <= fifo_rinc;
        out_valid_r <= out_temp;
    end
end
always @(*) begin
    out_valid = out_valid_r & (cs_clk3 == CLK3_OUT256);
end


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cs_clk3 <= CLK3_IDLE;
    else        cs_clk3 <= ns_clk3;
end
always @(*) begin
    case(cs_clk3)
        CLK3_IDLE:
            ns_clk3 = (fifo_rinc) ? CLK3_OUT256 : CLK3_IDLE;
        CLK3_OUT256:
            ns_clk3 = (out_cnt256_r == 257) ? CLK3_WAITLOW : CLK3_OUT256;
        CLK3_WAITLOW:
            ns_clk3 = (~fifo_rinc) ? CLK3_IDLE : CLK3_WAITLOW;
        default: ns_clk3 = cs_clk3;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) out_cnt256_r <= 0;
    else begin
        if (fifo_rinc)       out_cnt256_r <= out_cnt256_r + fifo_rinc;
        else                 out_cnt256_r <= 0;
    end
end

always @(*) begin
    if (out_valid) rand_num = fifo_rdata;
    else           rand_num = 'd0;
end

endmodule