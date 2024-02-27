//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   2023 ICLAB Fall Course
//   Lab03      : BRIDGE
//   Author     : Ting-Yu Chang
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : BRIDGE_encrypted.v
//   Module Name : BRIDGE
//   Release version : v1.0 (Release Date: Sep-2023)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

`define STATE_BITS 6

module BRIDGE(
    // Input Signals
    clk,
    rst_n,
    in_valid,
    direction,
    addr_dram,
    addr_sd,
    // Output Signals
    out_valid,
    out_data,
    // DRAM Signals
    AR_VALID, AR_ADDR, R_READY, AW_VALID, AW_ADDR, W_VALID, W_DATA, B_READY,
    AR_READY, R_VALID, R_RESP, R_DATA, AW_READY, W_READY, B_VALID, B_RESP,
    // SD Signals
    MISO,
    MOSI
);

// Input Signals
input clk, rst_n;
input in_valid;
input direction;
input [12:0] addr_dram;
input [15:0] addr_sd;

// Output Signals
output reg out_valid;
output reg [7:0] out_data;

// DRAM Signals
// write address channel
output reg [31:0] AW_ADDR;
output reg AW_VALID;
input AW_READY;
// write data channel
output reg W_VALID;
output reg [63:0] W_DATA;
input W_READY;
// write response channel
input B_VALID;
input [1:0] B_RESP;
output reg B_READY;
// read address channel
output reg [31:0] AR_ADDR;
output reg AR_VALID;
input AR_READY;
// read data channel
input [63:0] R_DATA;
input R_VALID;
input [1:0] R_RESP;
output reg R_READY;

// SD Signals
input MISO;
output reg MOSI;

//==============================================//
//       parameter & integer declaration        //
//==============================================//
/* Command & Data Block Formats */
parameter PREFIX = 2'b01;
parameter START_TOKEN = 8'hfe;
parameter CMD_READ  = 6'd17;
parameter CMD_WRITE = 6'd24;
parameter END_BIT = 1'b1;
parameter UNIT = 8;
/* States */
parameter IDLE  = `STATE_BITS'd0;
parameter FETCH = `STATE_BITS'd1;
// DRAM -> SD
parameter DRAM_SEND_ARADDR      = `STATE_BITS'd2;
parameter DRAM_GET_RDATA        = `STATE_BITS'd3;
parameter SD_SEND_CMD           = `STATE_BITS'd4;
parameter SD_WAIT_RESPONSE      = `STATE_BITS'd5;
parameter SD_CALIBRATE_UNITS    = `STATE_BITS'd6;
parameter SD_SEND_DATA          = `STATE_BITS'd7;
parameter SD_WAIT_DATA_RESPONSE = `STATE_BITS'd8;
parameter SD_WAIT_BUSY          = `STATE_BITS'd9;
// SD -> DRAM
parameter SD_WAIT_DATA          = `STATE_BITS'd16;
parameter SD_GATHER_DATA        = `STATE_BITS'd11;
parameter DRAM_SEND_AWADDR      = `STATE_BITS'd12;
parameter DRAM_WRITE_WDATA      = `STATE_BITS'd13;
parameter DRAM_WAIT_RESPOND     = `STATE_BITS'd14;
parameter DRAM_GAP              = `STATE_BITS'd15;
// OUTPUT
parameter OUTPUT                = `STATE_BITS'd10;


//==============================================//
//           reg & wire declaration             //
//==============================================//
reg [`STATE_BITS-1:0] c_state, n_state;
reg [87:0] data_block_r;
reg [47:0] cmd_block_r;
reg [12:0] addr_dram_r;
reg [15:0] addr_sd_r;
reg [6:0]  data_counter_r;
reg [5:0]  cmd_counter_r;
reg [3:0]  unit_calib_r;
reg        direction_r;

//==============================================//
//                    FSM                       //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        c_state <= IDLE;
    else
        c_state <= n_state;
end
always @(*) begin
    case(c_state)
        IDLE: n_state = FETCH;

        FETCH: begin
            if (in_valid) begin
                if (direction) begin /* Direction = 1 (SD -> DRAM) */
                    n_state = SD_SEND_CMD;
                end
                else begin /* Direction = 0 (DRAM -> SD) */
                    n_state = DRAM_SEND_ARADDR;
                end
            end
            else begin
                n_state = FETCH;
            end
        end

        /* Direction = 0 (>DRAM< -> SD) */
        DRAM_SEND_ARADDR:      n_state = (AR_READY) ? DRAM_GET_RDATA : DRAM_SEND_ARADDR;
        DRAM_GET_RDATA:        n_state = (R_VALID) ? SD_SEND_CMD : DRAM_GET_RDATA;

        /* Shared states */
        SD_SEND_CMD:           n_state = (cmd_counter_r == 6'd47) ? SD_WAIT_RESPONSE : SD_SEND_CMD;
        SD_WAIT_RESPONSE:      n_state = (~MISO) ? SD_CALIBRATE_UNITS : SD_WAIT_RESPONSE;
        SD_CALIBRATE_UNITS:    n_state = (unit_calib_r == 4'd14) ? ((direction_r) ? SD_WAIT_DATA : SD_SEND_DATA) : SD_CALIBRATE_UNITS;

        /* Direction = 1 (>SD< -> DRAM) */
        SD_WAIT_DATA:          n_state = (~MISO) ? SD_GATHER_DATA : SD_WAIT_DATA;
        SD_GATHER_DATA:        n_state = (data_counter_r == 7'd79) ? DRAM_SEND_AWADDR : SD_GATHER_DATA;

        /* Direction = 0 (DRAM -> >SD<) */
        SD_SEND_DATA:          n_state = (data_counter_r == 7'd87) ? SD_WAIT_DATA_RESPONSE : SD_SEND_DATA;
        SD_WAIT_DATA_RESPONSE: n_state = (data_counter_r == 7'd95) ? SD_WAIT_BUSY : SD_WAIT_DATA_RESPONSE;
        SD_WAIT_BUSY:          n_state = (MISO) ? OUTPUT : SD_WAIT_BUSY;

        /* Direction = 1 (SD -> >DRAM<) */
        DRAM_SEND_AWADDR:      n_state = (AW_READY) ? DRAM_WRITE_WDATA : DRAM_SEND_AWADDR;
        DRAM_WRITE_WDATA:      n_state = (W_READY) ? DRAM_WAIT_RESPOND : DRAM_WRITE_WDATA;
        DRAM_WAIT_RESPOND:     n_state = (B_VALID) ? DRAM_GAP : DRAM_WAIT_RESPOND;
        DRAM_GAP:              n_state = OUTPUT;

        OUTPUT: n_state = (unit_calib_r == 4'd7) ? IDLE : OUTPUT;
        default: n_state = IDLE;
    endcase
end
//==============================================//
//                  BRIDGE                      //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        addr_dram_r <= 13'd0;
        addr_sd_r   <= 16'd0;
        direction_r <= 1'b0;
    end
    else if (in_valid) begin
        addr_dram_r <= addr_dram;
        addr_sd_r   <= addr_sd;
        direction_r <= direction;
    end
    else begin
        addr_dram_r <= addr_dram_r;
        addr_sd_r   <= addr_sd_r;
        direction_r <= direction_r;
    end
end

// Command Block
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_block_r <= 48'd0;
    end
    // If direction is SD -> DRAM, Generate READ COMMAND when value is prepared when in_valid is 1.
    else if (direction && in_valid) begin
        cmd_block_r[47:46] <= PREFIX;
        cmd_block_r[45:40] <= CMD_READ;
        cmd_block_r[39:8]  <= addr_sd;
        cmd_block_r[7:0]   <= {CRC7({PREFIX, CMD_READ, {16'd0, addr_sd}}), END_BIT};
    end
    // Otherwise generate WRITE COMMAND when received R_DATA. (DRAM -> SD)
    else if (R_VALID) begin
        cmd_block_r[47:46] <= PREFIX;
        cmd_block_r[45:40] <= CMD_WRITE;
        cmd_block_r[39:8]  <= addr_sd_r;
        cmd_block_r[7:0]   <= {CRC7({PREFIX, CMD_WRITE, {16'd0, addr_sd_r}}), END_BIT};
    end
    else if (c_state == SD_SEND_CMD) begin
        cmd_block_r <= {cmd_block_r[46:0], cmd_block_r[47]};
    end
    else if (c_state == IDLE) begin
        cmd_block_r <= 48'd0;
    end
    else begin
        cmd_block_r <= cmd_block_r;
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_counter_r <= 6'd0;
    else if (c_state == SD_SEND_CMD)
        cmd_counter_r <= cmd_counter_r + 1'd1;
    else
        cmd_counter_r <= 6'd0;
end

// Data Block
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        data_block_r <= 88'd0;
    end
    else if (R_VALID) begin
        data_block_r[87:80] <= START_TOKEN;
        data_block_r[79:16] <= R_DATA;
        data_block_r[15:0]  <= CRC16_CCITT(R_DATA);
    end
    else if (c_state == SD_GATHER_DATA) begin
        data_block_r <= {data_block_r[86:0], MISO};
    end
    else if (c_state == SD_SEND_DATA) begin
        data_block_r <= {data_block_r[86:0], data_block_r[87]};
    end
    else if (c_state == IDLE) begin
        data_block_r <= 88'd0;
    end
    else begin
        data_block_r <= data_block_r;
    end
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_counter_r <= 7'd0;
    else if (c_state == SD_SEND_DATA || c_state == SD_WAIT_DATA_RESPONSE || c_state == SD_GATHER_DATA)
        data_counter_r <= data_counter_r + 1'd1;
    else
        data_counter_r <= 7'd0;
end

// Unit calibrator counter
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        unit_calib_r <= 4'd0;
    else if (c_state == SD_CALIBRATE_UNITS || c_state == OUTPUT)
        unit_calib_r <= unit_calib_r + 1'd1;
    else
        unit_calib_r <= 4'd0;
end

// out_valid
always @(*) out_valid = (c_state == OUTPUT);
// out_data
always @(*) begin
    if (c_state == OUTPUT) begin
        case(unit_calib_r)
            4'd0: out_data = data_block_r[79:72];
            4'd1: out_data = data_block_r[71:64];
            4'd2: out_data = data_block_r[63:56];
            4'd3: out_data = data_block_r[55:48];
            4'd4: out_data = data_block_r[47:40];
            4'd5: out_data = data_block_r[39:32];
            4'd6: out_data = data_block_r[31:24];
            4'd7: out_data = data_block_r[23:16];
            default: out_data = 8'd0;
        endcase
    end
    else begin
        out_data = 8'd0;
    end
end
//==============================================//
//                     AXI                      //
//==============================================//
/* Read Channel */
// AR_ADDR
always @(*) begin
    AR_ADDR = (c_state == DRAM_SEND_ARADDR) ? {19'd0 ,addr_dram_r} : 32'd0;
end
// AR_VALID
always @(*) AR_VALID = (c_state == DRAM_SEND_ARADDR);
// R_READY
always @(*) R_READY = (c_state == DRAM_GET_RDATA);

/* Write Channel */
// AW_ADDR
always @(*) begin
    AW_ADDR = (c_state == DRAM_SEND_AWADDR) ? {19'd0 ,addr_dram_r} : 32'd0;
end
// AW_VALID
always @(*) AW_VALID = (c_state == DRAM_SEND_AWADDR);
// W_DATA
always @(*) W_DATA = (c_state == DRAM_WRITE_WDATA) ? data_block_r[79:16] : 64'd0;
// W_VALID
always @(*) W_VALID = (c_state == DRAM_WRITE_WDATA);
// B_READY
always @(*) B_READY = (c_state == DRAM_WRITE_WDATA || c_state == DRAM_WAIT_RESPOND);


//==============================================//
//                     SPI                      //
//==============================================//
always @(*) begin
    if (c_state == SD_SEND_CMD)
        MOSI = cmd_block_r[47];
    else if (c_state == SD_SEND_DATA)
        MOSI = data_block_r[87];
    else
        MOSI = 1'b1;
end

/* CRC7 */
function automatic [6:0] CRC7;  // Return 7-bit result
    input [39:0] data;  // 40-bit data input
    reg [6:0] crc;
    integer i;
    reg data_in, data_out;
    parameter polynomial = 7'h9;  // x^7 + x^3 + 1 0001001

    begin
        crc = 7'd0;
        for (i = 0; i < 40; i = i + 1) begin
            data_in = data[39-i];
            data_out = crc[6];
            crc = crc << 1;  // Shift the CRC
            if (data_in ^ data_out) begin
                crc = crc ^ polynomial;
            end
        end
        CRC7 = crc;
    end
endfunction

/* CRC16 */
function automatic [15:0] CRC16_CCITT;
    // Try to implement CRC-16-CCITT function by yourself.
    input [63:0] data;
    reg [15:0] crc;
    integer i;
    reg data_in, data_out;
    parameter polynomial = 16'b0001_0000_0010_0001; // x^16 + x^12 + x^5 + 1

    begin
        crc = 16'd0;
        for (i = 0; i < 64; i = i + 1) begin
            data_in = data[63-i];
            data_out = crc[15];
            crc = crc << 1;
            if (data_in ^ data_out) begin
                crc = crc ^ polynomial;
            end
        end
        CRC16_CCITT = crc;
    end
endfunction


//==============================================//
//                  SD & DRAM                   //
//==============================================//
pseudo_DRAM u_DRAM (
    .clk(clk),
    .rst_n(rst_n),
    // write address channel
    .AW_ADDR(AW_ADDR),
    .AW_VALID(AW_VALID),
    .AW_READY(AW_READY),
    // write data channel
    .W_VALID(W_VALID),
    .W_DATA(W_DATA),
    .W_READY(W_READY),
    // write response channel
    .B_VALID(B_VALID),
    .B_RESP(B_RESP),
    .B_READY(B_READY),
    // read address channel
    .AR_ADDR(AR_ADDR),
    .AR_VALID(AR_VALID),
    .AR_READY(AR_READY),
    // read data channel
    .R_DATA(R_DATA),
    .R_VALID(R_VALID),
    .R_RESP(R_RESP),
    .R_READY(R_READY)
);

pseudo_SD u_SD (
    .clk(clk),
    .MOSI(MOSI),
    .MISO(MISO)
);

endmodule

