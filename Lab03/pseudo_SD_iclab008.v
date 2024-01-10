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
//   File Name   : pseudo_SD.v
//   Module Name : pseudo_SD
//   Release version : v1.0 (Release Date: Sep-2023)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module pseudo_SD (
    clk,
    MOSI,
    MISO
);
/* Input Signals */
input clk;
input MOSI;
/* Output Signals */
output reg MISO;
/* parameter & integer declaration */
parameter SD_p_r = "../00_TESTBED/SD_init.dat";
parameter PREFIX = 2'b01;
parameter START_TOKEN = 8'hfe;
parameter CMD_READ  = 6'd17;
parameter CMD_WRITE = 6'd24;
parameter END_BIT = 1'b1;
parameter UNIT = 8;
integer i, t, latency;
// integer latency;

/* reg & wire declaration */
reg  [63:0]  SD [0:65535];
reg  [79:0]  data_block;
reg  [47:0]  cmd;
wire [1:0]   prefix;
wire [5:0]   action;
wire [31:0]  addr;
wire [6:0]   crc7;
wire         end_bit;
wire [63:0]  data;
wire [15:0]  crc16;

initial begin
    $readmemh(SD_p_r, SD);
    cmd = 'd0;
    data_block = 'd0;
    MISO = 'b1;
    latency = 0;
end

//////////////////////////////////////////////////////////////////////
// Write your own task here
//////////////////////////////////////////////////////////////////////
assign {prefix, action, addr, crc7, end_bit} = cmd;
assign {data, crc16} = data_block;

always @(posedge clk) begin
    latency = 0;
    if (MOSI == 'b0) begin
        /* Get command */
        get_cmd_task;
        cmd_format_check_task; // Command format checks
        addr_range_check_task; // addr range checks
        crc7_check_task;       // crc7 checks

        /* wait 0 ~ 8 units*/
        t = $urandom_range(0,8);
        repeat(t*UNIT) @(posedge clk);

        /* response */
        resopnse_task;

        if (action == CMD_READ) begin
            /* wait 1 ~ 32 units*/
            t = $urandom_range(1,32);
            repeat(t*UNIT) @(posedge clk);

            /* Throw data to host */
            throw_data_task;
        end

        if (action == CMD_WRITE) begin
            /* wait 1 ~ 32 units -> when detect zero 0xFE(11111110)*/
            while (MOSI == 'b1) begin
                latency = latency + 1;
                @(posedge clk);
            end
            if (latency % UNIT != 0) begin
                $display();
                $display("Time units is not an integer!");
                SPEC_SD_5_FAIL_task;
            end
            else if (latency <= UNIT) begin
                $display();
                $display("Should wait at least 1 unit!");
                SPEC_SD_5_FAIL_task;
            end


            /* Get data */
            get_data_task;
            crc16_check_task; // crc16 check

            /* Data response */
            data_resopnse_task;
        end
    end
end

task get_cmd_task; begin
    // Note: cmd[47] is automatically set to 0
    for (i = 46; i >= 0; i = i - 1) begin
        @(posedge clk);
        cmd[i] = MOSI;
        // $display("%d %b_%b_%b_%b_%b", i, prefix, action, addr, crc7, end_bit);
    end
end endtask

task cmd_format_check_task; begin
    // $display("Jump into cmd_format_check_task at time %10dns", $time);
    // $display("%b %b %b %b %b", prefix, action, addr, crc7, end_bit);
    if (prefix !== PREFIX) begin
        $display();
        $display("Received prefix = %2b   !=   %2b", prefix, PREFIX);
        SPEC_SD_1_FAIL_task;
    end
    if (action !== CMD_READ && action !== CMD_WRITE) begin
        $display();
        $display("Received cmd = %6b", action);
        $display("READ         = %6b", CMD_READ);
        $display("WRITE        = %6b", CMD_WRITE);
        SPEC_SD_1_FAIL_task;
    end
    if (end_bit !== END_BIT) begin
        $display();
        $display("Received end_bit = %1b    !=    1", end_bit);
        SPEC_SD_1_FAIL_task;
    end
end endtask

task addr_range_check_task; begin
    // $display("Jump into addr_check_task");
    if (addr > 'd65535) begin
        SPEC_SD_2_FAIL_task;
    end
end endtask

task crc7_check_task; begin
    reg [6:0] crc7_check;
    // $display("Jump into crc_check_task");
    crc7_check = CRC7({prefix, action, addr});
    if (crc7 !== crc7_check) begin
        $display();
        $display("Received CRC7 = %8b", crc7);
        $display(" Checked CRC7 = %8b", crc7_check);
        SPEC_SD_3_FAIL_task;
    end
end endtask

task resopnse_task; begin
    MISO = 'b0;
    repeat(8) @(posedge clk);
    MISO = 'b1;
end endtask

task get_data_task; begin
    for (i = 79; i >= 0; i = i - 1) begin
        // $display("%d %b_%b  %h", i, data_block[79:16], data_block[15:0], data_block[79:16]);
        @(posedge clk);
        data_block[i] = MOSI;
    end
end endtask

task data_resopnse_task; begin
    MISO = 'b0;
    repeat(5) @(posedge clk);
    MISO = 'b1;
    @(posedge clk);
    MISO = 'b0;
    @(posedge clk);
    MISO = 'b1;
    @(posedge clk);
    MISO = 'b0;

    /* wait 0 ~ 32 units */
    t = $urandom_range(0,32);
    repeat(t*UNIT) @(posedge clk);
    MISO = 'b1;

    /* write */
    SD[addr] = data;

end endtask

task crc16_check_task; begin
    reg [15:0] crc16_check;
    // $display("Jump into crc_check_task");
    crc16_check = CRC16_CCITT(data);
    if (crc16 !== crc16_check) begin
        $display();
        $display("Received CRC16 = %16b", crc16);
        $display(" Checked CRC16 = %16b", crc16_check);
        SPEC_SD_4_FAIL_task;
    end
end endtask

task throw_data_task; begin
    reg [87:0] data_block_out;
    data_block_out[87:80] = START_TOKEN;
    data_block_out[79:16] = SD[addr];
    data_block_out[15:0]  = CRC16_CCITT(SD[addr]);
    for (i = 87; i >= 0; i = i - 1) begin
        MISO = data_block_out[i];
        @(posedge clk);
    end
    MISO = 'b1;
end endtask

//////////////////////////////////////////////////////////////////////

task SPEC_SD_1_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC SD-1 FAIL                              *");
    $display("*                  Command format should be correct.                    *");
    $display("*                    Error message from pseudo_SD.v                     *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_SD_2_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC SD-2 FAIL                              *");
    $display("*                       addr = %6d > 65535                           *", addr);
    $display("*                    Error message from pseudo_SD.v                     *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_SD_3_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                            SPEC SD-3 FAIL                             *");
    $display("*                      Error message from pseudo_SD.v                   *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_SD_4_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                            SPEC SD-4 FAIL                             *");
    $display("*                      Error message from pseudo_SD.v                   *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_SD_5_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                            SPEC SD-5 FAIL                             *");
    $display("*           Time between each transmission should be correct.           *");
    $display("*                      Error message from pseudo_SD.v                   *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task YOU_FAIL_task; begin
    $display("*                              FAIL!                                    *");
    $display("*                 Error message from pseudo_SD.v                        *");
end endtask

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

endmodule