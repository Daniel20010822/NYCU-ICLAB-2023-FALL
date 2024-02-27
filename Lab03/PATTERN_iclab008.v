`ifdef RTL
    `define CYCLE_TIME 40.0
`endif
`ifdef GATE
    `define CYCLE_TIME 40.0
`endif

`include "../00_TESTBED/pseudo_DRAM.v"
`include "../00_TESTBED/pseudo_SD.v"

module PATTERN(
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

/* Input for design */
output reg        clk, rst_n;
output reg        in_valid;
output reg        direction;
output reg [12:0] addr_dram;
output reg [15:0] addr_sd;

/* Output for pattern */
input        out_valid;
input  [7:0] out_data;

/* DRAM Signals */
// write address channel
input [31:0] AW_ADDR;
input AW_VALID;
output AW_READY;
// write data channel
input W_VALID;
input [63:0] W_DATA;
output W_READY;
// write response channel
output B_VALID;
output [1:0] B_RESP;
input B_READY;
// read address channel
input [31:0] AR_ADDR;
input AR_VALID;
output AR_READY;
// read data channel
output [63:0] R_DATA;
output R_VALID;
output [1:0] R_RESP;
input R_READY;

/* SD Signals */
output MISO;
input MOSI;

/* parameter and integer*/
real CYCLE = `CYCLE_TIME;
integer pat_read;
integer PAT_NUM;
integer total_latency, latency;
integer i_pat;
integer a, i, start, stop;
/* reg declaration */
reg dir;
reg [12:0] ad_dram;
reg [15:0] ad_sd;
reg [63:0] correct_data;


initial begin
    pat_read = $fopen("../00_TESTBED/Input.txt", "r");
    reset_signal_task;

    i_pat = 0;
    total_latency = 0;
    a = $fscanf(pat_read, "%d", PAT_NUM);
    for (i_pat = 1; i_pat <= PAT_NUM; i_pat = i_pat + 1) begin
        input_task;
        wait_out_valid_task;
        check_ans_task;
        total_latency = total_latency + latency;
        $display("PASS PATTERN NO.%4d", i_pat);
    end
    $fclose(pat_read);

    $writememh("../00_TESTBED/DRAM_final.dat", u_DRAM.DRAM);
    $writememh("../00_TESTBED/SD_final.dat", u_SD.SD);
    YOU_PASS_task;
end

//////////////////////////////////////////////////////////////////////
// Write your own task here
//////////////////////////////////////////////////////////////////////
/* define clock cycle */
always #(CYCLE/2.0) clk = ~clk;

task reset_signal_task; begin
    rst_n     = 'b1;
    in_valid  = 'b0;
    direction = 'bx;
    addr_dram = 'bx;
    addr_sd   = 'bx;
    correct_data  = 'd0;
    total_latency = 0;

    force clk = 0;

    #CYCLE; rst_n = 0;
    #CYCLE; rst_n = 1;

    if (out_valid !== 'b0 || out_data  !== 'b0 || AW_ADDR   !== 'b0 ||
        AW_VALID  !== 'b0 || W_VALID   !== 'b0 || W_DATA    !== 'b0 ||
        B_READY   !== 'b0 || AR_ADDR   !== 'b0 || AR_VALID  !== 'b0 ||
        R_READY   !== 'b0 || MOSI      !== 'b1)
    begin
        SPEC_MAIN_1_DEBUG_task;
        SPEC_MAIN_1_FAIL_task;
    end
    #CYCLE; release clk;
end endtask

task input_task; begin
    a = $fscanf(pat_read, "%d", dir);
    a = $fscanf(pat_read, "%d", ad_dram);
    a = $fscanf(pat_read, "%d", ad_sd);

    repeat(4) @(negedge clk);

    in_valid  = 'b1;
    direction = dir;
    addr_dram = ad_dram;
    addr_sd   = ad_sd;
    @(negedge clk);

    in_valid  = 'b0;
    direction = 'bx;
    addr_dram = 'bx;
    addr_sd   = 'bx;
end endtask

task wait_out_valid_task; begin
    latency = 0;

    /**************************************/
    /* Waiting for out_valid to become 1  */
    /* out_valid should be 0              */
    /* out_data  should be 0              */
    /**************************************/
    while (out_valid !== 1'b1) begin
        latency = latency + 1;

        // Read data to check SPEC_MAIN_5
        if (~dir && R_VALID)
            correct_data = R_DATA;
        else if (dir && W_VALID)
            correct_data = W_DATA;

        // When out_data is not asserted during out_valid = 0
        if (out_data !== 'd0) begin
            SPEC_MAIN_2_FAIL_task;
        end
        // Execution latency is over 10000 cycles
        if (latency == 10000) begin
            SPEC_MAIN_3_FAIL_task;
        end
        @(negedge clk);
    end

    total_latency = total_latency + latency;
end endtask

task check_ans_task; begin
    latency = 0;

    /****************************************************/
    /* Data read from DRAM has not been written into SD */
    /*                        or                        */
    /* Data read from SD has not been written into DRAM */
    /****************************************************/
    if (correct_data !== u_SD.SD[ad_sd] || correct_data !== u_DRAM.DRAM[ad_dram]) begin
        SPEC_MAIN_6_FAIL_DEBUG_task;
        SPEC_MAIN_6_FAIL_task;
    end


    /*******************************/
    /* In the next eight cycles    */
    /* out_valid should be 1       */
    /* out_data  should be correct */
    /*******************************/
    for (i = 0; i < 8; i = i + 1) begin
        latency = latency + 1;

        // If out_valid turns 0 during these eight cycles
        if (out_valid !== 'b1) begin
            SPEC_MAIN_4_FAIL_task1;
        end
        // If current out_data is not the correct answer
        if (out_data !== correct_data[(63 - i*8) -: 8]) begin
            SPEC_MAIN_5_FAIL_task;
        end

        @(negedge clk);
    end

    /**************************/
    /* After eight cycles     */
    /* out_valid should be 0  */
    /* out_data  should be 0  */
    /**************************/
    // If out_valid haven't turned 0
    if (out_valid !== 'b0) begin
        SPEC_MAIN_4_FAIL_task2;
    end
    // out_valid turned 0, but out_data still have value
    else if (out_data !== 'd0) begin
        SPEC_MAIN_2_FAIL_task;
    end

    total_latency = total_latency + latency;
end endtask
//////////////////////////////////////////////////////////////////////

task YOU_PASS_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                         Congratulations!                              *");
    $display("*                Your execution cycles = %5d cycles                  *", total_latency);
    $display("*                Your clock period = %.1f ns                            *", CYCLE);
    $display("*                Total Latency = %.1f ns                          *", total_latency*CYCLE);
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_MAIN_1_DEBUG_task; begin
    $display();
    $display("out_valid = %4d", out_valid);
    $display("out_data  = %4d", out_data);
    $display("AW_ADDR   = %4d", AW_ADDR);
    $display("AW_VALID  = %4d", AW_VALID);
    $display("W_VALID   = %4d", W_VALID);
    $display("W_DATA    = %4d", W_DATA);
    $display("B_READY   = %4d", B_READY);
    $display("AR_ADDR   = %4d", AR_ADDR);
    $display("AR_VALID  = %4d", AR_VALID);
    $display("R_READY   = %4d", R_READY);
    $display("MOSI      = %4d", MOSI);
    $display("at %4dns", $time);
end endtask

task SPEC_MAIN_1_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-1 FAIL                            *");
    $display("* All output signals should be reset after the reset signal is asserted.*");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    $finish;
end endtask

task SPEC_MAIN_2_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-2 FAIL                            *");
    $display("*       The out_data should be reset when your out_valid is low.        *");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_3_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-3 FAIL                            *");
    $display("*          The execution latency is limited in 10000 cycles.            *");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_4_FAIL_task1; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-4 FAIL                            *");
    $display("*       The out_valid and out_data must be asserted in 8 cycles.        *");
    $display("*                   out_valid becomes low in %1d cycles                   *", i);
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_4_FAIL_task2; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-4 FAIL                            *");
    $display("*       The out_valid and out_data must be asserted in 8 cycles.        *");
    $display("*                   out_valid is still high after 8 cycles              *");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_5_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-5 FAIL                            *");
    $display("*         The out_data should be correct when out_valid is high.        *");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_6_FAIL_task; begin
    $display();
    $display("*************************************************************************");
    $display("*                           SPEC MAIN-6 FAIL                            *");
    $display("*         The data in the DRAM and SD card should be correct when       *");
    $display("*                          out_valid is high.                           *");
    $display("*                      Error message from BRIDGE.v                      *");
    $display("*************************************************************************");
    $display();
    // repeat(2) @(negedge clk);
    $finish;
end endtask

task SPEC_MAIN_6_FAIL_DEBUG_task; begin
    $display();
    if (dir) $display("SD -> DRAM");
    else $display("DRAM -> SD");
    $display("Correct Data: %h", correct_data);
    $display("   Dram Data: %h", u_DRAM.DRAM[ad_dram]);
    $display("     SD Data: %h", u_SD.SD[ad_sd]);
end endtask

task YOU_FAIL_task; begin
    $display("*                              FAIL!                                    *");
    $display("*                    Error message from PATTERN.v                       *");
end endtask

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