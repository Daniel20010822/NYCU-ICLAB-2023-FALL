/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
NYCU Institute of Electronic
2023 Autumn IC Design Laboratory
Lab09: SystemVerilog Design and Verification
File Name   : PATTERN.sv
Module Name : PATTERN
Release version : v1.0 (Release Date: Nov-2023)
Author : Jui-Huang Tsai (erictsai.10@nycu.edu.tw)
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

`include "Usertype_BEV.sv"

program automatic PATTERN(input clk, INF.PATTERN inf);
import usertype::*;

//================================================================
// parameters & integer
//================================================================
parameter DRAM_p_r = "../00_TESTBED/DRAM/dram.dat";
int seed    = 587;
int PAT_NUM = 10000;
int total_latency, latency;
int i, pat;
logic[ 9*8:1] reset_color       = "\033[1;0m";
logic[10*8:1] txt_green_prefix  = "\033[1;32m";
logic[10*8:1] txt_blue_prefix   = "\033[1;34m";
//================================================================
// wire & registers
//================================================================
logic [7:0] golden_DRAM [((65536+8*256)-1):(65536+0)];  // 256 box
logic [63:0] dram_data, new_data;
Bev_Bal received_bal, updated_bal;
Error_Msg golden_err_msg;
logic golden_complete;
logic ING_OF_flag, NO_EXP_flag, NO_ING_flag;
logic [16:0] addr;
ING bt_ratio, gt_ratio, mlk_ratio, pj_ratio;
ING bt_cost, gt_cost, mlk_cost, pj_cost;
ING new_bt, new_gt, new_mlk, new_pj;
//================================================================
// class random
//================================================================

class random_action;
    function new(int seed);
        this.srandom(seed);
    endfunction

    rand Action action_id;

    constraint range{
        action_id inside{Make_drink, Supply, Check_Valid_Date};
    }
endclass

class random_boxid;
    function new(int seed);
        this.srandom(seed);
    endfunction

    rand logic [7:0] box_id;

    constraint range{
        box_id inside{[0:255]};
    }
endclass

class random_date;
    function new(int seed);
        this.srandom(seed);
    endfunction

    rand Month M;
    rand Day   D;

    constraint range{
        M inside{[1:12]};
        (M ==  1) -> D inside{[1:31]};
        (M ==  2) -> D inside{[1:28]};
        (M ==  3) -> D inside{[1:31]};
        (M ==  4) -> D inside{[1:30]};
        (M ==  5) -> D inside{[1:31]};
        (M ==  6) -> D inside{[1:30]};
        (M ==  7) -> D inside{[1:31]};
        (M ==  8) -> D inside{[1:31]};
        (M ==  9) -> D inside{[1:30]};
        (M == 10) -> D inside{[1:31]};
        (M == 11) -> D inside{[1:30]};
        (M == 12) -> D inside{[1:31]};
    }
endclass

class random_order;
    function new(int seed);
        this.srandom(seed);
    endfunction

    randc Order_Info order;

    constraint range{
        order.Bev_Type_O inside{
            Black_Tea,
            Milk_Tea,
            Extra_Milk_Tea,
            Green_Tea,
            Green_Milk_Tea,
            Pineapple_Juice,
            Super_Pineapple_Tea,
            Super_Pineapple_Milk_Tea };

        order.Bev_Size_O inside{
            L,
            M,
            S };
    }
endclass

class random_ing;
    function new(int seed);
        this.srandom(seed);
    endfunction
    rand ING black_tea;
    rand ING green_tea;
    rand ING milk;
    rand ING pineapple_juice;
    constraint range {
        black_tea inside{[0:4095]};
        green_tea inside{[0:4095]};
        milk inside{[0:4095]};
        pineapple_juice inside{[0:4095]};
    }
endclass

//================================================================
// initial
//================================================================
random_action rand_action = new(seed);
random_boxid  rand_boxid  = new(seed);
random_date   rand_date   = new(seed);
random_order  rand_order  = new(seed);
random_ing    rand_ing    = new(seed);

initial $readmemh(DRAM_p_r, golden_DRAM);

initial begin
    total_latency = 0;
    reset_signal_task;

    for (pat = 0; pat < PAT_NUM; pat = pat + 1) begin
        input_task;
        gen_golden_task;
        wait_valid_task;
        check_ans_task;
    end

    YOU_PASS_task;
end

//================================================================
// TASK
//================================================================
task reset_signal_task; begin
    inf.rst_n              = 'b1;
    inf.sel_action_valid   = 'b0;
    inf.type_valid         = 'b0;
    inf.size_valid         = 'b0;
    inf.date_valid         = 'b0;
    inf.box_no_valid       = 'b0;
    inf.box_sup_valid      = 'b0;
    inf.D                  = 'bx;


    #10; inf.rst_n = 'b0;
    #10; inf.rst_n = 'b1;

    // if (inf.out_valid !== 'b0 || inf.err_msg !== No_Err || inf.complete !== 'b0) begin
    //     OUTPUT_NOT_RESET_MSG;
    // end
end endtask


task input_task; begin
    i = rand_action.randomize();
    i = rand_order.randomize();
    i = rand_date.randomize();
    i = rand_boxid.randomize();
    i = rand_ing.randomize();

    // Action
    repeat($urandom_range(0,3)) @(negedge clk);
    throw_rand_action;

    case(rand_action.action_id)
        Make_drink: begin
            // 1. Type valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_type;
            // 2. Size valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_size;
            // 3. Date valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_date;
            // 4. Box_no valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_boxid;
        end
        Supply: begin
            // 1. Date valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_date;
            // 2. Box_no valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_boxid;
            // 3-1. black_tea
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_black_tea;
            // 3-2. green_tea
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_green_tea;
            // 3-3. milk
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_milk;
            // 3-4. pineapple_juice
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_pineapple_juice;
        end
        Check_Valid_Date: begin
            // 1. Date valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_date;
            // 2. Box_no valid
            repeat($urandom_range(0,3)) @(negedge clk);
            throw_rand_boxid;
        end
    endcase
end endtask


task gen_golden_task; begin
    // err_msg, complete
    golden_err_msg = No_Err;
    golden_complete = 1;
    // flags
    ING_OF_flag = 0;
    NO_EXP_flag = 0;
    NO_ING_flag = 0;

    // ACCESS DRAM
    addr = {5'b10000, {4'd0, rand_boxid.box_id} << 3};
    dram_data = {
        golden_DRAM[addr+7],
        golden_DRAM[addr+6],
        golden_DRAM[addr+5],
        golden_DRAM[addr+4],
        golden_DRAM[addr+3],
        golden_DRAM[addr+2],
        golden_DRAM[addr+1],
        golden_DRAM[addr+0]
    };
    received_bal.black_tea       = dram_data[63:52];
    received_bal.green_tea       = dram_data[51:40];
    received_bal.milk            = dram_data[31:20];
    received_bal.pineapple_juice = dram_data[19:8];
    received_bal.M = dram_data[35:32];
    received_bal.D = dram_data[4:0];

    case(rand_action.action_id)
        Make_drink: begin
            get_cost_task;

            // NO_ING_CHK
            if (received_bal.black_tea       - bt_cost  > received_bal.black_tea      ) NO_ING_flag = 1;
            if (received_bal.green_tea       - gt_cost  > received_bal.green_tea      ) NO_ING_flag = 1;
            if (received_bal.milk            - mlk_cost > received_bal.milk           ) NO_ING_flag = 1;
            if (received_bal.pineapple_juice - pj_cost  > received_bal.pineapple_juice) NO_ING_flag = 1;

            // NO_EXP_CHK
            if ((received_bal.M < rand_date.M) || (received_bal.M == rand_date.M && received_bal.D < rand_date.D))
                NO_EXP_flag = 1;


            updated_bal.black_tea       = (received_bal.black_tea       - bt_cost  > received_bal.black_tea      ) ? received_bal.black_tea       : received_bal.black_tea       - bt_cost;
            updated_bal.green_tea       = (received_bal.green_tea       - gt_cost  > received_bal.green_tea      ) ? received_bal.green_tea       : received_bal.green_tea       - gt_cost;
            updated_bal.milk            = (received_bal.milk            - mlk_cost > received_bal.milk           ) ? received_bal.milk            : received_bal.milk            - mlk_cost;
            updated_bal.pineapple_juice = (received_bal.pineapple_juice - pj_cost  > received_bal.pineapple_juice) ? received_bal.pineapple_juice : received_bal.pineapple_juice - pj_cost;
            updated_bal.M = received_bal.M;
            updated_bal.D = received_bal.D;


            // Update golden
            if (NO_EXP_flag)      begin golden_err_msg = No_Exp; golden_complete = 0; end
            else if (NO_ING_flag) begin golden_err_msg = No_Ing; golden_complete = 0; end
            else                  begin golden_err_msg = No_Err; golden_complete = 1; end

            // Write back to dram
            if (golden_err_msg == No_Err) begin
                new_data = {
                    updated_bal.black_tea,
                    updated_bal.green_tea,
                    {4'd0, updated_bal.M},
                    updated_bal.milk,
                    updated_bal.pineapple_juice,
                    {3'd0, updated_bal.D}
                };
                golden_DRAM[addr+7] = new_data[63:56];
                golden_DRAM[addr+6] = new_data[55:48];
                golden_DRAM[addr+5] = new_data[47:40];
                golden_DRAM[addr+4] = new_data[39:32];
                golden_DRAM[addr+3] = new_data[31:24];
                golden_DRAM[addr+2] = new_data[23:16];
                golden_DRAM[addr+1] = new_data[15:8];
                golden_DRAM[addr+0] = new_data[7:0];
            end
        end
        Supply: begin
            if ( received_bal.black_tea       + rand_ing.black_tea       < received_bal.black_tea      ) ING_OF_flag = 1;
            if ( received_bal.green_tea       + rand_ing.green_tea       < received_bal.green_tea      ) ING_OF_flag = 1;
            if ( received_bal.milk            + rand_ing.milk            < received_bal.milk           ) ING_OF_flag = 1;
            if ( received_bal.pineapple_juice + rand_ing.pineapple_juice < received_bal.pineapple_juice) ING_OF_flag = 1;


            updated_bal.black_tea       = ( received_bal.black_tea       + rand_ing.black_tea       < received_bal.black_tea      ) ? 4095 : received_bal.black_tea       + rand_ing.black_tea;
            updated_bal.green_tea       = ( received_bal.green_tea       + rand_ing.green_tea       < received_bal.green_tea      ) ? 4095 : received_bal.green_tea       + rand_ing.green_tea;
            updated_bal.milk            = ( received_bal.milk            + rand_ing.milk            < received_bal.milk           ) ? 4095 : received_bal.milk            + rand_ing.milk;
            updated_bal.pineapple_juice = ( received_bal.pineapple_juice + rand_ing.pineapple_juice < received_bal.pineapple_juice) ? 4095 : received_bal.pineapple_juice + rand_ing.pineapple_juice;
            updated_bal.M = rand_date.M;
            updated_bal.D = rand_date.D;


            // Update golden
            if (ING_OF_flag)      begin golden_err_msg = Ing_OF; golden_complete = 0; end
            else                  begin golden_err_msg = No_Err; golden_complete = 1; end

            // Write back to dram
            new_data = {
                updated_bal.black_tea,
                updated_bal.green_tea,
                {4'd0, updated_bal.M},
                updated_bal.milk,
                updated_bal.pineapple_juice,
                {3'd0, updated_bal.D}
            };
            golden_DRAM[addr+7] = new_data[63:56];
            golden_DRAM[addr+6] = new_data[55:48];
            golden_DRAM[addr+5] = new_data[47:40];
            golden_DRAM[addr+4] = new_data[39:32];
            golden_DRAM[addr+3] = new_data[31:24];
            golden_DRAM[addr+2] = new_data[23:16];
            golden_DRAM[addr+1] = new_data[15:8];
            golden_DRAM[addr+0] = new_data[7:0];
        end
        Check_Valid_Date: begin
            if ((received_bal.M < rand_date.M) || (received_bal.M == rand_date.M && received_bal.D < rand_date.D))
                NO_EXP_flag = 1;

            // Update golden
            if (NO_EXP_flag)      begin golden_err_msg = No_Exp; golden_complete = 0; end
            else                  begin golden_err_msg = No_Err; golden_complete = 1; end
        end
    endcase
end endtask


task wait_valid_task; begin
    latency = 0;

    while (inf.out_valid !== 'b1) begin
        latency = latency + 1;
        if (latency > 1000) begin
            LATENCY_LIMIT_EXCEED_MSG;
        end
        @(negedge clk);
    end

    total_latency = total_latency + latency;
end endtask





task check_ans_task; begin
    if (inf.err_msg !== golden_err_msg || inf.complete !== golden_complete) begin
        PAT_FAIL_MSG;
    end

    PAT_PASS_MSG;
    @(negedge clk);
end endtask




//================================================================
//   INPUT_TASK -> THROW TASKS
//================================================================

task throw_rand_action; begin
    inf.sel_action_valid = 'b1;
    inf.D.d_act[0] = rand_action.action_id;
    @(negedge clk);
    inf.sel_action_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_type; begin
    inf.type_valid = 'b1;
    inf.D.d_type[0] = rand_order.order.Bev_Type_O;
    @(negedge clk);
    inf.type_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_size; begin
    inf.size_valid = 'b1;
    inf.D.d_size[0] = rand_order.order.Bev_Size_O;
    @(negedge clk);
    inf.size_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_date; begin
    inf.date_valid = 'b1;
    inf.D.d_date[0] = {rand_date.M, rand_date.D};
    @(negedge clk);
    inf.date_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_boxid; begin
    inf.box_no_valid = 'b1;
    inf.D.d_box_no[0] = rand_boxid.box_id;
    @(negedge clk);
    inf.box_no_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_black_tea; begin
    inf.box_sup_valid = 'b1;
    inf.D.d_ing[0] = rand_ing.black_tea;
    @(negedge clk);
    inf.box_sup_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_green_tea; begin
    inf.box_sup_valid = 'b1;
    inf.D.d_ing[0] = rand_ing.green_tea;
    @(negedge clk);
    inf.box_sup_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_milk; begin
    inf.box_sup_valid = 'b1;
    inf.D.d_ing[0] = rand_ing.milk;
    @(negedge clk);
    inf.box_sup_valid = 'b0;
    inf.D = 'bx;
end endtask

task throw_rand_pineapple_juice; begin
    inf.box_sup_valid = 'b1;
    inf.D.d_ing[0] = rand_ing.pineapple_juice;
    @(negedge clk);
    inf.box_sup_valid = 'b0;
    inf.D = 'bx;
end endtask

//================================================================
//   GEN_GOLDEN_TASK -> GET_COST_TASK
//================================================================
task get_cost_task; begin
    case(rand_order.order.Bev_Type_O)
        Black_Tea: begin
            bt_ratio  = 240;
            gt_ratio  = 0;
            mlk_ratio = 0;
            pj_ratio  = 0;
        end
        Milk_Tea: begin
            bt_ratio  = 180;
            gt_ratio  = 0;
            mlk_ratio = 60;
            pj_ratio  = 0;
        end
        Extra_Milk_Tea: begin
            bt_ratio  = 120;
            gt_ratio  = 0;
            mlk_ratio = 120;
            pj_ratio  = 0;
        end
        Green_Tea: begin
            bt_ratio  = 0;
            gt_ratio  = 240;
            mlk_ratio = 0;
            pj_ratio  = 0;
        end
        Green_Milk_Tea: begin
            bt_ratio  = 0;
            gt_ratio  = 120;
            mlk_ratio = 120;
            pj_ratio  = 0;
        end
        Pineapple_Juice: begin
            bt_ratio  = 0;
            gt_ratio  = 0;
            mlk_ratio = 0;
            pj_ratio  = 240;
        end
        Super_Pineapple_Tea: begin
            bt_ratio  = 120;
            gt_ratio  = 0;
            mlk_ratio = 0;
            pj_ratio  = 120;
        end
        Super_Pineapple_Milk_Tea: begin
            bt_ratio  = 120;
            gt_ratio  = 0;
            mlk_ratio = 60;
            pj_ratio  = 60;
        end
    endcase

    case (rand_order.order.Bev_Size_O)
        L: begin
            bt_cost  = bt_ratio  * 4;
            gt_cost  = gt_ratio  * 4;
            mlk_cost = mlk_ratio * 4;
            pj_cost  = pj_ratio  * 4;
        end
        M: begin
            bt_cost  = bt_ratio  * 3;
            gt_cost  = gt_ratio  * 3;
            mlk_cost = mlk_ratio * 3;
            pj_cost  = pj_ratio  * 3;
        end
        S: begin
            bt_cost  = bt_ratio  * 2;
            gt_cost  = gt_ratio  * 2;
            mlk_cost = mlk_ratio * 2;
            pj_cost  = pj_ratio  * 2;
        end
    endcase
end endtask

//================================================================
//   MESSAGE DISPLAY TASKS
//================================================================

task OUTPUT_NOT_RESET_MSG; begin
    $display();
    $display("\033[31m");
    $display("*************************************************************************");
    $display();
    $display(" All output signals should be reset after the reset signal is asserted.");
    $display();
    $display("*************************************************************************");
    $display("\033[39m");
    $finish;
end endtask

task LATENCY_LIMIT_EXCEED_MSG; begin
    $display();
    $display("\033[31m");
    $display("*************************************************************************");
    $display();
    $display(" Execution latency exceed 1000 cycles. ");
    $display();
    $display("*************************************************************************");
    $display("\033[39m");
    $finish;
end endtask


task PAT_FAIL_MSG; begin
    $display();
    $display("\033[31m");
    $display("*************************************************************************");
    $display("                              Wrong Answer                               ");
    $display("*************************************************************************");
    $display();
    $display(" PATTERN NO.%3d ",pat);
    case(rand_action.action_id)
        Make_drink: $display(" ACTION: Make_drink");
        Supply: $display(" ACTION: Supply");
        Check_Valid_Date: $display(" ACTION: Check_Valid_Date");
    endcase
    $display(" Expected Err_msg = %d        Expected complete = %d ",golden_err_msg, golden_complete);
    $display("     Your Err_msg = %d            Your complete = %d ",inf.err_msg, inf.complete);
    $display();
    $display("*************************************************************************");
    $display("\033[39m");
    $finish;
end endtask

task PAT_PASS_MSG; begin
    $display("%0sPASS PATTERN NO.%4d, %0sCycles: %3d%0s",txt_blue_prefix, pat, txt_green_prefix, latency, reset_color);
end endtask

task YOU_PASS_task; begin
    $display ("----------------------------------------------------------------------------------------------------------------------");
    $display ("                                                  Congratulations                                                                        ");
    $display ("                                           You have passed all patterns!                                                                 ");
    $display ("                                           Your execution cycles = %5d cycles                                                            ", total_latency);
    $display ("                                           Your clock period = %.1f ns                                                               ", 12);
    $display ("                                           Total Latency = %.1f ns                                                               ", total_latency*12);
    $display ("----------------------------------------------------------------------------------------------------------------------");
    repeat(2)@(negedge clk);
    $finish;
end endtask

endprogram