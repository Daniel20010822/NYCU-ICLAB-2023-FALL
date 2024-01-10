/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
NYCU Institute of Electronic
2023 Autumn IC Design Laboratory
Lab10: SystemVerilog Coverage & Assertion
File Name   : CHECKER.sv
Module Name : CHECKER
Release version : v1.0 (Release Date: Nov-2023)
Author : Jui-Huang Tsai (erictsai.10@nycu.edu.tw)
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

`include "Usertype_BEV.sv"
module Checker(input clk, INF.CHECKER inf);
import usertype::*;


logic m_flag, s_flag, c_flag, low_flag;
logic [2:0] cnt_4;
logic [2:0] valid_cnt;

//=============================
//    Coverage Part
//=============================


class BEV;
    Bev_Type bev_type;
    Bev_Size bev_size;
endclass

BEV bev_info = new();

always_ff @(posedge clk) begin
    if (inf.type_valid) begin
        bev_info.bev_type = inf.D.d_type[0];
    end
end
always_ff @(posedge clk) begin
    if (inf.size_valid) begin
        bev_info.bev_size = inf.D.d_size[0];
    end
end


/*
1. Each case of Beverage_Type should be select at least 100 times.
*/


covergroup SPEC1 @(posedge clk iff(inf.type_valid));
    option.per_instance = 1;
    option.at_least = 100;
    btype: coverpoint bev_info.bev_type {
        bins type_bin [] = {[Black_Tea:Super_Pineapple_Milk_Tea]};
    }
endgroup


/*
2.	Each case of Bererage_Size should be select at least 100 times.
*/


covergroup SPEC2 @(posedge clk iff(inf.size_valid));
    option.per_instance = 1;
    option.at_least = 100;
    bsize: coverpoint bev_info.bev_size {
        bins size_bin [] = {L, M, S};
    }
endgroup


/*
3.	Create a cross bin for the SPEC1 and SPEC2. Each combination should be selected at least 100 times.
(Black Tea, Milk Tea, Extra Milk Tea, Green Tea, Green Milk Tea, Pineapple Juice, Super Pineapple Tea, Super Pineapple Tea) x (L, M, S)
*/

// FIXME: IFF STATEMENT
covergroup SPEC3 @(posedge clk iff(inf.size_valid));
    option.per_instance = 1;
    option.at_least = 100;
    coverpoint bev_info.bev_type;
    coverpoint bev_info.bev_size;
    cross bev_info.bev_type, bev_info.bev_size;
endgroup

/*
4.	Output signal inf.err_msg should be No_Err, No_Exp, No_Ing and Ing_OF, each at least 20 times. (Sample the value when inf.out_valid is high)
*/


covergroup SPEC4 @(posedge clk iff(inf.out_valid));
    option.per_instance = 1;
    option.at_least = 20;
    berrmsg: coverpoint inf.err_msg {
        bins err_msg_bin [] = {No_Err, No_Exp, No_Ing, Ing_OF};
    }
endgroup


/*
5.	Create the transitions bin for the inf.D.act[0] signal from [0:2] to [0:2]. Each transition should be hit at least 200 times. (sample the value at posedge clk iff inf.sel_action_valid)
*/


covergroup SPEC5 @(posedge clk iff(inf.sel_action_valid));
    option.per_instance = 1;
    option.at_least = 200;
    bact: coverpoint inf.D.d_act[0] {
        bins action_bin [] = ([Make_drink:Check_Valid_Date] => [Make_drink:Check_Valid_Date]);
    }
endgroup


/*
6.	Create a covergroup for material of supply action with auto_bin_max = 32, and each bin have to hit at least one time.
*/

covergroup SPEC6 @(posedge clk iff(inf.box_sup_valid));
    option.per_instance = 1;
    option.at_least = 1;
    bboxsup: coverpoint inf.D.d_ing[0] {
        option.auto_bin_max = 32;
    }
endgroup


/*
    Create instances of Spec1, Spec2, Spec3, Spec4, Spec5, and Spec6
*/
SPEC1 cov_inst_1 = new();
SPEC2 cov_inst_2 = new();
SPEC3 cov_inst_3 = new();
SPEC4 cov_inst_4 = new();
SPEC5 cov_inst_5 = new();
SPEC6 cov_inst_6 = new();

//=============================
//    Assertion
//=============================

/*
    If you need, you can declare some FSM, logic, flag, and etc. here.
*/

/*
    1. All outputs signals (including BEV.sv and bridge.sv) should be zero after reset.
*/


wire #(0.5) rst_delay = inf.rst_n;
property rule_01;
    @(negedge rst_delay) (
        inf.out_valid === 'b0 && inf.err_msg === 'b0 && inf.complete === 'b0 &&
        inf.C_addr === 'b0 && inf.C_data_w === 'b0 && inf.C_in_valid === 'b0 && inf.C_r_wb === 'b0 &&
        inf.C_out_valid === 'b0 && inf.C_data_r === 'b0 &&
        inf.AR_VALID === 'b0 && inf.AR_ADDR === 'b0 && inf.R_READY === 'b0 &&
        inf.AW_VALID === 'b0 && inf.AW_ADDR === 'b0 && inf.W_VALID === 'b0 && inf.W_DATA === 'b0 && inf.B_READY === 'b0
    );
endproperty

Assertion_01: assert property (rule_01)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 1 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end




/*
    2.	Latency should be less than 1000 cycles for each operation.
*/


property rule_02;
    @(negedge clk) inf.sel_action_valid |-> ## [1:1000] inf.out_valid;
endproperty

Assertion_02: assert property (rule_02)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 2 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end



/*
    3. If out_valid does not pull up, complete should be 0.
*/



property rule_03;
    @(negedge clk) (inf.out_valid === 'b1 && inf.complete === 'b1) |-> (inf.err_msg === No_Err);
endproperty

Assertion_03: assert property (rule_03)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 3 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end



/*
    4. Next input valid will be valid 1-4 cycles after previous input valid fall.
*/
always_ff @(posedge clk) begin
    if (inf.sel_action_valid) begin
        m_flag <= (inf.D.d_act[0] === Make_drink);
        s_flag <= (inf.D.d_act[0] === Supply);
        c_flag <= (inf.D.d_act[0] === Check_Valid_Date);
    end
end
always_ff @(posedge clk) begin
    if (inf.sel_action_valid)   cnt_4 <= 0;
    else if (inf.box_sup_valid) cnt_4 <= cnt_4 + 1'd1;
end

// MAKE_DRINK
property rule_04_m1;
    @(posedge clk) (inf.sel_action_valid && inf.D.d_act[0] === Make_drink) |-> ## [1:4] inf.type_valid;
endproperty
Assertion_04_m1: assert property (rule_04_m1)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_m2;
    @(posedge clk) (inf.type_valid && m_flag) |-> ## [1:4] inf.size_valid;
endproperty
Assertion_04_m2: assert property (rule_04_m2)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_m3;
    @(posedge clk) (inf.size_valid && m_flag) |-> ## [1:4] inf.date_valid;
endproperty
Assertion_04_m3: assert property (rule_04_m3)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_m4;
    @(posedge clk) (inf.date_valid && m_flag) |-> ## [1:4] inf.box_no_valid;
endproperty
Assertion_04_m4: assert property (rule_04_m4)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end

// SUPPLY
property rule_04_s1;
    @(posedge clk) (inf.sel_action_valid && inf.D.d_act[0] === Supply) |-> ## [1:4] inf.date_valid;
endproperty
Assertion_04_s1: assert property (rule_04_s1)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_s2;
    @(posedge clk) (inf.date_valid && s_flag) |-> ## [1:4] inf.box_no_valid;
endproperty
Assertion_04_s2: assert property (rule_04_s2)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_s3;
    @(posedge clk) (inf.box_no_valid && s_flag) |-> ## [1:4] inf.box_sup_valid;
endproperty
Assertion_04_s3: assert property (rule_04_s3)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_s4;
    @(posedge clk) (inf.box_sup_valid && s_flag && cnt_4 == 1) |-> ## [1:4] inf.box_sup_valid;
endproperty
Assertion_04_s4: assert property (rule_04_s4)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_s5;
    @(posedge clk) (inf.box_sup_valid && s_flag && cnt_4 == 2) |-> ## [1:4] inf.box_sup_valid;
endproperty
Assertion_04_s5: assert property (rule_04_s5)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end

// CHECK_DATE
property rule_04_c1;
    @(posedge clk) (inf.sel_action_valid && inf.D.d_act[0] === Check_Valid_Date) |-> ## [1:4] inf.date_valid;
endproperty
Assertion_04_c1: assert property (rule_04_c1)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
property rule_04_c2;
    @(posedge clk) (inf.date_valid && c_flag) |-> ## [1:4] inf.box_no_valid;
endproperty
Assertion_04_c2: assert property (rule_04_c2)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 4 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end



/*
    5. All input valid signals won't overlap with each other.
*/


assign valid_cnt = inf.sel_action_valid + inf.type_valid + inf.size_valid + inf.date_valid + inf.box_no_valid + inf.box_sup_valid;

property rule_05;
    @(posedge clk) valid_cnt <= 1;
endproperty

Assertion_05: assert property (rule_05)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 5 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end





/*
    6. Out_valid can only be high for exactly one cycle.
*/



property rule_06;
    @(negedge clk) inf.out_valid |=> !inf.out_valid;
endproperty

Assertion_06: assert property (rule_06)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 6 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end




/*
    7. Next operation will be valid 1-4 cycles after out_valid fall.
*/



property rule_07;
    @(negedge clk) inf.out_valid |-> ## [2:5] inf.sel_action_valid;
endproperty

Assertion_07: assert property (rule_07)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 7 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end



/*
    8. The input date from pattern should adhere to the real calendar. (ex: 2/29, 3/0, 4/31, 13/1 are illegal cases)
*/


property rule_08;
    @(posedge clk) (inf.date_valid) |-> (
        (inf.D.d_date[0].M ===  1 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M ===  2 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 28)) ||
        (inf.D.d_date[0].M ===  3 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M ===  4 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 30)) ||
        (inf.D.d_date[0].M ===  5 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M ===  6 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 30)) ||
        (inf.D.d_date[0].M ===  7 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M ===  8 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M ===  9 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 30)) ||
        (inf.D.d_date[0].M === 10 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31)) ||
        (inf.D.d_date[0].M === 11 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 30)) ||
        (inf.D.d_date[0].M === 12 && (1 <= inf.D.d_date[0].D && inf.D.d_date[0].D <= 31))    );
endproperty

Assertion_08: assert property (rule_08)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 8 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end




/*
    9. C_in_valid can only be high for one cycle and can't be pulled high again before C_out_valid
*/


property rule_09_1;
    @(negedge clk) inf.C_in_valid |=> inf.C_in_valid === 'b0;
endproperty

Assertion_09_1: assert property (rule_09_1)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 9 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end

always_ff @(posedge clk) begin
    if (inf.sel_action_valid)
        low_flag <= 0;
    else if (inf.C_in_valid)
        low_flag <= 1;
    else if (inf.C_out_valid)
        low_flag <= 0;
end

property rule_09_2;
    @(negedge clk) low_flag |=> inf.C_in_valid === 'b0;
endproperty

Assertion_09_2: assert property (rule_09_2)
else begin
    $display("*************************************************************************");
    $display();
    $display("Assertion 9 is violated");
    $display();
    $display("*************************************************************************");
    $fatal;
end
endmodule
