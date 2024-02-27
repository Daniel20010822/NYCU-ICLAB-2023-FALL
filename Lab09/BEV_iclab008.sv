module BEV(input clk, INF.BEV_inf inf);
import usertype::*;
// This file contains the definition of several state machines used in the BEV (Beverage) System RTL design.
// The state machines are defined using SystemVerilog enumerated types.
// The state machines are:
// - state_t: used to represent the overall state of the BEV system
//
// Each enumerated type defines a set of named states that the corresponding process can be in.
typedef enum logic [1:0]{
    IDLE,
    MAKE_DRINK,
    SUPPLY,
    CHECK_DATE
} state_t;

typedef enum logic [3:0]{
    IDLE_M,
    // GET D
    GET_TYPE_M,
    GET_SIZE_M,
    GET_DATE_M,
    GET_BOXNO_M,
    // ACCESS DRAM
    R_VALID_M,
    R_WAIT_M,
    CHK_EXP_M,
    CHK_ING_M,
    W_VALID_M,
    W_WAIT_M,
    CPLT_M
} mk_state_t;

typedef enum logic [3:0] {
    IDLE_S,
    GET_DATE_S,
    GET_BOXNO_S,
    GET_BT_S,
    GET_GT_S,
    GET_MLK_S,
    GET_PJ_S,
    R_VALID_S,
    R_WAIT_S,
    CHK_OF_S,
    W_VALID_S,
    W_WAIT_S,
    CPLT_S
} sup_state_t;

typedef enum logic [3:0] {
    IDLE_C,
    GET_DATE_C,
    GET_BOXNO_C,
    R_VALID_C,
    R_WAIT_C,
    CHK_EXP_C,
    CPLT_C
} cd_state_t;


// REGISTERS
state_t      state, nstate;
mk_state_t   mk_state, mk_nstate;
sup_state_t  sup_state, sup_nstate;
cd_state_t   cd_state, cd_nstate;

// INPUT REGISTERS
Action action_r;
Bev_Type type_r;
Bev_Size size_r;
Month M_r;
Day D_r;
Barrel_No box_id_r;

ING bt_r, bt_ratio, bt_cost;
ING gt_r, gt_ratio, gt_cost;
ING mlk_r, mlk_ratio, mlk_cost;
ING pj_r, pj_ratio, pj_cost;
Bev_Bal received_bal, updated_bal;

Error_Msg err_msg_r;
logic Ingredient_Overflow, Expired, No_Ingredient;
logic [12:0] updated_black_tea;
logic [12:0] updated_green_tea;
logic [12:0] updated_milk;
logic [12:0] updated_pineapple_juice;

//=================================
// TOP FSM
//=================================
always_ff @( posedge clk or negedge inf.rst_n) begin : TOP_FSM_SEQ
    if (!inf.rst_n) state <= IDLE;
    else            state <= nstate;
end
always_comb begin : TOP_FSM_COMB
    case(state)
        IDLE: begin
            if (inf.sel_action_valid) begin
                case(inf.D.d_act[0])
                    Make_drink:       nstate = MAKE_DRINK;
                    Supply:           nstate = SUPPLY;
                    Check_Valid_Date: nstate = CHECK_DATE;
                    default:          nstate = IDLE;
                endcase
            end
            else begin
                nstate = IDLE;
            end
        end

        MAKE_DRINK: nstate = (inf.out_valid) ? IDLE : MAKE_DRINK;
        SUPPLY    : nstate = (inf.out_valid) ? IDLE : SUPPLY;
        CHECK_DATE: nstate = (inf.out_valid) ? IDLE : CHECK_DATE;

        default: nstate = IDLE;
    endcase
end


//=================================
// MAKE_DRINK FSM
//=================================
always_ff @( posedge clk or negedge inf.rst_n) begin : MAKE_DRINK_FSM_SEQ
    if (!inf.rst_n) mk_state <= IDLE_M;
    else            mk_state <= mk_nstate;
end
always_comb begin : MAKE_DRINK_FSM_COMB
    case(mk_state)
        IDLE_M: begin
            if (inf.sel_action_valid) begin
                if (inf.D.d_act[0] == Make_drink)  mk_nstate = GET_TYPE_M;
                else                               mk_nstate = IDLE_M;
            end
            else begin
                mk_nstate = IDLE_M;
            end
        end

        // INPUT REGS
        GET_TYPE_M:   mk_nstate = (inf.type_valid)   ? GET_SIZE_M  : GET_TYPE_M;
        GET_SIZE_M:   mk_nstate = (inf.size_valid)   ? GET_DATE_M  : GET_SIZE_M;
        GET_DATE_M:   mk_nstate = (inf.date_valid)   ? GET_BOXNO_M : GET_DATE_M;
        GET_BOXNO_M:  mk_nstate = (inf.box_no_valid) ? R_VALID_M   : GET_BOXNO_M;
        // DRAM READ
        R_VALID_M:    mk_nstate = R_WAIT_M;
        R_WAIT_M:     mk_nstate = (inf.C_out_valid)  ? CHK_EXP_M   : R_WAIT_M;
        // CHECK
        CHK_EXP_M:    mk_nstate = (Expired)          ? CPLT_M      : CHK_ING_M;
        CHK_ING_M:    mk_nstate = (No_Ingredient)    ? CPLT_M      : W_VALID_M;
        // DRAM WRITE
        W_VALID_M:    mk_nstate = W_WAIT_M;
        W_WAIT_M:     mk_nstate = (inf.C_out_valid)  ? CPLT_M      : W_WAIT_M;
        // COMPLETE
        CPLT_M:       mk_nstate = IDLE_M;

        default: mk_nstate = IDLE_M;
    endcase
end


//=================================
// SUPPLY FSM
//=================================
always_ff @(posedge clk or negedge inf.rst_n) begin : SUPPLY_FSM_SEQ
    if (!inf.rst_n) sup_state <= IDLE_S;
    else            sup_state <= sup_nstate;
end
always_comb begin : SUPPLY_FSM_COMB
    case(sup_state)
        IDLE_S: begin
            if (inf.sel_action_valid) begin
                if (inf.D.d_act[0] == Supply)  sup_nstate = GET_DATE_S;
                else                           sup_nstate = IDLE_S;
            end
            else begin
                sup_nstate = IDLE_S;
            end
        end

        // INPUT REGS
        GET_DATE_S : sup_nstate = (inf.date_valid)    ? GET_BOXNO_S : GET_DATE_S;
        GET_BOXNO_S: sup_nstate = (inf.box_no_valid)  ? GET_BT_S    : GET_BOXNO_S;
        GET_BT_S   : sup_nstate = (inf.box_sup_valid) ? GET_GT_S    : GET_BT_S;
        GET_GT_S   : sup_nstate = (inf.box_sup_valid) ? GET_MLK_S   : GET_GT_S;
        GET_MLK_S  : sup_nstate = (inf.box_sup_valid) ? GET_PJ_S    : GET_MLK_S;
        GET_PJ_S   : sup_nstate = (inf.box_sup_valid) ? R_VALID_S   : GET_PJ_S;
        // DRAM READ
        R_VALID_S  : sup_nstate = R_WAIT_S;
        R_WAIT_S   : sup_nstate = (inf.C_out_valid)   ? CHK_OF_S    : R_WAIT_S;
        // CHECK OVERFLOW
        CHK_OF_S   : sup_nstate = W_VALID_S;
        // DRAM WRITE
        W_VALID_S  : sup_nstate = W_WAIT_S;
        W_WAIT_S   : sup_nstate = (inf.C_out_valid)   ? CPLT_S      : W_WAIT_S;
        // COMPLETE
        CPLT_S     : sup_nstate = IDLE_S;

        default: sup_nstate = IDLE_S;
    endcase
end


//=================================
// CHECK_DATE FSM
//=================================
always_ff @(posedge clk or negedge inf.rst_n) begin : CHECK_DATE_FSM_SEQ
    if (!inf.rst_n) cd_state <= IDLE_C;
    else            cd_state <= cd_nstate;
end
always_comb begin : CHECK_DATE_FSM_COMB
    case(cd_state)
        IDLE_C: begin
            if (inf.sel_action_valid) begin
                if (inf.D.d_act[0] == Check_Valid_Date)  cd_nstate = GET_DATE_C;
                else                                     cd_nstate = IDLE_C;
            end
            else begin
                cd_nstate = IDLE_C;
            end
        end

        // INPUT REGS
        GET_DATE_C:  cd_nstate = (inf.date_valid)   ? GET_BOXNO_C : GET_DATE_C;
        GET_BOXNO_C: cd_nstate = (inf.box_no_valid) ? R_VALID_C   : GET_BOXNO_C;
        // READ DRAM
        R_VALID_C:   cd_nstate = R_WAIT_C;
        R_WAIT_C:    cd_nstate = (inf.C_out_valid)  ? CHK_EXP_C   : R_WAIT_C;
        // CHECK EXPIRE
        CHK_EXP_C:   cd_nstate = CPLT_C;
        // COMPLETE
        CPLT_C:      cd_nstate = IDLE_C;

        default: cd_nstate = IDLE_C;
    endcase
end



//=================================
// INPUT REGISTERS
//=================================
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) action_r <= Make_drink;
    else begin
        if (inf.sel_action_valid)
            action_r <= inf.D.d_act[0];
    end
end
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) type_r <= Black_Tea;
    else begin
        if (state == IDLE)
            type_r <= Black_Tea;
        else if (inf.type_valid)
            type_r <= inf.D.d_type[0];
    end
end
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) size_r <= L;
    else begin
        if (state == IDLE)
            size_r <= L;
        else if (inf.size_valid)
            size_r <= inf.D.d_size[0];
    end
end
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) begin
        M_r <= 0;
        D_r <= 0;
    end
    else begin
        if (state == IDLE) begin
            M_r <= 0;
            D_r <= 0;
        end
        else if (inf.date_valid) begin
            M_r <= inf.D.d_date[0].M;
            D_r <= inf.D.d_date[0].D;
        end
    end
end
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) box_id_r <= 0;
    else begin
        if (state == IDLE)
            box_id_r <= 0;
        else if (inf.box_no_valid)
            box_id_r <= inf.D.d_box_no[0];
    end
end
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) begin
        bt_r  <= 0;
        gt_r  <= 0;
        mlk_r <= 0;
        pj_r  <= 0;
    end
    else begin
        if (state == IDLE) begin
            bt_r  <= 0;
            gt_r  <= 0;
            mlk_r <= 0;
            pj_r  <= 0;
        end
        else if (inf.box_sup_valid) begin
            case(sup_state)
                GET_BT_S : bt_r  <= inf.D.d_ing[0];
                GET_GT_S : gt_r  <= inf.D.d_ing[0];
                GET_MLK_S: mlk_r <= inf.D.d_ing[0];
                GET_PJ_S : pj_r  <= inf.D.d_ing[0];
            endcase
        end
    end
end



//=================================
// OUTPUT LOGIC
//=================================
// TO PATTERN
always_comb begin : OUTPUT_LOGIC
    if (mk_state == CPLT_M || sup_state == CPLT_S || cd_state == CPLT_C) begin
        inf.out_valid = 1'b1;
        inf.err_msg   = err_msg_r;
        inf.complete  = (err_msg_r == No_Err);
    end
    else begin
        inf.out_valid = 1'b0;
        inf.err_msg   = No_Err;
        inf.complete  = 1'b0;
    end
end

always_ff @(posedge clk or negedge inf.rst_n) begin : ERR_MSG_REG
    if (!inf.rst_n) err_msg_r <= No_Err;
    else begin
        if (state == IDLE) begin
            err_msg_r <= No_Err;
        end
        else if (mk_state == CHK_ING_M) begin
            err_msg_r <= (No_Ingredient) ? No_Ing : No_Err;
        end
        else if (sup_state == CHK_OF_S) begin
            err_msg_r <= (Ingredient_Overflow) ? Ing_OF : No_Err;
        end
        else if (mk_state == CHK_EXP_M || cd_state == CHK_EXP_C) begin
            err_msg_r <= (Expired) ? No_Exp : No_Err;
        end
    end
end

always_comb begin : ING_OF_CHK
    Ingredient_Overflow = (
        updated_black_tea[12]  ||
        updated_green_tea[12]  ||
        updated_milk[12]       ||
        updated_pineapple_juice[12] );
        // (bt_r  + received_bal.black_tea       < bt_r ) ||
        // (gt_r  + received_bal.green_tea       < gt_r ) ||
        // (mlk_r + received_bal.milk            < mlk_r) ||
        // (pj_r  + received_bal.pineapple_juice < pj_r ) );
end

always_comb begin : EXP_DATE_CHK
    Expired = (
        (received_bal.M < M_r) ||
        (received_bal.M == M_r && received_bal.D < D_r) );
end

always_comb begin : NO_ING_CHK
    No_Ingredient = (
        updated_black_tea[12]  ||
        updated_green_tea[12]  ||
        updated_milk[12]       ||
        updated_pineapple_juice[12] );
        // (received_bal.black_tea       - bt_cost  > received_bal.black_tea      ) ||
        // (received_bal.green_tea       - gt_cost  > received_bal.green_tea      ) ||
        // (received_bal.milk            - mlk_cost > received_bal.milk           ) ||
        // (received_bal.pineapple_juice - pj_cost  > received_bal.pineapple_juice) );
end

always_comb begin : OVERFLOW_CHECK
    case(state)
        MAKE_DRINK: begin
            updated_black_tea       = received_bal.black_tea        - bt_cost ;
            updated_green_tea       = received_bal.green_tea        - gt_cost ;
            updated_milk            = received_bal.milk             - mlk_cost;
            updated_pineapple_juice = received_bal.pineapple_juice  - pj_cost ;
        end
        SUPPLY: begin
            updated_black_tea       = bt_r  + received_bal.black_tea;
            updated_green_tea       = gt_r  + received_bal.green_tea;
            updated_milk            = mlk_r + received_bal.milk;
            updated_pineapple_juice = pj_r  + received_bal.pineapple_juice;
        end
        default: begin
            updated_black_tea       = 0;
            updated_green_tea       = 0;
            updated_milk            = 0;
            updated_pineapple_juice = 0;
        end
    endcase
end

// TO BRIDGE
always_comb begin
    if (mk_state == R_VALID_M || sup_state == R_VALID_S || cd_state == R_VALID_C) begin
        inf.C_addr = box_id_r;
        inf.C_r_wb = 1'b1;
        inf.C_in_valid = 1'b1;
        inf.C_data_w = 'd0;
    end
    else if (mk_state == W_VALID_M || sup_state == W_VALID_S) begin
        inf.C_addr = box_id_r;
        inf.C_r_wb = 1'b0;
        inf.C_in_valid = 1'b1;
        inf.C_data_w = {
            updated_bal.black_tea,
            updated_bal.green_tea,
            {4'd0, updated_bal.M},
            updated_bal.milk,
            updated_bal.pineapple_juice,
            {3'd0, updated_bal.D}
        };
    end
    else begin
        inf.C_addr = 'd0;
        inf.C_r_wb = 1'b0;
        inf.C_in_valid = 1'b0;
        inf.C_data_w = 'd0;
    end
end

always_ff @(posedge clk or negedge inf.rst_n) begin : RECEIVED_BAL
    if (!inf.rst_n) begin
        received_bal.black_tea       <= 0;
        received_bal.green_tea       <= 0;
        received_bal.milk            <= 0;
        received_bal.pineapple_juice <= 0;
        received_bal.M <= 0;
        received_bal.D <= 0;
    end
    else begin
        if (nstate == IDLE) begin
            received_bal.black_tea       <= 0;
            received_bal.green_tea       <= 0;
            received_bal.milk            <= 0;
            received_bal.pineapple_juice <= 0;
            received_bal.M <= 0;
            received_bal.D <= 0;
        end
        else if (inf.C_out_valid) begin
            received_bal.black_tea       <= inf.C_data_r[63:52];
            received_bal.green_tea       <= inf.C_data_r[51:40];
            received_bal.milk            <= inf.C_data_r[31:20];
            received_bal.pineapple_juice <= inf.C_data_r[19:8];
            received_bal.M <= inf.C_data_r[35:32];
            received_bal.D <= inf.C_data_r[4:0];
        end
    end
end

// TODO: Check if overflow is detected correctly.
// TODO: What if D.Month, D.Day is earlier than received.Month received.Day?
always_comb begin : UPDATED_BAL
    case(state)
        MAKE_DRINK: begin
            // updated_bal.black_tea       = received_bal.black_tea        - bt_cost ;
            // updated_bal.green_tea       = received_bal.green_tea        - gt_cost ;
            // updated_bal.milk            = received_bal.milk             - mlk_cost;
            // updated_bal.pineapple_juice = received_bal.pineapple_juice  - pj_cost ;
            updated_bal.black_tea       = updated_black_tea[11:0];
            updated_bal.green_tea       = updated_green_tea[11:0];
            updated_bal.milk            = updated_milk[11:0];
            updated_bal.pineapple_juice = updated_pineapple_juice[11:0];
            updated_bal.M = received_bal.M;
            updated_bal.D = received_bal.D;
        end
        SUPPLY: begin
            // updated_bal.black_tea       = (bt_r  + received_bal.black_tea < bt_r)       ? 4095 : bt_r  + received_bal.black_tea;
            // updated_bal.green_tea       = (gt_r  + received_bal.green_tea < gt_r)       ? 4095 : gt_r  + received_bal.green_tea;
            // updated_bal.milk            = (mlk_r + received_bal.milk      < mlk_r)      ? 4095 : mlk_r + received_bal.milk;
            // updated_bal.pineapple_juice = (pj_r  + received_bal.pineapple_juice < pj_r) ? 4095 : pj_r  + received_bal.pineapple_juice;
            updated_bal.black_tea       = (updated_black_tea[12]      ) ? 4095 : updated_black_tea[11:0];
            updated_bal.green_tea       = (updated_green_tea[12]      ) ? 4095 : updated_green_tea[11:0];
            updated_bal.milk            = (updated_milk[12]           ) ? 4095 : updated_milk[11:0];
            updated_bal.pineapple_juice = (updated_pineapple_juice[12]) ? 4095 : updated_pineapple_juice[11:0];
            updated_bal.M = M_r;
            updated_bal.D = D_r;
        end
        default: begin
            updated_bal.black_tea       = 0;
            updated_bal.green_tea       = 0;
            updated_bal.milk            = 0;
            updated_bal.pineapple_juice = 0;
            updated_bal.M = 0;
            updated_bal.D = 0;
        end
    endcase
end

always_comb begin : BEV_RATIO
    case(type_r)
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
end

// Add a stage from ratio -> cost
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) begin
        bt_cost  <= 0;
        gt_cost  <= 0;
        mlk_cost <= 0;
        pj_cost  <= 0;
    end
    else begin
        if (state == MAKE_DRINK) begin
            case (size_r)
                L: begin
                    bt_cost  <= bt_ratio  << 2 ;
                    gt_cost  <= gt_ratio  << 2 ;
                    mlk_cost <= mlk_ratio << 2 ;
                    pj_cost  <= pj_ratio  << 2 ;
                end
                M: begin
                    bt_cost  <= (bt_ratio  << 1) + bt_ratio  ;
                    gt_cost  <= (gt_ratio  << 1) + gt_ratio  ;
                    mlk_cost <= (mlk_ratio << 1) + mlk_ratio ;
                    pj_cost  <= (pj_ratio  << 1) + pj_ratio  ;
                end
                S: begin
                    bt_cost  <= bt_ratio  << 1;
                    gt_cost  <= gt_ratio  << 1;
                    mlk_cost <= mlk_ratio << 1;
                    pj_cost  <= pj_ratio  << 1;
                end
            endcase
        end
    end
end
// always_comb begin : BEV_REQUIRED_VOL
//     if (state == MAKE_DRINK) begin
//         case (size_r)
//             L: begin
//                 bt_cost  = bt_ratio  << 2 ;
//                 gt_cost  = gt_ratio  << 2 ;
//                 mlk_cost = mlk_ratio << 2 ;
//                 pj_cost  = pj_ratio  << 2 ;
//             end
//             M: begin
//                 bt_cost  = (bt_ratio  << 1) + bt_ratio  ;
//                 gt_cost  = (gt_ratio  << 1) + gt_ratio  ;
//                 mlk_cost = (mlk_ratio << 1) + mlk_ratio ;
//                 pj_cost  = (pj_ratio  << 1) + pj_ratio  ;
//             end
//             S: begin
//                 bt_cost  = bt_ratio  << 1;
//                 gt_cost  = gt_ratio  << 1;
//                 mlk_cost = mlk_ratio << 1;
//                 pj_cost  = pj_ratio  << 1;
//             end
//             default: begin
//                 bt_cost  = 0;
//                 gt_cost  = 0;
//                 mlk_cost = 0;
//                 pj_cost  = 0;
//             end
//         endcase
//     end
//     else begin
//         bt_cost  = 0;
//         gt_cost  = 0;
//         mlk_cost = 0;
//         pj_cost  = 0;
//     end
// end
endmodule