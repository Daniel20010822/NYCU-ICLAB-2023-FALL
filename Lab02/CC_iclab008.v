`define STATE_BITS 3
`define WIDTH 8
`define WIDTH_SHORT 6

module CC(
    //Input Port
    clk,
    rst_n,
	in_valid,
	mode,
    xi,
    yi,

    //Output Port
    out_valid,
	xo,
	yo
    );

input               clk, rst_n, in_valid;
input       [1:0]   mode;
input       [7:0]   xi, yi;

output reg          out_valid;
output reg  [7:0]   xo, yo;
//==============================================//
//             Parameter and Integer            //
//==============================================//
integer i;
parameter IDLE      = `STATE_BITS'd0;
parameter FETCH     = `STATE_BITS'd1;
parameter MODE0_OUT = `STATE_BITS'd2;
parameter MODE1_CAL = `STATE_BITS'd3;
parameter MODE1_OUT = `STATE_BITS'd4;
parameter MODE2_OUT = `STATE_BITS'd5;
//==============================================//
//            FSM State Declaration             //
//==============================================//
reg [`STATE_BITS - 1:0] current_state, next_state;

//==============================================//
//                 reg declaration              //
//==============================================//
/*Input Blocks*/
reg signed [`WIDTH - 1:0] x_r [0:3];
reg signed [`WIDTH - 1:0] y_r [0:3];
reg  [1:0]  mode_r;

/*Cal Block 1*/
wire signed [`WIDTH-1:0] xuL, xuR, xdL, xdR, yu, yd;
wire signed [`WIDTH:0] dy, dxL, dxR;
wire signed [`WIDTH-1:0] offset_L, offset_R;
wire neg_slope_L, neg_slope_R;
reg  signed [`WIDTH:0] test_pt_L0_x, test_pt_L0_y, test_pt_L0_dx, test_pt_L0_dy;
reg  signed [`WIDTH:0] test_pt_L1_x, test_pt_L1_y, test_pt_L1_dx, test_pt_L1_dy;
reg  signed [`WIDTH:0] test_pt_R0_x, test_pt_R0_y, test_pt_R0_dx, test_pt_R0_dy;
reg  signed [`WIDTH:0] test_pt_R1_x, test_pt_R1_y, test_pt_R1_dx, test_pt_R1_dy;
reg  signed [`WIDTH*2+1:0] SUB0_L0, SUB0_L1, SUB0_R0, SUB0_R1;
reg  signed [`WIDTH*2+1:0] SUB1_L0, SUB1_L1, SUB1_R0, SUB1_R1;
reg  signed [`WIDTH*2+1:0] region_test_L0, region_test_L1, region_test_R0, region_test_R1;
reg  signed [`WIDTH-1:0] xL_current_r, xR_current_r;
reg  signed [`WIDTH-1:0] xL_next, xR_next;
reg  signed [`WIDTH-1:0] x_cnt_r, y_cnt_r;
wire signed [`WIDTH:0] xL_current_r_ext, xR_current_r_ext;
wire signed [`WIDTH:0] x_cnt_r_ext, y_cnt_r_ext;

/*Cal Block 2*/
wire signed [`WIDTH_SHORT-1:0] x0, x1, x2, x3;
wire signed [`WIDTH_SHORT-1:0] y0, y1, y2, y3;
wire signed [`WIDTH_SHORT:0] a, b, c, d, e, f;
reg  signed [`WIDTH_SHORT*2+1:0] ac_p_bd, aa_p_bb, ee_p_ff;
reg  signed [`WIDTH_SHORT*2+1:0] ac_p_bd_r, aa_p_bb_r, ee_p_ff_r;
wire signed [24:0] LHS, RHS;
wire [1:0] relation;

/*Cal Block 3*/
reg  signed [15:0] x0y1, x1y2, x2y3, x3y0;
reg  signed [15:0] x1y0, x2y1, x3y2, x0y3;
reg  signed [15:0] det0, det1, det2, det3;
wire signed [16:0] area_m2signed_r;
wire [16:0] area_m2;
wire [15:0] area;

//==============================================//
//             Current State Block              //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= IDLE;
    end
    else begin
        current_state <= next_state;
    end
end

//==============================================//
//              Next State Block                //
//==============================================//
always @(*) begin
    case(current_state)
        IDLE: begin
            if (in_valid)
                next_state = FETCH;
            else
                next_state = IDLE;
        end
        FETCH: begin
            if (~in_valid) begin
                if (mode_r == 2'd0)
                    next_state = MODE0_OUT;
                else if (mode_r == 2'd1)
                    next_state = MODE1_CAL;
                else
                    next_state = MODE2_OUT;
            end
            else begin
                next_state = FETCH;
            end
        end

        /*Mode 0 Section*/
        MODE0_OUT: begin
            next_state = ({x_cnt_r, y_cnt_r} == {x_r[1], y_r[0]}) ? IDLE : MODE0_OUT;
        end

        /*Mode 1 Section*/
        MODE1_CAL: begin
            next_state = MODE1_OUT;
        end
        MODE1_OUT: begin
            next_state = IDLE;
        end

        /*Mode 2 Section*/
        MODE2_OUT: next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

//==============================================//
//                  Input Block                 //
//==============================================//
/*x0 ~ x4 & y0 ~ y4*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i <= 3; i = i + 1) begin
            x_r[i] <= 8'd0;
            y_r[i] <= 8'd0;
        end
    end
    else if (in_valid) begin
        for (i = 0; i <= 2; i = i + 1) begin
            x_r[i] <= x_r[i+1];
            y_r[i] <= y_r[i+1];
        end
        x_r[3] <= xi;
        y_r[3] <= yi;
    end
    else begin
        for (i = 0; i <= 3; i = i + 1) begin
            x_r[i] <= x_r[i];
            y_r[i] <= y_r[i];
        end
    end
end

/*mode*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode_r <= 2'd0;
    end
    else if (in_valid) begin
        mode_r <= mode;
    end
    else begin
        mode_r <= mode_r;
    end
end

//==============================================//
//              Calculation Block1              //
//==============================================//
/*
  Equivalent Signals:
  xuL = x0  |  yu  = y0  |  dy  = y0 - y2 = -d
  xuR = x1  |  yd  = y2  |  dxL = x0 - x2 = -c
  xdL = x2  |            |  dxR = x1 - x3
  xdR = x3  |            |
*/
assign xuL = x_r[0];
assign xuR = x_r[1];
assign xdL = x_r[2];
assign xdR = x_r[3];
assign yu  = y_r[0];
assign yd  = y_r[2];

assign dy  = { yu[`WIDTH-1],  yu} - { yd[`WIDTH-1],  yd};
assign dxL = {xuL[`WIDTH-1], xuL} - {xdL[`WIDTH-1], xdL};
assign dxR = {xuR[`WIDTH-1], xuR} - {xdR[`WIDTH-1], xdR};
// assign dy  = ~d + 1'd1; // y0 - y2, yu  - yd
// assign dxL = ~c + 1'd1; // x0 - x2, xuL - xdL
// assign dxR = x_r[1] - x_r[3]; // xuR - xdR

/*Next corner calculator*/
assign offset_L = (dxL > dy || dxL <= -dy) ? dxL / dy : `WIDTH'd0;
assign offset_R = (dxR > dy || dxR <= -dy) ? dxR / dy : `WIDTH'd0;

assign neg_slope_L = (xuL < xdL);
assign neg_slope_R = (xuR < xdR);

// extend original signals for further uses
assign x_cnt_r_ext = {x_cnt_r[`WIDTH-1], x_cnt_r};
assign y_cnt_r_ext = {y_cnt_r[`WIDTH-1], y_cnt_r};
assign xL_current_r_ext = {xL_current_r[`WIDTH-1], xL_current_r};
assign xR_current_r_ext = {xR_current_r[`WIDTH-1], xR_current_r};

always @(*) begin
    test_pt_L0_x  = xL_current_r_ext + offset_L;
    test_pt_L0_y  = y_cnt_r_ext + 1'd1;
    test_pt_L0_dx = test_pt_L0_x - x_r[2];
    test_pt_L0_dy = test_pt_L0_y - y_r[2];
    SUB0_L0 = dy  * test_pt_L0_dx;
    SUB1_L0 = dxL * test_pt_L0_dy;
    region_test_L0 = SUB0_L0 - SUB1_L0;
end
always @(*) begin
    test_pt_L1_x  = (neg_slope_L) ? xL_current_r_ext + offset_L - 1'd1 : xL_current_r_ext + offset_L + 1'd1;
    test_pt_L1_y  = y_cnt_r_ext + 1'd1;
    test_pt_L1_dx = test_pt_L1_x - x_r[2];
    test_pt_L1_dy = test_pt_L1_y - y_r[2];
    SUB0_L1 = dy  * test_pt_L1_dx;
    SUB1_L1 = dxL * test_pt_L1_dy;
    region_test_L1 = SUB0_L1 - SUB1_L1;
end
always @(*) begin
    test_pt_R0_x  = xR_current_r_ext + offset_R;
    test_pt_R0_y  = y_cnt_r_ext + 1'd1;
    test_pt_R0_dx = test_pt_R0_x - x_r[3];
    test_pt_R0_dy = test_pt_R0_y - y_r[3];
    SUB0_R0 = dy  * test_pt_R0_dx;
    SUB1_R0 = dxR * test_pt_R0_dy;
    region_test_R0 = SUB0_R0 - SUB1_R0;
end
always @(*) begin
    test_pt_R1_x  = (neg_slope_R) ? xR_current_r_ext + offset_R - 1'd1 : xR_current_r_ext + offset_R + 1'd1;
    test_pt_R1_y  = y_cnt_r_ext + 1'd1;
    test_pt_R1_dx = test_pt_R1_x - x_r[3];
    test_pt_R1_dy = test_pt_R1_y - y_r[3];
    SUB0_R1 = dy  * test_pt_R1_dx;
    SUB1_R1 = dxR * test_pt_R1_dy;
    region_test_R1 = SUB0_R1 - SUB1_R1;
end

// xL_current
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        xL_current_r <= 8'd0;
    end
    else if (current_state == FETCH) begin
        xL_current_r <= x_r[2];
    end
    else if (x_cnt_r == xR_current_r) begin
        xL_current_r <= xL_next;
    end
    else begin
        xL_current_r <= xL_current_r;
    end
end

// xL_next
always @(*) begin
    if (xuL == xdL) begin
        xL_next = test_pt_L0_x;
    end
    else if (!neg_slope_L) begin
        if (~(|region_test_L1)) // If test point 1 is on the line
            xL_next = test_pt_L1_x;
        else if (region_test_L0[`WIDTH*2+1] == region_test_L1[`WIDTH*2+1]) // If two test points are all on the left
            xL_next = test_pt_L1_x;
        else
            xL_next = test_pt_L0_x;
    end
    else begin
        if (~(|region_test_L0)) // If test point 0 is on the line
            xL_next = test_pt_L0_x;
        else if (region_test_L0[`WIDTH*2+1] == region_test_L1[`WIDTH*2+1]) // If two test points are all on the left
            xL_next = test_pt_L0_x;
        else
            xL_next = test_pt_L1_x;
    end
end

// xR_current
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        xR_current_r <= 8'd0;
    end
    else if (current_state == FETCH) begin
        xR_current_r <= x_r[3];
    end
    else if (x_cnt_r == xR_current_r) begin
        xR_current_r <= xR_next;
    end
    else begin
        xR_current_r <= xR_current_r;
    end
end

// xR_next
always @(*) begin
    if (xuR == xdR) begin
        xR_next = test_pt_R0_x;
    end
    else if (!neg_slope_R) begin
        if (~(|region_test_R1)) // If test point 1 is on the line
            xR_next = test_pt_R1_x;
        else if (region_test_R0[`WIDTH*2+1] == region_test_R1[`WIDTH*2+1]) // If two test points are all on the left
            xR_next = test_pt_R1_x;
        else
            xR_next = test_pt_R0_x;
    end
    else begin
        if (~(|region_test_R0)) // If test point 0 is on the line
            xR_next = test_pt_R0_x;
        else if (region_test_R0[`WIDTH*2+1] == region_test_R1[`WIDTH*2+1]) // If two test points are all on the left
            xR_next = test_pt_R0_x;
        else
            xR_next = test_pt_R1_x;
    end
end



/*Coordinate output count*/
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_cnt_r <= 8'd0;
    end
    else if (current_state == FETCH) begin
        x_cnt_r <= x_r[2]; // xdL
    end
    else if (current_state == MODE0_OUT) begin
        if (x_cnt_r == xR_current_r)
            x_cnt_r <= xL_next;
        else
            x_cnt_r <= x_cnt_r + 1'd1;
    end
    else begin
        x_cnt_r <= x_cnt_r;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y_cnt_r <= 8'd0;
    end
    else if (current_state == FETCH) begin
        y_cnt_r <= y_r[2]; // yd
    end
    else if (current_state == MODE0_OUT) begin
        if (x_cnt_r == xR_current_r)
            y_cnt_r <= y_cnt_r + 1'd1;
        else
            y_cnt_r <= y_cnt_r;
    end
    else begin
        y_cnt_r <= y_cnt_r;
    end
end



//==============================================//
//              Calculation Block2              //
//==============================================//
assign {x0, x1, x2, x3} = {x_r[0][5:0], x_r[1][5:0], x_r[2][5:0], x_r[3][5:0]};
assign {y0, y1, y2, y3} = {y_r[0][5:0], y_r[1][5:0], y_r[2][5:0], y_r[3][5:0]};

assign a = y1 - y0;
assign b = x0 - x1;
assign c = x2 - x0;
assign d = y2 - y0;
assign e = x2 - x3;
assign f = y2 - y3;

// //TODO: Convert to xuL, xdL form
// assign a = y_r[1] - y_r[0];
// assign b = x_r[0] - x_r[1];
// assign c = x_r[2] - x_r[0];
// assign d = y_r[2] - y_r[0];
// assign e = x_r[2] - x_r[3];
// assign f = y_r[2] - y_r[3];

assign ac_p_bd = a * c + b * d;
assign aa_p_bb = a * a + b * b;
assign ee_p_ff = e * e + f * f;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ac_p_bd_r <= 14'd0;
        aa_p_bb_r <= 14'd0;
        ee_p_ff_r <= 14'd0;
    end
    else if (current_state == MODE1_CAL) begin
        ac_p_bd_r <= ac_p_bd;
        aa_p_bb_r <= aa_p_bb;
        ee_p_ff_r <= ee_p_ff;
    end
    else if (current_state == MODE1_OUT) begin
        ac_p_bd_r <= ac_p_bd_r;
        aa_p_bb_r <= aa_p_bb_r;
        ee_p_ff_r <= ee_p_ff_r;
    end
    else begin
        ac_p_bd_r <= 14'd0;
        aa_p_bb_r <= 14'd0;
        ee_p_ff_r <= 14'd0;
    end
end

assign LHS = ac_p_bd_r * ac_p_bd_r;
assign RHS = aa_p_bb_r * ee_p_ff_r;

// L.H.S.          <=> R.H.S.
// (ax + by + c)^2 <=> (a^2 + b^2) * radius^2
assign relation = (LHS  > RHS) ? 2'd0 :
                  (LHS == RHS) ? 2'd2 : 2'd1;

//==============================================//
//              Calculation Block3              //
//==============================================//
always @(*) begin
    x0y1 = x_r[0] * y_r[1];
    x1y0 = x_r[1] * y_r[0];
    det0 = x0y1 - x1y0;
end
always @(*) begin
    x1y2 = x_r[1] * y_r[2];
    x2y1 = x_r[2] * y_r[1];
    det1 = x1y2 - x2y1;
end
always @(*) begin
    x2y3 = x_r[2] * y_r[3];
    x3y2 = x_r[3] * y_r[2];
    det2 = x2y3 - x3y2;
end
always @(*) begin
    x3y0 = x_r[3] * y_r[0];
    x0y3 = x_r[0] * y_r[3];
    det3 = x3y0 - x0y3;
end

assign area_m2signed_r = det0 + det1 + det2 + det3;
assign area_m2 = (area_m2signed_r[16]) ? ~area_m2signed_r + 1'd1 : area_m2signed_r;
assign area = area_m2 >> 1;

//==============================================//
//                Output Block                  //
//==============================================//
always @(*) begin
    case (current_state)
        MODE0_OUT: {xo, yo} = {x_cnt_r, y_cnt_r};
        MODE1_OUT: {xo, yo} = {14'd0, relation};
        MODE2_OUT: {xo, yo} = {area[15:8], area[7:0]};
        default:   {xo, yo} = {8'd0, 8'd0};
    endcase
end

always @(*) begin
    case (current_state)
        MODE0_OUT: out_valid = 1'd1;
        MODE1_OUT: out_valid = 1'd1;
        MODE2_OUT: out_valid = 1'd1;
        default: out_valid = 1'd0;
    endcase
end

endmodule
