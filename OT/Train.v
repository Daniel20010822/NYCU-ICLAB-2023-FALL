module Train(
    //Input Port
    clk,
    rst_n,
	in_valid,
	data,

    //Output Port
    out_valid,
	result
);

input        clk;
input 	     in_valid;
input        rst_n;
input  [3:0] data;
output   reg out_valid;
output   reg result;

// TRAIN_STATE
parameter WAITING    = 2'd0;
parameter IN_STATION = 2'd1;
parameter DEPARTED   = 2'd2;
parameter BLOCKED    = 2'd3;
// FSM
parameter IDLE       = 2'd0;
parameter INPUT      = 2'd1;
parameter OUTPUT     = 2'd2;
integer i;

reg [2:0] cs, ns;
reg [1:0] train_state_r     [1:10];
reg [1:0] train_state_next  [1:10];
reg       cur_result;

// FSM
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  cs <= 0;
    else         cs <= ns;
end
always @(*) begin
    case (cs)
        IDLE: begin
            if (in_valid)  ns = INPUT;
            else           ns = IDLE;
        end
        INPUT: begin
            if (~in_valid) ns = OUTPUT;
            else           ns = INPUT;
        end
        OUTPUT: begin
            ns = IDLE;
        end
        default: ns = IDLE;
    endcase
end


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 1; i <= 10; i = i + 1)
            train_state_r[i] <= WAITING;
    end
    else if (cs == IDLE) begin
        for (i = 1; i <= 10; i = i + 1)
            train_state_r[i] <= WAITING;
    end
    else if (cs == INPUT && in_valid) begin
        for (i = 1; i <= 10; i = i + 1)
            train_state_r[i] <= train_state_next[i];
    end
end

always @(*) begin
    if (cs == INPUT && in_valid) begin
    // if (cs == INPUT || cs == OUTPUT) begin
        if ((   train_state_r[2]  == IN_STATION || train_state_r[3]  == IN_STATION || train_state_r[4]  == IN_STATION || train_state_r[5]  == IN_STATION || train_state_r[6]  == IN_STATION ||
                train_state_r[7]  == IN_STATION || train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd1) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 1) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if ((  train_state_r[3]  == IN_STATION || train_state_r[4]  == IN_STATION || train_state_r[5]  == IN_STATION || train_state_r[6]  == IN_STATION ||
                    train_state_r[7]  == IN_STATION || train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd2) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 2) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[4]  == IN_STATION || train_state_r[5]  == IN_STATION || train_state_r[6]  == IN_STATION || train_state_r[7]  == IN_STATION ||
                   train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd3) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 3) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[5]  == IN_STATION || train_state_r[6]  == IN_STATION || train_state_r[7]  == IN_STATION ||
                   train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd4) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 4) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[6]  == IN_STATION || train_state_r[7]  == IN_STATION || train_state_r[8]  == IN_STATION ||
                   train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd5) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 5) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[7]  == IN_STATION || train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd6) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 6) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[8]  == IN_STATION || train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd7) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 7) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[9]  == IN_STATION || train_state_r[10] == IN_STATION   ) && data == 'd8) begin

            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 8) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end

        else if (( train_state_r[10] == IN_STATION   ) && data == 'd9) begin
            for (i = 1; i <= 10; i = i + 1) begin
                if (i == 9) train_state_next[i] = BLOCKED;
                else        train_state_next[i] = train_state_r[i];
            end
        end


        else begin
            for (i = 1; i <= 10; i = i + 1) begin
                if (i == data)                              train_state_next[i] = DEPARTED;
                else if (i < data) begin
                    if      (train_state_r[i] == BLOCKED)   train_state_next[i] = BLOCKED;
                    else if (train_state_r[i] == DEPARTED)  train_state_next[i] = DEPARTED;
                    else                                    train_state_next[i] = IN_STATION;
                end
                else                                        train_state_next[i] = train_state_r[i];
            end
        end
    end
    else begin
        for (i = 1; i <= 10; i = i + 1)
            train_state_next[i] = WAITING;
    end
end

always @(*) begin
    cur_result = (
        train_state_r[1] != BLOCKED &&
        train_state_r[2] != BLOCKED &&
        train_state_r[3] != BLOCKED &&
        train_state_r[4] != BLOCKED &&
        train_state_r[5] != BLOCKED &&
        train_state_r[6] != BLOCKED &&
        train_state_r[7] != BLOCKED &&
        train_state_r[8] != BLOCKED &&
        train_state_r[9] != BLOCKED &&
        train_state_r[10] != BLOCKED  );
end

always @(*) begin
    if (cs == OUTPUT) begin
        out_valid = 1;
        result = cur_result;
    end
    else begin
        out_valid = 0;
        result = 0;
    end
end
endmodule