module bridge(input clk, INF.bridge_inf inf);

typedef enum logic [1:0] {IDLE, READ, WRITE, OUT} axi_state_t;
typedef enum logic [3:0] {
    IDLE_R,
    ARVALID_HIGH,
    RREADY_HIGH
} read_state_t;

typedef enum logic [3:0] {
    IDLE_W,
    AWVALID_HIGH,
    WVALID_HIGH,
    WAIT_BVALID
} write_state_t;

axi_state_t    axi_state, axi_nstate;
read_state_t   r_state, r_nstate;
write_state_t  w_state, w_nstate;

logic [63:0] out_data_r;



//================================================================
// logic
//================================================================
// Outputs (C_out_valid, C_data_r)
assign inf.C_out_valid = (axi_state == OUT);
always_comb begin
    if (axi_state == OUT) inf.C_data_r = out_data_r;
    else                  inf.C_data_r = 0;
end

always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) out_data_r <= 0;
    else begin
        if (inf.R_VALID && inf.R_READY)
            out_data_r <= inf.R_DATA;
    end
end

// Read (AR_VALID, AR_ADDR, R_READY)
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) inf.AR_ADDR <= 17'h00000;
    else begin
        if (inf.C_in_valid) inf.AR_ADDR <= {5'b10000, {4'd0, inf.C_addr} << 3};
    end
end
assign inf.AR_VALID = (r_state == ARVALID_HIGH);
assign inf.R_READY  = (r_state == RREADY_HIGH);

// Write (AW_VALID, AW_ADDR, W_VALID, W_DATA, B_READY)
always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) inf.AW_ADDR <= 17'h00000;
    else begin
        if (inf.C_in_valid) inf.AW_ADDR <= {5'b10000, {4'd0, inf.C_addr} << 3};
    end
end

assign inf.AW_VALID = (w_state == AWVALID_HIGH);
assign inf.W_VALID  = (w_state == WVALID_HIGH);
assign inf.B_READY  = (w_state == WVALID_HIGH || w_state == WAIT_BVALID);

always_ff @(posedge clk or negedge inf.rst_n) begin
    if (!inf.rst_n) inf.W_DATA <= 64'd0;
    else begin
        if (inf.C_in_valid) inf.W_DATA <= inf.C_data_w;
    end
end



//================================================================
// state
//================================================================
always_ff @(posedge clk or negedge inf.rst_n) begin : AXI_FSM_SEQ
    if (!inf.rst_n)  axi_state <= IDLE;
    else             axi_state <= axi_nstate;
end
always_comb begin : AXI_FSM_COMB
    case(axi_state)
        IDLE: begin
            if (inf.C_in_valid) axi_nstate = (inf.C_r_wb) ? READ : WRITE;
            else                axi_nstate = IDLE;
        end

        READ:   axi_nstate = (inf.R_VALID && inf.R_READY) ? OUT : READ;
        WRITE:  axi_nstate = (inf.B_VALID && inf.B_READY) ? OUT : WRITE;
        OUT:    axi_nstate = IDLE;
        default: axi_nstate = IDLE;
    endcase
end

always_ff @(posedge clk or negedge inf.rst_n) begin : AXI_READ_FSM_SEQ
    if (!inf.rst_n)  r_state <= IDLE_R;
    else             r_state <= r_nstate;
end
always_comb begin : AXI_READ_FSM_COMB
    case(r_state)
        IDLE_R: begin
            if (axi_state == IDLE)  r_nstate = (inf.C_in_valid && inf.C_r_wb == 1) ? ARVALID_HIGH : IDLE_R;
            else                    r_nstate = IDLE_R;
        end

        ARVALID_HIGH:       r_nstate = (inf.AR_READY) ? RREADY_HIGH : ARVALID_HIGH;
        RREADY_HIGH:        r_nstate = (inf.R_VALID)  ? IDLE_R : RREADY_HIGH;
        default: r_nstate = IDLE_R;
    endcase
end


always_ff @(posedge clk or negedge inf.rst_n) begin : AXI_WRITE_FSM_SEQ
    if (!inf.rst_n)  w_state <= IDLE_W;
    else             w_state <= w_nstate;
end
always_comb begin : AXI_WRITE_FSM_COMB
    case(w_state)
        IDLE_W: begin
            if (axi_state == IDLE)  w_nstate = (inf.C_in_valid && inf.C_r_wb == 0) ? AWVALID_HIGH : IDLE_W;
            else                    w_nstate = IDLE_W;
        end

        AWVALID_HIGH:  w_nstate = (inf.AW_READY) ? WVALID_HIGH : AWVALID_HIGH;
        WVALID_HIGH:   w_nstate = (inf.W_READY)  ? WAIT_BVALID : WVALID_HIGH;
        WAIT_BVALID:   w_nstate = (inf.B_VALID)  ? IDLE_W      : WAIT_BVALID;
        default: w_nstate = IDLE_W;
    endcase
end

endmodule