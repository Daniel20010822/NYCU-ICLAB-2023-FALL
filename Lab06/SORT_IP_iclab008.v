//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//    (C) Copyright System Integration and Silicon Implementation Laboratory
//    All Right Reserved
//		Date		: 2023/10
//		Version		: v1.0
//   	File Name   : SORT_IP.v
//   	Module Name : SORT_IP
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module SORT_IP #(parameter IP_WIDTH = 8) (
    // Input signals
    IN_character, IN_weight,
    // Output signals
    OUT_character
);

//================================================================
// INPUT & OUTPUT
//================================================================
input  [IP_WIDTH*4-1:0]  IN_character;
input  [IP_WIDTH*5-1:0]  IN_weight;

output [IP_WIDTH*4-1:0]  OUT_character;

//================================================================
// REG & WIRES
//================================================================
reg [3:0] character [0:7];
reg [4:0] weight    [0:7];
reg [8:0] pair      [0:7];
reg [8:0] layer     [0:5][0:7];
//================================================================
// DESIGN
//================================================================
integer idx;
// always @(*) begin
//     for (idx = 0; idx < IP_WIDTH; idx = idx + 1) begin
//         character[idx] = IN_character[(IP_WIDTH - idx - 1)*4 + 3 -: 4];
//         weight[idx]    = IN_weight[(IP_WIDTH - idx - 1)*5 + 4 -: 5];
//     end
// end
always @(*) begin
    for (idx = 0; idx < 8; idx = idx + 1) begin
        character[idx] = 0;
        weight[idx]    = 0;
    end
    for (idx = 0; idx < IP_WIDTH; idx = idx + 1) begin
        character[idx] = IN_character[idx*4 + 3 -: 4];
        weight[idx]    = IN_weight[idx*5 + 4 -: 5];
    end
end
always @(*) begin
    for (idx = 0; idx < 8; idx = idx + 1) begin
        pair[idx] = {weight[idx], character[idx]};
    end
end

always @(*) begin
    {layer[0][0], layer[0][2]} = (pair[0][8:4] > pair[2][8:4] || (pair[0][8:4] == pair[2][8:4] && pair[0][3:0] > pair[2][3:0])) ? {pair[0], pair[2]} : {pair[2], pair[0]};
    {layer[0][1], layer[0][3]} = (pair[1][8:4] > pair[3][8:4] || (pair[1][8:4] == pair[3][8:4] && pair[1][3:0] > pair[3][3:0])) ? {pair[1], pair[3]} : {pair[3], pair[1]};
    {layer[0][4], layer[0][6]} = (pair[4][8:4] > pair[6][8:4] || (pair[4][8:4] == pair[6][8:4] && pair[4][3:0] > pair[6][3:0])) ? {pair[4], pair[6]} : {pair[6], pair[4]};
    {layer[0][5], layer[0][7]} = (pair[5][8:4] > pair[7][8:4] || (pair[5][8:4] == pair[7][8:4] && pair[5][3:0] > pair[7][3:0])) ? {pair[5], pair[7]} : {pair[7], pair[5]};

    {layer[1][0], layer[1][4]} = (layer[0][0][8:4] > layer[0][4][8:4] || (layer[0][0][8:4] == layer[0][4][8:4] && layer[0][0][3:0] > layer[0][4][3:0])) ? {layer[0][0], layer[0][4]} : {layer[0][4], layer[0][0]};
    {layer[1][1], layer[1][5]} = (layer[0][1][8:4] > layer[0][5][8:4] || (layer[0][1][8:4] == layer[0][5][8:4] && layer[0][1][3:0] > layer[0][5][3:0])) ? {layer[0][1], layer[0][5]} : {layer[0][5], layer[0][1]};
    {layer[1][2], layer[1][6]} = (layer[0][2][8:4] > layer[0][6][8:4] || (layer[0][2][8:4] == layer[0][6][8:4] && layer[0][2][3:0] > layer[0][6][3:0])) ? {layer[0][2], layer[0][6]} : {layer[0][6], layer[0][2]};
    {layer[1][3], layer[1][7]} = (layer[0][3][8:4] > layer[0][7][8:4] || (layer[0][3][8:4] == layer[0][7][8:4] && layer[0][3][3:0] > layer[0][7][3:0])) ? {layer[0][3], layer[0][7]} : {layer[0][7], layer[0][3]};

    {layer[2][0], layer[2][1]} = (layer[1][0][8:4] > layer[1][1][8:4] || (layer[1][0][8:4] == layer[1][1][8:4] && layer[1][0][3:0] > layer[1][1][3:0])) ? {layer[1][0], layer[1][1]} : {layer[1][1], layer[1][0]};
    {layer[2][2], layer[2][3]} = (layer[1][2][8:4] > layer[1][3][8:4] || (layer[1][2][8:4] == layer[1][3][8:4] && layer[1][2][3:0] > layer[1][3][3:0])) ? {layer[1][2], layer[1][3]} : {layer[1][3], layer[1][2]};
    {layer[2][4], layer[2][5]} = (layer[1][4][8:4] > layer[1][5][8:4] || (layer[1][4][8:4] == layer[1][5][8:4] && layer[1][4][3:0] > layer[1][5][3:0])) ? {layer[1][4], layer[1][5]} : {layer[1][5], layer[1][4]};
    {layer[2][6], layer[2][7]} = (layer[1][6][8:4] > layer[1][7][8:4] || (layer[1][6][8:4] == layer[1][7][8:4] && layer[1][6][3:0] > layer[1][7][3:0])) ? {layer[1][6], layer[1][7]} : {layer[1][7], layer[1][6]};

    {layer[3][2], layer[3][4]} = (layer[2][2][8:4] > layer[2][4][8:4] || (layer[2][2][8:4] == layer[2][4][8:4] && layer[2][2][3:0] > layer[2][4][3:0])) ? {layer[2][2], layer[2][4]} : {layer[2][4], layer[2][2]};
    {layer[3][3], layer[3][5]} = (layer[2][3][8:4] > layer[2][5][8:4] || (layer[2][3][8:4] == layer[2][5][8:4] && layer[2][3][3:0] > layer[2][5][3:0])) ? {layer[2][3], layer[2][5]} : {layer[2][5], layer[2][3]};
    layer[3][0] = layer[2][0];
    layer[3][1] = layer[2][1];
    layer[3][6] = layer[2][6];
    layer[3][7] = layer[2][7];

    {layer[4][1], layer[4][4]} = (layer[3][1][8:4] > layer[3][4][8:4] || (layer[3][1][8:4] == layer[3][4][8:4] && layer[3][1][3:0] > layer[3][4][3:0])) ? {layer[3][1], layer[3][4]} : {layer[3][4], layer[3][1]};
    {layer[4][3], layer[4][6]} = (layer[3][3][8:4] > layer[3][6][8:4] || (layer[3][3][8:4] == layer[3][6][8:4] && layer[3][3][3:0] > layer[3][6][3:0])) ? {layer[3][3], layer[3][6]} : {layer[3][6], layer[3][3]};
    layer[4][0] = layer[3][0];
    layer[4][2] = layer[3][2];
    layer[4][5] = layer[3][5];
    layer[4][7] = layer[3][7];

    {layer[5][1], layer[5][2]} = (layer[4][1][8:4] > layer[4][2][8:4] || (layer[4][1][8:4] == layer[4][2][8:4] && layer[4][1][3:0] > layer[4][2][3:0])) ? {layer[4][1], layer[4][2]} : {layer[4][2], layer[4][1]};
    {layer[5][3], layer[5][4]} = (layer[4][3][8:4] > layer[4][4][8:4] || (layer[4][3][8:4] == layer[4][4][8:4] && layer[4][3][3:0] > layer[4][4][3:0])) ? {layer[4][3], layer[4][4]} : {layer[4][4], layer[4][3]};
    {layer[5][5], layer[5][6]} = (layer[4][5][8:4] > layer[4][6][8:4] || (layer[4][5][8:4] == layer[4][6][8:4] && layer[4][5][3:0] > layer[4][6][3:0])) ? {layer[4][5], layer[4][6]} : {layer[4][6], layer[4][5]};
    layer[5][0] = layer[4][0];
    layer[5][7] = layer[4][7];
end

genvar i;
generate
    for (i = 0; i < IP_WIDTH; i = i + 1) begin: loop
        assign OUT_character[(4*i)+3 -: 4] = layer[5][IP_WIDTH - i - 1][3:0];
    end
endgenerate

endmodule