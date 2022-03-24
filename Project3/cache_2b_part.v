`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/21 10:19:34
// Design Name: 
// Module Name: cache_2b_part
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module cache_2b_part(
input read_or_write_in, //one bit for read or write input, 0 for read, 1 for write
input [9:0] address_in, //the address for read or write
input [31:0] write_data_in, //the data for writing in the memory
input [127:0] read_data_in, //data from the main meory
output reg hit,//decide if the address hit the cache
output reg [9:0] address_out, //the address for requsting main memory
output reg [31:0] read_data_out, //output the reading data
output reg [127:0] write_data_out, //the data for writing the dirty block
output reg read_or_write_out //the read or write out to the main memory
);
/*for each block, tag field = 5, valid size = 1, dirty size =1, word size = 4*32 =128, the size is 128+5+1+1=135*/
reg [134:0] cache [0:3];
reg LRU1,LRU2;
reg [1:0] blockindex;

integer i;
initial begin//initialize the cache memory and other registers
    for (i=0;i<4;i=i+1) begin
        cache[i]=135'b0;
    end
    hit=1'b0;
    LRU1=1'b0;
    LRU2=1'b0;
    blockindex=2'b00;
end

always @(read_or_write_in, address_in) begin
    hit<=1'b0;
    address_out<=address_in;
    if (address_in[4]==1'b0) begin//kdetermine if the address hit and upgrade the LRU block
        if (address_in[9:5]==cache[0][132:128] && cache[0][134]==1'b1) begin
            hit<=1'b1;
            blockindex<=2'b00;
            LRU1<=1'b0;
        end
        else if (address_in[9:5]==cache[1][132:128] && cache[1][134]==1'b1) begin
            hit<=1'b1;
            blockindex<=2'b01;
            LRU1<=1'b1;
        end
    end
    if (address_in[4]==1'b1) begin
        if (address_in[9:5]==cache[2][132:128] && cache[2][134]==1'b1) begin
            hit<=1'b1;
            blockindex<=2'b10;
            LRU2<=1'b0;
        end
        else if (address_in[9:5]==cache[3][132:128] && cache[3][134]==1'b1) begin
            hit<=1'b1;
            blockindex<=2'b11;
            LRU2<=1'b1;
        end
    end
    if (hit==1'b0)begin//get the block needed to be replaced and upgrade the LRU block in the set
        if (address_in[4]==1'b0) begin
            if (cache[0][134]==1'b0) begin
                blockindex<=2'b00;
                LRU1<=1'b0;
            end
            else if (cache[1][134]==1'b0) begin
                blockindex<=2'b01;
                LRU1<=1'b1;
            end
            else begin
                if (LRU1==1'b0) begin
                    blockindex<=2'b01;
                    LRU1<=1'b1;
                end 
                else begin
                    blockindex<=2'b00;
                    LRU1<=1'b0;
                end
            end
        end
        if (address_in[4]==1'b1) begin
            if (cache[2][134]==1'b0) begin
                blockindex<=2'b10;
                LRU2<=1'b0;
            end
            else if (cache[3][134]==1'b0) begin
                blockindex<=2'b11;
                LRU2<=1'b1;
            end
            else begin
                if (LRU2==1'b0) begin
                    blockindex<=2'b11;
                    LRU2<=1'b1;
                end 
                else begin
                    blockindex<=2'b10;
                    LRU2<=1'b0;
                end
            end
        end
        if (cache[blockindex][133]==1'b1) begin//deal with the dirty case
            read_or_write_out<=1'b1;
            write_data_out<=cache[blockindex];
            address_out<={cache[blockindex][132:128],address_in[4],4'b0};
        end
        read_or_write_out<=1'b0;//get the data from the main memory
        cache[blockindex][132:128]<=address_in[9:5];
        address_out<=address_in;
        cache[blockindex][127:0]<=read_data_in;
        cache[blockindex][133]<=1'b0;//the dirty signal become zero after reading.
        cache[blockindex][134]<=1'b1;//the valid signal become valid after read the data.
    end
    if (read_or_write_in==1'b0) begin//read case
        case (address_in[3:2])
            2'b00: read_data_out<=cache[blockindex][31:0];
            2'b01: read_data_out<=cache[blockindex][63:32];
            2'b10: read_data_out<=cache[blockindex][95:64];
            2'b11: read_data_out<=cache[blockindex][127:96];
            default: read_data_out<=32'b0;
        endcase
    end
    if (read_or_write_in==1'b1) begin//write case
        case (address_in[3:2])
            2'b00: cache[blockindex][31:0]<=write_data_in;
            2'b01: cache[blockindex][63:32]<=write_data_in;
            2'b10: cache[blockindex][95:64]<=write_data_in;
            2'b11: cache[blockindex][127:96]<=write_data_in;
        endcase
        cache[blockindex][133]<=1'b1;
    end
end
endmodule
