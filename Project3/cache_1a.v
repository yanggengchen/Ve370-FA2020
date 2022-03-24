`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/19 21:58:55
// Design Name: 
// Module Name: cache_1a
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


module cache_1a(cpu_write, cpu_address, cpu_write_data, cpu_read_data, cpu_hit, mem_write, mem_address, mem_write_data, mem_read_data);
      input       cpu_write;
      input       [9:0]cpu_address;
      input       [31:0]cpu_write_data;
      output reg  [31:0]cpu_read_data;
      output reg  cpu_hit;
      output reg  mem_write;
      output reg  [9:0]mem_address;
      output reg  [127:0]mem_write_data;
      input       [127:0]mem_read_data;
// we have 4 blocks, each block we have 4 words, so we need the size to be 4*4 = 16 
// the length of each word in line is 128/4 = 32 
      reg         [31:0]cache[15:0];
      reg         V[3:0];
// the size of tag is 10-2(byte off)-2(word off)-2(index) = 4
      reg         [3:0]tag[3:0];
// initialize all the reg
      integer     i;
      integer     j;
      wire        [3:0]my_tag;
      wire        [1:0]my_index;
      wire        [3:0]cache_address;
      wire        [1:0]my_wordoff;
      wire        [5:0]my_address;
      assign      my_tag = cpu_address[9:6];
      assign      my_index = cpu_address[5:4];
      assign      cache_address = cpu_address[5:2];
      assign      my_wordoff = cpu_address[3:2];
      assign      my_address = cpu_address[9:4];

      initial begin
            for(i = 0 ; i < 16 ; i = i + 1)begin
                  cache[i] = 32'b0;
            end
            for(j = 0 ; j < 4 ; j = j + 1)begin
                  V[j] = 0;
                  tag[j] = 4'b0;
            end
      end
      
      always@(*)begin
      //write cache and write mem
            if(cpu_write == 1)begin                  
                  mem_address <= my_address;
                  if(V[my_index] == 1 && tag[my_index] == my_tag)begin
                        cpu_hit <= 1;
                        //write cache  
                        cache[cache_address]   <= cpu_write_data;
                        //write mem
                        mem_write_data[127:96] <= cache[my_index * 4];
                        mem_write_data[95:64]  <= cache[my_index * 4 + 1];
                        mem_write_data[63:32]  <= cache[my_index * 4 + 2];
                        mem_write_data[31:0]   <= cache[my_index * 4 + 3];
                  end
                  else begin
                        cpu_hit <= 0;
                        V[my_index] <= 1;
                        tag[my_index] <= my_tag;
                        mem_write <= 0; // to read data from mem
                        //fetch block
                        cache[my_index * 4] <= mem_read_data[127:96];
                        cache[my_index * 4 + 1] <= mem_read_data[95:64];
                        cache[my_index * 4 + 2] <= mem_read_data[63:32];
                        cache[my_index * 4 + 3] <= mem_read_data[31:0];
                        //write cache
                        cache[cache_address]   <= cpu_write_data;
                        //write mem
                        mem_write_data[127:96] <= cache[my_index * 4];
                        mem_write_data[95:64]  <= cache[my_index * 4 + 1];
                        mem_write_data[63:32]  <= cache[my_index * 4 + 2];
                        mem_write_data[31:0]   <= cache[my_index * 4 + 3];

                  end
                  mem_write <= 1; //write mem
            end
            else begin
                  mem_write <= 0;
                  mem_address <= my_address;
                  if(V[my_index] == 1 && tag[my_index] == my_tag)begin
                        cpu_hit <= 1;
                        cpu_read_data <= cache[cache_address];
                  end
                  else begin
                        cpu_hit <= 0;
                        V[my_index] <= 1;
                        tag[my_index] <= my_tag;
                        //fetch block
                        cache[my_index * 4] <= mem_read_data[127:96];
                        cache[my_index * 4 + 1] <= mem_read_data[95:64];
                        cache[my_index * 4 + 2] <= mem_read_data[63:32];
                        cache[my_index * 4 + 3] <= mem_read_data[31:0];
                        //output
                        cpu_read_data <= cache[cache_address];
                  end
            end
      end
endmodule
