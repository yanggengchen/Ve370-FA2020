`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/11/21 11:21:48
// Design Name: 
// Module Name: cache_1b
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


module cache_1b(cpu_write, cpu_address, cpu_write_data, cpu_read_data, cpu_hit, mem_write, mem_address, mem_write_data, mem_read_data);
      input       cpu_write;
      input       [9:0]cpu_address;
      input       [31:0]cpu_write_data;
      output reg  [31:0]cpu_read_data;
      output reg  cpu_hit;
      output reg  mem_write;
      output reg  [9:0]mem_address;
      output reg  [127:0]mem_write_data;
      input       [127:0]mem_read_data;
      reg         [31:0]cache[15:0];
      reg         V[3:0];
// the size of tag is 10-2(byte off)-2(word off)-1(set index) = 5
      reg         [4:0]tag[3:0];
      reg         used[3:0];
      integer     i;
      integer     j;
      wire        [4:0]my_tag;
      wire        my_set_index;
      wire        [1:0]my_wordoff;
      wire        [5:0]my_address;
      assign      my_tag = cpu_address[9:5];
      assign      my_set_index = cpu_address[4];
      assign      my_wordoff = cpu_address[3:2];
      assign      my_address = cpu_address[9:4];

      initial begin
            for(i = 0 ; i < 16 ; i = i + 1)begin
                  cache[i] = 32'b0;
            end
            for(j = 0 ; j < 4 ; j = j + 1)begin
                  V[j] = 0;
                  used[j] = 0;
                  tag[j] = 5'b0;
            end
      end

      always@(*)begin
            mem_address <= my_address;
            if(cpu_write == 1)begin
                  if(V[my_set_index * 2] == 1 && tag[my_set_index * 2] == my_tag)begin
                        cpu_hit <= 1;
                        //write cache
                        cache[my_set_index * 8 + my_wordoff] <= cpu_write_data;                        
                        //write mem
                        mem_write <= 1;
                        mem_write_data[127:96] <= cache[my_set_index * 8];
                        mem_write_data[95:64]  <= cache[my_set_index * 8 + 1];
                        mem_write_data[63:32]  <= cache[my_set_index * 8 + 2];
                        mem_write_data[31:0]   <= cache[my_set_index * 8 + 3];
                        //set used
                        used[my_set_index * 2] <= 1;
                        used[my_set_index * 2 + 1] <= 0;
                  end
                  else if(V[my_set_index * 2 + 1] == 1 && tag[my_set_index * 2 + 1] == my_tag)begin
                        cpu_hit <= 1;
                        //write cache
                        cache[my_set_index * 8 + 4 + my_wordoff] <= cpu_write_data;
                        //write mem
                        mem_write <= 1;
                        mem_write_data[127:96] <= cache[my_set_index * 8 + 4];
                        mem_write_data[95:64]  <= cache[my_set_index * 8 + 5];
                        mem_write_data[63:32]  <= cache[my_set_index * 8 + 6];
                        mem_write_data[31:0]   <= cache[my_set_index * 8 + 7];
                        //set used
                        used[my_set_index * 2] <= 0;
                        used[my_set_index * 2 + 1] <= 1;
                  end
                  else begin
                        cpu_hit <= 0;
                        if(V[my_set_index * 2] == 0 || (V[my_set_index * 2 + 1] == 1 && used[my_set_index *2 + 1] == 1))begin
                              //write cache
                              cache[my_set_index * 8 + my_wordoff] <= cpu_write_data;
                              tag[my_set_index * 2] <= my_tag;
                              V[my_set_index * 2] <= 1;
                              //write mem
                              mem_write <= 1;
                              mem_write_data[127:96] <= cache[my_set_index * 8];
                              mem_write_data[95:64]  <= cache[my_set_index * 8 + 1];
                              mem_write_data[63:32]  <= cache[my_set_index * 8 + 2];
                              mem_write_data[31:0]   <= cache[my_set_index * 8 + 3];
                              //set used
                              used[my_set_index * 2] <= 1;
                              used[my_set_index * 2 + 1] <= 0;
                        end
                        else begin
                              //write cache
                              cache[my_set_index * 8 + 4+ my_wordoff] <= cpu_write_data;
                              tag[my_set_index * 2 + 1] <= my_tag;
                              V[my_set_index * 2 + 1] <= 1;
                              //write mem
                              mem_write <= 1;
                              mem_write_data[127:96] <= cache[my_set_index * 8 + 4];
                              mem_write_data[95:64]  <= cache[my_set_index * 8 + 5];
                              mem_write_data[63:32]  <= cache[my_set_index * 8 + 6];
                              mem_write_data[31:0]   <= cache[my_set_index * 8 + 7];
                              //set used
                              used[my_set_index * 2] <= 0;
                              used[my_set_index * 2 + 1] <= 1;
                        end                      
                  end
            end
            else begin
                  mem_write <= 0;
                  if(V[my_set_index * 2] == 1 && tag[my_set_index * 2] == my_tag)begin
                        cpu_hit <= 1;
                        cpu_read_data <= cache[my_set_index * 8 + my_wordoff];
                        //set used
                        used[my_set_index * 2] <= 1;
                        used[my_set_index * 2 + 1] <= 0;
                  end
                  else if(V[my_set_index * 2] == 1 && tag[my_set_index * 2] == my_tag)begin
                        cpu_hit <= 1;
                        cpu_read_data <= cache[my_set_index * 8 + 4 + my_wordoff];
                        //set used
                        used[my_set_index * 2] <= 0;
                        used[my_set_index * 2 + 1] <= 1;
                  end
                  else begin
                        cpu_hit <= 0;
                        if(V[my_set_index * 2] == 0 || (V[my_set_index * 2 + 1] == 1 && used[my_set_index *2 + 1] == 1))begin
                              //fetch block
                              cache[my_set_index * 8] <= mem_read_data[127:96];
                              cache[my_set_index * 8 + 1] <= mem_read_data[95:64];
                              cache[my_set_index * 8 + 2] <= mem_read_data[63:32];
                              cache[my_set_index * 8 + 3] <= mem_read_data[31:0];
                              tag[my_set_index * 2] <= my_tag;
                              V[my_set_index * 2] <= 1;
                              //output the read result
                              cpu_read_data <= cache[my_set_index * 8 + my_wordoff];
                              //set used
                              used[my_set_index * 2] <= 1;
                              used[my_set_index * 2 + 1] <= 0;
                        end
                        else begin
                              //fetch block
                              cache[my_set_index * 8 + 4] <= mem_read_data[127:96];
                              cache[my_set_index * 8 + 5] <= mem_read_data[95:64];
                              cache[my_set_index * 8 + 6] <= mem_read_data[63:32];
                              cache[my_set_index * 8 + 7] <= mem_read_data[31:0];
                              tag[my_set_index * 2 + 1] <= my_tag;
                              V[my_set_index * 2 + 1] <= 1;
                              //output the read result
                              cpu_read_data <= cache[my_set_index * 8 + 4 + my_wordoff];
                              //set used
                              used[my_set_index * 2] <= 0;
                              used[my_set_index * 2 + 1] <= 1;
                        end
                  end
            end
      end
endmodule
