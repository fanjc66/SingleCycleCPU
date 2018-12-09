`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/05/26 11:09:24
// Design Name: 
// Module Name: test
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

module test;
    
    // input
    reg clk;
    reg rst_n;
    
    // output
    wire [31:0] inst;
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] alu_res;
    wire [31:0] write_data;
    wire [4:0] write_addr;
    wire write_en;
    wire [4:0] rs_addr;
    wire [31:0] rs_data;
    wire [4:0] rt_addr;
    wire [31:0] rt_data;
    wire [31:0] imm;
    wire [31:0] br_tgt;
    wire [31:0] mem_read;
    wire [`ALU_OP_BUS]  alu_op;
    wire zero;
    wire b_ctrl, jump, branch;
    wire take_branch;
    wire [1:0] fetch_rw;
    wire halt;
    
    main main(
        .clk(clk),
        .rst_n(rst_n),
        .inst(inst),
        .pc(pc),
        .next_pc(next_pc),
        .alu_res(alu_res),
        .reg_write_data(write_data),
        .reg_write_addr(write_addr),
        .reg_write_en(write_en),
        .rs_addr(rs_addr),
        .rs_data(rs_data),
        .rt_addr(rt_addr),
        .rt_data(rt_data),
        .imm(imm),
        .branch_target(br_tgt),
        .mem_read(mem_read),
        .alu_op(alu_op),
        .zero(zero),
        .b_ctrl(b_ctrl),
        .jump(jump),
        .branch(branch),
        .take_branch(take_branch),
        .pc_src(fetch_rw),
        .halt(halt));
        
    initial 
    begin
        // Initialize Inputs
        clk = 0;
        rst_n = 0;
        #25;
            rst_n = 1;
        #25
        clk = !clk;
        forever #50 
        begin
            clk = !clk;
        end
    end
        
endmodule
