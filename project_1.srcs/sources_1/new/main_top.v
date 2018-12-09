`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/11/16 00:05:25
// Design Name: 
// Module Name: main_top
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


module main_top(
    input clk,
    input btn0,
    output [6:0] seg,
    output reg [3:0] an,
    input [15:0] sw,
    output reg [15:0] LED
    );
    
    wire trigger;
    debouncer deb(clk, btn0, trigger);
    
    wire rst_n = sw[0];
    wire [2:0] mode = sw[15:13];
    
    reg [3:0] S[0:3];
    reg [3:0] BCD;
    reg [1:0] display_digit = 0;
    reg [3:0] display_digit_bit;
    wire clk_display;
    
    clock_div#(50000, 19) div2(clk, rst_n, clk_display);
    bcd_encoder encoder(1, BCD, seg);
    
    // output
    wire [31:0] inst;
    wire [31:0] pc;
    wire [31:0] next_pc;
    wire [31:0] alu_res;
    wire [31:0] reg_write_data;
    wire [31:0] reg_write_read;
    wire [4:0] reg_write_addr;
    wire reg_write_en;
    wire [4:0] rs_addr;
    wire [31:0] rs_data;
    wire [4:0] rt_addr;
    wire [31:0] rt_data;
    wire [31:0] imm;
    wire [31:0] branch_target;
    wire [31:0] mem_read;
    wire [`ALU_OP_BUS]  alu_op;
    wire zero;
    wire b_ctrl, jump, branch;
    wire take_branch;
    wire [1:0] fetch_rw;
    wire [5:0] opcode;
    wire halt;
    
    main main(
        .clk(trigger),
        .rst_n(rst_n),
        .inst(inst),
        .pc(pc),
        .next_pc(next_pc),
        .alu_res(alu_res),
        .reg_write_data(reg_write_data),
        .reg_write_addr(reg_write_addr),
        .reg_write_read(reg_write_read),
        .reg_write_en(reg_write_en),
        .rs_addr(rs_addr),
        .rs_data(rs_data),
        .rt_addr(rt_addr),
        .rt_data(rt_data),
        .imm(imm),
        .branch_target(branch_target),
        .mem_read(mem_read),
        .alu_op(alu_op),
        .zero(zero),
        .b_ctrl(b_ctrl),
        .jump(jump),
        .branch(branch),
        .take_branch(take_branch),
        .pc_src(fetch_rw),
        .halt(halt));
    
    always @(posedge clk_display)
        display_digit = display_digit + 1;
    
    always @*
    begin
        case(display_digit)
            0: an <= 4'b1110;
            1: an <= 4'b1101;
            2: an <= 4'b1011;
            3: an <= 4'b0111;
            default: an <= 4'b0000;
        endcase
        BCD <= S[display_digit];
    end
    
    always @*
    begin
        LED[0] <= 1;
        LED[1] <= fetch_rw;
        LED[2] <= zero;
        LED[3] <= reg_write_en;
        LED[4] <= b_ctrl;
        LED[5] <= take_branch;
        LED[6] <= jump;
        LED[7] <= branch;
        LED[15] <= halt;
    end
    
    always @*
        case(mode)
            0: begin {S[3], S[2]} <= pc[7:0]; {S[1], S[0]} <= next_pc[7:0]; end
            1: begin S[3] <= rs_addr / 10; S[2] <= rs_addr % 10; {S[1], S[0]} <= rs_data[7:0]; end
            2: begin S[3] <= rt_addr / 10; S[2] <= rt_addr % 10; {S[1], S[0]} <= rt_data[7:0]; end
            3: begin {S[3], S[2]} <= alu_res[7:0]; {S[1], S[0]} <= mem_read[7:0]; end
            4: begin S[3] <= reg_write_addr / 10; S[2] <= reg_write_addr % 10; {S[1], S[0]} <= reg_write_data[7:0]; end
            5: begin {S[3], S[2]} <= imm[7:0]; {S[1], S[0]} <= branch_target[7:0]; end
            6: begin S[3] <= alu_op; {S[2], S[1], S[0]} <= alu_res[11:0]; end
            7: begin {S[3], S[2]} <= reg_write_read; {S[1], S[0]} <= 0; end
            default: begin {S[3], S[2], S[1], S[0]} <= 0; end
        endcase
endmodule
