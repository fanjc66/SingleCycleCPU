`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Sun Yat-sen University
// Engineer: Yuhui Huang
// 
// Design Name: Execution Stage Controller
// Module Name: execution
// Project Name: SimpleCPU
// Target Devices: Basys3
// Tool Versions: Vivado 2018.1
// Description: 
//
// Dependencies: None
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`include "defines.v"

module execution#(parameter DATA_WIDTH = 32)(
    input clk,
    input rst_n,
    input stall,

    input [`DATA_BUS] pc,
    input [`DATA_BUS] raw_inst,
    input [`INST_BUS] inst,
    input [`DATA_BUS] imm,

    input [`ALU_OP_WIDTH-1:0] alu_op,
    input [`DATA_BUS] alu_rs,
    input [`DATA_BUS] alu_rt,

    input jump,
    input branch,
    
    // Output ports

    output [`DATA_BUS] res,
    output take_branch,
    output [`DATA_BUS] branch_target
);
    wire [`DATA_BUS] alu_rd, move_res;

    assign take_branch = branch && alu_rd;
    
    assign res = alu_rd;
    
    reg [`DATA_BUS] raw_bt;
    wire [`DATA_BUS] pc4 = pc + 4;
    
    always @*
    begin
        raw_bt <= 0;
        if (branch)
        begin
            if (jump)
            begin
                if (inst == `INST_J)
                    raw_bt <= { pc4[DATA_WIDTH-1:28], raw_inst[`RAW_INST_ADDR_BUS], 2'b00 };
                else // only j
                    raw_bt <= 0;
            end
            else // conditional branch instruction
            begin
                raw_bt <= pc4 + {imm[DATA_WIDTH-3:0], 2'b00}; // pc + 4 + 4 * imm
            end
        end
    end
    
    assign branch_target = jump == `JUMP_REG ? alu_rs : raw_bt;

    // ==== Data forwarding ===

    arithmetic_logic_unit #(.DATA_WIDTH(DATA_WIDTH)) alu(
        .stall(stall),
        .op(alu_op),
        .rs(alu_rs),
        .rt(alu_rt),
        .rd(alu_rd)
    );
endmodule