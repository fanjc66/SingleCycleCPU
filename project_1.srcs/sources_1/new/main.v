`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/05/25 09:12:25
// Design Name: 
// Module Name: main
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


`include "defines.v"

module main(
    input clk,
    input rst_n,
    
    output [31:0] inst,
    output [31:0] pc,
    output [31:0] next_pc,
    output [31:0] alu_res,
    output [31:0] reg_write_data,
    output [4:0] reg_write_addr,
    output [31:0] reg_write_read,
    output reg_write_en,
    output [4:0] rs_addr,
    output [31:0] rs_data,
    output [4:0] rt_addr,
    output [31:0] rt_data,
    output [31:0] imm,
    output [31:0] branch_target,
    output [31:0] mem_read,
    output [`ALU_OP_BUS] alu_op,
    output zero,
    output b_ctrl,
    output jump,
    output branch,
    output take_branch,
    output [1:0] pc_src,
    output halt);
    
    localparam DATA_WIDTH = 32;
    localparam DATA_PER_BYTE_WIDTH = 2;
    
    wire fetch_rw;
    wire [`DATA_BUS] fetch_write;
    wire [`DATA_BUS] fetch_pc;
    wire [`DATA_BUS] fetch_next_pc;
    wire [`DATA_BUS] imem_data;
    
    wire [`DATA_BUS] dec_pc = fetch_pc;
    wire [`DATA_BUS] dec_raw_inst = imem_data;
    wire [`VREG_BUS] dec_vrs_addr;
    wire [`VREG_BUS] dec_vrt_addr;
    wire [`DATA_BUS] dec_rs_data;
    wire [`DATA_BUS] dec_rt_data;
    wire [`VREG_BUS] dec_virtual_write_addr;
    wire [`INST_BUS] dec_inst;
    wire [`ALU_OP_BUS] dec_exec_op;
    wire dec_b_ctrl;
    wire dec_mem_enable;
    wire [`DATA_BUS] dec_imm;
    wire dec_wb_src;
    wire dec_wb_reg;
    wire dec_branch, dec_illegal, dec_halt;
    wire [1:0] dec_jump;
    
    wire [`DATA_BUS] exec_pc = dec_pc;
    wire [`DATA_BUS] exec_raw_inst = dec_raw_inst;
    wire [`INST_BUS] exec_inst = dec_inst;
    wire [`DATA_BUS] exec_imm = dec_imm;
    wire exec_mem_enable = dec_mem_enable;
    wire [`DATA_BUS] exec_mem_write = dec_rt_data;
    wire exec_wb_src = dec_wb_src;
    wire exec_wb_reg = dec_wb_reg;
    wire exec_jump = dec_jump;
    wire exec_branch = dec_branch;
    wire [`DATA_BUS] exec_branch_target;
    wire [`ALU_OP_BUS] exec_alu_op = dec_exec_op;
    wire [`DATA_BUS] exec_alu_rs = dec_rs_data;
    wire [`DATA_BUS] exec_alu_rt = dec_b_ctrl ? dec_imm : dec_rt_data;
    wire [`DATA_BUS] exec_res;
    wire exec_take_branch;
        
    wire [`DATA_BUS] dmem_res = exec_res;
    reg  [`DATA_BUS] dmem_write;
    wire dmem_wb_src = exec_wb_src;
    
    wire [`DATA_BUS] dmem_mem_read;
    wire [`DATA_BUS] dmem_mem_read_raw;
    wire [`DATA_BUS] dmem_mem_write;
    wire [3:0] dmem_mem_sel;
    wire dmem_mem_enable = exec_mem_enable;
    wire dmem_mem_rw;
    wire dmem_ready;
    wire dmem_mem_rw_valid;

    wire [`VREG_BUS] wb_virtual_write_addr = dec_virtual_write_addr;
    wire [`DATA_BUS] wb_write = dmem_write;
    
    assign inst = dec_raw_inst;
    assign pc = fetch_pc;
    assign next_pc = fetch_next_pc;
    assign alu_res = exec_res;
    assign reg_write_data = wb_write;
    assign reg_write_addr = wb_virtual_write_addr;
    assign reg_write_en = dec_wb_reg;
    assign rs_addr = dec_vrs_addr;
    assign rs_data = dec_rs_data;
    assign rt_addr = dec_vrt_addr;
    assign rt_data = dec_rt_data;
    assign imm = dec_imm;
    assign branch_target = exec_branch_target;
    assign mem_read = dmem_mem_read;
    assign alu_op = exec_alu_op;
    assign zero = exec_res == 0;
    assign b_ctrl = dec_b_ctrl;
    assign jump = dec_jump;
    assign branch = dec_branch;
    assign take_branch = exec_take_branch;
    assign pc_src = fetch_rw;
    assign halt = dec_halt;
    
    assign fetch_rw = exec_take_branch;
    assign fetch_write = exec_branch_target;
    
    fetch_unit #(.DATA_WIDTH(DATA_WIDTH)) fetch(
        .clk(clk),
        .rst_n(rst_n),
        .stall(dec_halt),
        
        .rw(fetch_rw),
        .write(fetch_write),
        
        .pc(fetch_pc),
        .next_pc(fetch_next_pc)
    );
    
    inst_cache#(.DATA_WIDTH(DATA_WIDTH))
                   icache(
           .clk(clk),
           .rst_n(rst_n),
   
           // request
           .addr(fetch_pc),
   
           .data(imem_data)
       );
    
    decoder #(.DATA_WIDTH(DATA_WIDTH)) decode(
`ifdef DEBUG_DEC
        .stall(`FALSE),
        .pc(dec_pc),
`endif
        .raw_inst(dec_raw_inst),
        .rs_addr(dec_vrs_addr),
        .rt_addr(dec_vrt_addr),
        .wb_reg(dec_wb_reg), // rd_enable
        .rd_addr(dec_virtual_write_addr),
        .imm(dec_imm),
        .inst(dec_inst),
        .exec_op(dec_exec_op),
        .b_ctrl(dec_b_ctrl),
        .mem_enable(dec_mem_enable),
        .wb_src(dec_wb_src),
        .jump(dec_jump),
        .branch(dec_branch),
        .halt(dec_halt),
        .illegal(dec_illegal)
    );

    register_file #(.DATA_WIDTH(DATA_WIDTH))
                    regfile(
        .clk(clk),
        .rst_n(rst_n),
        .stall_in(`FALSE),

        .rs_addr(dec_vrs_addr),
        .rs_data(dec_rs_data),
        .rt_addr(dec_vrt_addr),
        .rt_data(dec_rt_data),
        .rw(dec_wb_reg),
        .write_addr(wb_virtual_write_addr),
        .write_data(wb_write),
        .write_read(reg_write_read)
    );
    
    execution #(.DATA_WIDTH(DATA_WIDTH)) exe(
        .clk(clk),
        .rst_n(rst_n),
        .stall(`FALSE),
        
        .pc(exec_pc),
        .raw_inst(exec_raw_inst),
        .inst(exec_inst),
        .imm(exec_imm),
        
        .alu_op(exec_alu_op),
        .alu_rs(exec_alu_rs),
        .alu_rt(exec_alu_rt),

        .jump(exec_jump),
        .branch(exec_branch),
        
        .res(exec_res),
        .take_branch(exec_take_branch),
        .branch_target(exec_branch_target)
    );
        
    memory_access #(.DATA_WIDTH(DATA_WIDTH)) mem(
        .clk(clk),
        .rst_n(rst_n),
        
        .inst_in(exec_inst),
        .alu_res_in(exec_res),
        .mem_rw_out(dmem_mem_rw),
        .mem_write_in(exec_mem_write),
        .mem_write_out(dmem_mem_write),
        .mem_read_in(dmem_mem_read_raw),
        .mem_read_out(dmem_mem_read)
    );
        
    always @* // select which value should we write back to register file.
    begin
        if (dmem_wb_src == `WB_ALU) // if this instruction writes the value calculated back to register.
        begin
            dmem_write <= dmem_res; // ALU result passed through pipeline.
        end
        else // if this instruction is load-like inst.
        begin
            dmem_write <= dmem_mem_read; // write the value read from memory.
        end
    end
    
    data_cache #(.DATA_WIDTH(DATA_WIDTH))
                dcache(
        .clk(clk),
        .rst_n(rst_n),
        
        .addr(dmem_res),
        .enable(dmem_mem_enable),
        .rw(dmem_mem_rw),

        .write(dmem_mem_write),
        .read(dmem_mem_read_raw)
    );
    
endmodule
