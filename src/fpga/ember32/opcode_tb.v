`timescale 1ns / 1ps

//****************************************************************************
// Ember CPU Decoder Testbench
//
// Copyright 2025 Tom Gambill, IARI Ventures, LLC (IARITech.com)

// 
// BSD 2-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************


// Tests for decoder
module opcode_tb;

wire sys_rst;

wire 31:0]  instruction;
    
wire        inst_illegal;
wire        inst_noop;
wire        inst_halt;
wire        inst_trap;
wire        inst_rtu;
wire        inst_branch;
wire        inst_mov;
wire        inst_ldi;
wire        inst_load;
wire        inst_store;
wire        inst_alu;
    
wire [5:0]  reg_dest;
wire [5:0]  reg_srcA;
wire [3:0]  reg_srcA;
    
wire [13:0] imm_val;
wire        imm_val_en;
    
wire [21:0] branch_offset;
wire        branch_imm_en;
    
wire [13:0] addr_offset;
wire        addr_predec_postinc;

opcode_tb opcode_test(
    sys_rst,                        

    instruction,        

    inst_illegal, inst_noop, inst_halt,inst_trap, inst_rtu, inst_branch, inst_mov, inst_ldi, inst_load, inst_store, inst_alu,           

    reg_dest, reg_srcA, reg_srcB,           

    imm_val, imm_val_en,          

    branch_offset, branch_imm_en,      
        
    addr_offset, addr_predec_postinc );
   
   
   
endmodule   