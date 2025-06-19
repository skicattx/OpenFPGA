`timescale 1ns / 1ns

//****************************************************************************
// Ember CPU ALU Implementation
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


// Ember CPU ALU
module alu( 
        input           sys_clk,    // System Clock (For Divide)
        input           sys_rst_n,  // System _Reset
        input [3:0]     operation,  // ALU Operation
    
        input [31:0]    srcA_val,   // Src A Value 
        input [31:0]    srcB_val,   // Src B/Imm Value

        output[31:0]    result,     // Result Value    
        
        output [4:0]    cc,         // alu condition and exception bits
        output          busy        // Operation (divide, etc.) needs more time
    );

    
    //*************************************************************************
    // Status Register Bits (TODO: Move to header? `define?)
    localparam eCC_Z    = 1<<0;     // EQ/Z  -- NE
    localparam eCC_C    = 1<<1;     // C     -- NC
    localparam eCC_N    = 1<<2;     // LT/N -- GE
    localparam eCC_V    = 1<<3;     // V
    localparam eCC_D    = 1<<4;     // D - Div 0

    // One Hot bits for operations
    localparam add_bit  = 1<<'h0; 
    localparam sub_bit  = 1<<'h1;
        
    localparam mul_bit  = 1<<'h2; 
    localparam mhs_bit  = 1<<'h3; 
    localparam mhu_bit  = 1<<'h4; 
    localparam div_bit  = 1<<'h5; 
    localparam divu_bit = 1<<'h6; 
    
    localparam and_bit  = 1<<'h9; 
    localparam or_bit   = 1<<'ha; 
    localparam xor_bit  = 1<<'hb; 
    localparam lsr_bit  = 1<<'hc; 
    localparam lsl_bit  = 1<<'hd; 
    localparam asr_bit  = 1<<'he; 


    //*************************************************************************
    // Do all the things in parallel
    wire [15:0] onehot_op = 1 << operation;
    wire [7:0]  onehot_width = 1 << operation;
    
    
    //*************************************************************************
    wire mul_sign = onehot_op[mhs_bit];

    wire signed [32:0] signed1 = {mul_sign, srcA_val};
    wire signed [32:0] signed2 = {mul_sign, srcB_val};
    wire signed [63:0] multiply = signed1 * signed2;    





    wire zero;
    wire carry;
    wire negative;
    wire overflow;
    wire div_zero = 0;

    assign cc = (zero ? eCC_Z : 0) | (carry ? eCC_C : 0) | (negative ? eCC_N : 0) | (overflow ? eCC_V : 0) | (div_zero ? eCC_D : 0);



    assign busy = 0;    

/*
    //*************************************************************************
    // Single Cycle ALU Operations
    wire [31:0] aluOut_base =
     (funct3Is[0]  ? instr[30] & instr[5] ? aluMinus[31:0] : aluPlus : 32'b0) |
     (funct3Is[1]  ? leftshift                                       : 32'b0) |
     (funct3Is[2]  ? {31'b0, LT}                                     : 32'b0) |
     (funct3Is[3]  ? {31'b0, LTU}                                    : 32'b0) |
     (funct3Is[4]  ? aluIn1 ^ aluIn2                                 : 32'b0) |
     (funct3Is[5]  ? shifter                                         : 32'b0) |
     (funct3Is[6]  ? aluIn1 | aluIn2                                 : 32'b0) |
     (funct3Is[7]  ? aluIn1 & aluIn2                                 : 32'b0) ;

   wire [31:0] aluOut_muldiv =
     (  funct3Is[0]   ?  multiply[31: 0] : 32'b0) | // 0:MUL
     ( |funct3Is[3:1] ?  multiply[63:32] : 32'b0) | // 1:MULH, 2:MULHSU, 3:MULHU
     (  instr[14]     ?  div_sign ? -divResult : divResult : 32'b0) ; 
                                                 // 4:DIV, 5:DIVU, 6:REM, 7:REMU
   
   wire [31:0] aluOut = isALUreg & funcM ? aluOut_muldiv : aluOut_base;

*/










endmodule