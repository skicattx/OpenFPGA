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
        input [2:0]     data_width, // ALU Operation data width 
    
        input [31:0]    srcA_val,   // Src A Value 
        input [31:0]    srcB_val,   // Src B/Imm Value

        output[31:0]    result,     // Result Value    
        
        output [4:0]    cc,         // alu condition and exception bits
        output          busy        // Operation (divide, etc.) needs more time
    );

    `include "ember.vh"
    /*
    //*************************************************************************
    // Status Register Bits (TODO: Move to header? `define?)
    localparam eCC_Z    = 1<<0;     // EQ/Z  -- NE
    localparam eCC_C    = 1<<1;     // C     -- NC
    localparam eCC_N    = 1<<2;     // LT/N -- GE
    localparam eCC_V    = 1<<3;     // V
    localparam eCC_D    = 1<<4;     // D - Div 0

    // One Hot bits for operations
    localparam add_bit  = 'h0; 
    localparam sub_bit  = 'h1;
        
    localparam mul_bit  = 'h2; 
    localparam mhs_bit  = 'h3; 
    localparam mhu_bit  = 'h4; 
    localparam div_bit  = 'h5; 
    localparam divu_bit = 'h6; 
    
    localparam and_bit  = 'h9; 
    localparam or_bit   = 'ha; 
    localparam xor_bit  = 'hb; 
    localparam lsr_bit  = 'hc; 
    localparam lsl_bit  = 'hd; 
    localparam asr_bit  = 'he; 
    
    
    localparam Width_bit_w  = 'h0;
    localparam Width_bit_h  = 'h1;
    localparam Width_bit_sh = 'h2;
    localparam Width_bit_b  = 'h3;
    localparam Width_bit_sb = 'h4;
                           */

    //*************************************************************************
    // Do all the things in parallel
    wire [15:0] onehot_op = 1 << operation;
    wire [7:0]  onehot_width = 1 << data_width;
    
    
    //*************************************************************************
    // Always do signed mul, but set the 33rd bit to 1 or 0 depending on operation
    wire signed [32:0] signedA = {onehot_op[mhs_bit] & srcA_val[31], srcA_val};
    wire signed [32:0] signedB = {onehot_op[mhs_bit] & srcB_val[31], srcB_val};
    wire signed [63:0] multiply = signedA * signedB;    

    // ALU Ops
    wire [32:0] sum_  = srcA_val + srcB_val;    
    wire        sum_c = onehot_width[Width_bit_w] ? sum_[32] : onehot_width[Width_bit_h]|onehot_width[Width_bit_sh] ? sum_[16] : sum_[8];
    wire [32:0] dif_  = srcA_val - srcB_val;    
    wire        dif_c = onehot_width[Width_bit_w] ? dif_[32] : onehot_width[Width_bit_h]|onehot_width[Width_bit_sh] ? dif_[16] : dif_[8];
    wire [31:0] and_ = srcA_val & srcB_val;    
    wire [31:0] or_  = srcA_val | srcB_val;    
    wire [31:0] xor_ = srcA_val ^ srcB_val;    

    wire [31:0] lsr_ = onehot_width[Width_bit_w] ? srcA_val >> srcB_val[5:0] : onehot_width[Width_bit_h]|onehot_width[Width_bit_sh] ? srcA_val[15:0] >> srcB_val[4:0] : dif_[7:0] >> srcB_val[3:0];
    wire [31:0] lsl_ = srcA_val << srcB_val[5:0];    
    wire [31:0] asr_ = onehot_width[Width_bit_w] ? srcA_val >>> srcB_val[5:0] : onehot_width[Width_bit_h]|onehot_width[Width_bit_sh] ? srcA_val[15:0] >>> srcB_val[4:0] : dif_[7:0] >>> srcB_val[3:0];

    
    // ALU Flags
    wire zero = result==0;
    wire carry = (onehot_op[add_bit] & sum_c) | (onehot_op[sub_bit] & dif_c);
    wire negative = onehot_width[Width_bit_w] ? dif_[31] : onehot_width[Width_bit_h]|onehot_width[Width_bit_sh] ? dif_[15] : dif_[7];
    wire overflow = 0; // TODO: Probably remove this?
    wire div_zero = 0;

    assign cc = (zero ? eCC_Z : 0) | (carry ? eCC_C : 0) | (negative ? eCC_N : 0) | (overflow ? eCC_V : 0) | (div_zero ? eCC_D : 0);

    // ALU Result
    assign result = 
        (onehot_op[add_bit] ?  sum_[31:0] : 33'b0) |
        (onehot_op[sub_bit] ?  dif_[31:0] : 33'b0) |

        (onehot_op[mul_bit] ?  multiply[31:0]  : 33'b0) |
        (onehot_op[mhs_bit] ?  multiply[63:32] : 33'b0) |
        (onehot_op[mhu_bit] ?  multiply[63:32] : 33'b0) |
        (onehot_op[div_bit] ?  32'b0 : 33'b0) |
        (onehot_op[divu_bit] ? 32'b0 : 33'b0) |

        (onehot_op[and_bit] ?  and_ : 32'b0) |
        (onehot_op[or_bit]  ?  or_  : 32'b0) |
        (onehot_op[xor_bit] ?  xor_ : 32'b0) |
        (onehot_op[lsr_bit] ?  lsr_ : 32'b0) |
        (onehot_op[lsl_bit] ?  lsl_ : 32'b0) |
        (onehot_op[asr_bit] ?  asr_ : 32'b0);

        
    // Divide (TODO)
    assign busy = 0;    
  

endmodule