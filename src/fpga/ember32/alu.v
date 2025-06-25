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
        input           sys_clk,        // System Clock (For Divide)
        input           sys_rst_n,      // System _Reset
        
        input [15:0]    operation_oh,   // ALU Operation (As One Hot Value)
        input  [7:0]    data_width_oh,  // ALU Operation data width (As One Hot Value)

        input [31:0]    srcA_val,       // Src A Value 
        input [31:0]    srcB_val,       // Src B/Imm Value

        output[31:0]    result,         // ALU Result Value    

        output [7:0]    cc,             // alu condition and exception bits
        output          busy            // Operation (divide, etc.) needs more time
    );

    `include "ember.vh"
    
    // width mask input values
    wire [31:0] srcA_val_w = 
        data_width_oh[Width_bit_b] | data_width_oh[Width_bit_sb]  ? {{24'b0}, srcA_val[7:0]}  :
//        data_width_oh[Width_bit_sb] ? {{24{srcA_val[7]}},  srcA_val[7:0]}  :
        data_width_oh[Width_bit_h] | data_width_oh[Width_bit_sh] ? {{16'b0},  srcA_val[15:0]} :
//        data_width_oh[Width_bit_sh] ? {{16{srcA_val[15]}}, srcA_val[15:0]} : 
                                      srcA_val;

    wire [31:0] srcB_val_w = 
        data_width_oh[Width_bit_b] | data_width_oh[Width_bit_sb] ? {{24'b0}, srcB_val[7:0]}  :
//        data_width_oh[Width_bit_sb] ? {{24{srcB_val[7]}},  srcB_val[7:0]}  :
        data_width_oh[Width_bit_h] | data_width_oh[Width_bit_sh] ? {{16'b0}, srcB_val[15:0]} :
//        data_width_oh[Width_bit_sh] ? {{16{srcB_val[15]}}, srcB_val[15:0]} : 
                                      srcB_val;

    
    //*************************************************************************
    // Always do signed mul, but set the 33rd bit to 1 or 0 depending on operation
    wire signed [32:0] signedA = {operation_oh[mhs_bit] & srcA_val_w[31], srcA_val_w};
    wire signed [32:0] signedB = {operation_oh[mhs_bit] & srcB_val_w[31], srcB_val_w};
    wire signed [63:0] multiply = signedA * signedB;    

    // ALU Ops
    wire [32:0] sum_  = srcA_val_w + srcB_val_w;    
//    wire        sum_c = sum_[32];
    wire        sum_c = data_width_oh[Width_bit_w] ? sum_[32] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? sum_[16] : sum_[8];
    wire [32:0] dif_  = srcA_val_w - srcB_val_w;    
//    wire        dif_c = dif_[32];
    wire        dif_c = data_width_oh[Width_bit_w] ? dif_[32] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? dif_[16] : dif_[8];
    wire [31:0] and_ = srcA_val_w & srcB_val_w;    
    wire [31:0] or_  = srcA_val_w | srcB_val_w;    
    wire [31:0] xor_ = srcA_val_w ^ srcB_val_w;    

    wire [31:0] lsr_ = data_width_oh[Width_bit_w] ? srcA_val_w >> srcB_val_w[5:0] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? srcA_val_w[15:0] >> srcB_val_w[4:0] : dif_[7:0] >> srcB_val_w[3:0];
    wire [31:0] lsl_ = srcA_val_w << srcB_val_w[5:0];    
    wire [31:0] asr_ = data_width_oh[Width_bit_w] ? srcA_val_w >>> srcB_val_w[5:0] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? srcA_val_w[15:0] >>> srcB_val_w[4:0] : dif_[7:0] >>> srcB_val_w[3:0];

    
    // ALU Flags
    wire zero = result==0;
    wire carry = (operation_oh[add_bit] & sum_c) | (operation_oh[sub_bit] & dif_c);
    wire negative = data_width_oh[Width_bit_w] ? result[31] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? result[15] : result[7];
    wire overflow = 0; // TODO: Probably remove this?
    wire div_zero = 0;

    assign cc = (zero ? eCC_Z : 0) | (carry ? eCC_C : 0) | (negative ? eCC_N : 0) | (overflow ? eCC_V : 0) | (div_zero ? eCC_D : 0);

    // ALU Result
    wire [31:0] alu_result = 
        (operation_oh[add_bit] ?  sum_[31:0] : 32'b0) |
        (operation_oh[sub_bit] ?  dif_[31:0] : 32'b0) |

        (operation_oh[mul_bit] ?  multiply[31:0]  : 32'b0) |
        (operation_oh[mhs_bit] ?  multiply[63:32] : 32'b0) |
        (operation_oh[mhu_bit] ?  multiply[63:32] : 32'b0) |
        (operation_oh[div_bit] ?  32'b0 : 33'b0) |
        (operation_oh[divu_bit] ? 32'b0 : 33'b0) |

        (operation_oh[and_bit] ?  and_ : 32'b0) |
        (operation_oh[or_bit]  ?  or_  : 32'b0) |
        (operation_oh[xor_bit] ?  xor_ : 32'b0) |
        (operation_oh[lsr_bit] ?  lsr_ : 32'b0) |
        (operation_oh[lsl_bit] ?  lsl_ : 32'b0) |
        (operation_oh[asr_bit] ?  asr_ : 32'b0);

        
    // Divide (TODO)
    assign busy = 0;    
  
    assign result = 
        data_width_oh[Width_bit_b]  ? {{24'b0},              alu_result[7:0]}  :
        data_width_oh[Width_bit_sb] ? {{24{alu_result[7]}},  alu_result[7:0]}  :
        data_width_oh[Width_bit_h]  ? {{16'b0},              alu_result[15:0]} :
        data_width_oh[Width_bit_sh] ? {{16{alu_result[15]}}, alu_result[15:0]}  : 
                                      alu_result;



endmodule