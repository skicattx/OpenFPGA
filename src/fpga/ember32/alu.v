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
    
    // width mask input values (unsigned for add, sub, etc...signed for mul)
    wire [31:0] srcA_val_u = 
        data_width_oh[Width_bit_b] | data_width_oh[Width_bit_sb] ? {{24'b0}, srcA_val[7:0]}  :
        data_width_oh[Width_bit_h] | data_width_oh[Width_bit_sh] ? {{16'b0}, srcA_val[15:0]} :
                                      srcA_val;

    wire [31:0] srcB_val_u = 
        data_width_oh[Width_bit_b] | data_width_oh[Width_bit_sb] ? {{24'b0}, srcB_val[7:0]}  :
        data_width_oh[Width_bit_h] | data_width_oh[Width_bit_sh] ? {{16'b0}, srcB_val[15:0]} :
                                      srcB_val;

    wire [31:0] srcA_val_s, srcB_val_s;
    assign {srcA_val_s, srcB_val_s} = 
        data_width_oh[Width_bit_b]  ? { {{24'b0},            srcA_val[7:0]},  {{24'b0},            srcB_val[7:0]}  } :
        data_width_oh[Width_bit_sb] ? { {{24{srcA_val[7]}},  srcA_val[7:0]},  {{24{srcB_val[7]}},  srcB_val[7:0]}  } :
        data_width_oh[Width_bit_h]  ? { {{16'b0},            srcA_val[15:0]}, {{16'b0},            srcB_val[15:0]} } :
        data_width_oh[Width_bit_sh] ? { {{16{srcA_val[15]}}, srcA_val[15:0]}, {{16{srcB_val[15]}}, srcB_val[15:0]} } : 
                                      { srcA_val, srcB_val };
    
    //*************************************************************************
    // ALU Ops

    // Always do signed mul, but set the 33rd bit to 1 or 0 depending on operation
    wire signed_op = operation_oh[mhs_bit] | data_width_oh[Width_bit_sb] | data_width_oh[Width_bit_sh];
    wire signed [32:0] signedA = {signed_op & srcA_val_s[31], srcA_val_s};
    wire signed [32:0] signedB = {signed_op & srcB_val_s[31], srcB_val_s};
    wire signed [63:0] multiply = signedA * signedB;    

    wire [32:0] sum_  = srcA_val_u + srcB_val_u;
    wire [32:0] dif_  = srcA_val_u - srcB_val_u;
    
    // ALU Flags
    wire sum_c, dif_c, srcA_sign, srcB_sign, result_sign;
    assign {sum_c, dif_c, srcA_sign, srcB_sign, result_sign } = 
        data_width_oh[Width_bit_w] ?                             {sum_[32], dif_[32], srcA_val[31], srcB_val[31], result[31]} : 
        data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? {sum_[16], dif_[16], srcA_val[15], srcB_val[15], result[15]} : 
                                                                 { sum_[8],  dif_[8],  srcA_val[7],  srcB_val[7],  result[7]};

    wire overflow_add = !(srcA_sign ^ srcB_sign) & (srcA_sign ^ result_sign);
    wire overflow_sub = (srcA_sign ^ srcB_sign) & (srcA_sign ^ result_sign);
    wire overflow, carry;
    assign {overflow, carry} = operation_oh[add_bit] ? {overflow_add, sum_c} : 
                               operation_oh[sub_bit] ? {overflow_sub, dif_c} : {1'b0, 1'b0};
    wire zero = result==0;
    wire div_zero = 0;

    assign cc = (zero ? eCC_Z : 0) | (carry ? eCC_C : 0) | (result_sign ? eCC_N : 0) | (overflow ? eCC_V : 0) | (div_zero ? eCC_D : 0); 

    wire [31:0] and_ = srcA_val_u & srcB_val_u;    
    wire [31:0] or_  = srcA_val_u | srcB_val_u;    
    wire [31:0] xor_ = srcA_val_u ^ srcB_val_u;    

    wire [31:0] lsr_ = data_width_oh[Width_bit_w] ? srcA_val_u >> srcB_val_u[5:0] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? srcA_val_u[15:0] >> srcB_val_u[4:0] : dif_[7:0] >> srcB_val_u[3:0];
    wire [31:0] lsl_ = srcA_val_u << srcB_val_u[5:0];    
    wire [31:0] asr_ = data_width_oh[Width_bit_w] ? srcA_val_u >>> srcB_val_u[5:0] : data_width_oh[Width_bit_h]|data_width_oh[Width_bit_sh] ? srcA_val_u[15:0] >>> srcB_val_u[4:0] : dif_[7:0] >>> srcB_val_u[3:0];
    

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