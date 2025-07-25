`timescale 1ns / 1ns

//****************************************************************************
// Ember Shared Defines
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


//*************************************************************************
// CPU System Modes
localparam eSystemMode_Super    = 1'b0;        // 
localparam eSystemMode_User     = 1'b1;        // 


//*************************************************************************
// Status Register Flag Values
localparam eCC_Z    = 1<<0;     // EQ/Z  -- NE
localparam eCC_C    = 1<<1;     // C     -- NC
localparam eCC_N    = 1<<2;     // LT/N -- GE
localparam eCC_V    = 1<<3;     // V (Unsupported currently)

localparam eCC_D    = 1<<4;     // D - Div 0
localparam eCC_G    = 1<<5;     // G - Global Interrupt Enable/!Supervisor Mode (Set/Unset to switch to User/Super Mode)
localparam eCC_X    = 1<<6;     // X - Exception Flag
localparam eCC_T    = 1<<7;     // T - User Interrupt (Software Interrupt)

//    localparam eCC_AccVR        = 1<<16;    // Access Violation Read
//    localparam eCC_AccVW        = 1<<17;    // Access Violation Write


//*************************************************************************
// One Hot bits for ALU operations
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


//*************************************************************************
// One Hot bits for Data Width field
localparam Width_bit_w  = 'h0;
localparam Width_bit_h  = 'h1;
localparam Width_bit_sh = 'h2;
localparam Width_bit_b  = 'h3;
localparam Width_bit_sb = 'h4;
                           

//*************************************************************************
// Condition Code Values
localparam eBranch_NA       = 3'b000;   // Always
localparam eBranch_EQ       = 3'b001;   // (.eq or .z) Ra == Rb / Z
localparam eBranch_NE       = 3'b010;   // (.ne or .nz) Ra != Rb / !Z
localparam eBranch_LT       = 3'b011;   // (.lt or .ng) Ra < Rb && Ra-Rb < 0 (signed)
localparam eBranch_GE       = 3'b100;   // (.ge or .p) Ra >= Rb && Ra-Rb >= 0 (signed)
localparam eBranch_C        = 3'b101;   // (.c) Ra < Rb && Ra-Rb < 0 (unsigned)
localparam eBranch_NC       = 3'b110;   // (.nc) Ra >= Rb && Ra - Rb >= 0 (unsigned)
localparam eBranch_V        = 3'b111;   // (.v) Overflow
