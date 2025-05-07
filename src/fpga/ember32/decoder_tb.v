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
module decoder_tb;

wire sys_clk; 
wire sys_rst;

wire [31:0] data_in;
wire [31:0] data_out;
//wire [ADDRESS_WIDTH-1:0] address_out;
wire [19:0] address_out;

wire data_read_enable;
wire data_write_enable;
wire addr_ZeroPage;
wire addr_RAM;
wire addr_VRAM;
wire addr_HighPageROM;

decoder decoder_test(
    sys_clk, sys_rst,                        

    data_in,                
    data_out,               
    address_out,

    data_read_enable,              
    data_write_enable,             

    addr_ZeroPage, addr_RAM, addr_VRAM, addr_HighPageROM);


    
    
    
    
endmodule
		