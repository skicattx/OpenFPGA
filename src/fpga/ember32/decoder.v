`timescale 1ns / 1ps

//****************************************************************************
// Ember CPU Instruction ? Implementation
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

//`define ADDRESS_WIDTH   20     // Address width for Ember 1MB system bus (10 bits are ignored, plus 1 for VRAM and 1 for ROM

// Ember32 instruction decoder
module decoder( 
    input sys_clk,                          // System Clock
    input sys_rst,                          // System Reset

    input  [31:0] data_in,                  // Data bus input/read
    output [31:0] data_out,                 // Data bus output/write
//    output [`ADDRESS_WIDTH-1:0] address_out, // (Sub-) Address to read from
    output [19:0] address_out, // (Sub-) Address to read from
    
    output data_read_enable,                // True when data is requested on data_in (address_out and addr* bits will be valid)
    output data_write_enable,               // True when data on data_out is ready to be written (address_out and addr* bits will be valid)

	output addr_ZeroPage,                   // True when address represents a Zero Page Address     ($xxxxVVVV) 
	output addr_RAM,                        // True when address represents a RAM Address           ($xxxVVVVV) 
	output addr_VRAM,                       // True when address represents a VRAM Address          ($4xxVVVVV) 
	output addr_HighPage                    // True when address represents a High Page Address     ($8xxxVVVV) 	
);


// Stages:  reset, pc_fetch, ip_stall?, decode, execute, retire, halt



endmodule