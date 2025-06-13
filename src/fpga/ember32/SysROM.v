`timescale 1ns / 1ns

//****************************************************************************
// Ember CPU System ROM (Firmware) Implementation
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


// Ember System ROM Firmware (infered block RAM for now)
module SysROM( 
        input               sys_clk,            // System Clock
        
        input      [13:0]   mem_address,        // Address to read (upper 14 of 16 bits [word aligned, 32-bit])
        output reg [31:0]   mem_data_read,      // Data output/write

        input               mem_read_enable,    // High for at least one clock when memory read is requested
        output              mem_read_wait       // High when memory read is busy (unused for bram implementation...could be used for SDRAM, etc. though)
    );

    
    assign mem_read_wait = 0;

    // The actual memory array    
//    reg [31:0] reg_array [2**13-1:0];
    reg [31:0] reg_array [255:0]; // TODO: Just 1024 bytes for now...

    // For simulation
    integer i;
    initial begin
        for( i = 0; i < 256; i = i + 1 ) begin
            reg_array[i] <= 0;
        end
    end

    always @(posedge sys_clk) 
    begin
        if (mem_read_enable)
//            mem_data_out = reg_array[mem_address];
            mem_data_read <= reg_array[mem_address[7:0]];
    end

endmodule