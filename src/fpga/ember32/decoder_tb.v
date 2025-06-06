`timescale 1ns / 1ns

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

    reg  sys_clk; 
    reg  sys_rst;

    reg  data_read_ready;
    reg  data_write_complete;

    reg  [31:0] data_in;
    wire [31:0] data_out;

    wire data_read_request;
    wire data_write_request;

    wire [20-1:0] address_out;
//    wire [decoder.ADDRESS_WIDTH-1:0] address_out;

    wire addr_RAM;
    wire addr_VRAM;
    wire addr_HighPageROM;
  
    
    decoder decoder_test(
        sys_clk, sys_rst,                        

        data_read_ready, data_write_complete,
        
        data_in, data_out,               
        
        data_read_request, data_write_request,             
        
        address_out,

        addr_RAM, addr_VRAM, addr_HighPageROM );

        
    // Sequence of tests   
    initial begin
        sys_clk = 0;
        sys_rst = 0;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #6;
        sys_rst = 1;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #12;
        data_in = 0;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #21;
        sys_rst = 0;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #8;
        data_read_ready = 1;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #30;
        data_read_ready = 0;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #31;
        data_in = 'h04000000;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        #40;
        $display("data_in=0x%8h, data_read_request=%d, address_out=%5h, addr_RAM=%d, addr_VRAM=%d, addr_HighPageROM=%d", data_in, data_read_request, address_out, addr_RAM, addr_VRAM, addr_HighPageROM);
        
//            $finish;
    end


    always #5 sys_clk = ~sys_clk;
    
    
    
    
endmodule
        