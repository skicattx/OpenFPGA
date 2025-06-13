`timescale 1ns / 1ns

//****************************************************************************
// Ember System Main Board Implementation
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


// Ember32 Module Implementation 
module system_top(
        input          sys_clk,          // System Clock
        input          sys_rst_n         // System _Reset
    );

    
    wire    [31:0]  mem_address;         // Address to read/write
    wire    [31:0]  mem_data_write;      // Data output/write
    wire     [3:0]  mem_data_write_mask; // Data byte write bitmask
            
    wire    [31:0]  mem_data_read;       // Data bus input/read                                

    wire            mem_read_strobe;     // High when read is requested ?
    wire            mem_write_strobe;    // High when write is requested ?
        
    wire            mem_read_wait;       // High when memory read is busy
    wire            mem_write_wait;      // High when memory write is busy

    wire            addr_RAM;            // True when address represents a RAM Address           ($xxxVVVVV) 
    wire            addr_VRAM;           // True when address represents a VRAM Address          ($4xxVVVVV) 
    wire            addr_HighPage;       // True when address represents a High Page Address     ($8xxxVVVV) 	

    
    
    wire            RAM_read_wait;
    wire            VRAM_read_wait;
    wire            ROM_read_wait;
    assign  mem_read_wait = /*RAM_read_wait | VRAM_read_wait | */ ROM_read_wait;
   
   
    // Never wait for now
    assign  mem_write_wait = 0;


    
    //*************************************************************************
    // 
    assign  addr_VRAM = mem_address[31:30] == 2'b01;      
    assign  addr_HighPage = mem_address[31] == 1'b1;  
    assign  addr_RAM = mem_address[31] == 2'b00;  
    
    
    //*************************************************************************
    // Ember CPU
    ember CPU(
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),

        .mem_address_out(mem_address),    
        .mem_data_write(mem_data_write),       
        .mem_data_write_mask(mem_data_write_mask),  

        .mem_data_read(mem_data_read),                             

        .mem_read_strobe(mem_read_strobe),    
        .mem_write_strobe(mem_write_strobe),   

        .mem_read_wait(mem_read_wait),      
        .mem_write_wait(mem_write_wait)
    );
    
    
    
    //*************************************************************************
    // System ROM/Firmware
    wire ROM_enable = addr_HighPage & mem_read_strobe;
    
    SysROM ROM(
        .sys_clk(sys_clk),        
        
        .mem_address(mem_address[15:2]), // word-aligned, 32-bit read always        
        .mem_data_read(mem_data_read),   
        
        .mem_read_enable(ROM_enable),
        .mem_read_wait(ROM_read_wait)   
    );

    
endmodule