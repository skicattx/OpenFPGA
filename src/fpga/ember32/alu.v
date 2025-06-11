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
        input           sys_clk,                // System Clock
        input           sys_rst_n,              // System _Reset
        
        input [31:0]    srcA_val,               // Src A Value 
        input [31:0]    srcB_val,               // Src B/Imm Value
        output [31:0]   result,                 // Result Value    

        output          busy                    // Operation (divide, etc.) needs more time

/*      
        input                           data_read_ready,        // True when data_in is valid after data_read_enable is true
        input                           data_write_complete,    // True when data_out has been written after data_write_enable is true
        input  [31:0]                   data_in,                // Data bus input/read                                
        output reg [31:0]               data_out,               // Data bus output/write
            
        output reg                      data_read_request,      // True when data is requested on data_in (address_out and addr* bits will be valid)
        output reg                      data_write_request,     // True when data on data_out is ready to be written (address_out and addr* bits will be valid)

        output reg [ADDRESS_WIDTH-1:0]  address_out,            // (Sub-) Address to read from
                                
        output reg                      addr_RAM,               // True when address represents a RAM Address           ($xxxVVVVV) 
        output reg                      addr_VRAM,              // True when address represents a VRAM Address          ($4xxVVVVV) 
        output reg                      addr_HighPage           // True when address represents a High Page Address     ($8xxxVVVV) 	
*/        
    );

    
    //*************************************************************************
    // #defines
    localparam eStage_Reset         = 0;        // Reset State


    assign busy = 0;    

endmodule