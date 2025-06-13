`timescale 10ns / 1ns

//****************************************************************************
// Ember System Testbench
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
module system_tb;

    reg  sys_clk; 
    reg  sys_rst_n;
  
    
    system_top system(
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n)
    );
    
    // Sequence of tests   
    initial begin
        sys_clk = 0;
        sys_rst_n = 1;
        #3;
        sys_rst_n = 0;
        #2.3;
//        $display("mem_address=$%8h (mem_data_write==$%8h, mem_data_write_mask=4'b%4b, mem_write_strobe=%d) (mem_data_read=$%8h, mem_read_strobe=%d)", mem_address, mem_data_write, mem_data_write_mask, mem_write_strobe, mem_data_read, mem_read_strobe);
        sys_rst_n = 1;
        #5;
//        $display("mem_address=$%8h (mem_data_write==$%8h, mem_data_write_mask=4'b%4b, mem_write_strobe=%d) (mem_data_read=$%8h, mem_read_strobe=%d)", mem_address, mem_data_write, mem_data_write_mask, mem_write_strobe, mem_data_read, mem_read_strobe);

        #100;
        $stop;
    end


    always #1 sys_clk = ~sys_clk;
    
    
    
    
endmodule
        