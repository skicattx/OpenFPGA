`timescale 1ns / 1ns

//****************************************************************************
// Ember CPU Instruction Opcode Decoder Implementation
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


//****************************************************************************
// OpCodes
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 
//  op_nop   = 6'h00  //     |         Opcode        |                                          Unused                                                       | 
//  op_halt  = 6'h01  //     |         Opcode        |                                          Unused                                                       |        
//  op_trap  = 6'h02  //     |         Opcode        |                                          Unused                                                       |
//  op_rtu   = 6'h03  //     |         Opcode        |                                          Unused                                                       |
//      Branch        // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00|
//  op_bra   = 6'h06  //     |         Opcode        | Cond Code |Imm| Immediate /  Source Reg A /        Or Signed Immediate 22-bits                        |
//  op_brl   = 6'h07  //     |         Opcode        | Cond Code |Imm| Immediate /  Source Reg A /        Or Signed Immediate 22-bits                        |
// Special
//  op_mov   = 6'h0A  //     |         Opcode        |   Width   |umD|hiR| Dest Register |umA|hiR| Source Reg    |                 Unused                    |
//  op_ldi   = 6'h0B  //     |         Opcode        |   Width   | Dest Register | H | ? |HPF|     16-bit Immediate Value (signed extended with S bit)       |
//      Memory
//  op_ld    = 6'h0E  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |rA+|        Signed 14-bit Offset - Register A              |
//  op_st    = 6'h0F  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |-rD|      Signed 14-bit Offset - Dest Register             |
//      ALU           // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 
//  op_add   = 6'h10  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_sub   = 6'h11  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 
//  op_mul   = 6'h12  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_mhs   = 6'h13  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_mhu   = 6'h14  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_div   = 6'h15  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_divu  = 6'h16  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_rem   = 6'h17  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_remu  = 6'h18  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 
//  op_and   = 6'h19  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_or    = 6'h1a  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_xor   = 6'h1b  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_lsr   = 6'h1c  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_lsl   = 6'h1d  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_asr   = 6'h1e  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 



// Ember32 instruction decoder
module opcode(
        input [31:0]   instruction,        // Active instruction word
        
        output         inst_illegal,       // Illegal Instruction detected
        output         inst_noop,          // No-op instruction
        output         inst_halt,          // Halt instruction
        output         inst_trap,          // Trap instruction
        output         inst_rtu,           // Return to User Mode instruction
        output         inst_branch,        // Branch instruction
        output         inst_mov,           // Move instruction
        output         inst_ldi,           // Load Immediate instruction
        output         inst_load,          // Memory Load instruction
        output         inst_store,         // Memory Store instruction
        output         inst_alu,           // ALU instruction

        output [2:0]    branch_cond,        // Branch condition code
        output          branch_imm_en,      // Immediate value used instead of srcA index  
        output [21:0]   branch_offset,      // Signed (aligned / <<2) address offset imbedded into bra/brl instruction (in place of srcA)
        
        output [2:0]    data_width,         // Data read/write width code ]
        output [5:0]    reg_mov_dest,       // Dest register index for MOV instruction
        output [5:0]    reg_mov_src,        // Src register index for MOV instruction  
        
        output          ldi_high_half,      // LDI writes immediate value to high 16-bit half word
        output          ldi_high_page_fill, // LDI Writes all 1s or all 0s depending on this bit[16]
        output [15:0]   ldi_imm,            // LDI immediate value
        
        output          addr_predec_postinc,// pre-decrement or post-increment loads and stores (e.g. PUSH/POP pseudo instructions)  
        output [13:0]   addr_offset,        // Signed address offset imbedded into instruction

        output [3:0]    reg_dest,           // Output/Destination register index
        output [3:0]    reg_srcA,           // Src register A index (Also Branch reg)
        output [3:0]    reg_srcB,           // Src register B index
       
        output          imm_val_en,         // Immediate value used instead of srcB index in ALU 
        output [13:0]   imm_val             // Immediate value imbedded into instruction (in place of srcB)
       
    );

    assign inst_halt    = (instruction[31:26] == 6'b000000);
    assign inst_noop    = (instruction[31:26] == 6'b000001);
    assign inst_trap    = (instruction[31:26] == 6'b000010);
    assign inst_rtu     = (instruction[31:26] == 6'b000011);
    assign inst_branch  = (instruction[31:27] == 5'b00011);
    assign inst_mov     = (instruction[31:26] == 6'b001010);
    assign inst_ldi     = (instruction[31:26] == 6'b001011);
    assign inst_load    = (instruction[31:26] == 6'b001110);
    assign inst_store   = (instruction[31:26] == 6'b001111);
    assign inst_alu     = (instruction[31:30] == 2'b01);
    assign inst_illegal = ~(inst_halt | inst_noop | inst_trap | inst_rtu | inst_branch | inst_mov | inst_ldi | inst_load | inst_store | inst_alu);

    assign branch_cond          = instruction[25:23];       
    assign branch_imm_en        = instruction[22];    
    assign branch_offset        = instruction[21:0];     
     
    assign data_width           = instruction[25:23];         
    assign reg_mov_dest         = instruction[22:17];       
    assign reg_mov_src          = instruction[16:11];       
     
    assign ldi_high_half        = instruction[18];      
    assign ldi_high_page_fill   = instruction[16];  
    assign ldi_imm              = instruction[15:0];            
     
    assign addr_predec_postinc  = instruction[14];
    assign addr_offset          = instruction[13:0];        
     
    assign reg_dest             = instruction[22:19];
    assign reg_srcA             = instruction[18:15];
    assign reg_srcB             = instruction[13:10];
     
     
    assign imm_val_en           = instruction[14];         
    assign imm_val              = instruction[13:0];   


endmodule          