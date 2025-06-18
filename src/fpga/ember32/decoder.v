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
//  op_divu  = 6'h16  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      |  | 
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 
//  op_and   = 6'h19  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_or    = 6'h1a  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_xor   = 6'h1b  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_lsr   = 6'h1c  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_lsl   = 6'h1d  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//  op_asr   = 6'h1e  //     |         Opcode        |   Width   | Dest Register | Source Reg A  |Imm| Source Reg B  /      Or Signed Immediate 14-bits      | 
//                    // BIT | 31| 30| 29| 28| 27| 26| 25| 24| 23| 22| 21| 20| 19| 18| 17| 16| 15| 14| 13| 12| 11| 10| 09| 08| 07| 06| 05| 04| 03| 02| 01| 00| 


`define DATA_WIDTH_W        'b000   // TODO: This is UNsigned...though I think the assembler and emulator currently treat imm values as signed...
`define DATA_WIDTH_H        'b001
`define DATA_WIDTH_SH       'b010
`define DATA_WIDTH_B        'b011
`define DATA_WIDTH_SB       'b100
`define DATA_WIDTH_HH       'b101   // Unsupported
`define DATA_WIDTH_BB       'b110   // Unsupported
`define DATA_WIDTH_BBBB     'b111   // Unsupported


// Ember32 instruction decoder
module decoder(
        input [31:0]    instruction,            // Active instruction word
                        
        output          inst_illegal,           // Illegal Instruction detected
        output          inst_noop,              // No-op instruction
        output          inst_halt,              // Halt instruction
        output          inst_trap,              // Trap instruction
        output          inst_rtu,               // Return to User Mode instruction
        output          inst_branch,            // Branch instruction
        output          inst_mov,               // Move instruction
        output          inst_ldi,               // Load Immediate instruction
        output          inst_load,              // Memory Load instruction
        output          inst_store,             // Memory Store instruction
        output          inst_alu,               // ALU instruction
        
        output [2:0]    branch_cond,            // Branch condition code
        output          branch_imm_en,          // Immediate value used instead of srcA index  
        output [31:0]   branch_offset,          // Sign-extended 24-bit (22-bit*4) branch offset (in place of srcA)
                    
        output [2:0]    data_width,             // Data read/write width code ]
                    
        output          ldi_high_half,          // LDI writes only upper 16 bits of fully resolved immediate value that is returned
                    
        output          addr_predec_postinc,    // pre-decrement or post-increment loads and stores (e.g. PUSH/POP pseudo instructions)  
        output [31:0]   addr_offset,            // Signed address offset imbedded into instruction
            
        output [5:0]    reg_dest,               // {user}(high}{Output/Destination register index}
        output [5:0]    reg_srcA,               // {user}(high}{Src register A index (Also Branch reg)}
        output [3:0]    reg_srcB,               // Src register B index
                
        output          imm_val_en,             // Immediate value used instead of srcB index in ALU 
        output [31:0]   imm_value               // Sign or 0 extended immediate value (masked from data_width code)
      
    );

    assign  inst_noop    = (instruction[31:26] == 6'b000000);
    assign  inst_halt    = (instruction[31:26] == 6'b000001);
    assign  inst_trap    = (instruction[31:26] == 6'b000010);
    assign  inst_rtu     = (instruction[31:26] == 6'b000011);
    assign  inst_branch  = (instruction[31:27] == 5'b00011);
    assign  inst_mov     = (instruction[31:26] == 6'b001010);
    assign  inst_ldi     = (instruction[31:26] == 6'b001011);
    assign  inst_load    = (instruction[31:26] == 6'b001110);
    assign  inst_store   = (instruction[31:26] == 6'b001111);
    assign  inst_alu     = (instruction[31:30] == 2'b01);
    assign  inst_illegal = ~(inst_halt | inst_noop | inst_trap | inst_rtu | inst_branch | inst_mov | inst_ldi | inst_load | inst_store | inst_alu);
    
    assign  branch_cond          = instruction[25:23];       
    assign  branch_imm_en        = instruction[22];    
    assign  branch_offset        = {{9{instruction[21]}}, instruction[20:0], {2{1'b0}}};     
    
    assign  data_width           = instruction[25:23];
        
    assign  ldi_high_half        = instruction[18];      
        
    assign  addr_predec_postinc  = instruction[14];
    assign  addr_offset          = {{19{instruction[13]}}, instruction[12:0]};
        
    assign  reg_dest             = inst_mov ? instruction[22:17] : {{2'b00}, instruction[22:19]};   // Mov instruction has two additional index bits
    assign  reg_srcA             = inst_mov ? instruction[16:11] : {{2'b00}, instruction[18:15]};   // Mov instruction has two additional index bits    
    assign  reg_srcB             = instruction[13:10];        
        
    assign  imm_val_en           = instruction[14];         
    
    
    // Decode in parallel the various formats for packed immediate bits
    wire [31:0] imm_8u   = {{24{1'b0}},            instruction[7:0]};   // 8-bit unsigned extended
    wire [31:0] imm_8s   = {{23{instruction[7]}},  instruction[6:0]};   // 8-bit signed extended
    wire [31:0] imm_14s  = {{19{instruction[13]}}, instruction[12:0]};  // 14-bit signed extended
    wire [31:0] imm_14u  = {{18{1'b0}},            instruction[13:0]};  // 14-bit unsigned extended
    
    wire [31:0] imm32  = (data_width==`DATA_WIDTH_B)  ? imm_8u  :
                         (data_width==`DATA_WIDTH_SB) ? imm_8s  :
                         (data_width==`DATA_WIDTH_SH) ? imm_14s : 
                                                    imm_14u;

    wire [31:0] imm_ldi_b   = ldi_high_half ? {{8{1'b0}},           instruction[7:0], {16{1'b0}}} : {{24{1'b0}},           instruction[7:0]}; // 8-bit unsigned extended to high half : 8-bit unsigned extended to word
    wire [31:0] imm_ldi_sb  = ldi_high_half ? {{9{instruction[7]}}, instruction[6:0], {16{1'b0}}} : {{25{instruction[7]}}, instruction[6:0]}; // 8-bit signed extended to high half   : 8-bit signed extended to word
    wire [31:0] imm_ldi_h   = ldi_high_half ? {instruction[15:0],   {16{1'b0}}} : {{16{1'b0}},            instruction[15:0]};     // 16-bit to high half : 16-bit unsigned extended to word
    wire [31:0] imm_ldi_sh  = ldi_high_half ? {instruction[15:0],   {16{1'b0}}} : {{17{instruction[15]}}, instruction[14:0]};     // 16-bit to high half : 16-bit signed extended to word
    wire [31:0] imm_ldi_w   = ldi_high_half ? {instruction[15:0],   {16{1'b0}}} : {{16{instruction[16]}}, instruction[15:0]};     // 16-bit to high half : 17-bit sign extended to word
                                             
    wire [31:0] imm_ldi = (data_width==`DATA_WIDTH_B)  ? imm_ldi_b  :
                          (data_width==`DATA_WIDTH_SB) ? imm_ldi_sb :
                          (data_width==`DATA_WIDTH_H)  ? imm_ldi_h  : 
                          (data_width==`DATA_WIDTH_SH) ? imm_ldi_sh : 
                                                         imm_ldi_w;
                             
    assign  imm_value = inst_ldi ? imm_ldi : imm32;       

            
endmodule        