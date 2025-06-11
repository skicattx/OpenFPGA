`timescale 1ns / 1ns

//****************************************************************************
// Ember CPU Implementation
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
module ember
    #(
        parameter RESET_ADDRESS = 32'hFFFF0000 
    )
    ( 
        input          sys_clk,             // System Clock
        input          sys_rst_n,           // System _Reset

        output [31:0]  mem_address_out,     // Address to read/write
        output [31:0]  mem_data_out,        // Data output/write
        output [3:0]   mem_data_out_mask,   // Data byte write bitmask
        
        input  [31:0]  mem_data_in,         // Data bus input/read                                

        output         mem_read_strobe,     // High when read is requested ?
        output         mem_write_strobe,    // High when write is requested ?
        
        input          mem_read_wait,       // High when memory read is busy
        input          mem_write_wait       // High when memory write is busy

//        output reg                      addr_RAM,               // True when address represents a RAM Address           ($xxxVVVVV) 
//        output reg                      addr_VRAM,              // True when address represents a VRAM Address          ($4xxVVVVV) 
//        output reg                      addr_HighPage           // True when address represents a High Page Address     ($8xxxVVVV) 	
    );

    assign mem_data_out_mask = 4'b1000;
    
    //*************************************************************************
    // #defines
    localparam eSystemMode_Super    = 1'b0;        // 
    localparam eSystemMode_User     = 1'b1;        // 

    // Special registers    
    localparam eReg_Zero        = 0;        // 
    localparam eReg_LR          = 14;       // 
    localparam eReg_SP          = 15;       // 
    localparam eReg_CC          = 16;       // 
    localparam eReg_PC          = 17;       // 

    // Status Register Bits
    localparam eCC_Z            = 1<<0;     // EQ/Z  -- NE
    localparam eCC_C            = 1<<1;     // C     -- NC
    localparam eCC_N            = 1<<2;     // LT/N -- GE
    localparam eCC_V            = 1<<3;     // V
    
    localparam eCC_D            = 1<<4;     // D - (clears GIE)
    localparam eCC_G            = 1<<5;     // G - Global Interrupt Enable/!Supervisor Mode (Set/Unset to switch to User/Super Mode) (Setting also clears D, X, T, )4, // D - (clears GIE)
    localparam eCC_X            = 1<<6;     // X - (clears GIE)5, // G - Global Interrupt Enable/!Supervisor Mode (Set/Unset to switch to User/Super Mode) (Setting also clears D, X, T, )
    localparam eCC_T            = 1<<7;     // T - User Interrupt (Soft Interrupt, clears GIE)0, // EQ/Z  -- NE

    localparam eCC_AccVR        = 1<<16;    // Access Violation Read
    localparam eCC_AccVW        = 1<<17;    // Access Violation Write

    // Condition Codes
    localparam eBranch_NA       = 3'b000;   // Always
    localparam eBranch_EQ       = 3'b001;   // (.eq or .z) Ra == Rb / Z
    localparam eBranch_NE       = 3'b010;   // (.ne or .nz) Ra != Rb / !Z
    localparam eBranch_LT       = 3'b011;   // (.lt or .ng) Ra < Rb && Ra-Rb < 0 (signed)
    localparam eBranch_GE       = 3'b100;   // (.ge or .p) Ra >= Rb && Ra-Rb >= 0 (signed)
    localparam eBranch_C        = 3'b101;   // (.c) Ra < Rb && Ra-Rb < 0 (unsigned)
    localparam eBranch_NC       = 3'b110;   // (.nc) Ra >= Rb && Ra - Rb >= 0 (unsigned)
    localparam eBranch_V        = 3'b111;   // (.v) Overflow

    // State Machine
    localparam eFetchInstruction_bit        = 0;
    localparam eWait_FetchInstruction_bit   = 1;
    localparam eExecuteInstruction_bit      = 2;
    localparam eWait_ALU_or_MEM_bit         = 3;
    localparam eStateCount                  = 4;
    
    localparam eState_FetchInstruction      = 1 << eFetchInstruction_bit;
    localparam eState_Wait_FetchInstruction = 1 << eWait_FetchInstruction_bit;
    localparam eState_ExecuteInstruction    = 1 << eExecuteInstruction_bit;
    localparam eState_Wait_ALU_or_MEM       = 1 << eWait_ALU_or_MEM_bit;
    
//    localparam eState_FetchInstruction      = 4'b0001 << eFetchInstruction_bit;
//    localparam eState_Wait_FetchInstruction = 4'b0001 << eWait_FetchInstruction_bit;
//    localparam eState_ExecuteInstruction    = 4'b0001 << eExecuteInstruction_bit;
//    localparam eState_Wait_ALU_or_MEM       = 4'b0001 << eWait_ALU_or_MEM_bit;

    
    //*************************************************************************
    // CPU State Registers
    reg         system_mode;
    reg  [31:0] registers[0:1][0:31]; // [supervisor/user][register index]
    
    reg  [31:0] srcA_val;               // Src A Value 
    reg  [31:0] srcB_val;               // Src B/Imm Value
    wire [31:0] alu_result;             // Result Value    
    wire        alu_busy;               // High when ALU is busy (divide, etc.)

    
    
    
    //*************************************************************************
    // Opcode params       
    wire        inst_illegal;
    wire        inst_noop;
    wire        inst_halt;
    wire        inst_trap;
    wire        inst_rtu;
    wire        inst_branch;
    wire        inst_mov;
    wire        inst_ldi;
    wire        inst_load;
    wire        inst_store;
    wire        inst_alu;

    wire [2:0]  branch_cond;   
    wire        branch_imm_en;
    wire [31:0] branch_offset;

    wire [2:0]  data_width;         
    wire [5:0]  reg_mov_dest;       
    wire [5:0]  reg_mov_src;       

    wire        ldi_high_half;            

    wire        addr_predec_postinc;
    wire [31:0] addr_offset;   
        
    wire [5:0]  reg_dest;
    wire [5:0]  reg_srcA;
    wire [4:0]  reg_srcB;
        
    wire        imm_val_en;
    wire [31:0] imm_value;


    //*************************************************************************
    // Instruction Decoder
    decoder instruction_decoder(
        .instruction(mem_data_in),          // Non-registered, directly wired to memory input
        
        .inst_illegal(inst_illegal),
        .inst_noop(inst_noop),
        .inst_halt(inst_halt),
        .inst_trap(inst_trap),
        .inst_rtu(inst_rtu),
        .inst_branch(inst_branch),
        .inst_mov(inst_mov),
        .inst_ldi(inst_ldi),
        .inst_load(inst_load),
        .inst_store(inst_store),
        .inst_alu(inst_alu),
        .branch_cond(branch_cond),
        .branch_imm_en(branch_imm_en),
        .branch_offset(branch_offset),
        .data_width(data_width),
        .ldi_high_half(ldi_high_half),
        .addr_predec_postinc(addr_predec_postinc),
        .addr_offset(addr_offset),
        .reg_dest(reg_dest),
        .reg_srcA(reg_srcA),
        .reg_srcB(reg_srcB),
        .imm_val_en(imm_val_en),
        .imm_value(imm_value)
    );

   

    //*************************************************************************
    // ALU Implementation
    alu alu( 
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        
        .srcA_val(srcA_val),
        .srcB_val(srcB_val),
        .result(alu_result),
        
        .busy(alu_busy)
    );

    
    
    //*************************************************************************
    //
    wire [3:0] cc = registers[system_mode][eReg_CC][3:0];
    wire predicate = (branch_cond == eBranch_NA) |
                     (cc & eCC_Z ? branch_cond == eBranch_EQ : branch_cond == eBranch_NE) |
                     (cc & eCC_C ? branch_cond == eBranch_C  : branch_cond == eBranch_NC) |
                     (cc & eCC_N ? branch_cond == eBranch_LT : branch_cond == eBranch_GE) |
                     (cc & eCC_V && branch_cond == eBranch_V);
    
    
    
    //*************************************************************************
    // Various Program Counter, Load, Store Address Possibilities
    reg  [31:0] PC;                                 // The active PC (during fetch states)

    wire [31:0] PC_plus4 = PC + 4;                  // The next PC if we don't branch, or swap system modes, breakpoint handler, etc.
    
    wire [31:0] PC_branch = PC + branch_offset;     // The next PC if we're branching
    
    
//    wire [31:0] PC_new = inst_branch ? :
    
//    isJALR           ? {aluPlus[ADDR_WIDTH-1:1],1'b0} :
//    jumpToPCplusImm  ? PCplusImm :
//    PCplus4;
    
    
    wire [31:0] load_store_address = srcA_val + addr_offset; // Lowest 2 bits will be used for mask/shift for aligned memory access
    
    
    
    
    


    //*************************************************************************
    // State Machine (ONEHOT)
    
    wire waiting_for_data = inst_load | inst_store | alu_busy;      // Hold execution state for data to get where it's supposed to go
    
    
    reg [eStateCount-1:0] active_state; // (* onehot *)
//    reg [3:0] active_state; // (* onehot *)
    
    assign mem_address_out = active_state[eFetchInstruction_bit] | active_state[eWait_FetchInstruction_bit] ? PC : {load_store_address[31:2], 2'b00};


    always @(posedge sys_clk) 
    begin
        if(!sys_rst_n) 
        begin
            active_state                          <= eState_Wait_ALU_or_MEM;        // Just waiting for !mem_wbusy
            system_mode                           <= eSystemMode_Super;
            registers[eSystemMode_Super][eReg_PC] <= RESET_ADDRESS;
        end 
        else
        begin
//            (* parallel_case *)
            case(1'b1) // synthesis parallel_case 

                active_state[eWait_FetchInstruction_bit]: 
                    begin
                        if (!mem_read_wait) 
                        begin 
                            srcA_val <= registers[system_mode & reg_srcA[5]][reg_srcA[4:0]];
                            srcB_val <= registers[system_mode][reg_srcB];
//                            instr <= mem_rdata[31:2]; // Bits 0 and 1 are ignored (see
                            active_state <= eState_ExecuteInstruction;         // also the declaration of instr).
                        end
                    end

                active_state[eExecuteInstruction_bit]: 
                    begin
//                       PC <= PC_new;
//                       active_state <= waiting_for_data ? eState_Wait_ALU_or_MEM : eState_FetchInstruction;
                       active_state <= eState_FetchInstruction;
                    end

                active_state[eWait_ALU_or_MEM_bit]: 
                    begin
                        if (!alu_busy & !mem_read_wait & !mem_write_wait) 
                            active_state <= eState_FetchInstruction;
                    end

                default: 
                    begin // FETCH_INSTR
                        active_state <= eState_Wait_FetchInstruction;
                    end
                
            endcase
        end
    end
    
endmodule