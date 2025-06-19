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
        input          sys_nmi_n,           // Non-Maskable Interrupt (low on clk pos will set X bit in uCC - super PC on next clk)

        output [31:0]  mem_address_out,     // Address to read/write
        output [31:0]  mem_data_write,        // Data output/write
        output [3:0]   mem_data_write_mask,   // Data byte write bitmask
        
        input  [31:0]  mem_data_read,         // Data bus input/read                                

        output         mem_read_strobe,     // High when read is requested ?
        output         mem_write_strobe,    // High when write is requested ?
        
        input          mem_read_wait,       // High when memory read is busy
        input          mem_write_wait       // High when memory write is busy

//        output reg                      addr_RAM,               // True when address represents a RAM Address           ($xxxVVVVV) 
//        output reg                      addr_VRAM,              // True when address represents a VRAM Address          ($4xxVVVVV) 
//        output reg                      addr_HighPage           // True when address represents a High Page Address     ($8xxxVVVV) 	
    );

    assign mem_data_write_mask = 4'b1000;
    assign mem_read_strobe = 1'b1; // TODO: connect to signals...
    
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

    // Status Register Bits (TODO: Move to header? `define?)
    localparam eCC_Z            = 1<<0;     // EQ/Z  -- NE
    localparam eCC_C            = 1<<1;     // C     -- NC
    localparam eCC_N            = 1<<2;     // LT/N -- GE
    localparam eCC_V            = 1<<3;     // V
    
    localparam eCC_D            = 1<<4;     // D - (clears GIE)
    localparam eCC_G            = 1<<5;     // G - Global Interrupt Enable/!Supervisor Mode (Set/Unset to switch to User/Super Mode) (Setting also clears D, X, T, )4, // D - (clears GIE)
    localparam eCC_X            = 1<<6;     // X - (clears GIE)5, // G - Global Interrupt Enable/!Supervisor Mode (Set/Unset to switch to User/Super Mode) (Setting also clears D, X, T, )
    localparam eCC_T            = 1<<7;     // T - User Interrupt (Soft Interrupt, clears GIE)0, // EQ/Z  -- NE

//    localparam eCC_AccVR        = 1<<16;    // Access Violation Read
//    localparam eCC_AccVW        = 1<<17;    // Access Violation Write

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
    localparam eFetchInstruction_Wait_bit   = 1;    // Wait on memory fetch (as needed)
    localparam eExecuteInstruction_bit      = 2;
    localparam eExecuteInstruction_Wait_bit = 3;    // Wait on memory fetch/write or ALU (as needed)
    localparam eStateCount                  = 4;
    
    localparam eState_FetchInstruction          = 4'b0001 << eFetchInstruction_bit;
    localparam eState_FetchInstruction_Wait     = 4'b0001 << eFetchInstruction_Wait_bit;
    localparam eState_ExecuteInstruction        = 4'b0001 << eExecuteInstruction_bit;
    localparam eState_ExecuteInstruction_Wait   = 4'b0001 << eExecuteInstruction_Wait_bit;
    
//    localparam eState_FetchInstruction      = 4'b0001 << eFetchInstruction_bit;
//    localparam eState_Wait_FetchInstruction = 4'b0001 << eWait_FetchInstruction_bit;
//    localparam eState_ExecuteInstruction    = 4'b0001 << eExecuteInstruction_bit;
//    localparam eState_Wait_ALU_or_MEM       = 4'b0001 << eWait_ALU_or_MEM_bit;

    
    //*************************************************************************
    // CPU State Registers
    reg         system_mode;
    reg  [31:0] registers[0:1][0:17];//31]; // [supervisor/user][register index]
    
    reg  [31:0] srcA_val;               // Src A Value 
    reg  [31:0] srcB_val;               // Src B/Imm Value
    wire [31:0] alu_result;             // Result Value    
    wire [4:0]  alu_cc;                 // alu condition and exception bits     
    wire        alu_busy;               // High when ALU is busy (divide, etc.)

   
    
    //*************************************************************************
    // Opcode params       
    wire [5:0]  inst_opcode;
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
    wire [3:0]  reg_srcB;
        
    wire        imm_val_en;
    wire [31:0] imm_value;
    

    

    //*************************************************************************
    // Instruction Decoder
    decoder instruction_decoder(
        .instruction(mem_data_read),          // Non-registered, directly wired to memory input
        
        .inst_opcode(inst_opcode),
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
        .operation(inst_opcode[3:0]),
        .srcA_val(srcA_val),
        .srcB_val(srcB_val),
        
        .result(alu_result),
        .cc(alu_cc),
        
        .busy(alu_busy)
    );

    
    
    //*************************************************************************
    //
    wire predicate = (branch_cond == eBranch_NA) |
                     (alu_cc & eCC_Z ? branch_cond == eBranch_EQ : branch_cond == eBranch_NE) |
                     (alu_cc & eCC_C ? branch_cond == eBranch_C  : branch_cond == eBranch_NC) |
                     (alu_cc & eCC_N ? branch_cond == eBranch_LT : branch_cond == eBranch_GE) |
                     (alu_cc & eCC_V && branch_cond == eBranch_V);
    

    // Latches for these, as the wire from the decode is only valid for the Execute, not ExecuteWait State
    reg load_in_progress;
    reg store_in_progress;
    
    
    //*************************************************************************
    // Various Program Counter, Load, Store Address Possibilities
    wire [31:0] PC_plus4 = registers[system_mode][eReg_PC] + 4;                  // The next PC if we don't branch, or swap system modes, breakpoint handler, etc.
    
    wire [31:0] PC_branch = registers[system_mode][eReg_PC] + branch_offset;     // The next PC if we're branching
    
    
    wire [31:0] PC_new = inst_branch ? PC_branch : PC_plus4;
    
    
    wire [31:0] load_store_address = 0;//srcA_val + addr_offset; // Lowest 2 bits will be used for mask/shift for aligned memory access
    


    //*************************************************************************
    // State Machine (ONEHOT)
    wire fetch_waiting_for_data = mem_read_wait;                                    // Hold fetch state for load operation
    wire execute_needs_to_wait  = alu_busy | load_in_progress | store_in_progress;  // Execution will need to wait for data 
    wire execute_waiting        = alu_busy | mem_read_wait | mem_write_wait;        // Execution is waiting for data to get where it's supposed to go
    
    // Active CPU State
    reg [eStateCount-1:0] active_state; // (* onehot *)

    // Address that is always active on the address bus
    assign mem_address_out = (active_state[eFetchInstruction_bit] | active_state[eFetchInstruction_Wait_bit]) ? registers[system_mode][eReg_PC] : {load_store_address[31:2], 2'b00};
    
    // Value that will be written to the dest register (used only if this instruction writes to a register reg_result_en/reg_result_high_en)
    wire [31:0] reg_result =  inst_ldi        ? imm_value  : 
                              inst_mov        ? srcA_val   : alu_result;
    wire reg_result_en = inst_ldi | inst_mov; // TODO: BRA/BRL + predicate?
    wire reg_result_high_en = inst_ldi & ldi_high_half;

    // Compute the CC values for the next instruction (User or Super)
    wire [7:0] user_cc_new = (system_mode == eSystemMode_User) ?
                             (alu_cc | (inst_trap ? eCC_T : 0) | (inst_rtu|inst_illegal|inst_halt ? eCC_X : 0)) | (inst_trap|inst_rtu|inst_illegal|inst_halt|!sys_nmi_n ? 0 : eCC_G) :
                             registers[eSystemMode_User][eReg_CC][7:0] | (inst_rtu ? eCC_G : 0);

    wire [7:0] super_cc_new = alu_cc | (inst_illegal|inst_halt ? eCC_X : 0);

    // Based on the computed CC flag, which mode for the next instruction
    wire system_mode_new = (user_cc_new & eCC_G) ? eSystemMode_User : eSystemMode_Super;

    wire temp_mov_reg_bank = system_mode | reg_dest[5];

    always @(posedge sys_clk) 
    begin
        if(!sys_rst_n) 
        begin
            active_state        <= eState_ExecuteInstruction_Wait;        // Just waiting for !mem_wbusy
            system_mode         <= eSystemMode_Super;
            load_in_progress    <= 1'b0;
            store_in_progress   <= 1'b0;

            registers[eSystemMode_Super][eReg_PC]   <= RESET_ADDRESS;
            registers[eSystemMode_Super][eReg_Zero] <= 8'b00000000;
            registers[eSystemMode_Super][eReg_CC]   <= 8'b00000000;
            registers[eSystemMode_User][eReg_Zero ] <= 8'b00000000;
            registers[eSystemMode_User][eReg_CC ]   <= 8'b00000000;
        end 
        else
        begin
//            (* parallel_case *)
            case(1'b1) // synthesis parallel_case 


                active_state[eFetchInstruction_bit]: 
                    begin
                        active_state <= eState_FetchInstruction_Wait;
                    end

                active_state[eFetchInstruction_Wait_bit]: 
                    begin
                        if (!fetch_waiting_for_data) 
                        begin 
                            srcA_val            <= registers[system_mode | reg_srcA[5]][reg_srcA[4:0]];
                            srcB_val            <= registers[system_mode][reg_srcB];
                            active_state        <= eState_ExecuteInstruction;
                            
                            load_in_progress    <= inst_load;
                            store_in_progress   <= inst_store;
                        end
                    end

                active_state[eExecuteInstruction_bit]: 
                    begin
                        registers[system_mode][eReg_PC] <= PC_new;

                        if (execute_needs_to_wait) 
                            active_state <= eState_ExecuteInstruction_Wait;
                        else
                        begin
                            // Write any dest register?
                            if (reg_result_high_en)
                                registers[system_mode][reg_dest[4:0]][31:16] <= reg_result[31:16];
                            else if (reg_result_en)
                                registers[temp_mov_reg_bank][reg_dest[4:0]] <= reg_result; // Note, if this is a MOV pc, this takes priority over the assignment of PC_new above
//                                registers[system_mode | reg_dest[5]][reg_dest[4:0]] <= reg_result; // Note, if this is a MOV pc, this takes priority over the assignment of PC_new above
                            
                            // Write new CC values
                            registers[eSystemMode_User][eReg_CC][7:0] <= user_cc_new;
                            if (system_mode == eSystemMode_Super)
                                registers[eSystemMode_Super][eReg_CC][7:0] <= super_cc_new;

                            system_mode <= system_mode_new;                         
                            active_state <= eState_FetchInstruction;
                        end
                    end

                active_state[eExecuteInstruction_Wait_bit]: 
                    begin
                        if (!execute_waiting) 
                            active_state <= eState_FetchInstruction;

                            // TODO: write result when finished to reg, PC, or memory, etc.
                    end
            
            endcase
        end
    end
    
endmodule