`timescale 1ns / 1ns

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


// Ember32 instruction decoder
module decoder
    #(
        parameter ADDRESS_WIDTH = 20,
        parameter RESET_ADDRESS = 32'hFFFF0000 
    )
    ( 
        input                           sys_clk,                // System Clock
        input                           sys_rst,                // System Reset
                                    
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
    );

    
    //*************************************************************************
    // #defines
    localparam eStage_Reset         = 0;        // Reset State
    localparam eStage_Fetch         = 1;        // Fetch next instruction: address_out is valid - transition to Decode when 'read rdy' is signalled
//    localparam eStage_FetchStall    = 2;      // Fetch has not returned yet (Mem clk delay)
    localparam eStage_Decode        = 3;        // Opcode is loaded, waiting for settle of instruction/alu decode
    localparam eStage_LoadWait      = 4;        // Waiting for memory load from ld instruction (st will not wait)
    localparam eStage_Retire        = 5;        // Save result to register/mem?
    localparam eStage_Halt          = 6;        // halt instruction, or breakpoint? waiting for external continue? (needed?)

    localparam eRegBank_Super       = 0;        // 
    localparam eRegBank_User        = 1;        // 

    localparam eReg_Zero            = 0;        // 
    localparam eReg_LR              = 14;       // 
    localparam eReg_SP              = 15;       // 
    localparam eReg_CC              = 16;       // 
    localparam eReg_PC              = 17;       // 
    localparam eReg_Cycles          = 18;       // 
    localparam eReg_T0              = 19;       // 
    localparam eReg_T1              = 20;       // 
    localparam eReg_T2              = 21;       // 

    
    //*************************************************************************
    // For non-pipelined state
    reg [2:0]  cpu_stage;    
    reg        cpu_usermode;
    reg [31:0] registers[0:1][0:31]; // [supervisor/user][register index]
    
    
    //*************************************************************************
    // Opcode params
    reg  [31:0] instruction_word;
        
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
    wire [31:0] ldi_imm_value;            

    wire        addr_predec_postinc;
    wire [13:0] addr_offset;   
        
    wire [3:0]  reg_dest;
    wire [3:0]  reg_srcA;
    wire [3:0]  reg_srcB;
        
    wire        imm_val_en;
    wire [31:0] imm_value;


    opcode opcode_decoder(
        .instruction(instruction_word),        
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
        .reg_mov_dest(reg_mov_dest),
        .reg_mov_src(reg_mov_src),
        .ldi_high_half(ldi_high_half),
        .ldi_imm_value(ldi_imm_value),
        .addr_predec_postinc(addr_predec_postinc),
        .addr_offset(addr_offset),
        .reg_dest(reg_dest),
        .reg_srcA(reg_srcA),
        .reg_srcB(reg_srcB),
        .imm_val_en(imm_val_en),
        .imm_value(imm_value)
    );
    

    
    
    




    always @(posedge sys_clk)
    begin
        if (sys_rst) begin
            // CPU Reset
            $display("decoder: Trigger Synchronous Reset");
            cpu_stage <= eStage_Reset;
        end
        else if (cpu_stage == eStage_Reset) begin
            // Next clock after reset: fetch
            $display("decoder: exiting Reset to Fetch");
            cpu_stage  <= eStage_Fetch; 
        end 
        else if (cpu_stage == eStage_Reset) begin
            // Next clock after reset: fetch
            $display("decoder: exiting Reset to Fetch");
            cpu_stage  <= eStage_Fetch; 
        end 
    end
    
    // Handle Data Ready 
    always @(posedge data_read_ready)
    begin
        if (cpu_stage == eStage_Fetch) begin
            // Fetch is complete, transition to Decoding
            $display("decoder: eStage_Fetch => data_read_ready");
            instruction_word = data_in;
            data_read_request = 0;
            cpu_stage = eStage_Decode;
        end
        else if (cpu_stage == eStage_LoadWait) begin
            // Load is complete, transition to Retire
            $display("decoder: eStage_LoadWait => data_read_ready");
            registers[cpu_usermode][reg_dest] = data_in;
            data_read_request = 0;
            cpu_stage = eStage_Retire;
        end
        
    end
    
    // Handle Stage Change 
    always @(cpu_stage)
    begin
        case (cpu_stage)
            eStage_Reset: begin
                $display("decoder: cpu_stage => eStage_Reset");
                data_out = 0;
                data_read_request = 0;
                data_write_request = 0;
                
                address_out = 0;
                addr_RAM = 0;                
                addr_VRAM = 0;                
                addr_HighPage = 0;
                
                cpu_usermode = eRegBank_Super;
                
                registers[eRegBank_User][eReg_Cycles] = 0;
                registers[eRegBank_User][eReg_CC] = 0;
                
                registers[eRegBank_Super][eReg_PC] = RESET_ADDRESS;
                registers[eRegBank_Super][eReg_CC] = 0;
                registers[eRegBank_Super][eReg_Cycles] = 0;
            end
            eStage_Fetch: begin
                $display("decoder: cpu_stage => eStage_Fetch");
                address_out = registers[eRegBank_Super][eReg_PC][ADDRESS_WIDTH-1:0]; // Set the PC on the address buss
                addr_RAM = !registers[eRegBank_Super][eReg_PC][31] && !registers[eRegBank_Super][eReg_PC][30];      // address 0b00?? ???? ... is repeating RAM starting at 0x00000000
                addr_VRAM = !registers[eRegBank_Super][eReg_PC][31] && registers[eRegBank_Super][eReg_PC][30];     // address 0b01?? ???? ... is repeating VRAM starting at 0x40000000
                addr_HighPage = registers[eRegBank_Super][eReg_PC][31] && registers[eRegBank_Super][eReg_PC][30];  // address 0b11?? ???? ... is repeating ROM starting at 0xC0000000
                data_read_request = 1; // Tell memory we want something
                registers[eRegBank_Super][eReg_PC] = registers[eRegBank_Super][eReg_PC] + 4;
            end
            eStage_Decode: begin
                $display("decoder: cpu_stage => eStage_Decode");
                data_read_request = 0;
/*                case (cpu_stage)
                    if (inst_illegal) begin
                    end
                    if (inst_noop) begin
                    end
                    if (inst_halt) begin
                    end
                    if (inst_trap) begin
                    end
                    if (inst_rtu) begin
                    end
                    if (inst_branch) begin
                    end
                    if (inst_mov) begin
                    end
                     if (inst_ldi) begin
                    end
                    if (inst_load) begin
                    end
                    if (inst_store) begin
                    end
                    if (inst_alu) begin
                    end
                endcase
*/                
                // Handle special instruction types here? (that need reads, writes, jumps, etc?)
            end
            eStage_LoadWait: begin
                $display("decoder: cpu_stage => eStage_LoadWait");
            end
            eStage_Retire: begin
                $display("decoder: cpu_stage => eStage_Retire");
            end
            eStage_Halt: begin
                $display("decoder: cpu_stage => eStage_Halt");
            end
        endcase
    end    
        
       
    
            // Default CPU states after a reset
        
        
//       registers[SysMode::supervisor][RegSet::gp][0] <= 0; 
//       registers[SysMode::user][RegSet::gp][0] <= 0;
//       for (integer i=1; i<16; i=i+1) begin
//           registers[SysMode::supervisor][RegSet::gp][i] <= i-3; 
//           registers[SysMode::user][RegSet::gp][i] <= i<<2;
//       end
//       registers[SysMode::supervisor][RegSet::system][SysReg::cc] <= 0; 
//       registers[SysMode::supervisor][RegSet::system][SysReg::pc] <= ISADefaults::SupervisorStartAddress; 
//       registers[SysMode::user][RegSet::system][SysReg::cc] <= 0;
//       registers[SysMode::user][RegSet::system][SysReg::pc] <= ISADefaults::UserStartAddress;
//       
//       curStage            <= PipelineStage::reset;
///        nextStage       <= PipelineStage::pc_fetch;
//       curUserMode         <= SysMode::supervisor;
//       nextUserMode        <= SysMode::supervisor;
//       nextAddress         <= ISADefaults::SupervisorStartAddress;
//       nextWriteResult     <= 1'b0;
//       nextResultALU       <= 0;
//       nextResultMOV       <= 0;
//       
//       nextWritePC         <= 1'b0;
//       nextResultPC        <= 0;
//        
//       nextTrap            <= 1'b0;  
//       nextIllegalOp       <= 1'b0;  
//       nextGlobalIntEnable <= 1'b0;  
//       nextDivZero         <= 1'b0;  
//       nextOverflow        <= 1'b0;
//       nextNegative        <= 1'b0;
//       nextCarry           <= 1'b0;
//       nextZero            <= 1'b0;
/*
    end
    else begin
        curStage <= nextStage;
        curUserMode <= nextUserMode;
        curAddress <= nextAddress;
        
        case (nextStage)
            PipelineStage::pc_fetch : begin
                // Increment PC
                registers[curUserMode][RegSet::system][SysReg::pc] <= registers[curUserMode][RegSet::system][SysReg::pc] + 4;                 
            end
            PipelineStage::decode : begin             

            end
            PipelineStage::execute : begin
                // Latch the various result values
                nextWriteResult <= active_operation.writeResult;
                nextResultALU <= active_operation.aluResult;
                nextResultMOV <= active_operation.movResult;
                
                nextWritePC <= active_operation.writePC;
                
                // For now, only MOV writes this, but when we add LD, BRA, BRL, etc. this might be written in different stages
                nextResultPC <= (active_operation.op.ldi.opCode == OpCode::op_mov) ? active_operation.movResult : 0;
                
                nextTrap            <= active_operation.trap;           
                nextIllegalOp       <= active_operation.illegalOp;      
                nextGlobalIntEnable <= active_operation.globalIntEnable;
                nextDivZero         <= active_operation.divZero;           

                nextOverflow        <= active_operation.overflow;
                nextNegative        <= active_operation.negative;
                nextCarry           <= active_operation.carry;
                nextZero            <= active_operation.zero;        
            end
            PipelineStage::retire : begin
                if (nextWriteResult) begin
                    if (active_operation.op.sys.opCode >= OpCode::op_add && active_operation.op.sys.opCode <= OpCode::op_asr) begin // TODO: make a Task()/Fn
                        // Store ALU Result Value and Flags
                        registers[curUserMode][RegSet::gp][active_operation.op.alu_rr.regDest] <= nextResultALU;
                        registers[curUserMode][RegSet::system][SysReg::cc][3:0] <= '{nextOverflow, nextNegative, nextCarry, nextZero};
                    end 
                    else if (active_operation.op.ldi.opCode == OpCode::op_ldi) begin
                        // LDI/LDIH Instructions
                        if (active_operation.op.ldi.hiLoFlag == LDIHiLo::highHalf)
                            registers[curUserMode][RegSet::gp][active_operation.op.ldi.regDest][31:16] <= Lib::MaskWidthCode16(active_operation.op.ldi.width, active_operation.op.ldi.immVal);
                        else    
                            registers[curUserMode][RegSet::gp][active_operation.op.ldi.regDest][31:0] <= Lib::MaskWidthCode16(active_operation.op.ldi.width, active_operation.op.ldi.immVal);
                    end 
                    else if (active_operation.op.ldi.opCode == OpCode::op_mov) begin
                        // Store MOV Result Value
                        registers[curUserMode | active_operation.op.mov.userRegD][active_operation.op.mov.regSetD][active_operation.op.mov.regDest] <= nextResultMOV;
                    end
                end
                
                if (nextWritePC)
                    nextAddress <= nextResultPC; // If this instruction is updating PC, write the value directly (since updated PC won't be valid until next clock)
                else
                    nextAddress <= registers[curUserMode][RegSet::system][SysReg::pc];

                registers[curUserMode][RegSet::system][SysReg::cc][7:4] <= '{nextTrap, nextIllegalOp, nextGlobalIntEnable, nextDivZero};
            end
            default : ;
        endcase
    end
end    
*/


endmodule