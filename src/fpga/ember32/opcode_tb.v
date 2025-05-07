`timescale 1ns / 1ps

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
module opcode_tb;

reg         sys_rst = 'b1;

reg  [31:0] instruction = 'h00;
    
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
wire [21:0] branch_offset;

wire [2:0]  data_width;         
wire [5:0]  reg_mov_dest;       
wire [5:0]  reg_mov_src;       

wire        ldi_high_half;      
wire        ldi_high_page_fill;    
wire [15:0] ldi_imm;            

wire        addr_predec_postinc;
wire [13:0] addr_offset;   
    
wire [3:0]  reg_dest;
wire [3:0]  reg_srcA;
wire [3:0]  reg_srcB;
    
wire        imm_val_en;
wire [13:0] imm_val;


opcode opcode_test(
    sys_rst, instruction,        
    inst_illegal, inst_noop, inst_halt,inst_trap, inst_rtu, inst_branch, inst_mov, inst_ldi, inst_load, inst_store, inst_alu,           
    branch_cond, branch_imm_en, branch_offset,      
    data_width, reg_mov_dest, reg_mov_src,        
    ldi_high_half, ldi_high_page_fill, ldi_imm,            
    addr_predec_postinc, addr_offset,        
    reg_dest, reg_srcA, reg_srcB,           
    imm_val_en, imm_val );
   
    // Sequence of instructions   
    initial begin
        instruction = 'hFFFFFFFF; // Invalid
        #5;
        instruction = 'h00000000; // Noop
        #5;
        instruction = 'h04000000; // halt
        #5;
        instruction = 'h08000000; // trap
        #5;
        instruction = 'h0C000000; // rtu
        #5;

        // bra
        instruction = 'h18008000;  
        #5
        instruction = 'h187fffff; 
        #5
        instruction = 'h18838000; 
        #5
        instruction = 'h18c00005; 
        #5

        // brl
        instruction = 'h1c008000; 
        #5
        instruction = 'h1c400003; 
        #5
        instruction = 'h1e848000; 
        #5
        instruction = 'h1e7ffffd; 
        #5;

        instruction = 'h28022000;
        #5;
        instruction = 'h28040000;
        #5;
        instruction = 'h28068800;
        #5;
        instruction = 'h28088000;
        #5;
        instruction = 'h28087000;
        #5;
        instruction = 'h28087800;
        #5;
        instruction = 'h28202000;
        #5;
        instruction = 'h28237800;
        #5;
        instruction = 'h28632000;
        #5;
        instruction = 'h28274800;
        #5;
        instruction = 'h28674800;
        #5;
        instruction = 'h28254800;
        #5;
        instruction = 'h282a0800;
        #5;

        // LDI
        instruction = 'h2c08ffff;
        #5;
        instruction = 'h2c111234;
        #5;
        instruction = 'h2c14ff00;
        #5;
        instruction = 'h2c1c1234;
        #5;
        instruction = 'h2c199876;
        #5;
        instruction = 'h2c1c5678;
        #5;
       
        instruction = 'h2d8800ff;
        #5;
        instruction = 'h2e1000f1;
        #5;
        instruction = 'h2c9c1234;
        #5;
        instruction = 'h2d18f876;
        #5;
        instruction = 'h2d9c5678;
        #5;
        instruction = 'h2e1c5678;        
        #5
        
        // LD
        instruction = 'h38408000;
        #5;
        instruction = 'h38410004;
        #5;
        instruction = 'h38413ffc;
        #5;
        instruction = 'h38713fff;
        #5;
        instruction = 'h3837c000;
        #5;
        
        // ST
        instruction = 'h3c0f0000;
        #5;
        instruction = 'h3c178004;
        #5;
        instruction = 'h3c12bffc;
        #5;
        instruction = 'h3c6200ff;
        #5;
        instruction = 'h3c7b4000;
        #5;
       
    
    end   
    
    
    
    task write_register(input user_mode, input high_reg, input [3:0] reg_index);
        begin

            if (user_mode) $write("u"); 
            
            if (high_reg)
                case (reg_index)    
                    'h0: $write("cc"); 
                    'h1: $write("pc"); 
                    'h2: $write("cycles"); 
                    'h3: $write("t0"); 
                    'h4: $write("t1"); 
                    'h5: $write("t2"); 
                    default: $write("hr%0d", reg_index); 
               endcase
            else
                case (reg_index)    
                    'h0: $write("zero"); 
                    'hE: $write("lr"); 
                    'hF: $write("sp"); 
                    default: $write("r%0d", reg_index); 
               endcase

       end        
    endtask   
   

    task write_width_code(input [2:0] data_width);
        begin
            case (data_width)    
                'b000: $write("      "); // 32-bit Word
                'b001: $write(".h    "); // 16-bit Half-Word
                'b010: $write(".sh   "); // 16-bit Half-Word (Sign-Extended to fill 32-bits)
                'b011: $write(".b    "); // 8-bit Byte 
                'b100: $write(".sb   "); // 8-bit Byte (Sign-Extended to fill 32-bits)
                'b101: $write(".hh   "); // 2 16-bit Half-Word Vector (Not Supported in 1.0)
                'b110: $write(".bb   "); // 2 8-bit Byte Vector (Not Supported in 1.0)
                'b111: $write(".bbbb "); // 4 8-bit Byte Vector (Not Supported in 1.0)      
           endcase
        end
    endtask   
    
    

    // At each instruction, look at the outputs
    always @(instruction) 
    begin
        #1; // Settle Delay, then read the output bits
        case ('d1)
            inst_illegal:   $display("Illegal Instruction");
            inst_noop:      $display("noop");
            inst_halt:      $display("halt");
            inst_trap:      $display("trap");
            inst_rtu:       $display("rtu");

            inst_branch:
                begin
                    if (instruction[26] == 'd1)
                        $write("brl");
                    else
                        $write("bra");
                        
                    case (branch_cond)    
                        'b000: $write("       ");
                        'b001: $write(".eq/z  ");
                        'b010: $write(".ne/nz ");
                        'b011: $write(".lt    ");
                        'b100: $write(".ge    ");
                        'b101: $write(".c     ");
                        'b110: $write(".nc    ");
                        'b111: $write(".v     ");
                   endcase

                    if (branch_imm_en == 'd1)
                        $write(" %4d", $signed(branch_offset) * 4);
                    else
                        write_register('b0, 'b0, reg_srcA);
                    
                    $write("\n");
                end

            inst_mov:
                begin
                    $write("mov");
                        
                    write_width_code(data_width);
                    $write(" ");
                    
                    write_register(reg_mov_dest[5], reg_mov_dest[4], reg_mov_dest[3:0]);
                    $write(", ");
                    write_register(reg_mov_src[5], reg_mov_src[4], reg_mov_src[3:0]);
                    
                    $write("\n");
                end
                
           inst_ldi:
                begin
                    if (ldi_high_half == 'd1) $write("ldih");
                    else                      $write("ldi");    
                    
                    write_width_code(data_width);
                    if (ldi_high_half == 'd0) $write(" ");

                    write_register('b0, 'b0, reg_dest[3:0]);

                    if (ldi_high_page_fill == 'd1)
                        $display(", $ffff%4h", ldi_imm);
                    else 
                        $display(", $%4h", ldi_imm);
                end      

           inst_load:
                begin
                    if (addr_predec_postinc == 'd1) 
                        begin
                            $write("pop      ");
                            write_register('b0, 'b0, reg_dest[3:0]);
                            $write("\n");                            
                        end    
                    else
                        begin
                            $write("ld");    
                            write_width_code(data_width);
                            $write("  ");
                            
                            write_register('b0, 'b0, reg_dest[3:0]);
                            $write(", (");
                            write_register('b0, 'b0, reg_srcA[3:0]);
                            
                            if      ($signed(imm_val) > 0) $display("+%0d)", imm_val);
                            else if ($signed(imm_val) < 0) $display("%0d)", $signed(imm_val));
                            else                           $display(")");
                        end      
                end
           inst_store:
                begin
                    if (addr_predec_postinc == 'd1) 
                        begin
                            $write("push     ");
                            write_register('b0, 'b0, reg_srcA[3:0]);
                            $write("\n");                            
                        end    
                    else
                        begin

                        $write("st");

                        write_width_code(data_width);
                        $write("  (");
                      
                        write_register('b0, 'b0, reg_dest[3:0]);
                        if      ($signed(imm_val) > 0) $write("+%0d), ", imm_val);
                        else if ($signed(imm_val) < 0) $write("%0d), ", $signed(imm_val));
                        else                           $write("), ");

                        write_register('b0, 'b0, reg_srcA[3:0]);
                        $write("\n");
                    end      
                end
           inst_alu:
                begin
                    $write("ALU");

                    write_width_code(data_width);

                    
                    write_register('b0, 'b0, reg_dest[3:0]);
                    $display(", $%4h", ldi_imm);
                end      
                
        endcase
    end
   
   

endmodule  