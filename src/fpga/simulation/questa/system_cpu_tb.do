vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/decoder.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/alu.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/ember.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/SysROM.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/system_top.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/system_tb.v}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L cyclonev_hssi_ver -L cyclonev_pcie_hip_ver -L rtl_work -L work -voptargs="+acc" system_tb

radix define Register {
'd0 "zero" -color #228799,
'd1 "r1", 'd2 "r2", 'd3 "r3", 'd4 "r4", 'd5 "r5", 'd6 "r6",'d7 "r7", 'd8 "r8", 'd9 "r9", 'd10 "r10", 'd11 "r11", 'd12 "r12", 'd13 "r13",
'd14 "lr"  -color #3277b8, 'd15 "sp"  -color #3277b8,
'd16 "cc"  -color #4277c9, 'd17 "pc"  -color #4277c9,
-default unsigned
}

radix define CPUState {
4'b0001 "Fetch"   -color #FFFFFF,
4'b0010 "FWait"   -color #FF8824,
4'b0100 "Execute" -color #11BB52,
4'b1000 "EWait"   -color #FF6644,
-default hex
}

radix define Width {
5'b00001 ".w"  -color #65BB75,
5'b00010 ".h"  -color #76a854,
5'b00100 ".sh" -color #87a875,
5'b01000 ".b"  -color #987774,
5'b10000 ".sb" -color #a977a8,
-default hex
}

radix define OpCode {
6'h00 "nop"  -color #FF7618,
6'h01 "halt" -color #FF7618,
6'h02 "trap" -color #FF7618,
6'h03 "rtu"  -color #FF7618,
6'h06 "bra"  -color #FFB918,
6'h07 "brl"  -color #FFB918,
6'h0A "mov"  -color #FFDD18,
6'h0B "ldi"  -color #FF9918,
6'h0E "ld"   -color #FF5518,
6'h0F "st"   -color #FF5518,
6'h10 "add"  -color #FF9618,
6'h11 "sub"  -color #FF9618,
6'h12 "mul"  -color #FF9618,
6'h13 "mhs"  -color #FF9618,
6'h14 "mhu"  -color #FF9618,
6'h15 "div"  -color #FF9618,
6'h16 "divu" -color #FF9618,
6'h19 "and"  -color #FF9618,
6'h1a "or"   -color #FF9618,
6'h1b "xor"  -color #FF9618,
6'h1c "lsr"  -color #FF9618,
6'h1d "lsl"  -color #FF9618,
6'h1e "asr"  -color #FF9618,
-default hex
}


add wave -color #FF22FF sim:/system_tb/system/CPU/sys_clk
add wave -color #AAAAAA sim:/system_tb/system/CPU/sys_rst_n

add wave -divider "OpCode Decoder"
add wave sim:/system_tb/system/CPU/instruction_decoder/instruction
add wave -radix OpCode sim:/system_tb/system/CPU/inst_opcode
add wave -radix Width sim:/system_tb/system/CPU/instruction_decoder/data_width_oh

add wave -group "Instruction Type"  \
        sim:/system_tb/system/CPU/instruction_decoder/inst_noop \
        sim:/system_tb/system/CPU/instruction_decoder/inst_halt \
        sim:/system_tb/system/CPU/instruction_decoder/inst_trap \
        sim:/system_tb/system/CPU/instruction_decoder/inst_rtu \
        sim:/system_tb/system/CPU/instruction_decoder/inst_branch \
        sim:/system_tb/system/CPU/instruction_decoder/inst_mov \
        sim:/system_tb/system/CPU/instruction_decoder/inst_ldi \
        sim:/system_tb/system/CPU/instruction_decoder/inst_load \
        sim:/system_tb/system/CPU/instruction_decoder/inst_store \
        sim:/system_tb/system/CPU/instruction_decoder/inst_alu 

add wave -radix Register sim:/system_tb/system/CPU/instruction_decoder/reg_srcA
add wave -radix Register sim:/system_tb/system/CPU/instruction_decoder/reg_srcB
add wave -radix Register sim:/system_tb/system/CPU/instruction_decoder/reg_dest
add wave -color #4444FF sim:/system_tb/system/CPU/instruction_decoder/imm_value
add wave -color #3333FF sim:/system_tb/system/CPU/instruction_decoder/imm_val_en
add wave sim:/system_tb/system/CPU/instruction_decoder/ldi_high_half
add wave sim:/system_tb/system/CPU/instruction_decoder/inst_store

add wave -divider "CPU State"
add wave -radix CPUState sim:/system_tb/system/CPU/active_state
add wave -color #66FF00 -radix binary sim:/system_tb/system/CPU/system_mode
add wave -color #66FF00 -radix binary sim:/system_tb/system/CPU/system_mode_new
add wave -color #99FF00 -radix binary sim:/system_tb/system/CPU/user_cc_new
add wave -color #99FF00 -radix binary sim:/system_tb/system/CPU/super_cc_new
add wave sim:/system_tb/system/CPU/mem_address_out
add wave sim:/system_tb/system/CPU/srcA_val 
add wave sim:/system_tb/system/CPU/srcB_val 

add wave sim:/system_tb/system/CPU/PC_new
add wave sim:/system_tb/system/CPU/PC_branch
add wave sim:/system_tb/system/CPU/PC_plus4

add wave -group "Supervisor Registers"  \
        -label lr sim:/system_tb/system/CPU/registers[0][14] -label sp sim:/system_tb/system/CPU/registers[0][15] \
        -label "cc(TXGD VNCZ)" -radix binary sim:/system_tb/system/CPU/registers[0][16][7:0] -label pc -radix hexadecimal sim:/system_tb/system/CPU/registers[0][17] \
        -label r sim:/system_tb/system/CPU/registers[0]

add wave -group "User Registers"  \
        -label lr sim:/system_tb/system/CPU/registers[1][14] -label sp sim:/system_tb/system/CPU/registers[1][15] \
        -label "cc(TXGD VNCZ)" -radix binary sim:/system_tb/system/CPU/registers[1][16][7:0] -label pc -radix hex sim:/system_tb/system/CPU/registers[1][17] \
        -label r sim:/system_tb/system/CPU/registers[1]       

add wave -divider "ALU"
add wave sim:/system_tb/system/CPU/alu/sum_ 
add wave sim:/system_tb/system/CPU/alu/sum_c 
add wave sim:/system_tb/system/CPU/alu/dif_ 
add wave sim:/system_tb/system/CPU/alu/dif_c 

add wave sim:/system_tb/system/CPU/alu/signedA
add wave sim:/system_tb/system/CPU/alu/signedB 
add wave sim:/system_tb/system/CPU/alu/multiply 

add wave -color #DFDD28 sim:/system_tb/system/CPU/alu/srcA_val_w 
add wave -color #DFDD28 sim:/system_tb/system/CPU/alu/srcB_val_w 
add wave -color #F0D0B0 sim:/system_tb/system/CPU/alu/alu_result 
add wave -color #FFDDB8 sim:/system_tb/system/CPU/alu/result 

add wave -color #6666FF sim:/system_tb/system/CPU/alu/zero
add wave -color #6666FF sim:/system_tb/system/CPU/alu/carry 
add wave -color #6666FF sim:/system_tb/system/CPU/alu/negative
add wave -color #6666FF sim:/system_tb/system/CPU/alu/carry 

view structure
view signals
run 2000ns
wave zoom range 0 800ns
