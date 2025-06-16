vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/decoder.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/alu.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/ember.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/SysROM.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/system_top.v}
vlog -vlog01compat -work work +incdir+D:/dev/git/OpenFPGA/src/fpga/ember32 {D:/dev/git/OpenFPGA/src/fpga/ember32/system_tb.v}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L cyclonev_hssi_ver -L cyclonev_pcie_hip_ver -L rtl_work -L work -voptargs="+acc" system_tb


radix define CPUState {
4'b0001 "Fetch"   -color #11BB52,
4'b0010 "FWait"   -color #FF8824,
4'b0100 "Execute" -color #11FF44,
4'b1000 "EWait"   -color #FF6644,
-default hex
}


add wave -color #FF22FF sim:/system_tb/system/CPU/sys_clk
add wave -color #AAAAAA sim:/system_tb/system/CPU/sys_rst_n

add wave -divider OpCode

add wave -divider "CPU State"
add wave -color #11FF00 -radix CPUState sim:/system_tb/system/CPU/active_state
add wave sim:/system_tb/system/CPU/mem_address_out
add wave sim:/system_tb/system/CPU/reg_srcA 
add wave sim:/system_tb/system/CPU/reg_srcB 

add wave sim:/system_tb/system/CPU/PC
add wave sim:/system_tb/system/CPU/PC_new
add wave sim:/system_tb/system/CPU/PC_branch
add wave sim:/system_tb/system/CPU/PC_plus4

add wave sim:/system_tb/system/CPU/registers

view structure
view signals
run 500ns
