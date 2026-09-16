transcript on

# Compile exactly the three RTL/testbench files in this project.  Do not use
# Quartus's synthesis top-level (dac60501_uart_top) as the simulation top;
# the testbench is tb_dac60501.
if {[file exists work]} {
    vdel -lib work -all
}
vlib work
vmap work work
vlog dac60501_uart_top.v dac60501_model.v tb_dac60501.v

# Override TARGET_MV on the command line when needed, for example:
# vsim -voptargs=+acc -gTARGET_MV=231 work.tb_dac60501
vsim -voptargs=+acc work.tb_dac60501

add wave -r sim:/tb_dac60501/*
run -alltranscript on

# Compile exactly the three RTL/testbench files in this project.  Do not use
# Quartus's synthesis top-level (dac60501_uart_top) as the simulation top;
# the testbench is tb_dac60501.
if {[file exists work]} {
    vdel -lib work -all
}
vlib work
vmap work work
vlog dac60501_uart_top.v dac60501_model.v tb_dac60501.v

# Override TARGET_MV on the command line when needed, for example:
# vsim -voptargs=+acc -gTARGET_MV=231 work.tb_dac60501
vsim -voptargs=+acc work.tb_dac60501

add wave -r sim:/tb_dac60501/*
run -all