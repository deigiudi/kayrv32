create_project vivado ../vivado -part xc7a50ticsg324-1L
set_property target_language Verilog [current_project]
add_files -norecurse /mnt/hgfs/RISC-V/1_Cores/Verilog_BenchRV32I/rtl/DecodedOP.vh
add_files -norecurse {../rtl/TOP_UART.vhdl ../rtl/pipMA_RV32.v ../rtl/TOP_CoreRV32.v ../rtl/pipID_RV32.v ../rtl/pipEX_RV32.v ../rtl/TOP_RV32.v ../rtl/UART_RX.vhdl ../rtl/pipIF_RV32.v ../rtl/CacheCTRL.v ../rtl/CacheD_RV32.v ../rtl/UART_TX.vhdl ../rtl/CacheI_RV32.v}

create_ip -name clk_wiz -vendor xilinx.com -library ip -version 6.0 -module_name clk_wiz_0
set_property -dict [list CONFIG.USE_MIN_POWER {true} CONFIG.USE_LOCKED {false} CONFIG.JITTER_SEL {No_Jitter} CONFIG.MMCM_CLKFBOUT_MULT_F {6.125} CONFIG.MMCM_CLKOUT0_DIVIDE_F {6.125} CONFIG.MMCM_CLKOUT0_DUTY_CYCLE {0.5} CONFIG.CLKOUT1_JITTER {149.849} CONFIG.CLKOUT1_PHASE_ERROR {130.058}] [get_ips clk_wiz_0]
generate_target {instantiation_template} [get_files /mnt/hgfs/RISC-V/1_Cores/Verilog_BenchRV32I/vivado2/vivado2.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]
update_compile_order -fileset sources_1

generate_target all [get_files  ../vivado/vivado.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]
catch { config_ip_cache -export [get_ips -all clk_wiz_0] }
export_ip_user_files -of_objects [get_files ../vivado/vivado.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci] -no_script -sync -force -quiet
create_ip_run [get_files -of_objects [get_fileset sources_1] ../vivado/vivado.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]
launch_runs  clk_wiz_0_synth_1
export_simulation -of_objects [get_files ../vivado/vivado.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci] -directory ../vivado/vivado.ip_user_files/sim_scripts -ip_user_files_dir ../vivado/vivado.ip_user_files -ipstatic_source_dir ../vivado/vivado.ip_user_files/ipstatic -lib_map_path [list {modelsim=../vivado/vivado.cache/compile_simlib/modelsim} {questa=../vivado/vivado.cache/compile_simlib/questa} {ies=../vivado/vivado.cache/compile_simlib/ies} {xcelium=../vivado/vivado.cache/compile_simlib/xcelium} {vcs=../vivado/vivado.cache/compile_simlib/vcs} {riviera=../vivado/vivado.cache/compile_simlib/riviera}] -use_ip_compiled_libs -force -quiet

set_property SOURCE_SET sources_1 [get_filesets sim_1]
add_files -fileset sim_1 -norecurse ../testbench/UART_TB.vhdl
set_property top TB_UART [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1

start_gui

synth_design -rtl -name rtl_1