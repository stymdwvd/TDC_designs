##################################################################
# SP701 Single-Channel TDC - ILA Debug Version Constraints
# Date: 2025-12-26
# Description: Simplified constraints with ILA integration
#
# FEATURES:
# - Removed SPI/UART pin assignments
# - Added ILA debug constraints
# - Simplified control interface pins
##################################################################

# Clock Constraints
##################################################################

# System Clock (100 MHz)
create_clock -period 10.000 -name sys_clk [get_ports sys_clk_p]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk_p]

# Reference Clock for Calibration (slightly different frequency)
create_clock -period 10.101 -name ref_clk [get_ports tdc_ref_clk]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_ref_clk]

# Clock Domain Crossing
set_clock_groups -asynchronous -group [get_clocks sys_clk] -group [get_clocks ref_clk]

# Pin Assignments - System Interface
##################################################################

# System Clock - Use appropriate pin for SP701 board
set_property PACKAGE_PIN R2 [get_ports sys_clk_p]

# System Reset (directly exposed button)
set_property PACKAGE_PIN T18 [get_ports sys_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports sys_rst_n]

# TDC Input Signals - High-Speed Inputs
##################################################################

# TDC Start Signal
set_property PACKAGE_PIN R16 [get_ports tdc_start]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_start]
set_property IBUF_LOW_PWR FALSE [get_ports tdc_start]

# TDC Stop Signal
set_property PACKAGE_PIN R17 [get_ports tdc_stop]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_stop]
set_property IBUF_LOW_PWR FALSE [get_ports tdc_stop]

# Reference Clock Input
set_property PACKAGE_PIN P16 [get_ports tdc_ref_clk]

# TDC Input Timing Constraints (asynchronous inputs)
set_input_delay -clock [get_clocks sys_clk] -min 0.000 [get_ports tdc_start]
set_input_delay -clock [get_clocks sys_clk] -max 2.000 [get_ports tdc_start]
set_input_delay -clock [get_clocks sys_clk] -min 0.000 [get_ports tdc_stop]
set_input_delay -clock [get_clocks sys_clk] -max 2.000 [get_ports tdc_stop]

# Control Interface - Direct Buttons/Switches
##################################################################

# TDC Enable (can use DIP switch or button)
set_property PACKAGE_PIN T20 [get_ports tdc_enable]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_enable]

# TDC Arm (button to arm for next measurement)
set_property PACKAGE_PIN T21 [get_ports tdc_arm]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_arm]

# Calibration Trigger (button to start calibration)
set_property PACKAGE_PIN U20 [get_ports calibration_trigger]
set_property IOSTANDARD LVCMOS33 [get_ports calibration_trigger]

# Edge Mode Selection (2-bit DIP switch)
set_property PACKAGE_PIN U21 [get_ports {edge_mode[0]}]
set_property PACKAGE_PIN V20 [get_ports {edge_mode[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {edge_mode[*]}]

# Control signals are slow - false path
set_false_path -from [get_ports sys_rst_n]
set_false_path -from [get_ports tdc_enable]
set_false_path -from [get_ports tdc_arm]
set_false_path -from [get_ports calibration_trigger]
set_false_path -from [get_ports {edge_mode[*]}]

# Status LEDs
##################################################################

set_property PACKAGE_PIN M14 [get_ports led_tdc_ready]
set_property PACKAGE_PIN M15 [get_ports led_measuring]
set_property PACKAGE_PIN L14 [get_ports led_data_valid]
set_property PACKAGE_PIN L15 [get_ports led_error]
set_property IOSTANDARD LVCMOS33 [get_ports led_*]
set_false_path -to [get_ports led_*]

# Metastability Protection Constraints
##################################################################

# Mark all synchronizer registers as ASYNC_REG
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*reset_sync_reg*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*start_edge_d1*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*start_edge_d2*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*stop_edge_d1*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*stop_edge_d2*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*captured_start_async*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*captured_stop_async*"}]
set_property ASYNC_REG TRUE [get_cells -hier -filter {name =~ "*ref_clk_sync*"}]

# Delay Line Timing Constraints
##################################################################

# CARRY4 delay line propagation constraints
set_max_delay -datapath_only -from [get_pins -hier -filter {name =~ "*delay_line_inst*carry_start*/CI"}] -to [get_pins -hier -filter {name =~ "*delay_line_inst*carry_start*/CO*"}] 0.050
set_max_delay -datapath_only -from [get_pins -hier -filter {name =~ "*delay_line_inst*carry_stop*/CI"}] -to [get_pins -hier -filter {name =~ "*delay_line_inst*carry_stop*/CO*"}] 0.050

# Capture register timing
set_max_delay -datapath_only -from [get_pins -hier -filter {name =~ "*delay_chain_start*"}] -to [get_pins -hier -filter {name =~ "*captured_start_async*"}] 0.500
set_max_delay -datapath_only -from [get_pins -hier -filter {name =~ "*delay_chain_stop*"}] -to [get_pins -hier -filter {name =~ "*captured_stop_async*"}] 0.500

# Calibration data false paths (multi-cycle by design)
set_false_path -from [get_cells -hier -filter {name =~ "*calibration_inst/histogram*"}]
set_false_path -from [get_cells -hier -filter {name =~ "*calibration_inst/bin_width_lut*"}]
set_false_path -from [get_cells -hier -filter {name =~ "*calibration_inst/tap_delay_lut*"}]
set_false_path -to [get_cells -hier -filter {name =~ "*local_tap_delay*"}]

# Physical Constraints for Delay Line
##################################################################

# CARRY4 delay line placement - vertical column
create_pblock pblock_delay_line_start
add_cells_to_pblock [get_pblocks pblock_delay_line_start] [get_cells -hier -filter {name =~ "*delay_line_inst*carry_start*"}]
resize_pblock [get_pblocks pblock_delay_line_start] -add {SLICE_X0Y25:SLICE_X0Y89}

create_pblock pblock_delay_line_stop
add_cells_to_pblock [get_pblocks pblock_delay_line_stop] [get_cells -hier -filter {name =~ "*delay_line_inst*carry_stop*"}]
resize_pblock [get_pblocks pblock_delay_line_stop] -add {SLICE_X1Y25:SLICE_X1Y89}

# Preserve delay line optimization
set_property KEEP_HIERARCHY TRUE [get_cells -hier -filter {name =~ "*delay_line_inst*"}]
set_property DONT_TOUCH TRUE [get_cells -hier -filter {name =~ "*delay_line_inst*carry*"}]

# High fanout constraints for delay chain
set_property MAX_FANOUT 1 [get_nets -hier -filter {name =~ "*delay_chain_start*"}]
set_property MAX_FANOUT 1 [get_nets -hier -filter {name =~ "*delay_chain_stop*"}]
set_property MAX_FANOUT 1 [get_nets -hier -filter {name =~ "*carry_chain_start*"}]
set_property MAX_FANOUT 1 [get_nets -hier -filter {name =~ "*carry_chain_stop*"}]

# ILA Debug Constraints
##################################################################

# Mark debug signals for ILA insertion
# These are automatically handled by mark_debug attributes in RTL
# But we can add explicit constraints if needed

# Create debug hub clock constraint (for ILA)
# This will be automatically created when ILA is inserted

# Debug hub placement (near BRAM for efficient routing)
# set_property C_CLK_INPUT_FREQ_HZ 100000000 [get_debug_cores dbg_hub]
# set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]

# Implementation Strategy
##################################################################

# Performance-focused strategy
set_property STRATEGY Performance_ExplorePostRoutePhysOpt [get_runs impl_1]

# Enable physical optimization
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

# Configuration Settings
##################################################################

set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]

# Fast configuration for debug iterations
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# Post-Route Reports
##################################################################

set_property STEPS.ROUTE_DESIGN.TCL.POST {
    # Timing reports
    report_timing_summary -delay_type max -report_unconstrained -check_timing_verbose -max_paths 10 -input_pins -file timing_summary.rpt

    # Resource utilization
    report_utilization -file utilization.rpt

    # Debug core report
    report_debug_core -file debug_cores.rpt

    # Delay line timing
    report_timing -from [get_pins -hier -filter {name =~ "*delay_line_inst*"}] -to [get_pins -hier -filter {name =~ "*captured*"}] -max_paths 20 -file delay_line_timing.rpt
} [get_runs impl_1]

##################################################################
# ILA SETUP INSTRUCTIONS
##################################################################
# After synthesis, use these TCL commands to set up ILA:
#
# 1. Open synthesized design:
#    open_run synth_1
#
# 2. Set up debug:
#    Tools -> Set Up Debug -> Next
#    Select nets with mark_debug attribute
#    Configure sample depth (e.g., 4096 samples)
#    Choose trigger settings
#
# 3. Or create ILA manually:
#    create_debug_core u_ila_0 ila
#    set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_0]
#    set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
#    set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
#    set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
#
# 4. Connect probes:
#    connect_debug_port u_ila_0/probe0 [get_nets {time_interval[*]}]
#    connect_debug_port u_ila_0/probe1 [get_nets {measurement_ready}]
#    ... etc.
#
# 5. Implement and generate bitstream
##################################################################
