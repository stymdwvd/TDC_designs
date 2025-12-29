# ================================================================
# SP701 Single-Channel TDC Constraints - NO CALIBRATION VERSION
# Date: 2025-12-28
# Target: Xilinx Spartan-7 XC7S100-FGGA676
#
# This version does NOT require a reference clock since there
# is no calibration system.
# ================================================================

# ----------------------------------------------------------------
# Clock Constraints
# ----------------------------------------------------------------

# System Clock (100 MHz)
create_clock -period 10.000 -name sys_clk [get_ports sys_clk_p]

# ----------------------------------------------------------------
# Clock Input (directly connected - no differential)
# ----------------------------------------------------------------

set_property PACKAGE_PIN R2 [get_ports sys_clk_p]
set_property IOSTANDARD LVCMOS33 [get_ports sys_clk_p]

# ----------------------------------------------------------------
# Reset
# ----------------------------------------------------------------

set_property PACKAGE_PIN T2 [get_ports sys_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports sys_rst_n]

# ----------------------------------------------------------------
# TDC Input Signals
# ----------------------------------------------------------------

# Start Signal (high-speed input)
set_property PACKAGE_PIN M1 [get_ports tdc_start]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_start]
set_property SLEW FAST [get_ports tdc_start]

# Stop Signal (high-speed input)
set_property PACKAGE_PIN N1 [get_ports tdc_stop]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_stop]
set_property SLEW FAST [get_ports tdc_stop]

# ----------------------------------------------------------------
# Control Signals
# ----------------------------------------------------------------

# TDC Enable
set_property PACKAGE_PIN P1 [get_ports tdc_enable]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_enable]

# TDC Arm
set_property PACKAGE_PIN R1 [get_ports tdc_arm]
set_property IOSTANDARD LVCMOS33 [get_ports tdc_arm]

# Edge Mode [1:0]
set_property PACKAGE_PIN L2 [get_ports {edge_mode[0]}]
set_property PACKAGE_PIN M2 [get_ports {edge_mode[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {edge_mode[*]}]

# ----------------------------------------------------------------
# Status LEDs
# ----------------------------------------------------------------

set_property PACKAGE_PIN E13 [get_ports led_tdc_ready]
set_property PACKAGE_PIN C13 [get_ports led_measuring]
set_property PACKAGE_PIN B13 [get_ports led_data_valid]
set_property PACKAGE_PIN A13 [get_ports led_error]
set_property IOSTANDARD LVCMOS33 [get_ports led_*]
set_property SLEW SLOW [get_ports led_*]
set_property DRIVE 8 [get_ports led_*]

# ----------------------------------------------------------------
# Timing Constraints
# ----------------------------------------------------------------

# Input delay for TDC signals (minimize for accuracy)
set_input_delay -clock sys_clk -min 0.0 [get_ports tdc_start]
set_input_delay -clock sys_clk -max 1.0 [get_ports tdc_start]
set_input_delay -clock sys_clk -min 0.0 [get_ports tdc_stop]
set_input_delay -clock sys_clk -max 1.0 [get_ports tdc_stop]

# False paths for asynchronous inputs (handled by synchronizers)
set_false_path -from [get_ports tdc_start] -to [get_pins -hierarchical *_d1_reg/D]
set_false_path -from [get_ports tdc_stop] -to [get_pins -hierarchical *_d1_reg/D]

# False path for reset
set_false_path -from [get_ports sys_rst_n]

# ----------------------------------------------------------------
# Delay Line Constraints
# ----------------------------------------------------------------

# Prevent optimization of delay line
set_property DONT_TOUCH true [get_cells -hierarchical *carry_start*]
set_property DONT_TOUCH true [get_cells -hierarchical *carry_stop*]

# Keep delay line elements together
set_property KEEP_HIERARCHY true [get_cells -hierarchical delay_line_inst]

# ----------------------------------------------------------------
# Physical Constraints for Delay Line Placement
# ----------------------------------------------------------------

# Place delay line CARRY4 elements in a single column
# Adjust SLICE range based on available resources
# set_property LOC SLICE_X50Y0 [get_cells -hierarchical carry_chain[0].carry_start]

# ----------------------------------------------------------------
# ILA Debug Constraints (optional)
# ----------------------------------------------------------------

# To use ILA, run "Set Up Debug" in Vivado after synthesis
# Signals marked with (* mark_debug = "true" *) will be detected

# ----------------------------------------------------------------
# Bitstream Configuration
# ----------------------------------------------------------------

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
