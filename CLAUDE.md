# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Project Overview

SP701 FPGA-based Time-to-Digital Converter (TDC) designs for high-resolution time interval measurement.

**Target Hardware:** Xilinx Spartan-7 XC7S100 (SP701 Evaluation Board)
**Repository:** https://github.com/stymdwvd/TDC_designs

## Project Structure

```
TDC_designs/
├── Claude_TDC_with_calibration/    # High-accuracy version (~10-20ps)
│   ├── SP701_TDC_DelayLine.v       # Delay line + calibration module
│   ├── SP701_TDC_Single_Core.v     # TDC core with calibration logic
│   ├── SP701_TDC_Single_Top.v      # Top-level with ILA debug
│   ├── SP701_TDC_Single_Testbench.v
│   └── SP701_TDC_Single_Constraints.xdc
│
└── Claude_TDC_no_calibration/      # Simplified version (~50-100ps)
    ├── SP701_TDC_DelayLine.v       # Delay line only
    ├── SP701_TDC_Single_Core.v     # TDC core with linear approximation
    ├── SP701_TDC_Single_Top.v      # Top-level with ILA debug
    ├── SP701_TDC_Single_Testbench.v
    └── SP701_TDC_Single_Constraints.xdc
```

## TDC Architecture

### Key Features
- **Delay Line:** CARRY4-based tapped delay chain (256 taps)
- **Capture:** Rising clock edge sampling
- **Bubble Suppression:** 3-tap majority voting
- **Thermometer Encoder:** Finds 1→0 transition point
- **Fine Resolution:** ~39ps per tap (10ns / 256 taps)

### Measurement Principle
```
Total Time = (coarse_count × clock_period) + (fine_tap × tap_delay)

Where:
- coarse_count = number of clock cycles between START and STOP
- clock_period = 10ns (100 MHz)
- fine_tap = tap position in delay line
- tap_delay = calibrated or linear (~39ps)
```

## Version Comparison

| Feature | With Calibration | No Calibration |
|---------|-----------------|----------------|
| Accuracy | ~10-20 ps | ~50-100 ps |
| External Clock | Required (tdc_ref_clk) | Not needed |
| Startup Time | ~4096 cycles calibration | Immediate |
| Resources | Higher | Lower |
| Complexity | More complex | Simpler |

## Build Instructions

### Vivado Project Setup
1. Create new RTL project targeting XC7S100-FGGA676
2. Add Verilog sources from chosen version folder
3. Add constraints (.xdc) file
4. Run Synthesis → Implementation → Generate Bitstream

### ILA Debug Setup
1. After synthesis, run "Set Up Debug"
2. Signals marked `(* mark_debug = "true" *)` will be detected
3. Create ILA core with appropriate depth (4096+ samples recommended)

## Pin Assignments

Key pins (defined in constraints):
- `sys_clk_p` (R2) - 100 MHz system clock
- `sys_rst_n` (T2) - Active-low reset
- `tdc_start` (M1) - Start signal input
- `tdc_stop` (N1) - Stop signal input
- `tdc_ref_clk` (K2) - Calibration reference clock (with_calibration only)

## Git Workflow

```bash
# Check status
git status

# After making changes
git add .
git commit -m "Description of changes"
git push

# Pull latest changes
git pull
```

## Design Notes

- Always use rising clock edge for measurement
- Default edge mode: Rising edge detection for START/STOP signals
- Calibration requires asynchronous reference clock (e.g., 12 MHz, 27 MHz)
- For simulation, use 1ps timescale for accurate delay modeling
