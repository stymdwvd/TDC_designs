# ================================================================
# Vivado Synthesis Script for SP701 TDC (No Calibration)
# Target: Xilinx Spartan-7 XC7S100-FGGA676
# ================================================================

# Set project parameters
set project_name "SP701_TDC_NoCalibration"
set project_dir [file dirname [info script]]
set output_dir "${project_dir}/vivado_project"

# Create output directory if it doesn't exist
file mkdir $output_dir

# Remove existing project if present
if {[file exists "${output_dir}/${project_name}.xpr"]} {
    file delete -force "${output_dir}/${project_name}.xpr"
    file delete -force "${output_dir}/${project_name}.cache"
    file delete -force "${output_dir}/${project_name}.runs"
    file delete -force "${output_dir}/${project_name}.hw"
    file delete -force "${output_dir}/${project_name}.ip_user_files"
    file delete -force "${output_dir}/${project_name}.srcs"
}

# Create project
create_project -force $project_name $output_dir -part xc7s100fgga676-1

# Set project properties
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

# Add RTL source files
add_files -norecurse [list \
    "${project_dir}/SP701_TDC_DelayLine.v" \
    "${project_dir}/SP701_TDC_Single_Core.v" \
    "${project_dir}/SP701_TDC_Single_Top.v" \
]

# Add constraints file
add_files -fileset constrs_1 -norecurse "${project_dir}/SP701_TDC_Single_Constraints.xdc"

# Set top module
set_property top sp701_tdc_single_top [current_fileset]

# Update compile order
update_compile_order -fileset sources_1

# Display project info
puts "================================================================"
puts "Project: $project_name"
puts "Part: xc7s100fgga676-1"
puts "Top Module: sp701_tdc_single_top"
puts "================================================================"

# Run Synthesis
puts "\nStarting Synthesis..."
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

# Check synthesis status
set synth_status [get_property STATUS [get_runs synth_1]]
set synth_progress [get_property PROGRESS [get_runs synth_1]]

puts "\n================================================================"
puts "Synthesis Status: $synth_status"
puts "Synthesis Progress: $synth_progress"
puts "================================================================"

# Open synthesized design and generate reports
if {$synth_status == "synth_design Complete!"} {
    open_run synth_1

    # Generate utilization report
    report_utilization -file "${output_dir}/utilization_synth.rpt"
    puts "\nUtilization report saved to: ${output_dir}/utilization_synth.rpt"

    # Generate timing summary
    report_timing_summary -file "${output_dir}/timing_summary_synth.rpt"
    puts "Timing report saved to: ${output_dir}/timing_summary_synth.rpt"

    # Print quick utilization summary
    puts "\n================================================================"
    puts "RESOURCE UTILIZATION SUMMARY"
    puts "================================================================"
    report_utilization -hierarchical -hierarchical_depth 2

    puts "\n================================================================"
    puts "SYNTHESIS COMPLETED SUCCESSFULLY"
    puts "================================================================"

    close_design

    # Run Implementation (Place & Route)
    puts "\nStarting Implementation (Place & Route)..."
    launch_runs impl_1 -jobs 4
    wait_on_run impl_1

    set impl_status [get_property STATUS [get_runs impl_1]]
    set impl_progress [get_property PROGRESS [get_runs impl_1]]

    puts "\n================================================================"
    puts "Implementation Status: $impl_status"
    puts "Implementation Progress: $impl_progress"
    puts "================================================================"

    if {$impl_status == "route_design Complete!"} {
        open_run impl_1

        # Generate post-implementation reports
        report_utilization -file "${output_dir}/utilization_impl.rpt"
        report_timing_summary -file "${output_dir}/timing_summary_impl.rpt"
        report_timing -nworst 10 -file "${output_dir}/timing_detail_impl.rpt"

        puts "\n================================================================"
        puts "POST-IMPLEMENTATION TIMING SUMMARY"
        puts "================================================================"
        report_timing_summary -no_header -no_detailed_paths

        puts "\n================================================================"
        puts "IMPLEMENTATION COMPLETED SUCCESSFULLY"
        puts "================================================================"
    } else {
        puts "\n================================================================"
        puts "IMPLEMENTATION FAILED"
        puts "Check logs at: ${output_dir}/${project_name}.runs/impl_1/"
        puts "================================================================"
    }
} else {
    puts "\n================================================================"
    puts "SYNTHESIS FAILED"
    puts "Check logs at: ${output_dir}/${project_name}.runs/synth_1/"
    puts "================================================================"
}

# Close project
close_project

puts "\nDone."
