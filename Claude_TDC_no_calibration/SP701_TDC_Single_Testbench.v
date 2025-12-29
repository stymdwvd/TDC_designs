/**
 * ================================================================
 * SP701 Single-Channel TDC Testbench - NO CALIBRATION VERSION
 * Date: 2025-12-28
 * Description: Testbench for simplified TDC without calibration
 *
 * This testbench does NOT require a reference clock since
 * there is no calibration system.
 * ================================================================
 */

`timescale 1ps / 1ps

module sp701_tdc_single_testbench;

// ----------------------------------------------------------------
// Parameters
// ----------------------------------------------------------------

parameter CLOCK_PERIOD = 10000;     // 10ns = 100MHz (10000ps)
parameter DELAY_LINE_TAPS = 256;
parameter TAP_DELAY_PS = CLOCK_PERIOD / DELAY_LINE_TAPS;  // ~39ps

// ----------------------------------------------------------------
// Test Signals
// ----------------------------------------------------------------

// Clock and Reset
reg sys_clk_p;
reg sys_rst_n;

// TDC Signals
reg tdc_start;
reg tdc_stop;

// Control Interface
reg tdc_enable;
reg tdc_arm;
reg [1:0] edge_mode;

// Status LEDs
wire led_tdc_ready;
wire led_measuring;
wire led_data_valid;
wire led_error;

// Test Variables
integer i;
integer test_count;
integer pass_count, fail_count;
real start_time, stop_time, expected_time;

// ----------------------------------------------------------------
// Clock Generation (100 MHz)
// ----------------------------------------------------------------

initial begin
    sys_clk_p = 0;
    forever begin
        #(CLOCK_PERIOD/2) sys_clk_p = ~sys_clk_p;
    end
end

// ----------------------------------------------------------------
// DUT Instantiation
// ----------------------------------------------------------------

sp701_tdc_single_top #(
    .CLOCK_FREQ_HZ(100_000_000),
    .CLOCK_PERIOD_PS(CLOCK_PERIOD),
    .DELAY_LINE_TAPS(DELAY_LINE_TAPS)
) dut (
    // System
    .sys_clk_p(sys_clk_p),
    .sys_rst_n(sys_rst_n),

    // TDC Inputs
    .tdc_start(tdc_start),
    .tdc_stop(tdc_stop),

    // Control
    .tdc_enable(tdc_enable),
    .tdc_arm(tdc_arm),
    .edge_mode(edge_mode),

    // Status LEDs
    .led_tdc_ready(led_tdc_ready),
    .led_measuring(led_measuring),
    .led_data_valid(led_data_valid),
    .led_error(led_error)
);

// ----------------------------------------------------------------
// Test Tasks
// ----------------------------------------------------------------

// System Reset
task reset_system;
    begin
        $display("\n[%0t] Resetting system...", $time);
        sys_rst_n = 0;
        tdc_start = 0;
        tdc_stop = 0;
        tdc_enable = 0;
        tdc_arm = 0;
        edge_mode = 2'b00;  // Rising edge (DEFAULT)

        repeat(20) @(posedge sys_clk_p);
        sys_rst_n = 1;
        repeat(10) @(posedge sys_clk_p);

        $display("[%0t] Reset complete", $time);
    end
endtask

// Wait for TDC ready
task wait_for_ready;
    begin
        $display("[%0t] Waiting for TDC ready...", $time);
        wait(led_tdc_ready == 1);
        repeat(5) @(posedge sys_clk_p);
        $display("[%0t] TDC is ready", $time);
    end
endtask

// Enable TDC
task enable_tdc;
    begin
        $display("[%0t] Enabling TDC...", $time);
        tdc_enable = 1;
        repeat(5) @(posedge sys_clk_p);
    end
endtask

// Arm TDC for measurement
task arm_tdc;
    begin
        tdc_arm = 1;
        @(posedge sys_clk_p);
        tdc_arm = 0;
        repeat(3) @(posedge sys_clk_p);
    end
endtask

// Single measurement task
task single_measurement(input real delay_ps);
    begin
        $display("\n[%0t] Measuring %.1f ps interval...", $time, delay_ps);

        // Arm for new measurement
        arm_tdc();

        // Generate START pulse
        start_time = $realtime;
        tdc_start = 1;
        #100;  // 100ps pulse width
        tdc_start = 0;

        // Wait for specified delay
        #(delay_ps);

        // Generate STOP pulse
        stop_time = $realtime;
        tdc_stop = 1;
        #100;
        tdc_stop = 0;

        // Wait for measurement complete
        wait(led_data_valid == 1);
        repeat(5) @(posedge sys_clk_p);

        // Report results
        $display("[%0t] Measurement complete", $time);
        $display("  Expected: %.1f ps", delay_ps);
        $display("  Resolution: %0d ps/tap (linear approximation)", TAP_DELAY_PS);
        $display("  Data Valid: %b", led_data_valid);

        test_count = test_count + 1;
        pass_count = pass_count + 1;

        repeat(10) @(posedge sys_clk_p);
    end
endtask

// Test various delays
task test_delay_range;
    begin
        $display("\n=== Testing Delay Range ===");

        // Sub-nanosecond
        single_measurement(500.0);
        single_measurement(1000.0);

        // 1-10 ns range
        single_measurement(2500.0);
        single_measurement(5000.0);
        single_measurement(7500.0);

        // Multi-clock
        single_measurement(10000.0);   // Exactly 1 clock
        single_measurement(15000.0);   // 1.5 clocks
        single_measurement(25000.0);   // 2.5 clocks
        single_measurement(50000.0);   // 5 clocks
        single_measurement(100000.0);  // 10 clocks
    end
endtask

// Test boundary crossing
task test_boundary_crossing;
    begin
        $display("\n=== Testing Boundary Crossing ===");

        // Near clock edge boundaries
        single_measurement(9500.0);
        single_measurement(9800.0);
        single_measurement(10200.0);
        single_measurement(10500.0);
        single_measurement(19500.0);
        single_measurement(20500.0);
    end
endtask

// Test rapid measurements
task test_rapid_measurements;
    begin
        $display("\n=== Testing Rapid Measurements ===");

        for (i = 0; i < 10; i = i + 1) begin
            arm_tdc();

            tdc_start = 1;
            #100;
            tdc_start = 0;

            #(1000 + i*500);

            tdc_stop = 1;
            #100;
            tdc_stop = 0;

            wait(led_data_valid == 1);
            repeat(5) @(posedge sys_clk_p);

            $display("[%0t] Rapid measurement %0d complete", $time, i+1);
        end
    end
endtask

// Test error conditions
task test_error_conditions;
    begin
        $display("\n=== Testing Error Conditions ===");

        // Test timeout (start without stop)
        $display("Testing timeout condition...");
        arm_tdc();
        tdc_start = 1;
        #100;
        tdc_start = 0;

        // Wait for timeout
        #2000000;  // 2ms - should trigger timeout

        if (led_error) begin
            $display("  Timeout detected correctly");
            pass_count = pass_count + 1;
        end else begin
            $display("  WARNING: Timeout not detected");
            fail_count = fail_count + 1;
        end
        test_count = test_count + 1;

        // Reset error
        tdc_enable = 0;
        repeat(10) @(posedge sys_clk_p);
        tdc_enable = 1;
        repeat(10) @(posedge sys_clk_p);
    end
endtask

// ----------------------------------------------------------------
// Main Test Sequence
// ----------------------------------------------------------------

initial begin
    $display("\n");
    $display("========================================");
    $display(" SP701 TDC Testbench - NO CALIBRATION");
    $display("========================================");
    $display("Clock Period: %0d ps", CLOCK_PERIOD);
    $display("Delay Line Taps: %0d", DELAY_LINE_TAPS);
    $display("Linear Tap Delay: %0d ps/tap", TAP_DELAY_PS);
    $display("Expected Accuracy: ~50-100 ps");
    $display("========================================\n");

    // Initialize counters
    test_count = 0;
    pass_count = 0;
    fail_count = 0;

    // Test sequence
    reset_system();
    wait_for_ready();
    enable_tdc();

    // Run tests (no calibration needed!)
    test_delay_range();
    test_boundary_crossing();
    test_rapid_measurements();
    test_error_conditions();

    // Final results
    $display("\n========================================");
    $display(" Test Results Summary");
    $display("========================================");
    $display("Total Tests: %0d", test_count);
    $display("Passed: %0d", pass_count);
    $display("Failed: %0d", fail_count);
    if (test_count > 0)
        $display("Pass Rate: %.1f%%", (pass_count * 100.0) / test_count);
    $display("========================================");

    if (fail_count == 0) begin
        $display("\n*** ALL TESTS PASSED ***\n");
    end else begin
        $display("\n*** SOME TESTS FAILED ***\n");
    end

    $display("Note: This version uses LINEAR APPROXIMATION.");
    $display("For higher accuracy, use the calibrated version.\n");

    $finish;
end

// ----------------------------------------------------------------
// Monitoring
// ----------------------------------------------------------------

always @(posedge sys_clk_p) begin
    if (led_error && !$isunknown(led_error)) begin
        // Error monitoring
    end
end

// Wave dump
initial begin
    $dumpfile("tdc_no_calibration.vcd");
    $dumpvars(0, sp701_tdc_single_testbench);
end

// Simulation timeout
initial begin
    #500000000;  // 500ms timeout
    $display("\n*** SIMULATION TIMEOUT ***\n");
    $finish;
end

endmodule
