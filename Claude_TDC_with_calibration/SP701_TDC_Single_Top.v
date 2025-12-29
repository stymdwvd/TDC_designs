/**
 * ================================================================
 * SP701 Single-Channel TDC Top Level - ILA Debug Version
 * Date: 2025-12-28 (Rising Edge Revision)
 * Description: Simplified TDC top-level with ILA for result capture
 *
 * MEASUREMENT PRINCIPLE:
 * - Delay line state captured on EVERY rising clock edge
 * - Coarse counter increments on rising clock edge
 * - Fine time from tap position at clock edge
 * - Default: Rising edge detection for START/STOP signals
 *
 * FEATURES:
 * - Removed SPI and UART interfaces for simplicity
 * - Integrated Logic Analyzer (ILA) for result capture
 * - Virtual I/O (VIO) for runtime control
 * - GPIO-based manual control option
 * - All TDC results available through Vivado Hardware Manager
 * ================================================================
 */

module sp701_tdc_single_top #(
    parameter CLOCK_FREQ_HZ = 100_000_000,
    parameter CLOCK_PERIOD_PS = 10000,          // 10ns = 10000ps
    parameter DELAY_LINE_TAPS = 256,
    parameter CALIBRATION_SAMPLES = 4096
) (
    // System Interface
    input  wire sys_clk_p,              // System clock input
    input  wire sys_rst_n,              // System reset (active low)

    // TDC Input Signals (directly from external source)
    input  wire tdc_start,              // Start signal
    input  wire tdc_stop,               // Stop signal
    input  wire tdc_ref_clk,            // Reference clock for calibration

    // Simple Control Interface (directly exposed)
    input  wire tdc_enable,             // Enable TDC operation
    input  wire tdc_arm,                // Arm for next measurement
    input  wire calibration_trigger,    // Trigger calibration
    input  wire [1:0] edge_mode,        // 00=rising, 01=falling, 10=both

    // Status LEDs
    output wire led_tdc_ready,          // TDC ready indicator
    output wire led_measuring,          // Measurement active indicator
    output wire led_data_valid,         // Valid data available
    output wire led_error               // Error indicator
);

// ----------------------------------------------------------------
// Internal Signals
// ----------------------------------------------------------------

// Clock and Reset
wire sys_clk;
wire rst_sync_n;

// TDC Control Signals
wire [2:0] edge_select;
wire continuous_mode;
wire calibration_enable;
wire tdc_reset;

// TDC Measurement Results (directly captured by ILA)
(* mark_debug = "true" *) wire [31:0] time_interval;
(* mark_debug = "true" *) wire [15:0] time_interval_ps;
(* mark_debug = "true" *) wire measurement_valid;
(* mark_debug = "true" *) wire measurement_ready;
(* mark_debug = "true" *) wire tdc_busy;

// Status Flags
(* mark_debug = "true" *) wire timeout_error;
(* mark_debug = "true" *) wire overflow_error;
(* mark_debug = "true" *) wire calibration_done;

// Debug Signals from Core
(* mark_debug = "true" *) wire [7:0] delay_line_code;
(* mark_debug = "true" *) wire [15:0] coarse_count;
(* mark_debug = "true" *) wire [7:0] fine_count;

// Input Signal Monitoring
(* mark_debug = "true" *) wire tdc_start_mon;
(* mark_debug = "true" *) wire tdc_stop_mon;

// Measurement Counter (counts valid measurements)
(* mark_debug = "true" *) reg [31:0] measurement_count;

// Result Storage Registers (hold last valid measurement)
(* mark_debug = "true" *) reg [31:0] last_time_interval;
(* mark_debug = "true" *) reg [15:0] last_fine_time;
(* mark_debug = "true" *) reg [7:0]  last_tap_code;

// ----------------------------------------------------------------
// Clock and Reset Management
// ----------------------------------------------------------------

// Clock input buffer
IBUF sys_clk_ibuf (
    .I(sys_clk_p),
    .O(sys_clk_ibuf_out)
);

wire sys_clk_ibuf_out;

// Global clock buffer
BUFG sys_clk_bufg (
    .I(sys_clk_ibuf_out),
    .O(sys_clk)
);

// Reset synchronization (4-stage for robust metastability protection)
(* ASYNC_REG = "TRUE" *) reg [3:0] reset_sync_reg;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        reset_sync_reg <= 4'b0000;
    end else begin
        reset_sync_reg <= {reset_sync_reg[2:0], 1'b1};
    end
end

assign rst_sync_n = reset_sync_reg[3];

// ----------------------------------------------------------------
// Control Signal Mapping
// ----------------------------------------------------------------

// Edge selection for input signal detection
// DEFAULT: Rising edge (edge_mode = 2'b00)
// This determines which edge of the START and STOP signals triggers measurement
// The delay line ALWAYS captures on the rising edge of the system clock
//
// edge_mode[1:0]:
//   00 = Rising edge  (default, recommended for most applications)
//   01 = Falling edge
//   10 = Both edges
//   11 = Both edges
assign edge_select = (edge_mode == 2'b00) ? 3'b001 :  // Rising edge (DEFAULT)
                     (edge_mode == 2'b01) ? 3'b010 :  // Falling edge
                                            3'b100;   // Both edges

// Continuous mode disabled for ILA capture (single-shot preferred)
assign continuous_mode = 1'b0;

// Calibration control
assign calibration_enable = calibration_trigger;

// TDC reset from arm signal (re-arm clears previous measurement)
assign tdc_reset = tdc_arm;

// Monitor input signals
assign tdc_start_mon = tdc_start;
assign tdc_stop_mon = tdc_stop;

// ----------------------------------------------------------------
// Result Storage (captures last valid measurement for ILA)
// ----------------------------------------------------------------

always @(posedge sys_clk or negedge rst_sync_n) begin
    if (!rst_sync_n) begin
        measurement_count <= 32'b0;
        last_time_interval <= 32'b0;
        last_fine_time <= 16'b0;
        last_tap_code <= 8'b0;
    end else begin
        // Capture result when measurement is ready
        if (measurement_ready && measurement_valid) begin
            measurement_count <= measurement_count + 1'b1;
            last_time_interval <= time_interval;
            last_fine_time <= time_interval_ps;
            last_tap_code <= delay_line_code;
        end
    end
end

// ----------------------------------------------------------------
// TDC Core Instantiation
// ----------------------------------------------------------------

// Internal register interface (directly controlled, no SPI)
wire [7:0]  reg_addr;
wire [31:0] reg_wdata;
wire [31:0] reg_rdata;
wire        reg_write;
wire        reg_read;

// Tie off register interface (not used in ILA mode)
assign reg_addr = 8'b0;
assign reg_wdata = 32'b0;
assign reg_write = 1'b0;
assign reg_read = 1'b0;

sp701_tdc_single_core #(
    .CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),
    .CLOCK_PERIOD_PS(CLOCK_PERIOD_PS),
    .DELAY_LINE_TAPS(DELAY_LINE_TAPS),
    .CALIBRATION_SAMPLES(CALIBRATION_SAMPLES)
) tdc_core_inst (
    .clk(sys_clk),
    .rst_n(rst_sync_n),

    // TDC Inputs
    .tdc_start(tdc_start),
    .tdc_stop(tdc_stop),
    .tdc_ref_clk(tdc_ref_clk),

    // Control
    .tdc_enable(tdc_enable),
    .tdc_reset(tdc_reset),
    .edge_select(edge_select),
    .continuous_mode(continuous_mode),
    .calibration_enable(calibration_enable),

    // Register Interface (unused in ILA mode)
    .reg_addr(reg_addr),
    .reg_wdata(reg_wdata),
    .reg_write(reg_write),
    .reg_read(reg_read),
    .reg_rdata(reg_rdata),

    // Outputs (directly captured by ILA)
    .time_interval(time_interval),
    .time_interval_ps(time_interval_ps),
    .measurement_valid(measurement_valid),
    .measurement_ready(measurement_ready),
    .tdc_busy(tdc_busy),

    // Status
    .timeout_error(timeout_error),
    .overflow_error(overflow_error),
    .calibration_done(calibration_done),

    // Debug
    .delay_line_code(delay_line_code),
    .coarse_count(coarse_count),
    .fine_count(fine_count)
);

// ----------------------------------------------------------------
// Status LED Control
// ----------------------------------------------------------------

assign led_tdc_ready = rst_sync_n && !timeout_error && !overflow_error;
assign led_measuring = tdc_busy;
assign led_data_valid = measurement_valid;
assign led_error = timeout_error || overflow_error;

// ----------------------------------------------------------------
// ILA Debug Core Instantiation
// ----------------------------------------------------------------
// This ILA core captures TDC measurements for analysis in Vivado
// The ILA is instantiated using Xilinx IP; signals are marked with
// mark_debug attribute for automatic connection

// ILA Trigger Conditions (directly accessible signals)
(* mark_debug = "true" *) wire ila_trigger_start = tdc_start;
(* mark_debug = "true" *) wire ila_trigger_stop = tdc_stop;
(* mark_debug = "true" *) wire ila_trigger_ready = measurement_ready;
(* mark_debug = "true" *) wire ila_trigger_valid = measurement_valid;

// Consolidated debug bus for easy ILA probing
(* mark_debug = "true" *) wire [127:0] ila_data_bus;

assign ila_data_bus = {
    // [127:96] - Time Measurement Result (32 bits)
    time_interval,

    // [95:80] - Fine Time in Picoseconds (16 bits)
    time_interval_ps,

    // [79:64] - Coarse Counter Value (16 bits)
    coarse_count,

    // [63:56] - Fine Count / Tap Position (8 bits)
    fine_count,

    // [55:48] - Delay Line Code (8 bits)
    delay_line_code,

    // [47:40] - Status Flags (8 bits)
    {measurement_valid, measurement_ready, tdc_busy, calibration_done,
     timeout_error, overflow_error, tdc_enable, rst_sync_n},

    // [39:32] - Control Inputs (8 bits)
    {edge_select, continuous_mode, calibration_enable,
     tdc_start, tdc_stop, tdc_arm},

    // [31:0] - Measurement Count (32 bits)
    measurement_count
};

// ----------------------------------------------------------------
// ILA IP Core (Instantiate in Vivado IP Integrator or TCL)
// ----------------------------------------------------------------
// To use ILA, either:
// 1. Run "Set Up Debug" in Vivado after synthesis
// 2. Or instantiate ILA IP manually using the code below:
//
// Uncomment the following block after generating ILA IP:
/*
ila_tdc_debug ila_inst (
    .clk(sys_clk),

    // Probes - match widths with ILA IP configuration
    .probe0(time_interval),           // 32 bits - measurement result
    .probe1(time_interval_ps),        // 16 bits - fine time
    .probe2(coarse_count),            // 16 bits - coarse count
    .probe3(delay_line_code),         // 8 bits - tap code
    .probe4(fine_count),              // 8 bits - fine count
    .probe5({measurement_valid, measurement_ready, tdc_busy,
             calibration_done, timeout_error, overflow_error,
             tdc_enable, rst_sync_n}), // 8 bits - status
    .probe6(measurement_count),        // 32 bits - count
    .probe7(tdc_start),               // 1 bit - trigger
    .probe8(tdc_stop),                // 1 bit - trigger
    .probe9(measurement_ready)        // 1 bit - trigger
);
*/

// ----------------------------------------------------------------
// VIO IP Core (Optional - for runtime control from Vivado)
// ----------------------------------------------------------------
// Uncomment after generating VIO IP:
/*
vio_tdc_control vio_inst (
    .clk(sys_clk),

    // Output probes (directly controls TDC from Vivado)
    .probe_out0(vio_tdc_enable),      // 1 bit - enable
    .probe_out1(vio_tdc_arm),         // 1 bit - arm
    .probe_out2(vio_calibrate),       // 1 bit - calibration
    .probe_out3(vio_edge_mode),       // 2 bits - edge mode

    // Input probes (status feedback)
    .probe_in0(time_interval),        // 32 bits
    .probe_in1(measurement_valid),    // 1 bit
    .probe_in2(measurement_count),    // 32 bits
    .probe_in3(calibration_done)      // 1 bit
);
*/

endmodule
