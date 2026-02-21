/**
 * ================================================================
 * SP701 Single-Channel TDC Top Level - NO CALIBRATION VERSION
 * Target: Xilinx Spartan-7 XC7S100 (SP701 Evaluation Board)
 * Clock:  100 MHz (10ns period)
 * Accuracy: ~50-100ps (linear approximation, no calibration)
 * ================================================================
 *
 * SYSTEM BLOCK DIAGRAM:
 *
 *   External Pins            Top Module                     Core Module
 *  +-----------+    +---------------------------+    +------------------+
 *  | sys_clk_p |--->| IBUF -> BUFG (sys_clk)    |--->| clk              |
 *  | sys_rst_n |--->| 4-stage reset sync        |--->| rst_n            |
 *  |           |    |                           |    |                  |
 *  | tdc_start |----+--- (direct to core) ------+--->| tdc_start        |
 *  | tdc_stop  |----+--- (direct to core) ------+--->| tdc_stop         |
 *  |           |    |                           |    |                  |
 *  | tdc_enable|--->| 2-stage sync -> enable    |--->| tdc_enable       |
 *  | tdc_arm   |--->| 2-stage sync -> reset     |--->| tdc_reset        |
 *  +-----------+    |                           |    |                  |
 *                   |   Result Storage           |<---| time_interval    |
 *  +-----------+    |   (last_time_interval,     |<---| measurement_ready|
 *  | LEDs      |<---|    last_fine_time,         |<---| measurement_valid|
 *  | ready     |<---|    last_tap_code,          |<---| tdc_busy         |
 *  | measuring |<---|    measurement_count)      |<---| timeout_error    |
 *  | valid     |<---|                           |<---| overflow_error   |
 *  | error     |<---|   ILA Debug Bus [95:0]     |    +------------------+
 *  +-----------+    +---------------------------+
 *
 * SIGNAL FLOW:
 * 1. sys_clk_p  -> IBUF -> BUFG -> sys_clk (all logic runs on this)
 * 2. sys_rst_n  -> 4-stage sync -> rst_sync_n (safe async reset release)
 * 3. tdc_enable -> 2-stage sync -> tdc_enable_sync (metastability safe)
 * 4. tdc_arm    -> 2-stage sync -> tdc_arm_sync -> tdc_reset
 * 5. tdc_start/stop go directly to Core (Core has its own 4-stage sync)
 * 6. Measurements captured via ILA debug probes (no external data bus)
 *
 * ================================================================
 */

module sp701_tdc_single_top #(
    parameter CLOCK_FREQ_HZ = 100_000_000,
    parameter CLOCK_PERIOD_PS = 10000,          // 10ns = 10000ps
    parameter DELAY_LINE_TAPS = 256
) (
    // System Interface
    input  wire sys_clk_p,              // System clock input
    input  wire sys_rst_n,              // System reset (active low)

    // TDC Input Signals
    input  wire tdc_start,              // Start signal
    input  wire tdc_stop,               // Stop signal

    // Simple Control Interface
    input  wire tdc_enable,             // Enable TDC operation
    input  wire tdc_arm,                // Arm for next measurement

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

// TDC Control Signals (synchronized)
wire tdc_enable_sync;
wire tdc_arm_sync;
wire continuous_mode;
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

// Debug Signals from Core
(* mark_debug = "true" *) wire [7:0] delay_line_code;
(* mark_debug = "true" *) wire [15:0] coarse_count;
(* mark_debug = "true" *) wire [7:0] fine_count;

// Input Signal Monitoring
(* mark_debug = "true" *) wire tdc_start_mon;
(* mark_debug = "true" *) wire tdc_stop_mon;

// Measurement Counter
(* mark_debug = "true" *) reg [31:0] measurement_count;

// Result Storage Registers
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

// Reset synchronization (4-stage)
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
// Input Synchronizers for Control Signals
// ----------------------------------------------------------------
// 2-stage synchronizers to prevent metastability

(* ASYNC_REG = "TRUE" *) reg [1:0] enable_sync_reg;
(* ASYNC_REG = "TRUE" *) reg [1:0] arm_sync_reg;

always @(posedge sys_clk or negedge rst_sync_n) begin
    if (!rst_sync_n) begin
        enable_sync_reg <= 2'b0;
        arm_sync_reg <= 2'b0;
    end else begin
        enable_sync_reg <= {enable_sync_reg[0], tdc_enable};
        arm_sync_reg <= {arm_sync_reg[0], tdc_arm};
    end
end

assign tdc_enable_sync = enable_sync_reg[1];
assign tdc_arm_sync = arm_sync_reg[1];

// ----------------------------------------------------------------
// Control Signal Mapping
// ----------------------------------------------------------------

// Continuous mode disabled for ILA capture
assign continuous_mode = 1'b0;

// TDC reset from arm signal (synchronized)
assign tdc_reset = tdc_arm_sync;

// Monitor input signals
assign tdc_start_mon = tdc_start;
assign tdc_stop_mon = tdc_stop;

// ----------------------------------------------------------------
// Result Storage
// ----------------------------------------------------------------

always @(posedge sys_clk or negedge rst_sync_n) begin
    if (!rst_sync_n) begin
        measurement_count <= 32'b0;
        last_time_interval <= 32'b0;
        last_fine_time <= 16'b0;
        last_tap_code <= 8'b0;
    end else begin
        if (measurement_ready && measurement_valid) begin
            measurement_count <= measurement_count + 1'b1;
            last_time_interval <= time_interval;
            last_fine_time <= time_interval_ps;
            last_tap_code <= delay_line_code;
        end
    end
end

// ----------------------------------------------------------------
// TDC Core Instantiation (No Calibration)
// ----------------------------------------------------------------

sp701_tdc_single_core #(
    .CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),
    .CLOCK_PERIOD_PS(CLOCK_PERIOD_PS),
    .DELAY_LINE_TAPS(DELAY_LINE_TAPS)
) tdc_core_inst (
    .clk(sys_clk),
    .rst_n(rst_sync_n),

    // TDC Inputs
    .tdc_start(tdc_start),
    .tdc_stop(tdc_stop),

    // Control
    .tdc_enable(tdc_enable_sync),
    .tdc_reset(tdc_reset),
    .continuous_mode(continuous_mode),

    // Outputs
    .time_interval(time_interval),
    .time_interval_ps(time_interval_ps),
    .measurement_valid(measurement_valid),
    .measurement_ready(measurement_ready),
    .tdc_busy(tdc_busy),

    // Status
    .timeout_error(timeout_error),
    .overflow_error(overflow_error),

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
// ILA Debug Signals
// ----------------------------------------------------------------

(* mark_debug = "true" *) wire ila_trigger_start = tdc_start;
(* mark_debug = "true" *) wire ila_trigger_stop = tdc_stop;
(* mark_debug = "true" *) wire ila_trigger_ready = measurement_ready;
(* mark_debug = "true" *) wire ila_trigger_valid = measurement_valid;

// Consolidated debug bus
(* mark_debug = "true" *) wire [95:0] ila_data_bus;

assign ila_data_bus = {
    // [95:64] - Time Measurement Result (32 bits)
    time_interval,

    // [63:48] - Fine Time in Picoseconds (16 bits)
    time_interval_ps,

    // [47:32] - Coarse Counter Value (16 bits)
    coarse_count,

    // [31:24] - Fine Count / Tap Position (8 bits)
    fine_count,

    // [23:16] - Delay Line Code (8 bits)
    delay_line_code,

    // [15:8] - Status Flags (8 bits)
    {measurement_valid, measurement_ready, tdc_busy, 1'b0,
     timeout_error, overflow_error, tdc_enable_sync, rst_sync_n},

    // [7:0] - Control Inputs (8 bits)
    {3'b001, continuous_mode, 1'b0,
     tdc_start, tdc_stop, tdc_arm}
};

endmodule
