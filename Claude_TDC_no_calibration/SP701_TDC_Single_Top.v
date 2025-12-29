/**
 * ================================================================
 * SP701 Single-Channel TDC Top Level - NO CALIBRATION VERSION
 * Date: 2025-12-28
 * Description: Simplified TDC without calibration, ILA for debug
 *
 * This version uses LINEAR APPROXIMATION - no external reference
 * clock needed for calibration.
 *
 * ACCURACY: ~50-100ps (sufficient for many applications)
 * ADVANTAGES:
 * - No calibration reference clock required
 * - Simpler interface
 * - Reduced resource usage
 * - Faster startup (no calibration delay)
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
// Control Signal Mapping
// ----------------------------------------------------------------

// Edge selection: 00=rising (default), 01=falling, 10=both
assign edge_select = (edge_mode == 2'b00) ? 3'b001 :  // Rising edge (DEFAULT)
                     (edge_mode == 2'b01) ? 3'b010 :  // Falling edge
                                            3'b100;   // Both edges

// Continuous mode disabled for ILA capture
assign continuous_mode = 1'b0;

// TDC reset from arm signal
assign tdc_reset = tdc_arm;

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

wire [7:0]  reg_addr;
wire [31:0] reg_wdata;
wire [31:0] reg_rdata;
wire        reg_write;
wire        reg_read;

// Tie off register interface
assign reg_addr = 8'b0;
assign reg_wdata = 32'b0;
assign reg_write = 1'b0;
assign reg_read = 1'b0;

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
    .tdc_enable(tdc_enable),
    .tdc_reset(tdc_reset),
    .edge_select(edge_select),
    .continuous_mode(continuous_mode),

    // Register Interface (unused)
    .reg_addr(reg_addr),
    .reg_wdata(reg_wdata),
    .reg_write(reg_write),
    .reg_read(reg_read),
    .reg_rdata(reg_rdata),

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
     timeout_error, overflow_error, tdc_enable, rst_sync_n},

    // [7:0] - Control Inputs (8 bits)
    {edge_select, continuous_mode, 1'b0,
     tdc_start, tdc_stop, tdc_arm}
};

endmodule
