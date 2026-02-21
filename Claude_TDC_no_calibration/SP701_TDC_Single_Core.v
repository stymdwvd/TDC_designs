/**
 * ================================================================
 * SP701 Single-Channel TDC Core - NO CALIBRATION VERSION
 * Target: Xilinx Spartan-7 XC7S100 (SP701 Evaluation Board)
 * Clock:  100 MHz (10ns period)
 * Accuracy: ~50-100ps (linear approximation, no calibration)
 * ================================================================
 *
 * BLOCK DIAGRAM:
 *
 *  From Top Module          Core Module
 *  +-----------+    +--------------------------------------------------+
 *  | tdc_start |--->|  4-stage       Edge Detect    2-cycle delay       |
 *  | tdc_stop  |--->|  sync regs --> (d3 & ~d4) --> compensation ----+  |
 *  |           |    |                                                |  |
 *  | clk       |--->|  Coarse Counter (32-bit free-running)         |  |
 *  | rst_n     |--->|       |                                       |  |
 *  |           |    |       v                                       v  |
 *  | tdc_enable|--->|  STATE MACHINE --------------------------------  |
 *  | tdc_reset |--->|  IDLE -> WAIT_START -> MEASURING -> PROC 1-3     |
 *  | cont_mode |--->|       |                    |           |         |
 *  |           |    |       |   start_edge:      |  stop_edge:         |
 *  |           |    |       |   capture coarse    |  capture coarse    |
 *  |           |    |       |   + fine tap        |  + fine tap        |
 *  |           |    |       |                    |           |         |
 *  |           |    |       |              PROCESSING PIPELINE         |
 *  |           |    |       |              Stage1: coarse_diff,        |
 *  |           |    |       |                      fine_diff           |
 *  |           |    |       |              Stage2: coarse×period_ps,   |
 *  |           |    |       |                      fine×period_ps      |
 *  |           |    |       |              Stage3: total_ps output     |
 *  +-----------+    |       |                          |               |
 *                   |       v                          v               |
 *  +-----------+    |  time_interval (32-bit ps)                       |
 *  | Outputs   |<---|  time_interval_ps (16-bit sub-ns)                |
 *  | to Top    |<---|  measurement_valid, measurement_ready            |
 *  | Module    |<---|  tdc_busy, timeout_error, overflow_error         |
 *  +-----------+    +--------------------------------------------------+
 *                         |
 *  +-----------+          |  Delay Line Instance
 *  | DelayLine |<-------->|  (CARRY4 chain + bubble suppression
 *  | Module    |          |   + tree encoder, 4-stage pipeline)
 *  +-----------+          |  Returns: encoded_start/stop [7:0]
 *
 * DATA FLOW (per measurement):
 *
 * 1. IDLE: Wait for tdc_enable. Clear errors on tdc_reset.
 *
 * 2. WAIT_START: Wait for start_edge (rising edge on tdc_start).
 *    On start_edge: latch coarse_counter → start_coarse_time
 *                   latch encoded_start  → start_fine_time
 *    Note: start_edge is delayed 2 cycles from raw edge detection
 *          to align with the 2-stage delay line pipeline.
 *
 * 3. MEASURING: Wait for stop_edge (rising edge on tdc_stop).
 *    On stop_edge: latch coarse_counter → stop_coarse_time
 *                  latch encoded_stop   → stop_fine_time
 *
 * 4. PROC_STAGE1: Compute differences
 *    coarse_diff = stop_coarse - start_coarse
 *    fine_diff   = stop_fine - start_fine  (with boundary correction)
 *
 * 5. PROC_STAGE2: Convert to picoseconds (fixed-point)
 *    coarse_ps = coarse_diff × CLOCK_PERIOD_PS
 *    fine_product = fine_diff × CLOCK_PERIOD_PS  (keep full precision)
 *
 * 6. PROC_STAGE3: Final result
 *    time_interval = coarse_ps + (fine_product >>> 8)
 *    Assert measurement_valid and measurement_ready
 *
 * FINE TIME CALCULATION (fixed-point, no truncation):
 *   fine_ps = (tap_diff × 10000) >>> 8
 *   This is equivalent to tap_diff × 39.0625 ps/tap
 *   (vs integer 39 ps/tap which loses 0.0625/tap)
 *
 * ================================================================
 */

module sp701_tdc_single_core #(
    parameter CLOCK_FREQ_HZ = 100_000_000,      // 100 MHz system clock
    parameter CLOCK_PERIOD_PS = 10000,          // 10ns = 10000ps
    parameter MAX_TIME_INTERVAL_NS = 1000000,   // 1ms maximum interval
    parameter DELAY_LINE_TAPS = 256,            // Number of delay line taps
    parameter DATA_WIDTH = 32                   // Output data width
) (
    // Clock and Reset
    input  wire clk,                    // System clock (100 MHz)
    input  wire rst_n,                  // Active low reset

    // TDC Input Signals
    input  wire tdc_start,              // TDC start pulse
    input  wire tdc_stop,               // TDC stop pulse

    // Control Interface
    input  wire tdc_enable,             // Enable TDC operation
    input  wire tdc_reset,              // Reset TDC measurement
    input  wire continuous_mode,        // Continuous measurement mode

    // TDC Output
    output reg  [DATA_WIDTH-1:0] time_interval,    // Measured time interval (ps)
    output reg  [15:0] time_interval_ps,           // Sub-nanosecond precision
    output reg  measurement_valid,                 // Measurement valid flag
    output reg  measurement_ready,                 // New measurement ready
    output reg  tdc_busy,                         // TDC busy measuring

    // Status and Error Flags
    output reg  timeout_error,                     // Measurement timeout
    output reg  overflow_error,                    // Counter overflow

    // Debug and Monitoring
    output wire [7:0] delay_line_code,            // Current delay line code
    output wire [15:0] coarse_count,              // Coarse counter value
    output wire [7:0] fine_count                  // Fine counter value
);

// ----------------------------------------------------------------
// Linear Tap Delay Calculation (No Calibration)
// ----------------------------------------------------------------
// Fine time = (tap_diff × CLOCK_PERIOD_PS) / DELAY_LINE_TAPS
// Since DELAY_LINE_TAPS is a power of 2, division becomes a right shift
// This avoids the truncation error of integer TAP_DELAY_PS (39 vs 39.0625)

localparam TAPS_SHIFT = $clog2(DELAY_LINE_TAPS);  // = 8 for 256 taps

// ----------------------------------------------------------------
// State Machine States
// ----------------------------------------------------------------

localparam [3:0]
    IDLE         = 4'b0000,
    WAIT_START   = 4'b0001,
    MEASURING    = 4'b0010,
    PROC_STAGE1  = 4'b0100,   // Processing pipeline stage 1
    PROC_STAGE2  = 4'b0101,   // Processing pipeline stage 2
    PROC_STAGE3  = 4'b0110,   // Processing pipeline stage 3
    ERROR        = 4'b1000;

reg [3:0] tdc_state, tdc_state_next;

// ----------------------------------------------------------------
// Input Edge Detection with Metastability Protection
// ----------------------------------------------------------------
// 4-stage synchronizer for asynchronous input signals
// Rising edge detection only

wire start_edge, stop_edge;
(* ASYNC_REG = "TRUE" *) reg start_edge_d1, start_edge_d2;
reg start_edge_d3, start_edge_d4;
(* ASYNC_REG = "TRUE" *) reg stop_edge_d1, stop_edge_d2;
reg stop_edge_d3, stop_edge_d4;

// Pipeline delay compensation registers
// The delay line module adds 2 extra pipeline stages (bubble reg + encoder reg)
// so edge detection must be delayed by 2 cycles to align with encoded tap values
reg start_edge_raw, stop_edge_raw;
reg start_edge_delay1, stop_edge_delay1;

// Raw edge detection on synchronized signals
wire start_edge_raw_w = start_edge_d3 & ~start_edge_d4;
wire stop_edge_raw_w = stop_edge_d3 & ~stop_edge_d4;

// Delayed edge detection (2 cycles to match delay line pipeline)
assign start_edge = start_edge_delay1;
assign stop_edge = stop_edge_delay1;

// ----------------------------------------------------------------
// Internal Registers
// ----------------------------------------------------------------

// Timing Counters
reg  [31:0] coarse_counter;
reg  [31:0] start_coarse_time;
reg  [31:0] stop_coarse_time;
reg  [7:0]  start_fine_time;
reg  [7:0]  stop_fine_time;
reg  [31:0] timeout_counter;

// Processing pipeline registers
reg  [31:0] coarse_diff_stage1;
reg  signed [16:0] fine_diff_stage1;
reg  boundary_crossing_stage1;
reg  [31:0] coarse_ps_stage2;
reg  signed [31:0] fine_product_stage2;  // fine_diff × CLOCK_PERIOD_PS (full precision)

// Delay Line Signals
wire [DELAY_LINE_TAPS-1:0] delay_line_start_taps;
wire [DELAY_LINE_TAPS-1:0] delay_line_stop_taps;
wire [7:0] start_tap_encoded;
wire [7:0] stop_tap_encoded;
wire start_valid, stop_valid;

// Timeout Detection
localparam TIMEOUT_CYCLES = CLOCK_FREQ_HZ / 1000; // 1ms timeout

// Debug outputs
assign delay_line_code = stop_tap_encoded;
assign coarse_count = coarse_counter[15:0];
assign fine_count = stop_fine_time;

// ----------------------------------------------------------------
// Input Synchronization (4-stage)
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_edge_d1 <= 1'b0;
        start_edge_d2 <= 1'b0;
        start_edge_d3 <= 1'b0;
        start_edge_d4 <= 1'b0;
        stop_edge_d1 <= 1'b0;
        stop_edge_d2 <= 1'b0;
        stop_edge_d3 <= 1'b0;
        stop_edge_d4 <= 1'b0;
        // Pipeline delay compensation
        start_edge_raw <= 1'b0;
        stop_edge_raw <= 1'b0;
        start_edge_delay1 <= 1'b0;
        stop_edge_delay1 <= 1'b0;
    end else begin
        start_edge_d1 <= tdc_start;
        start_edge_d2 <= start_edge_d1;
        start_edge_d3 <= start_edge_d2;
        start_edge_d4 <= start_edge_d3;
        stop_edge_d1 <= tdc_stop;
        stop_edge_d2 <= stop_edge_d1;
        stop_edge_d3 <= stop_edge_d2;
        stop_edge_d4 <= stop_edge_d3;
        // 2-cycle delay to align edge detection with encoder output
        start_edge_raw <= start_edge_raw_w;
        stop_edge_raw <= stop_edge_raw_w;
        start_edge_delay1 <= start_edge_raw;
        stop_edge_delay1 <= stop_edge_raw;
    end
end

// ----------------------------------------------------------------
// Coarse Counter (increments on every rising clock edge)
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        coarse_counter <= 32'b0;
    end else begin
        coarse_counter <= coarse_counter + 1'b1;
    end
end

// ----------------------------------------------------------------
// State Machine
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tdc_state <= IDLE;
    end else begin
        tdc_state <= tdc_state_next;
    end
end

always @(*) begin
    tdc_state_next = tdc_state;

    case (tdc_state)
        IDLE: begin
            if (tdc_enable) begin
                tdc_state_next = WAIT_START;
            end
        end

        WAIT_START: begin
            if (!tdc_enable) begin
                tdc_state_next = IDLE;
            end else if (start_edge) begin
                tdc_state_next = MEASURING;
            end else if (timeout_counter >= TIMEOUT_CYCLES) begin
                tdc_state_next = ERROR;
            end
        end

        MEASURING: begin
            if (!tdc_enable) begin
                tdc_state_next = IDLE;
            end else if (stop_edge) begin
                tdc_state_next = PROC_STAGE1;
            end else if (timeout_counter >= TIMEOUT_CYCLES) begin
                tdc_state_next = ERROR;
            end
        end

        PROC_STAGE1: tdc_state_next = PROC_STAGE2;
        PROC_STAGE2: tdc_state_next = PROC_STAGE3;
        PROC_STAGE3: tdc_state_next = continuous_mode ? WAIT_START : IDLE;

        ERROR: begin
            if (tdc_reset || !tdc_enable) begin
                tdc_state_next = IDLE;
            end
        end

        default: tdc_state_next = IDLE;
    endcase
end

// ----------------------------------------------------------------
// TDC Measurement Logic (Linear Approximation)
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_coarse_time <= 32'b0;
        stop_coarse_time <= 32'b0;
        start_fine_time <= 8'b0;
        stop_fine_time <= 8'b0;
        timeout_counter <= 32'b0;
        measurement_valid <= 1'b0;
        measurement_ready <= 1'b0;
        tdc_busy <= 1'b0;
        timeout_error <= 1'b0;
        overflow_error <= 1'b0;
        time_interval <= 32'b0;
        time_interval_ps <= 16'b0;

        // Pipeline registers
        coarse_diff_stage1 <= 32'b0;
        fine_diff_stage1 <= 17'sb0;
        boundary_crossing_stage1 <= 1'b0;
        coarse_ps_stage2 <= 32'b0;
        fine_product_stage2 <= 32'sb0;
    end else begin
        // Default values
        measurement_ready <= 1'b0;

        case (tdc_state)
            IDLE: begin
                tdc_busy <= 1'b0;
                timeout_counter <= 32'b0;
                timeout_error <= 1'b0;
                overflow_error <= 1'b0;
                if (tdc_reset) begin
                    measurement_valid <= 1'b0;
                end
            end

            WAIT_START: begin
                tdc_busy <= 1'b1;
                timeout_counter <= timeout_counter + 1'b1;
                if (start_edge) begin
                    // Capture on rising clock edge
                    start_coarse_time <= coarse_counter;
                    start_fine_time <= start_tap_encoded;
                    timeout_counter <= 32'b0;
                end
            end

            MEASURING: begin
                timeout_counter <= timeout_counter + 1'b1;
                if (stop_edge) begin
                    // Capture on rising clock edge
                    stop_coarse_time <= coarse_counter;
                    stop_fine_time <= stop_tap_encoded;
                end
            end

            // Pipeline Stage 1: Calculate differences
            PROC_STAGE1: begin
                coarse_diff_stage1 <= stop_coarse_time - start_coarse_time;

                // Fine time difference with boundary crossing detection
                if (stop_fine_time >= start_fine_time) begin
                    fine_diff_stage1 <= $signed({1'b0, stop_fine_time}) - $signed({1'b0, start_fine_time});
                    boundary_crossing_stage1 <= 1'b0;
                end else begin
                    fine_diff_stage1 <= $signed({1'b0, stop_fine_time}) - $signed({1'b0, start_fine_time}) +
                                       $signed({1'b0, DELAY_LINE_TAPS[8:0]});
                    boundary_crossing_stage1 <= 1'b1;
                end
            end

            // Pipeline Stage 2: Convert to picoseconds (FIXED-POINT)
            PROC_STAGE2: begin
                // Adjust coarse count if boundary crossing occurred
                if (boundary_crossing_stage1) begin
                    coarse_ps_stage2 <= (coarse_diff_stage1 - 1) * CLOCK_PERIOD_PS;
                end else begin
                    coarse_ps_stage2 <= coarse_diff_stage1 * CLOCK_PERIOD_PS;
                end

                // Full precision: fine_ps = (tap_diff × CLOCK_PERIOD_PS) >> TAPS_SHIFT
                // Keeps full product before shifting, avoids truncation error
                fine_product_stage2 <= fine_diff_stage1 * $signed({1'b0, CLOCK_PERIOD_PS[15:0]});
            end

            // Pipeline Stage 3: Final calculation
            PROC_STAGE3: begin
                if (coarse_diff_stage1 > (MAX_TIME_INTERVAL_NS / 10)) begin
                    overflow_error <= 1'b1;
                    measurement_valid <= 1'b0;
                end else begin
                    // Arithmetic right shift: divide product by DELAY_LINE_TAPS
                    // fine_ps_shifted = (fine_diff × CLOCK_PERIOD_PS) / 256
                    if (fine_product_stage2 >= 0) begin
                        time_interval <= coarse_ps_stage2 + (fine_product_stage2 >>> TAPS_SHIFT);
                        time_interval_ps <= fine_product_stage2[TAPS_SHIFT +: 16];
                    end else begin
                        time_interval <= coarse_ps_stage2;
                        time_interval_ps <= 16'b0;
                    end
                    measurement_valid <= 1'b1;
                    measurement_ready <= 1'b1;
                end
            end

            ERROR: begin
                tdc_busy <= 1'b0;
                timeout_error <= 1'b1;
                measurement_valid <= 1'b0;
            end

            default: begin
                tdc_busy <= 1'b0;
                timeout_error <= 1'b0;
                measurement_valid <= 1'b0;
            end
        endcase
    end
end

// ----------------------------------------------------------------
// Delay Line Instantiation
// ----------------------------------------------------------------

sp701_tdc_delay_line #(
    .NUM_TAPS(DELAY_LINE_TAPS),
    .DELAY_TYPE("CARRY")
) delay_line_inst (
    .clk(clk),
    .rst_n(rst_n),
    .input_pulse_start(tdc_start),
    .input_pulse_stop(tdc_stop),
    .delay_line_out_start(delay_line_start_taps),
    .delay_line_out_stop(delay_line_stop_taps),
    .encoded_start(start_tap_encoded),
    .encoded_stop(stop_tap_encoded),
    .start_valid(start_valid),
    .stop_valid(stop_valid)
);

endmodule
