/**
 * ================================================================
 * SP701 Single-Channel Time-to-Digital Converter (TDC) - Core Module
 * Date: 2025-01-15 (Original), 2025-12-28 (Rising Edge Revision)
 * Description: Optimized single-channel TDC implementation in Verilog
 *
 * MEASUREMENT PRINCIPLE:
 * - Delay line captures signal position on EVERY rising clock edge
 * - Coarse counter increments on each rising clock edge
 * - Fine time = tap position where signal stopped propagating
 * - Total time = (coarse_count × clock_period) + (fine_tap × tap_delay)
 *
 * REVISION NOTES:
 * - Rising clock edge capture for proper TDC operation
 * - Fixed fine-time boundary crossing arithmetic (handles wrap-around)
 * - Added 3-stage pipeline to PROCESSING state for timing closure
 * - Fixed calibration interface to use memory read (no array ports)
 * - Improved metastability protection (4-stage synchronizer)
 * ================================================================
 */

module sp701_tdc_single_core #(
    parameter CLOCK_FREQ_HZ = 100_000_000,      // 100 MHz system clock
    parameter CLOCK_PERIOD_PS = 10000,          // 10ns = 10000ps
    parameter MAX_TIME_INTERVAL_NS = 1000000,   // 1ms maximum interval
    parameter DELAY_LINE_TAPS = 256,            // Number of delay line taps
    parameter DATA_WIDTH = 32,                  // Output data width
    parameter ADDR_WIDTH = 8,                   // Address width for registers
    parameter CALIBRATION_SAMPLES = 4096        // Samples for calibration
) (
    // Clock and Reset
    input  wire clk,                    // System clock (100 MHz)
    input  wire rst_n,                  // Active low reset

    // TDC Input Signals
    input  wire tdc_start,              // TDC start pulse
    input  wire tdc_stop,               // TDC stop pulse
    input  wire tdc_ref_clk,            // Reference clock for calibration

    // Control Interface
    input  wire tdc_enable,             // Enable TDC operation
    input  wire tdc_reset,              // Reset TDC measurement
    input  wire [2:0] edge_select,      // Edge detection selection
    input  wire continuous_mode,        // Continuous measurement mode
    input  wire calibration_enable,     // Enable calibration

    // Configuration Registers Interface
    input  wire [ADDR_WIDTH-1:0] reg_addr,
    input  wire [DATA_WIDTH-1:0] reg_wdata,
    input  wire reg_write,
    input  wire reg_read,
    output reg  [DATA_WIDTH-1:0] reg_rdata,

    // TDC Output
    output reg  [DATA_WIDTH-1:0] time_interval,    // Measured time interval (ps)
    output reg  [15:0] time_interval_ps,           // Sub-nanosecond precision
    output reg  measurement_valid,                 // Measurement valid flag
    output reg  measurement_ready,                 // New measurement ready
    output reg  tdc_busy,                         // TDC busy measuring

    // Status and Error Flags
    output reg  timeout_error,                     // Measurement timeout
    output reg  overflow_error,                    // Counter overflow
    output reg  calibration_done,                  // Calibration complete

    // Debug and Monitoring
    output wire [7:0] delay_line_code,            // Current delay line code
    output wire [15:0] coarse_count,              // Coarse counter value
    output wire [7:0] fine_count                  // Fine counter value
);

// ----------------------------------------------------------------
// Internal Signals and Registers
// ----------------------------------------------------------------

// State Machine States
localparam [3:0]
    IDLE         = 4'b0000,
    WAIT_START   = 4'b0001,
    MEASURING    = 4'b0010,
    WAIT_STOP    = 4'b0011,
    PROC_STAGE1  = 4'b0100,   // Processing pipeline stage 1
    PROC_STAGE2  = 4'b0101,   // Processing pipeline stage 2
    PROC_STAGE3  = 4'b0110,   // Processing pipeline stage 3
    CALIBRATING  = 4'b0111,
    ERROR        = 4'b1000;

reg [3:0] tdc_state, tdc_state_next;

// ----------------------------------------------------------------
// Input Edge Detection with Metastability Protection
// ----------------------------------------------------------------
// 4-stage synchronizer for asynchronous input signals
// Default mode: Rising edge detection (edge_select[0] = 1)
// The synchronized edge tells us WHEN to capture the coarse counter
// The delay line output tells us WHERE in the clock period the edge arrived

wire start_edge, stop_edge;
(* ASYNC_REG = "TRUE" *) reg start_edge_d1, start_edge_d2;
reg start_edge_d3, start_edge_d4;
(* ASYNC_REG = "TRUE" *) reg stop_edge_d1, stop_edge_d2;
reg stop_edge_d3, stop_edge_d4;

// Edge detection on synchronized signals (d3/d4 for stability)
// Rising edge (default): detect 0->1 transition
// Falling edge: detect 1->0 transition
// Both edges: detect any transition
assign start_edge = (edge_select[0]) ? (start_edge_d3 & ~start_edge_d4) :  // Rising edge
                   (edge_select[1]) ? (~start_edge_d3 & start_edge_d4) :   // Falling edge
                                     (start_edge_d3 ^ start_edge_d4);      // Both edges

assign stop_edge = (edge_select[0]) ? (stop_edge_d3 & ~stop_edge_d4) :    // Rising edge
                  (edge_select[1]) ? (~stop_edge_d3 & stop_edge_d4) :     // Falling edge
                                    (stop_edge_d3 ^ stop_edge_d4);        // Both edges

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
reg  signed [16:0] fine_ps_stage2;
reg  [31:0] total_time_stage3;

// Delay Line Signals
wire [DELAY_LINE_TAPS-1:0] delay_line_start_taps;
wire [DELAY_LINE_TAPS-1:0] delay_line_stop_taps;
wire [7:0] start_tap_encoded;
wire [7:0] stop_tap_encoded;
wire start_valid, stop_valid;

// Calibration System - Memory Interface
wire calibration_complete_wire;
wire calibration_valid_wire;
reg  [7:0] cal_read_addr;
wire [15:0] cal_tap_delay_data;
wire [15:0] cal_bin_width_data;

// Calibration data cache (read during processing)
reg [15:0] start_tap_delay_cached;
reg [15:0] stop_tap_delay_cached;
reg calibration_data_valid;

// Configuration Registers
reg  [31:0] config_reg [15:0];
integer j;

// Timeout and Error Detection
localparam TIMEOUT_CYCLES = CLOCK_FREQ_HZ / 1000; // 1ms timeout

// Debug outputs
assign delay_line_code = stop_tap_encoded;
assign coarse_count = coarse_counter[15:0];
assign fine_count = stop_fine_time;

// ----------------------------------------------------------------
// Input Synchronization (4-stage for better metastability protection)
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
    end else begin
        start_edge_d1 <= tdc_start;
        start_edge_d2 <= start_edge_d1;
        start_edge_d3 <= start_edge_d2;
        start_edge_d4 <= start_edge_d3;
        stop_edge_d1 <= tdc_stop;
        stop_edge_d2 <= stop_edge_d1;
        stop_edge_d3 <= stop_edge_d2;
        stop_edge_d4 <= stop_edge_d3;
    end
end

// ----------------------------------------------------------------
// Coarse Counter (increments on every rising clock edge)
// ----------------------------------------------------------------
// This counter provides the coarse time measurement
// Each count = one clock period (10ns at 100MHz)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        coarse_counter <= 32'b0;
    end else begin
        // Increment on rising clock edge
        coarse_counter <= coarse_counter + 1'b1;
    end
end

// ----------------------------------------------------------------
// Main TDC State Machine
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
            if (tdc_enable && !calibration_enable) begin
                tdc_state_next = WAIT_START;
            end else if (calibration_enable) begin
                tdc_state_next = CALIBRATING;
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

        // 3-stage processing pipeline for timing closure
        PROC_STAGE1: begin
            tdc_state_next = PROC_STAGE2;
        end

        PROC_STAGE2: begin
            tdc_state_next = PROC_STAGE3;
        end

        PROC_STAGE3: begin
            tdc_state_next = continuous_mode ? WAIT_START : IDLE;
        end

        CALIBRATING: begin
            if (!calibration_enable) begin
                tdc_state_next = IDLE;
            end else if (calibration_complete_wire) begin
                tdc_state_next = IDLE;
            end
        end

        ERROR: begin
            if (tdc_reset || !tdc_enable) begin
                tdc_state_next = IDLE;
            end
        end

        default: tdc_state_next = IDLE;
    endcase
end

// ----------------------------------------------------------------
// Calibration Data Read Logic
// ----------------------------------------------------------------

// Read calibration data on-demand during processing
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cal_read_addr <= 8'b0;
        start_tap_delay_cached <= 16'b0;
        stop_tap_delay_cached <= 16'b0;
    end else begin
        case (tdc_state)
            MEASURING: begin
                // Pre-load start tap delay
                cal_read_addr <= start_fine_time;
            end

            PROC_STAGE1: begin
                // Cache start tap delay, request stop tap delay
                start_tap_delay_cached <= cal_tap_delay_data;
                cal_read_addr <= stop_fine_time;
            end

            PROC_STAGE2: begin
                // Cache stop tap delay
                stop_tap_delay_cached <= cal_tap_delay_data;
            end

            default: begin
                cal_read_addr <= 8'b0;
            end
        endcase
    end
end

// ----------------------------------------------------------------
// TDC Measurement Logic with Pipelined Processing
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
        fine_ps_stage2 <= 17'sb0;
        total_time_stage3 <= 32'b0;
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
                    // Capture on rising clock edge when start signal detected:
                    // - Coarse time: current clock cycle count
                    // - Fine time: tap position from delay line (captured at clock edge)
                    start_coarse_time <= coarse_counter;
                    start_fine_time <= start_tap_encoded;
                    timeout_counter <= 32'b0;
                end
            end

            MEASURING: begin
                timeout_counter <= timeout_counter + 1'b1;
                if (stop_edge) begin
                    // Capture on rising clock edge when stop signal detected:
                    // - Coarse time: current clock cycle count
                    // - Fine time: tap position from delay line (captured at clock edge)
                    stop_coarse_time <= coarse_counter;
                    stop_fine_time <= stop_tap_encoded;
                end
            end

            // Pipeline Stage 1: Calculate differences and detect boundary crossing
            PROC_STAGE1: begin
                // Coarse time difference (handles wrap-around automatically)
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

            // Pipeline Stage 2: Convert to picoseconds
            PROC_STAGE2: begin
                // Adjust coarse count if boundary crossing occurred
                if (boundary_crossing_stage1) begin
                    coarse_ps_stage2 <= (coarse_diff_stage1 - 1) * CLOCK_PERIOD_PS;
                end else begin
                    coarse_ps_stage2 <= coarse_diff_stage1 * CLOCK_PERIOD_PS;
                end

                // Use calibrated values if available
                if (calibration_data_valid) begin
                    if (stop_fine_time >= start_fine_time) begin
                        fine_ps_stage2 <= $signed({1'b0, stop_tap_delay_cached}) -
                                         $signed({1'b0, start_tap_delay_cached});
                    end else begin
                        fine_ps_stage2 <= $signed({1'b0, CLOCK_PERIOD_PS[15:0]}) -
                                         $signed({1'b0, start_tap_delay_cached}) +
                                         $signed({1'b0, stop_tap_delay_cached});
                    end
                end else begin
                    // Use linear approximation if not calibrated
                    fine_ps_stage2 <= (fine_diff_stage1 * $signed(CLOCK_PERIOD_PS)) /
                                     $signed(DELAY_LINE_TAPS);
                end
            end

            // Pipeline Stage 3: Final calculation and output
            PROC_STAGE3: begin
                // Check for overflow
                if (coarse_diff_stage1 > (MAX_TIME_INTERVAL_NS / 10)) begin
                    overflow_error <= 1'b1;
                    measurement_valid <= 1'b0;
                end else begin
                    // Calculate total time
                    if (fine_ps_stage2 >= 0) begin
                        total_time_stage3 <= coarse_ps_stage2 + fine_ps_stage2[15:0];
                    end else begin
                        total_time_stage3 <= coarse_ps_stage2;
                    end

                    time_interval <= coarse_ps_stage2 + (fine_ps_stage2 >= 0 ? fine_ps_stage2[15:0] : 16'b0);
                    time_interval_ps <= fine_ps_stage2[15:0];
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
// Calibration Status Management
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        calibration_done <= 1'b0;
        calibration_data_valid <= 1'b0;
    end else begin
        calibration_done <= calibration_complete_wire;

        // Mark calibration data valid when calibration completes
        if (calibration_complete_wire && calibration_valid_wire) begin
            calibration_data_valid <= 1'b1;
        end
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

// ----------------------------------------------------------------
// Calibration Module Instantiation (Updated Interface)
// ----------------------------------------------------------------

sp701_tdc_calibration #(
    .NUM_TAPS(DELAY_LINE_TAPS),
    .CALIBRATION_DEPTH(CALIBRATION_SAMPLES),
    .REF_PERIOD_PS(CLOCK_PERIOD_PS)
) calibration_inst (
    .clk(clk),
    .rst_n(rst_n),
    .calibration_start(calibration_enable && (tdc_state == CALIBRATING)),
    .reference_clock(tdc_ref_clk),
    .measured_tap_position(start_tap_encoded),
    .measurement_valid(start_valid),
    .calibration_complete(calibration_complete_wire),
    .calibration_valid(calibration_valid_wire),

    // Memory read interface
    .tap_read_addr(cal_read_addr),
    .tap_delay_data(cal_tap_delay_data),
    .bin_width_data(cal_bin_width_data)
);

// ----------------------------------------------------------------
// Configuration Register Interface
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Initialize configuration registers
        config_reg[0] <= 32'h00000001;  // TDC enable
        config_reg[1] <= 32'h00000001;  // Edge select (rising edge default)
        config_reg[2] <= 32'h00000000;  // Continuous mode
        config_reg[3] <= TIMEOUT_CYCLES; // Timeout value
        config_reg[4] <= CLOCK_PERIOD_PS; // Clock period in ps
        config_reg[5] <= DELAY_LINE_TAPS; // Number of taps
        // Initialize remaining registers to 0
        for (j = 6; j < 16; j = j + 1) begin
            config_reg[j] <= 32'h00000000;
        end
        reg_rdata <= 32'h00000000;
    end else begin
        // Write operation
        if (reg_write && reg_addr < 16) begin
            config_reg[reg_addr] <= reg_wdata;
        end

        // Read operation
        if (reg_read) begin
            case (reg_addr)
                8'h00: reg_rdata <= config_reg[0];
                8'h01: reg_rdata <= config_reg[1];
                8'h02: reg_rdata <= config_reg[2];
                8'h03: reg_rdata <= config_reg[3];
                8'h04: reg_rdata <= config_reg[4];
                8'h05: reg_rdata <= config_reg[5];
                8'h10: reg_rdata <= time_interval;
                8'h11: reg_rdata <= {16'h0000, time_interval_ps};
                8'h12: reg_rdata <= {24'h000000, delay_line_code};
                8'h13: reg_rdata <= {16'h0000, coarse_count};
                8'h14: reg_rdata <= {26'h0000000,
                                    overflow_error,
                                    timeout_error,
                                    calibration_done,
                                    measurement_ready,
                                    measurement_valid,
                                    tdc_busy};
                8'h15: reg_rdata <= {24'h000000, start_fine_time};
                8'h16: reg_rdata <= {24'h000000, stop_fine_time};
                8'h17: reg_rdata <= start_coarse_time;
                8'h18: reg_rdata <= stop_coarse_time;
                8'h19: reg_rdata <= {31'b0, calibration_data_valid};
                default: begin
                    if (reg_addr < 16) begin
                        reg_rdata <= config_reg[reg_addr];
                    end else begin
                        reg_rdata <= 32'h00000000;
                    end
                end
            endcase
        end
    end
end

endmodule
