/**
 * ================================================================
 * SP701 Single-Channel TDC Core - NO CALIBRATION VERSION
 * Date: 2025-12-28
 * Description: Simplified TDC without calibration system
 *
 * This version uses LINEAR APPROXIMATION for fine time calculation:
 *   fine_time_ps = tap_position × (clock_period_ps / num_taps)
 *
 * ACCURACY: ~50-100ps (vs ~10-20ps with calibration)
 * RESOURCES: Reduced (no calibration LUTs, simpler logic)
 *
 * MEASUREMENT PRINCIPLE:
 * - Delay line captures signal position on EVERY rising clock edge
 * - Coarse counter increments on rising clock edge
 * - Fine time = tap position × linear tap delay estimate
 * ================================================================
 */

module sp701_tdc_single_core #(
    parameter CLOCK_FREQ_HZ = 100_000_000,      // 100 MHz system clock
    parameter CLOCK_PERIOD_PS = 10000,          // 10ns = 10000ps
    parameter MAX_TIME_INTERVAL_NS = 1000000,   // 1ms maximum interval
    parameter DELAY_LINE_TAPS = 256,            // Number of delay line taps
    parameter DATA_WIDTH = 32,                  // Output data width
    parameter ADDR_WIDTH = 8                    // Address width for registers
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
    input  wire [2:0] edge_select,      // Edge detection selection
    input  wire continuous_mode,        // Continuous measurement mode

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

    // Debug and Monitoring
    output wire [7:0] delay_line_code,            // Current delay line code
    output wire [15:0] coarse_count,              // Coarse counter value
    output wire [7:0] fine_count                  // Fine counter value
);

// ----------------------------------------------------------------
// Linear Tap Delay Calculation (No Calibration)
// ----------------------------------------------------------------
// Assumes uniform tap delays across the delay line
// tap_delay = clock_period / num_taps
// For 100MHz (10ns) with 256 taps: ~39ps per tap

localparam TAP_DELAY_PS = CLOCK_PERIOD_PS / DELAY_LINE_TAPS;  // ~39ps

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
// Default mode: Rising edge detection (edge_select[0] = 1)

wire start_edge, stop_edge;
(* ASYNC_REG = "TRUE" *) reg start_edge_d1, start_edge_d2;
reg start_edge_d3, start_edge_d4;
(* ASYNC_REG = "TRUE" *) reg stop_edge_d1, stop_edge_d2;
reg stop_edge_d3, stop_edge_d4;

// Edge detection on synchronized signals
assign start_edge = (edge_select[0]) ? (start_edge_d3 & ~start_edge_d4) :  // Rising edge
                   (edge_select[1]) ? (~start_edge_d3 & start_edge_d4) :   // Falling edge
                                     (start_edge_d3 ^ start_edge_d4);      // Both edges

assign stop_edge = (edge_select[0]) ? (stop_edge_d3 & ~stop_edge_d4) :    // Rising edge
                  (edge_select[1]) ? (~stop_edge_d3 & stop_edge_d4) :     // Falling edge
                                    (stop_edge_d3 ^ stop_edge_d4);        // Both edges

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
reg  signed [16:0] fine_ps_stage2;
reg  [31:0] total_time_stage3;

// Delay Line Signals
wire [DELAY_LINE_TAPS-1:0] delay_line_start_taps;
wire [DELAY_LINE_TAPS-1:0] delay_line_stop_taps;
wire [7:0] start_tap_encoded;
wire [7:0] stop_tap_encoded;
wire start_valid, stop_valid;

// Configuration Registers
reg  [31:0] config_reg [7:0];
integer j;

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

            // Pipeline Stage 2: Convert to picoseconds (LINEAR APPROXIMATION)
            PROC_STAGE2: begin
                // Adjust coarse count if boundary crossing occurred
                if (boundary_crossing_stage1) begin
                    coarse_ps_stage2 <= (coarse_diff_stage1 - 1) * CLOCK_PERIOD_PS;
                end else begin
                    coarse_ps_stage2 <= coarse_diff_stage1 * CLOCK_PERIOD_PS;
                end

                // Linear approximation: fine_ps = tap_diff × tap_delay
                fine_ps_stage2 <= (fine_diff_stage1 * $signed(TAP_DELAY_PS));
            end

            // Pipeline Stage 3: Final calculation
            PROC_STAGE3: begin
                if (coarse_diff_stage1 > (MAX_TIME_INTERVAL_NS / 10)) begin
                    overflow_error <= 1'b1;
                    measurement_valid <= 1'b0;
                end else begin
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
// Configuration Register Interface
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (j = 0; j < 8; j = j + 1) begin
            config_reg[j] <= 32'b0;
        end
        config_reg[0] <= 32'h00000001;  // TDC enable
        config_reg[1] <= 32'h00000001;  // Edge select (rising)
        reg_rdata <= 32'b0;
    end else begin
        if (reg_write && reg_addr < 8) begin
            config_reg[reg_addr] <= reg_wdata;
        end

        if (reg_read) begin
            case (reg_addr)
                8'h00: reg_rdata <= config_reg[0];
                8'h01: reg_rdata <= config_reg[1];
                8'h10: reg_rdata <= time_interval;
                8'h11: reg_rdata <= {16'b0, time_interval_ps};
                8'h12: reg_rdata <= {24'b0, delay_line_code};
                8'h13: reg_rdata <= coarse_counter;
                8'h20: reg_rdata <= {28'b0, tdc_state};
                8'h21: reg_rdata <= {30'b0, timeout_error, overflow_error};
                default: reg_rdata <= 32'b0;
            endcase
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

endmodule
