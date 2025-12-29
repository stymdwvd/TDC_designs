/**
 * ================================================================
 * SP701 TDC Tapped Delay Line Module - NO CALIBRATION VERSION
 * Date: 2025-12-28
 * Description: Simplified delay line without calibration
 *
 * This version uses LINEAR APPROXIMATION for fine time calculation.
 * Suitable for applications where ~50-100ps accuracy is acceptable.
 *
 * MEASUREMENT PRINCIPLE:
 * - Captures delay line state on EVERY rising clock edge
 * - Fine time = tap_position × (clock_period / num_taps)
 * - No calibration needed - simpler and uses less resources
 * ================================================================
 */

module sp701_tdc_delay_line #(
    parameter NUM_TAPS = 256,           // Number of delay taps (must be multiple of 4 for CARRY4)
    parameter DELAY_TYPE = "CARRY"      // "LUT" or "CARRY" delay type (CARRY recommended)
) (
    input  wire clk,                    // System clock - captures on rising edge
    input  wire rst_n,                  // Active low reset
    input  wire input_pulse_start,      // Start pulse input
    input  wire input_pulse_stop,       // Stop pulse input
    output wire [NUM_TAPS-1:0] delay_line_out_start,  // Captured delay line for start
    output wire [NUM_TAPS-1:0] delay_line_out_stop,   // Captured delay line for stop
    output wire [7:0] encoded_start,    // Encoded tap position for start
    output wire [7:0] encoded_stop,     // Encoded tap position for stop
    output wire start_valid,            // Start encoding valid
    output wire stop_valid              // Stop encoding valid
);

// ----------------------------------------------------------------
// Internal Signals
// ----------------------------------------------------------------

// Delay chains - one extra tap for input
wire [NUM_TAPS:0] delay_chain_start;
wire [NUM_TAPS:0] delay_chain_stop;

// Captured state on rising clock edge (2-stage for metastability)
(* ASYNC_REG = "TRUE" *) reg [NUM_TAPS-1:0] captured_start_stage1;
(* ASYNC_REG = "TRUE" *) reg [NUM_TAPS-1:0] captured_stop_stage1;
reg [NUM_TAPS-1:0] captured_start_stage2;
reg [NUM_TAPS-1:0] captured_stop_stage2;

// Bubble-corrected outputs
wire [NUM_TAPS-1:0] start_bubble_corrected;
wire [NUM_TAPS-1:0] stop_bubble_corrected;

// Input assignment
assign delay_chain_start[0] = input_pulse_start;
assign delay_chain_stop[0] = input_pulse_stop;

// ----------------------------------------------------------------
// Delay Line Implementation
// ----------------------------------------------------------------

generate
    if (DELAY_TYPE == "LUT") begin : gen_lut_delays
        // LUT-based delay elements
        genvar i;
        for (i = 0; i < NUM_TAPS; i = i + 1) begin : delay_elements
            // LUT delay element for start pulse
            (* DONT_TOUCH = "TRUE", KEEP = "TRUE" *)
            LUT1 #(
                .INIT(2'b10)
            ) lut_delay_start (
                .O(delay_chain_start[i+1]),
                .I0(delay_chain_start[i])
            );

            // LUT delay element for stop pulse
            (* DONT_TOUCH = "TRUE", KEEP = "TRUE" *)
            LUT1 #(
                .INIT(2'b10)
            ) lut_delay_stop (
                .O(delay_chain_stop[i+1]),
                .I0(delay_chain_stop[i])
            );
        end
    end else begin : gen_carry_delays
        // CARRY4-based delay elements (recommended for better uniformity)
        // Each CARRY4 provides 4 taps, so we need NUM_TAPS/4 CARRY4 primitives
        localparam NUM_CARRY4 = NUM_TAPS / 4;

        // Intermediate signals for carry chain
        wire [NUM_CARRY4:0] carry_chain_start;
        wire [NUM_CARRY4:0] carry_chain_stop;
        wire [NUM_CARRY4*4-1:0] carry_out_start;
        wire [NUM_CARRY4*4-1:0] carry_out_stop;

        // First CARRY4 gets input signal
        assign carry_chain_start[0] = input_pulse_start;
        assign carry_chain_stop[0] = input_pulse_stop;

        genvar c;
        for (c = 0; c < NUM_CARRY4; c = c + 1) begin : carry_chain
            // CARRY4 for start delay line
            (* DONT_TOUCH = "TRUE", KEEP = "TRUE" *)
            CARRY4 carry_start (
                .CO({carry_chain_start[c+1], carry_out_start[c*4+2], carry_out_start[c*4+1], carry_out_start[c*4]}),
                .O(),  // Sum outputs unused, we use carry outputs
                .CI(carry_chain_start[c]),
                .CYINIT(1'b0),
                .DI(4'b0000),
                .S(4'b1111)  // All select bits high = propagate carry
            );

            // CARRY4 for stop delay line
            (* DONT_TOUCH = "TRUE", KEEP = "TRUE" *)
            CARRY4 carry_stop (
                .CO({carry_chain_stop[c+1], carry_out_stop[c*4+2], carry_out_stop[c*4+1], carry_out_stop[c*4]}),
                .O(),
                .CI(carry_chain_stop[c]),
                .CYINIT(1'b0),
                .DI(4'b0000),
                .S(4'b1111)
            );

            // Map carry outputs to delay chain
            assign delay_chain_start[c*4+1] = carry_out_start[c*4];
            assign delay_chain_start[c*4+2] = carry_out_start[c*4+1];
            assign delay_chain_start[c*4+3] = carry_out_start[c*4+2];
            assign delay_chain_start[c*4+4] = carry_chain_start[c+1];

            assign delay_chain_stop[c*4+1] = carry_out_stop[c*4];
            assign delay_chain_stop[c*4+2] = carry_out_stop[c*4+1];
            assign delay_chain_stop[c*4+3] = carry_out_stop[c*4+2];
            assign delay_chain_stop[c*4+4] = carry_chain_stop[c+1];
        end
    end
endgenerate

// ----------------------------------------------------------------
// Rising Clock Edge Capture
// ----------------------------------------------------------------
// This is the core TDC measurement principle:
// - Signal propagates through delay line asynchronously
// - At each rising clock edge, we sample the delay line state
// - The captured thermometer code shows how far the signal traveled
// - Fine time = tap position where signal stopped (1->0 transition)

// Stage 1: Direct capture from delay line on rising clock edge
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        captured_start_stage1 <= {NUM_TAPS{1'b0}};
        captured_stop_stage1 <= {NUM_TAPS{1'b0}};
    end else begin
        // Capture delay line state on EVERY rising clock edge
        captured_start_stage1 <= delay_chain_start[NUM_TAPS:1];
        captured_stop_stage1 <= delay_chain_stop[NUM_TAPS:1];
    end
end

// Stage 2: Synchronization/pipeline stage for timing closure
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        captured_start_stage2 <= {NUM_TAPS{1'b0}};
        captured_stop_stage2 <= {NUM_TAPS{1'b0}};
    end else begin
        captured_start_stage2 <= captured_start_stage1;
        captured_stop_stage2 <= captured_stop_stage1;
    end
end

// ----------------------------------------------------------------
// Bubble Suppression
// ----------------------------------------------------------------

sp701_bubble_suppressor #(
    .WIDTH(NUM_TAPS)
) bubble_fix_start (
    .thermometer_in(captured_start_stage2),
    .thermometer_out(start_bubble_corrected)
);

sp701_bubble_suppressor #(
    .WIDTH(NUM_TAPS)
) bubble_fix_stop (
    .thermometer_in(captured_stop_stage2),
    .thermometer_out(stop_bubble_corrected)
);

// Output assignment
assign delay_line_out_start = start_bubble_corrected;
assign delay_line_out_stop = stop_bubble_corrected;

// ----------------------------------------------------------------
// Thermometer Encoding
// ----------------------------------------------------------------

sp701_thermometer_encoder #(
    .INPUT_WIDTH(NUM_TAPS),
    .OUTPUT_WIDTH(8)
) encoder_start (
    .thermometer_in(start_bubble_corrected),
    .binary_out(encoded_start),
    .valid(start_valid)
);

sp701_thermometer_encoder #(
    .INPUT_WIDTH(NUM_TAPS),
    .OUTPUT_WIDTH(8)
) encoder_stop (
    .thermometer_in(stop_bubble_corrected),
    .binary_out(encoded_stop),
    .valid(stop_valid)
);

endmodule

/**
 * ================================================================
 * Bubble Suppressor Module
 * Description: Removes spurious transitions in thermometer code
 * using 3-tap majority voting
 * ================================================================
 */
module sp701_bubble_suppressor #(
    parameter WIDTH = 256
) (
    input  wire [WIDTH-1:0] thermometer_in,
    output wire [WIDTH-1:0] thermometer_out
);

genvar i;
generate
    // First bit - use 2-tap voting with next bit
    assign thermometer_out[0] = (thermometer_in[0] & thermometer_in[1]) |
                                 (thermometer_in[0] & thermometer_in[0]);

    // Middle bits - 3-tap majority voting
    for (i = 1; i < WIDTH-1; i = i + 1) begin : majority_vote
        // Majority of 3: output 1 if 2 or more inputs are 1
        assign thermometer_out[i] = (thermometer_in[i-1] & thermometer_in[i]) |
                                    (thermometer_in[i] & thermometer_in[i+1]) |
                                    (thermometer_in[i-1] & thermometer_in[i+1]);
    end

    // Last bit - use 2-tap voting with previous bit
    assign thermometer_out[WIDTH-1] = (thermometer_in[WIDTH-1] & thermometer_in[WIDTH-2]) |
                                       (thermometer_in[WIDTH-1] & thermometer_in[WIDTH-1]);
endgenerate

endmodule

/**
 * ================================================================
 * SP701 TDC Thermometer to Binary Encoder
 * Description: Converts thermometer code to binary by finding
 * the 1->0 transition point (where propagating edge stopped)
 * ================================================================
 */
module sp701_thermometer_encoder #(
    parameter INPUT_WIDTH = 256,
    parameter OUTPUT_WIDTH = 8
) (
    input  wire [INPUT_WIDTH-1:0] thermometer_in,
    output reg  [OUTPUT_WIDTH-1:0] binary_out,
    output reg  valid
);

integer i;
reg found_transition;
reg [OUTPUT_WIDTH-1:0] transition_point;

always @(*) begin
    binary_out = {OUTPUT_WIDTH{1'b0}};
    valid = 1'b0;
    found_transition = 1'b0;
    transition_point = {OUTPUT_WIDTH{1'b0}};

    // Special case: all zeros means no signal propagated
    if (thermometer_in == {INPUT_WIDTH{1'b0}}) begin
        binary_out = {OUTPUT_WIDTH{1'b0}};
        valid = 1'b0;
    end
    // Special case: all ones means signal propagated through entire chain
    else if (thermometer_in == {INPUT_WIDTH{1'b1}}) begin
        binary_out = INPUT_WIDTH[OUTPUT_WIDTH-1:0] - 1'b1;
        valid = 1'b1;
    end
    else begin
        // Find the transition point: last 1 before first 0
        for (i = 0; i < INPUT_WIDTH-1; i = i + 1) begin
            if (!found_transition) begin
                if (thermometer_in[i] && !thermometer_in[i+1]) begin
                    transition_point = i[OUTPUT_WIDTH-1:0];
                    found_transition = 1'b1;
                end
            end
        end

        // Check last bit
        if (!found_transition && thermometer_in[INPUT_WIDTH-1]) begin
            transition_point = (INPUT_WIDTH-1);
            found_transition = 1'b1;
        end

        binary_out = transition_point;
        valid = found_transition;
    end
end

endmodule
