/**
 * ================================================================
 * SP701 TDC Tapped Delay Line Module - NO CALIBRATION VERSION
 * Target: Xilinx Spartan-7 XC7S100 (SP701 Evaluation Board)
 * Clock:  100 MHz (10ns period), 256 CARRY4 taps (~39ps/tap)
 * ================================================================
 *
 * This file contains 3 modules:
 *   1. sp701_tdc_delay_line      - Main delay line with pipeline
 *   2. sp701_bubble_suppressor   - 3-tap majority vote filter
 *   3. sp701_thermometer_encoder - Tree-based 256→8 encoder
 *
 * ARCHITECTURE OVERVIEW:
 *
 *  input_pulse_start ─┐    input_pulse_stop ─┐
 *                     v                       v
 *  ┌──────────────────────────────────────────────────────────┐
 *  │           CARRY4 DELAY CHAINS (256 taps each)            │
 *  │  [0]──[1]──[2]──...──[255]    [0]──[1]──[2]──...──[255] │
 *  │   Each CARRY4 = 4 taps, so 64 CARRY4 primitives/chain   │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │ 256 bits              │ 256 bits
 *                 v                       v
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  STAGE 1: Capture Register (ASYNC_REG)                   │
 *  │  Samples all 256 taps on rising clock edge               │
 *  │  Result: thermometer code  111...110...000                │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  STAGE 2: Sync Register                                  │
 *  │  Second flip-flop stage for metastability protection     │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  BUBBLE SUPPRESSOR (combinational)                       │
 *  │  3-tap majority vote on each bit:                        │
 *  │    out[i] = maj(in[i-1], in[i], in[i+1])                │
 *  │  Boundaries: out[0]=in[0]&in[1], out[N-1]=in[N-2]&in[N-1]│
 *  │  Fixes: 1110111 → 1111111  and  1110011 → 1110011       │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  STAGE 3: Bubble Register                                │
 *  │  Pipeline break between bubble suppressor and encoder    │
 *  │  (This was the critical path before: 26 LUT levels)      │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  THERMOMETER ENCODER (combinational, tree-based)         │
 *  │  256 bits → 8-bit binary tap position                    │
 *  │  Split into 16 groups of 16 bits:                        │
 *  │    - Each group: find local 1→0 transition (parallel)    │
 *  │    - Group select: priority chain (16 entries, ~4 LUTs)  │
 *  │    - MUX: pick position from selected group              │
 *  │  Output: {group[3:0], position[3:0]} = tap 0..255        │
 *  │  Max LUT depth: ~9 levels (fits in 10ns easily)          │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *  ┌──────────────────────────────────────────────────────────┐
 *  │  STAGE 4: Encoder Register                               │
 *  │  Registered 8-bit output for clean timing                │
 *  └──────────────┬───────────────────────┬───────────────────┘
 *                 │                       │
 *                 v                       v
 *           encoded_start [7:0]     encoded_stop [7:0]
 *           start_valid             stop_valid
 *
 * LATENCY: 4 clock cycles from capture to encoded output.
 * Core module compensates with 2-cycle edge delay (stages 3+4).
 *
 * THERMOMETER CODE EXAMPLE (256 taps, pulse arrived at tap 42):
 *   in:  111...1 (taps 0-41) 000...0 (taps 42-255)
 *   Encoder reads: group 2 (taps 32-47) has transition at bit 9
 *   Output: {4'd2, 4'd9} = 8'd41  (last 1 position)
 *
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

// Bubble-corrected outputs (combinational from suppressor)
wire [NUM_TAPS-1:0] start_bubble_corrected;
wire [NUM_TAPS-1:0] stop_bubble_corrected;

// Pipeline stage 3: Registered bubble-corrected data
reg [NUM_TAPS-1:0] start_bubble_reg;
reg [NUM_TAPS-1:0] stop_bubble_reg;

// Encoder outputs (combinational)
wire [7:0] encoded_start_comb;
wire [7:0] encoded_stop_comb;
wire start_valid_comb;
wire stop_valid_comb;

// Pipeline stage 4: Registered encoder outputs
reg [7:0] encoded_start_reg;
reg [7:0] encoded_stop_reg;
reg start_valid_reg;
reg stop_valid_reg;

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
// Rising Clock Edge Capture (Stage 1 & 2)
// ----------------------------------------------------------------

// Stage 1: Direct capture from delay line on rising clock edge
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        captured_start_stage1 <= {NUM_TAPS{1'b0}};
        captured_stop_stage1 <= {NUM_TAPS{1'b0}};
    end else begin
        captured_start_stage1 <= delay_chain_start[NUM_TAPS:1];
        captured_stop_stage1 <= delay_chain_stop[NUM_TAPS:1];
    end
end

// Stage 2: Synchronization/pipeline stage
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
// Bubble Suppression (Combinational)
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

// ----------------------------------------------------------------
// Pipeline Stage 3: Register Bubble-Corrected Data
// ----------------------------------------------------------------
// This breaks the critical path between bubble suppressor and encoder

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_bubble_reg <= {NUM_TAPS{1'b0}};
        stop_bubble_reg <= {NUM_TAPS{1'b0}};
    end else begin
        start_bubble_reg <= start_bubble_corrected;
        stop_bubble_reg <= stop_bubble_corrected;
    end
end

// Output assignment (from registered bubble data)
assign delay_line_out_start = start_bubble_reg;
assign delay_line_out_stop = stop_bubble_reg;

// ----------------------------------------------------------------
// Thermometer Encoding (Combinational - Tree-Based)
// ----------------------------------------------------------------

sp701_thermometer_encoder #(
    .INPUT_WIDTH(NUM_TAPS),
    .OUTPUT_WIDTH(8)
) encoder_start (
    .thermometer_in(start_bubble_reg),
    .binary_out(encoded_start_comb),
    .valid(start_valid_comb)
);

sp701_thermometer_encoder #(
    .INPUT_WIDTH(NUM_TAPS),
    .OUTPUT_WIDTH(8)
) encoder_stop (
    .thermometer_in(stop_bubble_reg),
    .binary_out(encoded_stop_comb),
    .valid(stop_valid_comb)
);

// ----------------------------------------------------------------
// Pipeline Stage 4: Register Encoder Outputs
// ----------------------------------------------------------------

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        encoded_start_reg <= 8'b0;
        encoded_stop_reg <= 8'b0;
        start_valid_reg <= 1'b0;
        stop_valid_reg <= 1'b0;
    end else begin
        encoded_start_reg <= encoded_start_comb;
        encoded_stop_reg <= encoded_stop_comb;
        start_valid_reg <= start_valid_comb;
        stop_valid_reg <= stop_valid_comb;
    end
end

assign encoded_start = encoded_start_reg;
assign encoded_stop = encoded_stop_reg;
assign start_valid = start_valid_reg;
assign stop_valid = stop_valid_reg;

endmodule

/**
 * ================================================================
 * Bubble Suppressor Module
 * ================================================================
 * Removes spurious 0-bubbles in thermometer code using 3-tap
 * majority voting. A "bubble" is a stray 0 in a run of 1s, caused
 * by metastability or routing skew in the CARRY4 chain.
 *
 * Example: 1111_0111_1100 → 1111_1111_1100 (bubble at bit 4 fixed)
 *
 * Algorithm per bit:
 *   out[i] = majority(in[i-1], in[i], in[i+1])
 *          = (A&B) | (B&C) | (A&C)
 *
 * Boundary handling:
 *   out[0]   = in[0] & in[1]          (assume 0 below bit 0)
 *   out[N-1] = in[N-2] & in[N-1]      (assume 0 above bit N-1)
 *
 * Combinational only (~2-3 LUT levels). Registered in stage 3.
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
    // First bit - majority vote with implicit 0 below: majority(0, in[0], in[1])
    assign thermometer_out[0] = thermometer_in[0] & thermometer_in[1];

    // Middle bits - 3-tap majority voting
    for (i = 1; i < WIDTH-1; i = i + 1) begin : majority_vote
        // Majority of 3: output 1 if 2 or more inputs are 1
        assign thermometer_out[i] = (thermometer_in[i-1] & thermometer_in[i]) |
                                    (thermometer_in[i] & thermometer_in[i+1]) |
                                    (thermometer_in[i-1] & thermometer_in[i+1]);
    end

    // Last bit - majority vote with implicit 0 above: majority(in[W-2], in[W-1], 0)
    assign thermometer_out[WIDTH-1] = thermometer_in[WIDTH-2] & thermometer_in[WIDTH-1];
endgenerate

endmodule

/**
 * ================================================================
 * SP701 TDC Thermometer to Binary Encoder - TREE-BASED
 * ================================================================
 * Converts 256-bit thermometer code to 8-bit binary tap position.
 * Finds the 1→0 transition point (last '1' in the thermometer).
 *
 *  Input thermometer:  1111...1100...0000  (256 bits)
 *                              ^
 *                      transition point = tap position
 *
 * ARCHITECTURE (hierarchical, 16 groups × 16 bits):
 *
 *   thermometer_in [255:0]
 *   ├── Group 0  [15:0]   → has_transition? position?  ─┐
 *   ├── Group 1  [31:16]  → has_transition? position?   │
 *   ├── Group 2  [47:32]  → has_transition? position?   │
 *   ├── ...                                              ├→ Priority
 *   ├── Group 14 [239:224]→ has_transition? position?   │   Select
 *   └── Group 15 [255:240]→ has_transition? position?  ─┘
 *                                                         │
 *                           selected_group [3:0] ←────────┘
 *                           selected_position [3:0] ←── MUX
 *                                    │
 *                                    v
 *                     binary_out = {group, position} [7:0]
 *
 * GROUP SELECTION PRIORITY:
 *   1st: Lowest group with internal transition (mixed 1s and 0s)
 *   2nd: Highest all-ones group (transition at group boundary)
 *
 * TIMING: ~9 LUT levels max (was 26 with linear scan).
 * Combinational only. Registered in pipeline stage 4.
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

// Tree parameters: 16 groups of 16 bits
localparam NUM_GROUPS = 16;
localparam GROUP_SIZE = INPUT_WIDTH / NUM_GROUPS;  // = 16

// Per-group signals
wire [NUM_GROUPS-1:0] group_has_transition;  // Does this group contain a 1->0 transition?
wire [NUM_GROUPS-1:0] group_has_ones;        // Does this group have any 1s?
wire [3:0] group_position [0:NUM_GROUPS-1];  // Local transition position within each group

// Group analysis: find transition within each 16-bit group
genvar g, b;
generate
    for (g = 0; g < NUM_GROUPS; g = g + 1) begin : group_encode
        wire [GROUP_SIZE-1:0] group_bits;
        assign group_bits = thermometer_in[g*GROUP_SIZE +: GROUP_SIZE];

        // Check if group has any 1s
        assign group_has_ones[g] = |group_bits;

        // A group has the transition if it contains both 1s and 0s,
        // OR it's all 1s but the next group starts with 0
        // Simplified: transition is at the boundary between last 1-group and first 0-group,
        // or within a group that has mixed 1s and 0s
        wire group_all_ones;
        assign group_all_ones = &group_bits;

        // Group has internal transition if it has ones but isn't all ones
        assign group_has_transition[g] = group_has_ones[g] & ~group_all_ones;

        // Find local transition point within this group (first 1->0 from LSB)
        // Use small priority encoder for 16 bits
        reg [3:0] local_pos;
        reg local_found;
        integer idx;

        always @(*) begin
            local_pos = 4'b0;
            local_found = 1'b0;
            for (idx = 0; idx < GROUP_SIZE-1; idx = idx + 1) begin
                if (!local_found && group_bits[idx] && !group_bits[idx+1]) begin
                    local_pos = idx[3:0];
                    local_found = 1'b1;
                end
            end
            // Check if last bit is 1 (transition at group boundary)
            if (!local_found && group_bits[GROUP_SIZE-1]) begin
                local_pos = (GROUP_SIZE - 1);
                local_found = 1'b1;
            end
        end

        assign group_position[g] = local_pos;
    end
endgenerate

// Group selection: find the lowest group that has an internal transition.
// If no group has an internal transition, find the highest all-ones group
// (the transition is at the boundary between last all-ones group and next group).
reg [3:0] selected_group;
reg [3:0] selected_position;
reg found;

always @(*) begin
    selected_group = 4'b0;
    selected_position = 4'b0;
    found = 1'b0;

    // Priority: find lowest group with internal transition
    if (group_has_transition[0])       begin selected_group = 4'd0;  found = 1'b1; end
    else if (group_has_transition[1])  begin selected_group = 4'd1;  found = 1'b1; end
    else if (group_has_transition[2])  begin selected_group = 4'd2;  found = 1'b1; end
    else if (group_has_transition[3])  begin selected_group = 4'd3;  found = 1'b1; end
    else if (group_has_transition[4])  begin selected_group = 4'd4;  found = 1'b1; end
    else if (group_has_transition[5])  begin selected_group = 4'd5;  found = 1'b1; end
    else if (group_has_transition[6])  begin selected_group = 4'd6;  found = 1'b1; end
    else if (group_has_transition[7])  begin selected_group = 4'd7;  found = 1'b1; end
    else if (group_has_transition[8])  begin selected_group = 4'd8;  found = 1'b1; end
    else if (group_has_transition[9])  begin selected_group = 4'd9;  found = 1'b1; end
    else if (group_has_transition[10]) begin selected_group = 4'd10; found = 1'b1; end
    else if (group_has_transition[11]) begin selected_group = 4'd11; found = 1'b1; end
    else if (group_has_transition[12]) begin selected_group = 4'd12; found = 1'b1; end
    else if (group_has_transition[13]) begin selected_group = 4'd13; found = 1'b1; end
    else if (group_has_transition[14]) begin selected_group = 4'd14; found = 1'b1; end
    else if (group_has_transition[15]) begin selected_group = 4'd15; found = 1'b1; end

    // If no internal transition found, check for all-ones boundary case
    // The transition is at the last bit of the highest all-ones group
    if (!found) begin
        if      (group_has_ones[15]) begin selected_group = 4'd15; found = 1'b1; end
        else if (group_has_ones[14]) begin selected_group = 4'd14; found = 1'b1; end
        else if (group_has_ones[13]) begin selected_group = 4'd13; found = 1'b1; end
        else if (group_has_ones[12]) begin selected_group = 4'd12; found = 1'b1; end
        else if (group_has_ones[11]) begin selected_group = 4'd11; found = 1'b1; end
        else if (group_has_ones[10]) begin selected_group = 4'd10; found = 1'b1; end
        else if (group_has_ones[9])  begin selected_group = 4'd9;  found = 1'b1; end
        else if (group_has_ones[8])  begin selected_group = 4'd8;  found = 1'b1; end
        else if (group_has_ones[7])  begin selected_group = 4'd7;  found = 1'b1; end
        else if (group_has_ones[6])  begin selected_group = 4'd6;  found = 1'b1; end
        else if (group_has_ones[5])  begin selected_group = 4'd5;  found = 1'b1; end
        else if (group_has_ones[4])  begin selected_group = 4'd4;  found = 1'b1; end
        else if (group_has_ones[3])  begin selected_group = 4'd3;  found = 1'b1; end
        else if (group_has_ones[2])  begin selected_group = 4'd2;  found = 1'b1; end
        else if (group_has_ones[1])  begin selected_group = 4'd1;  found = 1'b1; end
        else if (group_has_ones[0])  begin selected_group = 4'd0;  found = 1'b1; end
    end

    // MUX: select position from chosen group
    case (selected_group)
        4'd0:  selected_position = group_position[0];
        4'd1:  selected_position = group_position[1];
        4'd2:  selected_position = group_position[2];
        4'd3:  selected_position = group_position[3];
        4'd4:  selected_position = group_position[4];
        4'd5:  selected_position = group_position[5];
        4'd6:  selected_position = group_position[6];
        4'd7:  selected_position = group_position[7];
        4'd8:  selected_position = group_position[8];
        4'd9:  selected_position = group_position[9];
        4'd10: selected_position = group_position[10];
        4'd11: selected_position = group_position[11];
        4'd12: selected_position = group_position[12];
        4'd13: selected_position = group_position[13];
        4'd14: selected_position = group_position[14];
        4'd15: selected_position = group_position[15];
        default: selected_position = 4'b0;
    endcase

    // Final output: {group_index[3:0], local_position[3:0]}
    binary_out = {selected_group, selected_position};
    valid = found;
end

endmodule
