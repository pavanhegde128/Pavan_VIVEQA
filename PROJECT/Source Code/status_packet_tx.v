`timescale 1ns / 1ps
// ============================================================
// status_packet_tx.v
// Builds an ASCII line:
//   S,<state>,<mode>,<runtime>,<start>,<stop>,<fault>,<maxrt>,<disp>,<relay>\n
// and streams it out through uart_tx once per `send_tick` pulse.
//
// <mode> is the active 7-segment display mode (0-5, matching the
// sw3/4/5 mode mux in machine_runtime_monitor): 0=runtime(sec),
// 1=start_count, 2=stop_count, 3=fault_count, 4=state, 5=max_runtime(sec).
// Sent so downstream consumers (e.g. the ESP32/Blynk bridge) can tell
// whether <disp> is a time value (modes 0/5, in seconds) or a plain
// count/state (modes 1-4), instead of guessing.
//
// <disp> is the exact 16-bit value currently shown on the
// 7-segment display (machine_runtime_monitor's display_data,
// the same value bin_to_bcd/max7219_driver render) - so the
// Blynk app can mirror whatever digits are on the board.
//
// <relay> is the FPGA's actual relay output pin (1 = energized,
// 0 = de-energized), sent as a single '0'/'1' character so the
// Blynk dashboard's LED mirrors the real hardware signal directly
// rather than being inferred from state.
//
// Each counter is sent as decimal ASCII with leading zeros
// suppressed (always at least 1 digit). Implemented as a single
// flat FSM - one numeric field is sent via a shared digit-index
// counter that is reloaded with a new source field at each comma.
// ============================================================
module status_packet_tx (
    input  wire        clk,
    input  wire        rst,

    input  wire        send_tick,      // pulse: start sending one packet
    input  wire [1:0]  state_in,
    input  wire [2:0]  mode_in,        // NEW: active display mode (0-5)
    input  wire [31:0] runtime_in,
    input  wire [31:0] start_in,
    input  wire [31:0] stop_in,
    input  wire [31:0] fault_in,
    input  wire [31:0] maxrt_in,
    input  wire [15:0] disp_in,        // live 7-segment value
    input  wire        relay_in,       // actual relay output state

    output wire        tx              // serial out pin
);

    // ---- byte-level UART transmitter ----
    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(.CLK_FREQ(24_000_000), .BAUD_RATE(9600)) u_tx (
        .clk      (clk),
        .rst      (rst),
        .tx_data  (tx_byte),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .tx       (tx)
    );

    // ---- decimal digit extraction: binary -> 10 BCD-like nibbles ----
    // digits_buf[0] = most significant digit ... digits_buf[9] = least significant
    function [39:0] bin_to_digits; // returns 10 packed 4-bit digits, MSB digit first
        input [31:0] value;
        integer i;
        reg [31:0] v;
        reg [3:0]  d [0:9];
        begin
            v = value;
            for (i = 9; i >= 0; i = i - 1) begin
                d[i] = v % 10;
                v    = v / 10;
            end
            bin_to_digits = {d[0], d[1], d[2], d[3], d[4],
                              d[5], d[6], d[7], d[8], d[9]};
        end
    endfunction

    // latched digit buffers for the field currently/about to be sent
    reg [39:0] digits_runtime, digits_start, digits_stop, digits_fault, digits_maxrt, digits_disp;
    reg [39:0] active_digits;     // field currently being streamed
    reg [3:0]  digit_pos;         // 0 = MSB digit ... 9 = LSB digit
    reg        leading_zero;      // still suppressing leading zeros

    // pulls digit at position `digit_pos` out of active_digits (MSB-first packing)
    wire [3:0] cur_digit = (digit_pos == 4'd0) ? active_digits[39:36] :
                           (digit_pos == 4'd1) ? active_digits[35:32] :
                           (digit_pos == 4'd2) ? active_digits[31:28] :
                           (digit_pos == 4'd3) ? active_digits[27:24] :
                           (digit_pos == 4'd4) ? active_digits[23:20] :
                           (digit_pos == 4'd5) ? active_digits[19:16] :
                           (digit_pos == 4'd6) ? active_digits[15:12] :
                           (digit_pos == 4'd7) ? active_digits[11:8]  :
                           (digit_pos == 4'd8) ? active_digits[7:4]   :
                                                  active_digits[3:0];

    // ---- FSM states ----
    localparam ST_IDLE    = 5'd0;
    localparam ST_S       = 5'd1;
    localparam ST_C1      = 5'd2;
    localparam ST_STATE   = 5'd3;
    localparam ST_CM      = 5'd26;  // NEW: comma after <state>, before <mode>
    localparam ST_MODE    = 5'd27;  // NEW: sends <mode> digit
    localparam ST_C2      = 5'd4;
    localparam ST_LOAD_RT = 5'd5;
    localparam ST_NUM_RT  = 5'd6;
    localparam ST_C3      = 5'd7;
    localparam ST_LOAD_SC = 5'd8;
    localparam ST_NUM_SC  = 5'd9;
    localparam ST_C4      = 5'd10;
    localparam ST_LOAD_SP = 5'd11;
    localparam ST_NUM_SP  = 5'd12;
    localparam ST_C5      = 5'd13;
    localparam ST_LOAD_FL = 5'd14;
    localparam ST_NUM_FL  = 5'd15;
    localparam ST_C6      = 5'd16;
    localparam ST_LOAD_MX = 5'd17;
    localparam ST_NUM_MX  = 5'd18;
    localparam ST_C7      = 5'd19;
    localparam ST_LOAD_DP = 5'd20;
    localparam ST_NUM_DP  = 5'd21;
    localparam ST_C8      = 5'd22;
    localparam ST_RELAY   = 5'd23;
    localparam ST_NL      = 5'd24;
    localparam ST_DONE    = 5'd25;

    reg [4:0] pstate;
    reg [4:0] next_after_num; // which comma-state to go to after a numeric field

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pstate   <= ST_IDLE;
            tx_start <= 1'b0;
            tx_byte  <= 8'h00;
            digit_pos <= 4'd0;
            leading_zero <= 1'b1;
        end
        else begin
            tx_start <= 1'b0; // default: only pulse high for exactly 1 cycle

            case (pstate)

            // -------------------------------------------------
            ST_IDLE: begin
                if (send_tick && !tx_busy) begin
                    digits_runtime <= bin_to_digits(runtime_in);
                    digits_start   <= bin_to_digits(start_in);
                    digits_stop    <= bin_to_digits(stop_in);
                    digits_fault   <= bin_to_digits(fault_in);
                    digits_maxrt   <= bin_to_digits(maxrt_in);
                    digits_disp    <= bin_to_digits({16'd0, disp_in});
                    pstate <= ST_S;
                end
            end

            // -------------------------------------------------
            ST_S: begin
                if (!tx_busy) begin
                    tx_byte  <= "S";
                    tx_start <= 1'b1;
                    pstate   <= ST_C1;
                end
            end

            ST_C1: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_STATE;
            end

            ST_STATE: if (!tx_busy) begin
                tx_byte  <= 8'h30 + {6'd0, state_in}; // '0'..'3'
                tx_start <= 1'b1;
                pstate   <= ST_CM;
            end

            // ---- comma, then the active display-mode digit (0-5) ----
            ST_CM: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_MODE;
            end

            ST_MODE: if (!tx_busy) begin
                tx_byte  <= 8'h30 + {5'd0, mode_in}; // '0'..'5'
                tx_start <= 1'b1;
                pstate   <= ST_C2;
            end

            ST_C2: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_RT;
            end

            // ---- runtime_count field ----
            ST_LOAD_RT: begin
                active_digits  <= digits_runtime;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C3;
                pstate         <= ST_NUM_RT;
            end

            ST_NUM_RT: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1; // skip leading zero, no tx
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C3: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_SC;
            end

            // ---- start_count field ----
            ST_LOAD_SC: begin
                active_digits  <= digits_start;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C4;
                pstate         <= ST_NUM_SC;
            end

            ST_NUM_SC: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1;
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C4: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_SP;
            end

            // ---- stop_count field ----
            ST_LOAD_SP: begin
                active_digits  <= digits_stop;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C5;
                pstate         <= ST_NUM_SP;
            end

            ST_NUM_SP: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1;
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C5: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_FL;
            end

            // ---- fault_count field ----
            ST_LOAD_FL: begin
                active_digits  <= digits_fault;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C6;
                pstate         <= ST_NUM_FL;
            end

            ST_NUM_FL: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1;
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C6: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_MX;
            end

            // ---- max_runtime field ----
            ST_LOAD_MX: begin
                active_digits  <= digits_maxrt;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C7;
                pstate         <= ST_NUM_MX;
            end

            ST_NUM_MX: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1;
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C7: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_LOAD_DP;
            end

            // ---- live 7-segment display value field ----
            ST_LOAD_DP: begin
                active_digits  <= digits_disp;
                digit_pos      <= 4'd0;
                leading_zero   <= 1'b1;
                next_after_num <= ST_C8;
                pstate         <= ST_NUM_DP;
            end

            ST_NUM_DP: if (!tx_busy) begin
                if (leading_zero && cur_digit == 4'd0 && digit_pos != 4'd9) begin
                    digit_pos <= digit_pos + 1'd1;
                end
                else begin
                    leading_zero <= 1'b0;
                    tx_byte  <= 8'h30 + cur_digit;
                    tx_start <= 1'b1;
                    if (digit_pos == 4'd9) pstate <= next_after_num;
                    else                   digit_pos <= digit_pos + 1'd1;
                end
            end

            ST_C8: if (!tx_busy) begin
                tx_byte  <= ",";
                tx_start <= 1'b1;
                pstate   <= ST_RELAY;
            end

            // ---- live relay output bit (single '0'/'1' character) ----
            ST_RELAY: if (!tx_busy) begin
                tx_byte  <= relay_in ? "1" : "0";
                tx_start <= 1'b1;
                pstate   <= ST_NL;
            end

            ST_NL: if (!tx_busy) begin
                tx_byte  <= "\n";
                tx_start <= 1'b1;
                pstate   <= ST_DONE;
            end

            ST_DONE: if (!tx_busy) begin
                pstate <= ST_IDLE;
            end

            default: pstate <= ST_IDLE;

            endcase
        end
    end

endmodule
