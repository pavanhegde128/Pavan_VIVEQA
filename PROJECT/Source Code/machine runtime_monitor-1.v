`timescale 1ns / 1ps
// ============================================================
// machine_runtime_monitor.v   (core, with added observation outputs)
// State machine logic is IDENTICAL to your original module.
// Only addition: state_out / runtime_out / start_out / stop_out /
// fault_out / maxrt_out ports so the top level can stream them
// to the ESP32 without reaching into internal regs hierarchically.
// ============================================================
module machine_runtime_monitor (
    input  wire        clk_24mhz,
    input  wire        rst,

    input  wire        sw0_start,
    input  wire        sw1_stop,
    input  wire        sw2_fault,

    input  wire        sw3_mode0,
    input  wire        sw4_mode1,
    input  wire        sw5_mode2,

    output wire        relay,
    output wire        buzzer,
    output reg  [15:0] display_data,

    // ---- new observation outputs (added, logic unchanged) ----
    output wire [1:0]  state_out,
    output wire [31:0] runtime_out,
    output wire [31:0] start_out,
    output wire [31:0] stop_out,
    output wire [31:0] fault_out,
    output wire [31:0] maxrt_out,
    output wire [2:0]  mode_out    // NEW: active display mode (sw5,sw4,sw3), so
                                    // downstream (status_packet_tx -> ESP32) can
                                    // tell whether disp_in is a time value (modes
                                    // 0/5 = runtime/max_runtime, in seconds) or a
                                    // plain count/state (modes 1/2/3/4)
);

    localparam IDLE    = 2'b00;
    localparam RUNNING = 2'b01;
    localparam FAULT   = 2'b10;
    localparam STOPPED = 2'b11;

    reg [1:0]  state;

    reg [31:0] runtime_count;
    reg [31:0] current_run;
    reg [31:0] start_count;
    reg [31:0] stop_count;
    reg [31:0] fault_count;
    reg [31:0] max_runtime;

    // ---- expose internal regs without touching logic below ----
    assign state_out   = state;
    assign runtime_out  = runtime_count;
    assign start_out    = start_count;
    assign stop_out     = stop_count;
    assign fault_out     = fault_count;
    assign maxrt_out     = max_runtime;

    // ---- 1-second tick ----
    reg [24:0] sec_div;
    wire one_sec = (sec_div == 25'd23_999_999);

    always @(posedge clk_24mhz or posedge rst) begin
        if (rst)          sec_div <= 25'd0;
        else if (one_sec) sec_div <= 25'd0;
        else              sec_div <= sec_div + 1'd1;
    end

    // ---- Buzzer tone generator (~732 Hz) ----
    reg [21:0] buzzer_counter;

    always @(posedge clk_24mhz or posedge rst) begin
        if (rst) buzzer_counter <= 22'd0;
        else     buzzer_counter <= buzzer_counter + 1'd1;
    end

    assign buzzer = (state == FAULT) ? buzzer_counter[13] : 1'b0;
    assign relay  = (state == RUNNING);

    // ---- State machine (unchanged) ----
    always @(posedge clk_24mhz or posedge rst) begin
        if (rst) begin
            state         <= IDLE;
            runtime_count <= 32'd0;
            current_run   <= 32'd0;
            start_count   <= 32'd0;
            stop_count    <= 32'd0;
            fault_count   <= 32'd0;
            max_runtime   <= 32'd0;
        end
        else begin
            case (state)

            IDLE: begin
                if (sw2_fault) begin
                    state       <= FAULT;
                    fault_count <= fault_count + 1'd1;
                end
                else if (sw0_start) begin
                    state       <= RUNNING;
                    start_count <= start_count + 1'd1;
                    current_run <= 32'd0;
                end
            end

            RUNNING: begin
                if (one_sec) begin
                    runtime_count <= runtime_count + 1'd1;
                    current_run   <= current_run   + 1'd1;
                end
                if (sw2_fault) begin
                    state <= FAULT;
                    if (current_run > max_runtime)
                        max_runtime <= current_run;
                    fault_count <= fault_count + 1'd1;
                end
                else if (sw1_stop) begin
                    state <= STOPPED;
                    if (current_run > max_runtime)
                        max_runtime <= current_run;
                    stop_count <= stop_count + 1'd1;
                end
            end

            FAULT: begin
                if (!sw2_fault)
                    state <= IDLE;
            end

            STOPPED: begin
                if (sw2_fault) begin
                    state       <= FAULT;
                    fault_count <= fault_count + 1'd1;
                end
                else if (sw0_start) begin
                    state       <= RUNNING;
                    start_count <= start_count + 1'd1;
                    current_run <= 32'd0;
                end
            end

            endcase
        end
    end

    // ---- Display mux (unchanged) ----
    wire [2:0] mode = {sw5_mode2, sw4_mode1, sw3_mode0};
    assign mode_out = mode;

    always @(*) begin
        case (mode)
            3'b000:  display_data = runtime_count[15:0];
            3'b001:  display_data = start_count[15:0];
            3'b010:  display_data = stop_count[15:0];
            3'b011:  display_data = fault_count[15:0];
            3'b100:  display_data = {14'd0, state};
            3'b101:  display_data = max_runtime[15:0];
            default: display_data = 16'h0000;
        endcase
    end

endmodule
