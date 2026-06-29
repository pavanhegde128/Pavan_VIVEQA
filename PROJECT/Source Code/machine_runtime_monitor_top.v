`timescale 1ns / 1ps
// ============================================================
// machine_runtime_monitor_top.v   (Blynk/ESP32-enabled version)
// Board : AT-STLN-ARTIX 7-001  (XC7A35T-FTG256-1)
// Clock : 24 MHz  (D13)
//
// Adds:
//   - PMOD IO_0 (T2) = UART TX  -> ESP32 RX   (status packets out)
//   - PMOD IO_1 (R3) = UART RX  <- ESP32 TX   (start/stop commands in)
//   Remote start/stop are OR'd with the existing physical switches,
//   so panel controls keep working exactly as before.
// ============================================================
module machine_runtime_monitor_top (
    input  wire clk_24mhz,
    input  wire rst,

    input  wire sw0_start,
    input  wire sw1_stop,
    input  wire sw2_fault,
    input  wire sw3_mode0,
    input  wire sw4_mode1,
    input  wire sw5_mode2,

    output wire relay,
    output wire buzzer,

    output wire seg_din,
    output wire seg_cs,
    output wire seg_clk,

    // ---- new: ESP32 link via PMOD ----
    output wire esp_uart_tx,   // PMOD IO_0 (T2) -> ESP32 RX
    input  wire esp_uart_rx    // PMOD IO_1 (R3) <- ESP32 TX
);

    // ====================================================
    // Remote command decode (from ESP32 / Blynk app)
    // ====================================================
    wire [7:0] cmd_rx_data;
    wire       cmd_rx_valid;
    wire       remote_start, remote_stop;
    wire       remote_fault, remote_mode0, remote_mode1, remote_mode2;
    wire       remote_reset;

    // The push-button reset is active-HIGH and feeds every register's
    // async reset in this design (see XDC: button "0", pin A13). A
    // remote reset command is OR'd onto the same net so "push button 0"
    // can be triggered from Blynk exactly like the physical button.
    wire rst_eff = rst | remote_reset;

    uart_rx #(.CLK_FREQ(24_000_000), .BAUD_RATE(9600)) u_cmd_rx (
        .clk      (clk_24mhz),
        .rst      (rst_eff),
        .rx       (esp_uart_rx),
        .rx_data  (cmd_rx_data),
        .rx_valid (cmd_rx_valid)
    );

    remote_cmd_decoder u_cmd_decode (
        .clk          (clk_24mhz),
        .rst          (rst_eff),
        .rx_data      (cmd_rx_data),
        .rx_valid     (cmd_rx_valid),
        .remote_start (remote_start),
        .remote_stop  (remote_stop),
        .remote_fault (remote_fault),
        .remote_mode0 (remote_mode0),
        .remote_mode1 (remote_mode1),
        .remote_mode2 (remote_mode2),
        .remote_reset (remote_reset)
    );

    // Merge physical switches with remote commands.
    // Either source can drive each input - panel controls keep
    // working exactly as before, Blynk just ORs on top of them.
    wire sw0_start_eff = sw0_start | remote_start;
    wire sw1_stop_eff  = sw1_stop  | remote_stop;
    wire sw2_fault_eff = sw2_fault | remote_fault;
    wire sw3_mode0_eff = sw3_mode0 | remote_mode0;
    wire sw4_mode1_eff = sw4_mode1 | remote_mode1;
    wire sw5_mode2_eff = sw5_mode2 | remote_mode2;

    // ====================================================
    // Core state machine (unmodified logic, extra outputs added)
    // ====================================================
    wire [15:0] display_data;
    wire [1:0]  state_obs;
    wire [2:0]  mode_obs;
    wire [31:0] runtime_obs, start_obs, stop_obs, fault_obs, maxrt_obs;

    machine_runtime_monitor u_core (
        .clk_24mhz     (clk_24mhz),
        .rst           (rst_eff),
        .sw0_start     (sw0_start_eff),
        .sw1_stop      (sw1_stop_eff),
        .sw2_fault     (sw2_fault_eff),
        .sw3_mode0     (sw3_mode0_eff),
        .sw4_mode1     (sw4_mode1_eff),
        .sw5_mode2     (sw5_mode2_eff),
        .relay         (relay),
        .buzzer        (buzzer),
        .display_data  (display_data),
        .state_out     (state_obs),
        .runtime_out   (runtime_obs),
        .start_out     (start_obs),
        .stop_out      (stop_obs),
        .fault_out     (fault_obs),
        .maxrt_out     (maxrt_obs),
        .mode_out      (mode_obs)
    );

    // ====================================================
    // 7-segment display path (unchanged)
    // ====================================================
    wire [3:0] bcd3, bcd2, bcd1, bcd0;

    bin_to_bcd u_bcd (
        .bin (display_data),
        .d3  (bcd3),
        .d2  (bcd2),
        .d1  (bcd1),
        .d0  (bcd0)
    );

    max7219_driver u_max7219 (
        .clk_24mhz (clk_24mhz),
        .d0        (bcd3),
        .d1        (bcd2),
        .d2        (bcd1),
        .d3        (bcd0),
        .seg_cs    (seg_cs),
        .seg_clk   (seg_clk),
        .seg_din   (seg_din)
    );

    // ====================================================
    // Status packet transmitter -> ESP32 (every 500 ms)
    // ====================================================
    // 24 MHz / 12,000,000 = 2 Hz -> one packet every 500 ms
    reg [23:0] tick_div;
    wire       send_tick = (tick_div == 24'd11_999_999);

    always @(posedge clk_24mhz or posedge rst_eff) begin
        if (rst_eff)        tick_div <= 24'd0;
        else if (send_tick) tick_div <= 24'd0;
        else                tick_div <= tick_div + 1'd1;
    end

    status_packet_tx u_status_tx (
        .clk         (clk_24mhz),
        .rst         (rst_eff),
        .send_tick   (send_tick),
        .state_in    (state_obs),
        .mode_in     (mode_obs),
        .runtime_in  (runtime_obs),
        .start_in    (start_obs),
        .stop_in     (stop_obs),
        .fault_in    (fault_obs),
        .maxrt_in    (maxrt_obs),
        .disp_in     (display_data),
        .relay_in    (relay),
        .tx          (esp_uart_tx)
    );

endmodule
