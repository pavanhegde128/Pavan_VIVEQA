`timescale 1ns / 1ps
// ============================================================
// tb_uart_link.v
// Standalone testbench for the new UART status/command path.
//
// Changes from original:
//   - Replaced fork...join_any (SystemVerilog) with Verilog-2001
//     compatible polling loops for remote_start / remote_stop.
//   - Removed bare begin...end blocks wrapping fork that caused
//     [HDL 9-806] syntax errors in Vivado Verilog mode.
// ============================================================
module tb_uart_link;

    reg clk_24mhz = 0;
    reg rst = 1;
    always #20.8333 clk_24mhz = ~clk_24mhz; // ~24 MHz

    // ----------------------------------------------------------
    // Test 1: status_packet_tx -> loopback -> uart_rx -> ASCII capture
    // ----------------------------------------------------------
    reg        send_tick;
    reg [1:0]  state_in;
    reg [2:0]  mode_in;
    reg [31:0] runtime_in, start_in, stop_in, fault_in, maxrt_in;
    reg [15:0] disp_in;
    reg        relay_in;
    wire       tx_line;

    status_packet_tx u_status (
        .clk        (clk_24mhz),
        .rst        (rst),
        .send_tick  (send_tick),
        .state_in   (state_in),
        .mode_in    (mode_in),
        .runtime_in (runtime_in),
        .start_in   (start_in),
        .stop_in    (stop_in),
        .fault_in   (fault_in),
        .maxrt_in   (maxrt_in),
        .disp_in    (disp_in),
        .relay_in   (relay_in),
        .tx         (tx_line)
    );

    // Loop TX straight into RX to capture bytes
    wire [7:0] cap_data;
    wire       cap_valid;

    uart_rx #(.CLK_FREQ(24_000_000), .BAUD_RATE(9600)) u_capture (
        .clk      (clk_24mhz),
        .rst      (rst),
        .rx       (tx_line),
        .rx_data  (cap_data),
        .rx_valid (cap_valid)
    );

    // Collect captured bytes into a string for display
    reg [8*64-1:0] captured_line;
    integer cap_idx;

    always @(posedge clk_24mhz) begin
        if (rst) begin
            cap_idx       <= 0;
            captured_line <= 0;
        end
        else if (cap_valid) begin
            if (cap_data == "\n") begin
                $display("  [T1] Captured packet: %s", captured_line);
                cap_idx       <= 0;
                captured_line <= 0;
            end
            else begin
                captured_line[8*cap_idx +: 8] <= cap_data;
                cap_idx <= cap_idx + 1;
            end
        end
    end

    // ----------------------------------------------------------
    // Test 2/3: bit-banged byte driver -> uart_rx -> remote_cmd_decoder
    // ----------------------------------------------------------
    reg  cmd_line = 1'b1; // idle high
    wire [7:0] cmd_rx_data;
    wire       cmd_rx_valid;
    wire       remote_start, remote_stop;
    wire       remote_fault, remote_mode0, remote_mode1, remote_mode2;
    wire       remote_reset;

    uart_rx #(.CLK_FREQ(24_000_000), .BAUD_RATE(9600)) u_cmd_rx (
        .clk      (clk_24mhz),
        .rst      (rst),
        .rx       (cmd_line),
        .rx_data  (cmd_rx_data),
        .rx_valid (cmd_rx_valid)
    );

    remote_cmd_decoder u_decode (
        .clk          (clk_24mhz),
        .rst          (rst),
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

    // Bit period at 9600 baud = 104,166.67 ns
    task send_byte;
        input [7:0] b;
        integer i;
        begin
            cmd_line = 1'b0;       // start bit
            #104166;
            for (i = 0; i < 8; i = i + 1) begin
                cmd_line = b[i];
                #104166;
            end
            cmd_line = 1'b1;       // stop bit
            #104166;
        end
    endtask

    // ----------------------------------------------------------
    // Pulse-detect helpers (set by always blocks, cleared by initial)
    // ----------------------------------------------------------
    reg got_start;
    reg got_stop;

    always @(posedge clk_24mhz) begin
        if (remote_start) got_start <= 1'b1;
        if (remote_stop)  got_stop  <= 1'b1;
    end

    // Timeout counter (re-used across tests)
    integer timeout_cnt;

    // ----------------------------------------------------------
    // Main stimulus
    // ----------------------------------------------------------
    initial begin
        $dumpfile("tb_uart_link.vcd");
        $dumpvars(0, tb_uart_link);

        state_in   = 2'd0;
        mode_in    = 3'd0;
        runtime_in = 32'd0;
        start_in   = 32'd0;
        stop_in    = 32'd0;
        fault_in   = 32'd0;
        maxrt_in   = 32'd0;
        disp_in    = 16'd0;
        relay_in   = 1'b0;
        send_tick  = 0;
        got_start  = 0;
        got_stop   = 0;

        #200;
        rst = 0;
        #200;

        // ---- T1: send a status packet with known values ----
        state_in   = 2'd1;   // RUNNING
        mode_in    = 3'd0;   // display mode = runtime (seconds)
        runtime_in = 32'd1234;
        start_in   = 32'd5;
        stop_in    = 32'd2;
        fault_in   = 32'd1;
        maxrt_in   = 32'd987;
        disp_in    = 16'd1234;
        relay_in   = 1'b1;   // relay energized while RUNNING

        send_tick = 1;
        @(posedge clk_24mhz);
        send_tick = 0;

        // Wait long enough for ~45 bytes at 9600 baud
        #2_200_000;

        // ---- T2: send ASCII '1' -> expect remote_start pulse ----
        $display("\n[T2] Sending '1' (start command)...");
        got_start = 0;
        send_byte(8'h31); // ASCII '1'

        // Poll for up to ~500 us after the byte
        timeout_cnt = 0;
        while (got_start == 1'b0 && timeout_cnt < 24000) begin
            @(posedge clk_24mhz);
            timeout_cnt = timeout_cnt + 1;
        end

        if (got_start)
            $display("  PASS  remote_start pulsed");
        else
            $display("  FAIL  remote_start never pulsed (timeout)");

        #50000;

        // ---- T3: send ASCII '0' -> expect remote_stop pulse ----
        $display("\n[T3] Sending '0' (stop command)...");
        got_stop = 0;
        send_byte(8'h30); // ASCII '0'

        timeout_cnt = 0;
        while (got_stop == 1'b0 && timeout_cnt < 24000) begin
            @(posedge clk_24mhz);
            timeout_cnt = timeout_cnt + 1;
        end

        if (got_stop)
            $display("  PASS  remote_stop pulsed");
        else
            $display("  FAIL  remote_stop never pulsed (timeout)");

        #50000;

        // ---- T4: send ASCII 'F' -> expect remote_fault to TOGGLE ----
        $display("\n[T4] Sending 'F' (fault toggle)...");
        send_byte(8'h46); // ASCII 'F'
        #50000;
        if (remote_fault === 1'b1)
            $display("  PASS  remote_fault toggled to 1");
        else
            $display("  FAIL  remote_fault did not toggle (got %b)", remote_fault);

        // toggle back off, confirm it returns to 0
        send_byte(8'h46);
        #50000;
        if (remote_fault === 1'b0)
            $display("  PASS  remote_fault toggled back to 0");
        else
            $display("  FAIL  remote_fault did not toggle back (got %b)", remote_fault);

        // ---- T5: send ASCII 'A'/'B'/'C' -> expect mode latches to TOGGLE ----
        $display("\n[T5] Sending 'A','B','C' (mode toggles)...");
        send_byte(8'h41); // 'A' -> remote_mode0
        send_byte(8'h42); // 'B' -> remote_mode1
        send_byte(8'h43); // 'C' -> remote_mode2
        #50000;
        if (remote_mode0 === 1'b1 && remote_mode1 === 1'b1 && remote_mode2 === 1'b1)
            $display("  PASS  remote_mode0/1/2 all toggled to 1");
        else
            $display("  FAIL  mode latches incorrect (mode0=%b mode1=%b mode2=%b)",
                      remote_mode0, remote_mode1, remote_mode2);

        // ---- T6: send ASCII 'R' -> expect remote_reset to PULSE ----
        $display("\n[T6] Sending 'R' (reset command)...");
        send_byte(8'h52); // ASCII 'R'
        // remote_reset is a single-cycle pulse, sample it the cycle it fires
        timeout_cnt = 0;
        while (remote_reset !== 1'b1 && timeout_cnt < 24000) begin
            @(posedge clk_24mhz);
            timeout_cnt = timeout_cnt + 1;
        end
        if (remote_reset === 1'b1)
            $display("  PASS  remote_reset pulsed");
        else
            $display("  FAIL  remote_reset never pulsed (timeout)");

        $display("\n=== tb_uart_link complete - inspect captured packet text above ===");
        $finish;
    end

    // Watchdog
    initial begin
        #20_000_000;
        $display("WATCHDOG timeout");
        $finish;
    end

endmodule
