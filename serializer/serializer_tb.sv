`timescale 1ns/1ps

// ============================================================================
// Testbench: serializer (8N1 UART framing)
//
// The DUT drives a standard UART TX line:
//   idle = 1 | start-bit = 0 | 8 data bits LSB-first | stop-bit = 1
//
// The TB contains a bit-accurate UART receiver that:
//   1. Detects the falling edge of TX (start bit).
//   2. Samples at the mid-point of each baud period.
//   3. Reconstructs and checks every byte against the expected queue.
// ============================================================================
module serializer_tb;

  // --------------------------------------------------------------------------
  // Parameters — must match DUT
  // --------------------------------------------------------------------------
  localparam int DATA_WIDTH = 8;
  localparam int FIFO_DEPTH = 16;
  localparam int CLK_PERIOD = 10;        // ns  (100 MHz clock)
  localparam int BAUD_DIV   = 16;        // Small value for fast simulation
                                         // (real: 868 for 115200 @ 100 MHz)

  // --------------------------------------------------------------------------
  // Signals
  // --------------------------------------------------------------------------
  logic                  clk;
  logic                  rst_n;
  logic [DATA_WIDTH-1:0] data_in;
  logic                  valid_in;
  logic                  ready_in;
  logic                  tx;

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  serializer #(
    .DATA_WIDTH (DATA_WIDTH),
    .FIFO_DEPTH (FIFO_DEPTH),
    .BAUD_DIV   (BAUD_DIV)
  ) dut (
    .clk      (clk),
    .rst_n    (rst_n),
    .data_in  (data_in),
    .valid_in (valid_in),
    .ready_in (ready_in),
    .tx       (tx)
  );

  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------
  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // --------------------------------------------------------------------------
  // Scoreboard
  // --------------------------------------------------------------------------
  bit [DATA_WIDTH-1:0] exp_q [$];   // Expected bytes in transmission order
  int                  errors;
  int                  bytes_sent;
  int                  bytes_checked;

  // --------------------------------------------------------------------------
  // abort_rx_frame: triggered by stimulus to kill uart_rx mid-frame on reset
  // --------------------------------------------------------------------------
  event abort_rx_frame;

  // --------------------------------------------------------------------------
  // UART Receiver — samples TX and reconstructs bytes
  // --------------------------------------------------------------------------
  // Each iteration wraps recv_frame and an abort_watcher in fork/join_any so
  // that ->abort_rx_frame (fired before a DUT reset) cleanly aborts any
  // in-progress frame without leaving the monitor in an inconsistent state.
  initial begin : uart_rx
    logic [DATA_WIDTH-1:0] rx_byte;
    bit   [DATA_WIDTH-1:0] expected;
    int                    baud_half;
    int                    baud_full;
    bit                    aborted;

    baud_half = (BAUD_DIV * CLK_PERIOD) / 2;  // half a baud in ns
    baud_full =  BAUD_DIV * CLK_PERIOD;        // full baud in ns

    forever begin
      aborted = 0;

      fork
        // ---- Branch A: receive one complete UART frame ----
        begin
          // Wait for the falling edge of TX (start bit)
          @(negedge tx);

          // Advance to the middle of the start bit and verify it is still 0
          #(baud_half);
          if (tx === 1'b0) begin
            // Sample 8 data bits, LSB first, at mid-baud
            for (int i = 0; i < DATA_WIDTH; i++) begin
              #(baud_full);
              rx_byte[i] = tx;
            end

            // Sample stop bit
            #(baud_full);
            if (tx !== 1'b1) begin
              $error("[%0t] Stop bit error: tx=0 (framing error)", $time);
              errors++;
            end

            // Compare against scoreboard
            bytes_checked++;
            if (exp_q.size() == 0) begin
              $error("[%0t] Unexpected byte received: 0x%02h (scoreboard empty)",
                     $time, rx_byte);
              errors++;
            end else begin
              expected = exp_q.pop_front();
              if (rx_byte !== expected) begin
                $error("[%0t] DATA MISMATCH: expected 0x%02h, got 0x%02h",
                       $time, expected, rx_byte);
                errors++;
              end else begin
                $display("[%0t] OK  byte 0x%02h", $time, rx_byte);
              end
            end
          end
          // else: false start (tx already high) — silently discard
        end

        // ---- Branch B: abort watcher ----
        begin
          @abort_rx_frame;
          aborted = 1;
        end
      join_any
      disable fork;   // kill whichever branch is still running

      if (aborted) begin
        // Reset is about to happen; wait for TX to return to idle then resume
        @(posedge rst_n);
        wait (tx === 1'b1);
      end
    end
  end

  // --------------------------------------------------------------------------
  // TX idle-high check: TX must never be X/Z
  // --------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst_n && ^tx === 1'bx) begin
      $error("[%0t] TX is X/Z", $time);
      errors++;
    end
  end

  // --------------------------------------------------------------------------
  // Reset assertion: TX must be high during reset
  // --------------------------------------------------------------------------
  property p_tx_high_in_reset;
    @(posedge clk) !rst_n |=> (tx === 1'b1);
  endproperty
  a_tx_reset: assert property (p_tx_high_in_reset)
    else begin $error("[%0t] TX not idle-high after reset", $time); errors++; end

  // --------------------------------------------------------------------------
  // Tasks
  // --------------------------------------------------------------------------
  // Reset the DUT
  task automatic do_reset(input int cycles = 4);
    valid_in = 0;
    data_in  = 0;
    rst_n    = 0;
    repeat (cycles) @(posedge clk);
    #1 rst_n = 1;
  endtask

  // Send one byte: drives valid_in/data_in until ready_in handshake
  task automatic send_byte(input [DATA_WIDTH-1:0] d);
    valid_in = 1;
    data_in  = d;
    @(posedge clk iff (ready_in === 1'b1));
    exp_q.push_back(d);
    bytes_sent++;
    #1;
  endtask

  // Deassert valid_in for n cycles
  task automatic idle_cycles(input int n);
    valid_in = 0;
    data_in  = $urandom;   // random garbage while valid is low
    repeat (n) @(posedge clk);
    #1;
  endtask

  // Wait until all queued bytes have been received by the UART RX monitor
  task automatic wait_drain(input int max_baud_periods = 500);
    int timeout_ns;
    timeout_ns = max_baud_periods * BAUD_DIV * CLK_PERIOD;
    valid_in = 0;
    fork
      begin : wait_loop
        while (exp_q.size() != 0) @(posedge clk);
        // Extra gap: at least one full UART frame of idle to confirm stop bit done
        repeat (BAUD_DIV * 12) @(posedge clk);
        disable watchdog_drain;
      end
      begin : watchdog_drain
        #(timeout_ns * 1ns);
        $error("[%0t] TIMEOUT in wait_drain (pending=%0d)", $time, exp_q.size());
        errors++;
        disable wait_loop;
      end
    join
    #1;
    if (tx !== 1'b1) begin
      $error("[%0t] TX not idle-high after drain", $time);
      errors++;
    end
  endtask

  // --------------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------------
  initial begin
    errors = 0; bytes_sent = 0; bytes_checked = 0;
    rst_n = 1; valid_in = 0; data_in = 0;
    #1;

    // ---- Test 1: Reset ----
    $display("\n=== Test 1: Reset ===");
    do_reset(4);
    repeat (2) @(posedge clk); #1;
    if (tx !== 1'b1) begin $error("TX not idle-high after reset"); errors++; end
    if (ready_in !== 1'b1) begin $error("ready_in not high after reset"); errors++; end

    // ---- Test 2: Single bytes — verify framing and bit order ----
    $display("\n=== Test 2: Single bytes (LSB-first framing) ===");
    send_byte(8'hA5);
    idle_cycles(1);
    wait_drain();

    send_byte(8'h01);   // only bit 0 set → first data bit = 1
    idle_cycles(1);
    wait_drain();

    send_byte(8'h80);   // only bit 7 set → last data bit = 1
    idle_cycles(1);
    wait_drain();

    // ---- Test 3: 16 back-to-back bytes (fill FIFO) ----
    $display("\n=== Test 3: 16 back-to-back bytes ===");
    for (int i = 0; i < FIFO_DEPTH; i++) begin
      send_byte(8'h10 + i[7:0]);
    end
    valid_in = 0;
    wait_drain(1000);

    // ---- Test 4: Saturation (valid_in always high) ----
    $display("\n=== Test 4: Saturation (valid_in always 1) ===");
    begin
      int saw_not_ready = 0;
      for (int i = 0; i < 60; i++) begin
        valid_in = 1;
        data_in  = $urandom;
        @(posedge clk);
        if (ready_in === 1'b1) begin
          exp_q.push_back(data_in);
          bytes_sent++;
        end else begin
          saw_not_ready++;
        end
        #1;
      end
      valid_in = 0;
      if (saw_not_ready == 0)
        $display("NOTE: ready_in never deasserted under saturation");
      else
        $display("ready_in deasserted %0d times under saturation (expected)", saw_not_ready);
    end
    wait_drain(2000);

    // ---- Test 5: Random traffic with gaps ----
    $display("\n=== Test 5: Random traffic with gaps ===");
    for (int i = 0; i < 50; i++) begin
      send_byte($urandom);
      case ($urandom_range(0, 3))
        0:       ;
        1:       idle_cycles(BAUD_DIV);
        2:       idle_cycles($urandom_range(1, BAUD_DIV * 3));
        default: idle_cycles($urandom_range(BAUD_DIV * 3, BAUD_DIV * 10));
      endcase
    end
    wait_drain(5000);

    // ---- Test 6: Corner patterns ----
    $display("\n=== Test 6: Corner patterns ===");
    send_byte(8'h00);
    send_byte(8'hFF);
    send_byte(8'h55);
    send_byte(8'hAA);
    send_byte(8'h0F);
    send_byte(8'hF0);
    valid_in = 0;
    wait_drain();

    // ---- Test 7: Reset mid-transmission ----
    $display("\n=== Test 7: Reset in middle of transmission ===");
    for (int i = 0; i < 4; i++) send_byte(8'h30 + i[7:0]);
    valid_in = 0;
    // Wait until TX goes low (start bit of some frame), then reset mid-frame.
    // Signal the uart_rx monitor FIRST so it aborts cleanly, then clear the
    // scoreboard and assert reset.
    @(negedge tx);
    repeat (BAUD_DIV / 2) @(posedge clk);
    ->abort_rx_frame;    // tell uart_rx to discard the in-progress frame
    #1;
    exp_q.delete();      // scoreboard reflects the abort
    do_reset(4);
    repeat (2) @(posedge clk); #1;
    if (tx !== 1'b1) begin $error("TX not idle-high after mid-frame reset"); errors++; end
    // Verify normal operation after reset
    send_byte(8'hC3);
    send_byte(8'h3C);
    valid_in = 0;
    wait_drain();

    // ---- Summary ----
    $display("\n==================================================");
    $display(" Bytes sent    : %0d", bytes_sent);
    $display(" Bytes checked : %0d", bytes_checked);
    if (errors == 0 && exp_q.size() == 0)
      $display(" RESULT: *** TEST PASSED ***");
    else
      $display(" RESULT: *** TEST FAILED *** (%0d errors)", errors);
    $display("==================================================\n");
    $finish;
  end

  // --------------------------------------------------------------------------
  // Global watchdog
  // --------------------------------------------------------------------------
  initial begin
    #(CLK_PERIOD * BAUD_DIV * 20000);
    $error("GLOBAL TIMEOUT");
    $finish;
  end

  // --------------------------------------------------------------------------
  // Abort on too many errors
  // --------------------------------------------------------------------------
  localparam int MAX_ERRORS = 10;
  always @(posedge clk) begin
    if (errors >= MAX_ERRORS) begin
      $display("\n*** Aborting: %0d errors reached ***", errors);
      $display(" RESULT: *** TEST FAILED ***");
      $finish;
    end
  end

  // --------------------------------------------------------------------------
  // Waveform dump
  // --------------------------------------------------------------------------
  initial begin
    $dumpfile("serializer_tb.vcd");
    $dumpvars(0, serializer_tb);
  end

endmodule
