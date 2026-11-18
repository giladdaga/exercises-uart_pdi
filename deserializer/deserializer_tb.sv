`timescale 1ns/1ps

// ============================================================================
// Testbench: deserializer (8N1 UART framing)
//
// The TB drives a standard UART RX line into the DUT:
//   idle = 1 | start-bit = 0 | 8 data bits LSB-first | stop-bit = 1
//
// The TB contains a bit-accurate UART transmitter that:
//   1. Drives each UART frame at the configured baud rate.
//   2. Transmits data bits LSB-first.
//   3. Monitors the DUT parallel output (Ready/Valid handshake).
//
// The DUT outputs received bytes over a Ready/Valid interface:
//   data_out, valid_out, ready_out
// ============================================================================
module deserializer_tb;

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
  logic                  rx;
  logic [DATA_WIDTH-1:0] data_out;
  logic                  valid_out;
  logic                  ready_out;
  logic                  error_out;

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  deserializer #(
    .DATA_WIDTH (DATA_WIDTH),
    .FIFO_DEPTH (FIFO_DEPTH),
    .BAUD_DIV   (BAUD_DIV)
  ) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .rx        (rx),
    .data_out  (data_out),
    .valid_out (valid_out),
    .ready_out (ready_out),
    .error_out (error_out)
  );

  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------
  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // --------------------------------------------------------------------------
  // Scoreboard
  // --------------------------------------------------------------------------
  bit [DATA_WIDTH-1:0] exp_q [$];   // Expected bytes in reception order
  int                  errors;
  int                  bytes_sent;
  int                  bytes_checked;

  // --------------------------------------------------------------------------
  // Parallel output monitor — clocked, samples DUT output handshake
  // --------------------------------------------------------------------------
  // Symmetric to serializer_tb's uart_rx monitor: captures every byte that
  // the DUT presents (valid_out && ready_out) and checks it against exp_q.
  logic [DATA_WIDTH-1:0] chk_expected;

  always @(posedge clk) begin
    if (rst_n && valid_out && ready_out) begin
      bytes_checked++;
      if (exp_q.size() == 0) begin
        $error("[%0t] Unexpected byte received: 0x%02h (scoreboard empty)",
               $time, data_out);
        errors++;
      end else begin
        chk_expected = exp_q.pop_front();
        if (data_out !== chk_expected) begin
          $error("[%0t] DATA MISMATCH: expected 0x%02h, got 0x%02h",
                 $time, chk_expected, data_out);
          errors++;
        end else begin
          $display("[%0t] OK  byte 0x%02h", $time, data_out);
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // X/Z checks on DUT outputs
  // --------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst_n && ^{valid_out, error_out} === 1'bx) begin
      $error("[%0t] X/Z on valid_out/error_out", $time);
      errors++;
    end
  end

  // --------------------------------------------------------------------------
  // Reset assertion: valid_out must be 0 after reset
  // --------------------------------------------------------------------------
  property p_valid_out_in_reset;
    @(posedge clk) !rst_n |=> (valid_out === 1'b0);
  endproperty
  a_valid_reset: assert property (p_valid_out_in_reset)
    else begin $error("[%0t] valid_out not 0 after reset", $time); errors++; end

  // --------------------------------------------------------------------------
  // Tasks
  // --------------------------------------------------------------------------
  // Reset the DUT; keep RX idle-high during reset
  task automatic do_reset(input int cycles = 4);
    rx        = 1'b1;
    ready_out = 1'b1;
    rst_n     = 0;
    repeat (cycles) @(posedge clk);
    #1 rst_n = 1;
  endtask

  // Transmit one UART 8N1 frame on the RX line and push byte to scoreboard.
  // Aligns the start bit to just after a posedge so DUT detects it cleanly.
  task automatic send_byte(input [DATA_WIDTH-1:0] d);
    int baud_ns;
    baud_ns = BAUD_DIV * CLK_PERIOD;

    // Push to scoreboard before transmitting so monitor has it ready
    exp_q.push_back(d);
    bytes_sent++;

    // Start bit
    @(posedge clk); #1;
    rx = 1'b0;
    #(baud_ns - 1);        // hold for one baud period from #1 offset

    // 8 data bits, LSB first
    for (int i = 0; i < DATA_WIDTH; i++) begin
      rx = d[i];
      #(baud_ns);
    end

    // Stop bit (line returns to idle)
    rx = 1'b1;
    #(baud_ns);
  endtask

  // Inject a UART frame with a corrupted stop bit (stop bit = 0 instead of 1).
  // Does NOT push to exp_q — DUT asserts error_out and suppresses valid_out.
  task automatic send_byte_bad_stop(input [DATA_WIDTH-1:0] d);
    int baud_ns;
    baud_ns = BAUD_DIV * CLK_PERIOD;

    @(posedge clk); #1;
    rx = 1'b0;              // start bit
    #(baud_ns - 1);
    for (int i = 0; i < DATA_WIDTH; i++) begin
      rx = d[i];
      #(baud_ns);
    end
    rx = 1'b0;              // bad stop bit (should be 1)
    #(baud_ns);
    rx = 1'b1;              // restore idle
    #(baud_ns);
  endtask

  // Keep RX idle for n clock cycles
  task automatic idle_cycles(input int n);
    repeat (n) @(posedge clk);
    #1;
  endtask

  // Wait until all queued bytes have been captured by the output monitor
  task automatic wait_drain(input int max_baud_periods = 500);
    int timeout_ns;
    timeout_ns = max_baud_periods * BAUD_DIV * CLK_PERIOD;
    ready_out = 1'b1;
    fork
      begin : wait_loop
        while (exp_q.size() != 0) @(posedge clk);
        // Extra gap: a few baud periods to let the last handshake settle
        repeat (BAUD_DIV * 4) @(posedge clk);
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
    if (valid_out !== 1'b0 && exp_q.size() == 0) begin
      // Allow one extra drain window for the skid register to flush
      repeat (BAUD_DIV * 4) @(posedge clk); #1;
    end
  endtask

  // --------------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------------
  initial begin
    errors = 0; bytes_sent = 0; bytes_checked = 0;
    rx = 1'b1; rst_n = 1; ready_out = 1;
    #1;

    // ---- Test 1: Reset ----
    $display("\n=== Test 1: Reset ===");
    do_reset(4);
    repeat (2) @(posedge clk); #1;
    if (valid_out !== 1'b0) begin $error("valid_out != 0 after reset"); errors++; end
    if (rx     !== 1'b1) begin $error("rx not idle-high after reset"); errors++; end

    // ---- Test 2: Single bytes — verify framing and bit order ----
    $display("\n=== Test 2: Single bytes (LSB-first framing) ===");
    send_byte(8'hA5);
    wait_drain();

    send_byte(8'h01);   // only bit 0 set → first data bit = 1
    wait_drain();

    send_byte(8'h80);   // only bit 7 set → last data bit = 1
    wait_drain();

    // ---- Test 3: 16 back-to-back bytes ----
    $display("\n=== Test 3: 16 back-to-back bytes ===");
    for (int i = 0; i < FIFO_DEPTH; i++) begin
      send_byte(8'h10 + i[7:0]);
    end
    wait_drain(1000);

    // ---- Test 4: Output back-pressure (ready_out = 0) ----
    // Symmetric to serializer Test 4 "saturation": here we stress the output
    // side by keeping ready_out low while the DUT receives bytes.
    $display("\n=== Test 4: Output back-pressure (ready_out low) ===");
    begin
      int saw_valid_stalled = 0;
      ready_out = 1'b0;
      for (int i = 0; i < 8; i++) send_byte(8'hA0 + i[7:0]);
      // Count cycles where valid_out is high but we're not accepting
      repeat (BAUD_DIV * 4) begin
        @(posedge clk);
        if (valid_out === 1'b1) saw_valid_stalled++;
        #1;
      end
      if (saw_valid_stalled == 0)
        $display("NOTE: valid_out never raised under back-pressure (check timing)");
      else
        $display("valid_out stalled %0d cycles under back-pressure (expected)", saw_valid_stalled);
      ready_out = 1'b1;
    end
    wait_drain(500);

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
    wait_drain();

    // ---- Test 7: Reset mid-reception ----
    // Symmetric to serializer Test 7: reset the DUT while it is actively
    // receiving a byte, then verify clean restart and correct operation.
    $display("\n=== Test 7: Reset in middle of reception ===");
    begin
      // Start sending 4 bytes in background, assert reset partway through
      fork
        begin : sender
          for (int i = 0; i < 4; i++) send_byte(8'h30 + i[7:0]);
        end
        begin : resetter
          // Wait until the DUT has started receiving (past START state):
          // let one full baud period pass after the first start bit.
          @(negedge rx);                      // start bit begins
          repeat (BAUD_DIV + BAUD_DIV/2) @(posedge clk);  // mid-data
          exp_q.delete();                     // discard; reset will abort reception
          rst_n = 0;
          rx    = 1'b1;                       // force idle during reset
          repeat (4) @(posedge clk);
          #1 rst_n = 1;
        end
      join_any
      disable fork;
      rx = 1'b1;   // ensure idle after fork killed
    end
    repeat (2) @(posedge clk); #1;
    if (valid_out !== 1'b0) begin
      $error("valid_out != 0 after mid-reception reset");
      errors++;
    end
    // Verify normal operation after reset
    send_byte(8'hC3);
    send_byte(8'h3C);
    wait_drain();

    // ---- Test 8: Framing error (bad stop bit) ----
    $display("\n=== Test 8: Framing error (bad stop bit) ===");
    send_byte_bad_stop(8'h42);
    repeat (BAUD_DIV * 4) @(posedge clk); #1;
    if (error_out !== 1'b1) begin
      $error("[%0t] error_out not asserted after framing error", $time);
      errors++;
    end else begin
      $display("[%0t] OK  error_out asserted on framing error", $time);
    end
    // Reset after error
    // do_reset(1);
    // repeat (2) @(posedge clk); #1;
    // Normal byte after framing error should still work
    send_byte(8'hBE);
    wait_drain();

    // ---- Test 9: Random ready_out throttling ----
    $display("\n=== Test 9: Random ready_out throttling ===");
    fork
      begin
        for (int i = 0; i < 20; i++) send_byte($urandom);
      end
      begin
        for (int i = 0; i < 400; i++) begin
          @(posedge clk);
          #1 ready_out = ($urandom_range(0, 10) > 2); // ~70% ready
        end
        #1 ready_out = 1;
      end
    join
    ready_out = 1;
    wait_drain(2000);

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
    $dumpfile("deserializer_tb.vcd");
    $dumpvars(0, deserializer_tb);
  end

endmodule
