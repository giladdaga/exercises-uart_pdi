`timescale 1ns/1ps

// ============================================================================
// Testbench: uart_pdi (UART Parallel Data Interface)
//
// The DUT implements a PDI protocol over 8N1 UART:
//
//   READ:
//     RX  receives: 'R' | idx[3] | idx[2] | idx[1] | idx[0]
//                       | len[3] | len[2] | len[1] | len[0]
//     TX  sends:    data[0] | data[1] | ... | data[N] | 'F'
//     (data = din[idx .. idx+len-1], LSB-first within each byte, zero-padded)
//
//   WRITE:
//     RX  receives: 'W' | idx[3] | idx[2] | idx[1] | idx[0]
//                       | len[3] | len[2] | len[1] | len[0]
//                       | data[0] | ... | data[N] | 'F'
//     dout[idx .. idx+len-1] is updated; rest retains its previous value.
//     dout = 0 on reset.
//
// 'R' = 0x52, 'W' = 0x57, 'F' = 0x46
// Index and length fields are 4-byte big-endian.
// ============================================================================
module uart_pdi_tb;

  // --------------------------------------------------------------------------
  // Parameters — DATA_WIDTH kept small for fast simulation
  // --------------------------------------------------------------------------
  localparam int DATA_WIDTH      = 32;   // DUT default is 1024; 32 is enough to test
  localparam int BYTE_LENGTH     = 8;
  localparam int BITS_FOR_INDEX  = 4;    // 4-byte (32-bit) index field
  localparam int BITS_FOR_LENGTH = 4;    // 4-byte (32-bit) length field
  localparam int CLK_PERIOD      = 10;   // ns  (100 MHz)
  localparam int BAUD_DIV        = 16;   // Small value for fast simulation

  localparam logic [7:0] CMD_R = 8'h52;  // 'R'
  localparam logic [7:0] CMD_W = 8'h57;  // 'W'
  localparam logic [7:0] CMD_F = 8'h46;  // 'F'

  // --------------------------------------------------------------------------
  // Signals
  // --------------------------------------------------------------------------
  logic                  clk;
  logic                  rst_n;
  logic                  rx;
  logic                  tx;
  logic [DATA_WIDTH-1:0] din;
  logic [DATA_WIDTH-1:0] dout;

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  uart_pdi #(
    .DATA_WIDTH      (DATA_WIDTH),
    .BYTE_LENGTH     (BYTE_LENGTH),
    .BITS_FOR_INDEX  (BITS_FOR_INDEX),
    .BITS_FOR_LENGTH (BITS_FOR_LENGTH),
    .BAUD_DIV        (BAUD_DIV)
  ) dut (
    .clk   (clk),
    .rst_n (rst_n),
    .rx    (rx),
    .tx    (tx),
    .din   (din),
    .dout  (dout)
  );

  // --------------------------------------------------------------------------
  // Clock
  // --------------------------------------------------------------------------
  initial clk = 0;
  always #(CLK_PERIOD/2) clk = ~clk;

  // --------------------------------------------------------------------------
  // Scoreboard — expected bytes that should appear on TX
  // --------------------------------------------------------------------------
  bit [7:0] exp_q [$];
  int       errors;
  int       bytes_driven;    // bytes sent into DUT via RX
  int       bytes_checked;   // bytes checked from DUT TX output

  // --------------------------------------------------------------------------
  // abort_rx_frame: triggered by stimulus to kill uart_rx mid-frame on reset
  // --------------------------------------------------------------------------
  event abort_rx_frame;

  // --------------------------------------------------------------------------
  // UART Receiver — samples TX and reconstructs bytes
  // --------------------------------------------------------------------------
  // Each iteration wraps recv_frame and an abort_watcher in fork/join_any so
  // that ->abort_rx_frame cleanly aborts any in-progress frame on DUT reset.
  initial begin : uart_rx
    logic [7:0] rx_byte;
    bit   [7:0] expected;
    int         baud_half;
    int         baud_full;
    bit         aborted;

    baud_half = (BAUD_DIV * CLK_PERIOD) / 2;
    baud_full =  BAUD_DIV * CLK_PERIOD;

    forever begin
      aborted = 0;

      fork
        // ---- Branch A: receive one complete UART frame ----
        begin
          @(negedge tx);

          #(baud_half);
          if (tx === 1'b0) begin
            for (int i = 0; i < BYTE_LENGTH; i++) begin
              #(baud_full);
              rx_byte[i] = tx;
            end

            #(baud_full);
            if (tx !== 1'b1) begin
              $error("[%0t] Stop bit error on TX (framing error)", $time);
              errors++;
            end

            bytes_checked++;
            if (exp_q.size() == 0) begin
              $error("[%0t] Unexpected byte on TX: 0x%02h (scoreboard empty)",
                     $time, rx_byte);
              errors++;
            end else begin
              expected = exp_q.pop_front();
              if (rx_byte !== expected) begin
                $error("[%0t] TX MISMATCH: expected 0x%02h, got 0x%02h",
                       $time, expected, rx_byte);
                errors++;
              end else begin
                $display("[%0t] OK  TX byte 0x%02h", $time, rx_byte);
              end
            end
          end
          // else: false start — tx already high, silently discard
        end

        // ---- Branch B: abort watcher ----
        begin
          @abort_rx_frame;
          aborted = 1;
        end
      join_any
      disable fork;

      if (aborted) begin
        @(posedge rst_n);
        wait (tx === 1'b1);
      end
    end
  end

  // --------------------------------------------------------------------------
  // TX X/Z check
  // --------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst_n && ^tx === 1'bx) begin
      $error("[%0t] TX is X/Z", $time);
      errors++;
    end
  end

  // --------------------------------------------------------------------------
  // Assertions
  // --------------------------------------------------------------------------
  property p_tx_high_in_reset;
    @(posedge clk) !rst_n |=> (tx === 1'b1);
  endproperty
  a_tx_reset: assert property (p_tx_high_in_reset)
    else begin $error("[%0t] TX not idle-high after reset", $time); errors++; end

  property p_dout_zero_in_reset;
    @(posedge clk) !rst_n |=> (dout === '0);
  endproperty
  a_dout_reset: assert property (p_dout_zero_in_reset)
    else begin $error("[%0t] dout not zero after reset", $time); errors++; end

  // --------------------------------------------------------------------------
  // Tasks
  // --------------------------------------------------------------------------

  // Reset the DUT; keep RX idle-high during reset
  task automatic do_reset(input int cycles = 4);
    rx    = 1'b1;
    din   = '0;
    rst_n = 0;
    repeat (cycles) @(posedge clk);
    #1 rst_n = 1;
  endtask

  // Transmit one UART 8N1 frame on the RX line.
  // Aligns the start bit to just after a posedge so DUT detects it cleanly.
  // Does NOT push to exp_q — caller decides whether to expect a TX response.
  task automatic send_uart_byte(input [7:0] d);
    int baud_ns;
    baud_ns = BAUD_DIV * CLK_PERIOD;
    bytes_driven++;

    @(posedge clk); #1;
    rx = 1'b0;              // start bit
    #(baud_ns - 1);

    for (int i = 0; i < BYTE_LENGTH; i++) begin
      rx = d[i];
      #(baud_ns);
    end

    rx = 1'b1;              // stop bit (line returns to idle)
    #(baud_ns);
  endtask

  // Send a 32-bit value big-endian (4 bytes, MSB first)
  task automatic send_32b_be(input [31:0] val);
    send_uart_byte(val[31:24]);
    send_uart_byte(val[23:16]);
    send_uart_byte(val[15: 8]);
    send_uart_byte(val[ 7: 0]);
  endtask

  // Send a READ command: 'R' | idx (4B BE) | len (4B BE)
  task automatic send_read_cmd(input [31:0] idx, input [31:0] len);
    send_uart_byte(CMD_R);
    send_32b_be(idx);
    send_32b_be(len);
  endtask

  // Send a WRITE command: 'W' | idx (4B BE) | len (4B BE) | data bytes | 'F'
  // write_val[0] maps to dout[idx], write_val[1] to dout[idx+1], etc.
  // Data is packed LSB-first within each byte; last byte is zero-padded.
  task automatic send_write_cmd(
    input [31:0]           idx,
    input [31:0]           len,
    input [DATA_WIDTH-1:0] write_val
  );
    int      num_bytes;
    logic [7:0] byte_val;
    int         bit_pos;

    send_uart_byte(CMD_W);
    send_32b_be(idx);
    send_32b_be(len);

    num_bytes = int'((len + 7) / 8);
    for (int b = 0; b < num_bytes; b++) begin
      byte_val = 8'h00;
      for (int i = 0; i < BYTE_LENGTH; i++) begin
        bit_pos = b * BYTE_LENGTH + i;
        if (bit_pos < int'(len))
          byte_val[i] = write_val[bit_pos];
        // else zero-pad (byte_val[i] stays 0)
      end
      send_uart_byte(byte_val);
    end

    send_uart_byte(CMD_F);
  endtask

  // Pre-load exp_q with the bytes the DUT should transmit for a READ response.
  // Reads din_val[idx +: len], packs LSB-first with zero-padding, then 'F'.
  task automatic expect_read_response(
    input [DATA_WIDTH-1:0] din_val,
    input [31:0]           idx,
    input [31:0]           len
  );
    int         num_bytes;
    logic [7:0] byte_val;
    int         bit_pos;

    num_bytes = int'((len + 7) / 8);
    for (int b = 0; b < num_bytes; b++) begin
      byte_val = 8'h00;
      for (int i = 0; i < BYTE_LENGTH; i++) begin
        bit_pos = b * BYTE_LENGTH + i;
        if (bit_pos < int'(len))
          byte_val[i] = din_val[int'(idx) + bit_pos];
        // else zero-pad
      end
      exp_q.push_back(byte_val);
    end
    exp_q.push_back(CMD_F);
  endtask

  // Wait until all queued bytes have been captured by the TX monitor
  task automatic wait_drain(input int max_baud_periods = 2000);
    int timeout_ns;
    timeout_ns = max_baud_periods * BAUD_DIV * CLK_PERIOD;
    fork
      begin : wait_loop
        while (exp_q.size() != 0) @(posedge clk);
        // Extra gap: at least one full UART frame of idle to confirm stop bit
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

  // Wait a few baud periods then check that dout matches expected_full exactly
  task automatic check_dout(
    input [DATA_WIDTH-1:0] expected_full,
    input string           test_name
  );
    repeat (BAUD_DIV * 2) @(posedge clk); #1;
    if (dout !== expected_full) begin
      $error("[%0t] %s — dout MISMATCH: expected 0x%08h, got 0x%08h",
             $time, test_name, expected_full, dout);
      errors++;
    end else begin
      $display("[%0t] OK  %s — dout=0x%08h", $time, test_name, dout);
    end
  endtask

  // --------------------------------------------------------------------------
  // Stimulus
  // --------------------------------------------------------------------------
  initial begin
    errors = 0; bytes_driven = 0; bytes_checked = 0;
    rx = 1'b1; rst_n = 1; din = '0;
    #1;

    // ---- Test 1: Reset ----
    $display("\n=== Test 1: Reset ===");
    do_reset(4);
    repeat (2) @(posedge clk); #1;
    if (tx   !== 1'b1) begin $error("TX not idle-high after reset");  errors++; end
    if (dout !== '0  ) begin $error("dout not zero after reset");     errors++; end
    $display("[%0t] OK  reset: tx=1, dout=0", $time);

    // ---- Test 2: Read 32 bits (full DATA_WIDTH word) ----
    $display("\n=== Test 2: Read 32 bits (full word) ===");
    do_reset();
    din = 32'hA5C33A5A;
    expect_read_response(din, 32'd0, 32'd32);
    send_read_cmd(32'd0, 32'd32);
    wait_drain();

    // ---- Test 3: Write 8 bits at index 0 ----
    $display("\n=== Test 3: Write 8 bits at index 0 ===");
    do_reset();
    send_write_cmd(32'd0, 32'd8, 32'h000000AB);
    check_dout(32'h000000AB, "write 8b@idx=0");

    // ---- Test 4: Write 8 bits at index 24 ----
    $display("\n=== Test 4: Write 8 bits at index 24 ===");
    // dout currently = 0x000000AB from Test 3; now write 0xCD into [31:24]
    send_write_cmd(32'd24, 32'd8, 32'h000000CD);
    check_dout(32'hCD0000AB, "write 8b@idx=24 retains lower bytes");

    // ---- Test 5: Write retains other bits ----
    $display("\n=== Test 5: Write retains other bits ===");
    do_reset();
    // Set full dout to known pattern first
    send_write_cmd(32'd0, 32'd32, 32'hFFFFFFFF);
    check_dout(32'hFFFFFFFF, "initial full write");
    // Overwrite only bits [7:4] with 0 → dout should be 0xFFFFFF0F
    send_write_cmd(32'd4, 32'd4, 32'h00000000);
    check_dout(32'hFFFFFF0F, "partial 4b@idx=4 retains surrounding bits");

    // ---- Test 6: Read partial 8 bits at index 8 ----
    $display("\n=== Test 6: Read 8 bits at index 8 ===");
    do_reset();
    din = 32'hDEADBEEF;
    // din[15:8] = 0xBE
    expect_read_response(din, 32'd8, 32'd8);   // expects 0xBE, then 'F'
    send_read_cmd(32'd8, 32'd8);
    wait_drain();

    // ---- Test 7: Read non-multiple-of-8 bits (zero-padding) ----
    $display("\n=== Test 7: Read 12 bits (zero-padding) ===");
    do_reset();
    // din = 0x00000FF0 → bits[4:15] = 0xFF0
    // Read 12 bits from idx=4:
    //   byte[0] = din[4..11]  = 0b11110000 = 0xF0  (bits 4..11, LSB-first → 0xFF? let's think)
    //   din[4]=1,din[5]=1,...din[11]=1 (8 ones) → byte[0] = 0xFF
    //   byte[1] = din[12]=1,din[13]=1,din[14]=0,din[15]=0,pad=0000 → 0x03
    //   Then 'F'
    din = 32'h00000FF0;
    expect_read_response(din, 32'd4, 32'd12);
    send_read_cmd(32'd4, 32'd12);
    wait_drain();

    // ---- Test 8: Read 1 bit ----
    $display("\n=== Test 8: Read 1 bit ===");
    do_reset();
    din = 32'h00000001;
    expect_read_response(din, 32'd0, 32'd1);   // expects 0x01, then 'F'
    send_read_cmd(32'd0, 32'd1);
    wait_drain();

    do_reset();
    din = 32'h00000000;
    expect_read_response(din, 32'd0, 32'd1);   // expects 0x00, then 'F'
    send_read_cmd(32'd0, 32'd1);
    wait_drain();

    // ---- Test 9: Sequential writes accumulate ----
    $display("\n=== Test 9: Sequential writes accumulate ===");
    do_reset();
    send_write_cmd(32'd0,  32'd8, 32'h000000AA);   // dout[7:0]   = 0xAA
    send_write_cmd(32'd8,  32'd8, 32'h000000BB);   // dout[15:8]  = 0xBB
    send_write_cmd(32'd16, 32'd8, 32'h000000CC);   // dout[23:16] = 0xCC
    send_write_cmd(32'd24, 32'd8, 32'h000000DD);   // dout[31:24] = 0xDD
    check_dout(32'hDDCCBBAA, "sequential 4x8b writes");

    // ---- Test 10: Reset mid-read operation ----
    $display("\n=== Test 10: Reset in middle of read ===");
    do_reset();
    din = 32'hCAFEBABE;
    fork
      begin : mid_read_sender
        // Send a full read command (9 UART bytes total)
        send_read_cmd(32'd0, 32'd32);
      end
      begin : mid_read_resetter
        // Wait partway through command reception (approx 3 frames in), then abort and reset
        repeat (3 * BAUD_DIV * 10) @(posedge clk);
        ->abort_rx_frame;
        #1;
        exp_q.delete();
        do_reset(4);
      end
    join_any
    disable fork;
    rx = 1'b1;

    repeat (BAUD_DIV * 2) @(posedge clk); #1;
    if (tx   !== 1'b1) begin $error("[%0t] TX not idle-high after mid-read reset", $time); errors++; end
    if (dout !== '0  ) begin $error("[%0t] dout not zero after mid-read reset",    $time); errors++; end
    // Verify normal operation resumes after reset
    din = 32'hDEADBEEF;
    expect_read_response(din, 32'd0, 32'd8);   // read 1 byte → 0xEF + 'F'
    send_read_cmd(32'd0, 32'd8);
    wait_drain();

    // ---- Test 11: Random write/read round-trips ----
    $display("\n=== Test 11: Random write/read round-trips ===");
    begin
      logic [DATA_WIDTH-1:0] rand_val;
      logic [DATA_WIDTH-1:0] expected_dout;
      int                    rand_idx;
      int                    rand_len;

      do_reset();
      expected_dout = '0;

      for (int iter = 0; iter < 8; iter++) begin
        // Pick a random aligned byte write to keep things simple
        rand_idx = ($urandom_range(0, (DATA_WIDTH/8) - 1)) * 8;
        rand_len = 8;
        rand_val = $urandom;

        // Update expected dout model
        for (int b = 0; b < rand_len; b++)
          expected_dout[rand_idx + b] = rand_val[b];

        send_write_cmd(rand_idx, rand_len, rand_val);
        check_dout(expected_dout, $sformatf("rand iter=%0d write idx=%0d", iter, rand_idx));

        // Now read back via din (set din = expected_dout for a round-trip check)
        din = expected_dout;
        expect_read_response(din, rand_idx, rand_len);
        send_read_cmd(rand_idx, rand_len);
        wait_drain();
      end
    end

    // ---- Summary ----
    $display("\n==================================================");
    $display(" Bytes driven  : %0d", bytes_driven);
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
    #(CLK_PERIOD * BAUD_DIV * 200000);
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
    $dumpfile("uart_pdi_tb.vcd");
    $dumpvars(0, uart_pdi_tb);
  end

endmodule
