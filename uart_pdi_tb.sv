`timescale 1ns / 1ps

module uart_pdi_tb#(
    parameter DATA_WIDTH = 8,
    parameter BYTE_LENGTH = 8,
    parameter BITS_FOR_LENGTH = 4,
    parameter BITS_FOR_INDEX = 4
);

    // =========================================================================
    // 1. Parameters & Constants
    // =========================================================================
    localparam time CLK_PERIOD = 10ns;      // Clock period (e.g. 100 MHz)
    localparam time TIMEOUT    = 100000ns; // Simulation watchdog limit
    

    // =========================================================================
    // 2. Signals (SystemVerilog 'logic')
    // =========================================================================
    // Standard control signals
    logic clk;
    logic rst_n;

    // DUT interface signals placeholder
    logic rx;                               // UART RX (serial input from external initiator)
    logic tx;                               // UART TX (serial output to external initiator)
    logic [DATA_WIDTH-1:0] din;             // Data IN (input to UFP)
    logic [DATA_WIDTH-1:0] dout;            // Data OUT (output from UFP)


    // =========================================================================
    // 3. Clock Generation
    // =========================================================================
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end


    // =========================================================================
    // 4. Waveform Dump (SimVision / GTKWave)
    // =========================================================================
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, uart_pdi_tb);
    end


    // =========================================================================
    // 5. Watchdog / Timeout Mechanism
    // =========================================================================
    initial begin
        #TIMEOUT;
        $display("\n[ERROR] Simulation reached timeout (%0t) - Aborting!", $time);
        $finish;
    end


    // =========================================================================
    // 6. Device Under Test (DUT) Instantiation
    // =========================================================================
    uart_pdi  #(
        .DATA_WIDTH(DATA_WIDTH),
        .BYTE_LENGTH(BYTE_LENGTH)
    ) uart_pdi_dut (
        .clk(clk),
        .rst_n(rst_n),
        .rx(rx),
        .tx(tx),
        .din(din),
        .dout(dout)
    );
    

    // =========================================================================
    // 7. Helper Tasks
    // =========================================================================
    task automatic reset_sequence();
        rst_n <= 1'b0;                      // Assert reset
        repeat (5) @(posedge clk);          // Wait cycles
        rst_n <= 1'b1;                      // Deassert reset
        @(posedge clk);
        $display("[INFO] System reset completed at time %0t", $time);
    endtask


    // =========================================================================
    // 8. Main Test Process
    // =========================================================================
    initial begin
        // Initialize inputs
        din = '0;
        rx = 1'b1;

        // Apply reset sequence
        reset_sequence();

        // Stimulus execution
        $display("[INFO] Starting test scenarios at time %0t", $time);

        // Add test vectors / stimulus here
        repeat (10) @(posedge clk);

        // Finish simulation
        $display("\n==========================================");
        $display("   TEST PASSED SUCCESSFULLY at %0t", $time);
        $display("==========================================\n");
        $finish;
    end

endmodule
