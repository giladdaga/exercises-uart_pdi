`timescale 1ns / 1ps

// =============================================================================
// Module: deserializer
// Description: Receives serial data over a standard UART RX line using 8N1
//              framing (1 start bit, 8 data bits LSB-first, no parity, 1 stop
//              bit) at the configured baud rate, buffers received bytes in an
//              internal FIFO, and outputs them over a Ready/Valid interface.
//              The RX line is idle-high.
// =============================================================================
module deserializer #(
    parameter int DATA_WIDTH = 8,    // Must be 8 for standard UART
    parameter int FIFO_DEPTH = 16,   // Internal FIFO depth in bytes
    parameter int BAUD_DIV   = 868   // Clock cycles per baud period
                                     // e.g. 100 MHz / 115200 baud ≈ 868
) (
    input  logic                  clk,       // System clock
    input  logic                  rst_n,     // Active-low asynchronous reset

    // UART serial input
    input  logic                  rx,        // UART RX line (idle = 1)

    // Parallel output interface (Ready/Valid handshake)
    output logic [DATA_WIDTH-1:0] data_out,  // Received byte
    output logic                  valid_out, // High when data_out is valid
    input  logic                  ready_out, // High when downstream can accept a byte

    // Error line
    output logic                  error_out  // Error flag, high when stop bit 
                                             // is not high or when pushing data 
                                             // while FIFO is full
);

    localparam int BAUD_COUNTER_WIDTH = $clog2(BAUD_DIV);
    localparam int BIT_COUNTER_WIDTH  = $clog2(DATA_WIDTH);
    localparam int HALF_BAUD_DIV      = BAUD_DIV / 2;

    // =========================================================================
    // Internal FIFO
    // =========================================================================
    logic                  fifo_full;
    logic                  fifo_empty;
    logic                  fifo_write_en;
    logic                  fifo_read_en;
    logic [DATA_WIDTH-1:0] fifo_data_out;

    fifo #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .FIFO_WIDTH (DATA_WIDTH)
    ) u_fifo (
        .clk      (clk),
        .reset    (!rst_n),
        .write_en (fifo_write_en),
        .data_in  (shift_reg),
        .empty    (fifo_empty),
        .read_en  (fifo_read_en),
        .full     (fifo_full),
        .data_out (fifo_data_out)
    );

    // =========================================================================
    // Input Synchronizer (2-FF synchronizer for asynchronous RX line)
    // =========================================================================
    logic current_rx1; // 1st FF of rx, can be meta-stable.
    logic current_rx;
    logic prev_rx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_rx1 <= 1'b1;
            current_rx  <= 1'b1;
            prev_rx     <= 1'b1;
        end else begin
            current_rx1 <= rx;
            current_rx  <= current_rx1;
            prev_rx     <= current_rx;
        end
    end

    // rx falling edge is detected when the previous rx value was 1 and the current 
    // rx value is 0.
    // This is a necessary condition for the start of a UART frame.
    // A delay of 1 clock cycle is introduced to avoid metastability. 
    logic rx_falling_edge;
    assign rx_falling_edge = prev_rx && !current_rx;

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE  = 2'b00,   // RX idle (high), waiting for start bit falling edge
        START = 2'b01,   // Waiting half baud period to sample center of start bit
        DATA  = 2'b10,   // Sampling 8 data bits at bit centers, LSB first
        STOP  = 2'b11    // Sampling center of stop bit (high) and pushing to FIFO
    } state_t;

    state_t                        state;
    logic [BAUD_COUNTER_WIDTH-1:0] baud_counter; // Counts down within a baud period
    logic [BIT_COUNTER_WIDTH-1:0]  bit_counter;  // Current data bit index (0-7)
    logic [DATA_WIDTH-1:0]         shift_reg;    // Byte being assembled
    logic                          error_reg;    // Error flag

    // baud_tick is high at the last clock cycle of each baud period
    // half_baud_tick is high at the middle clock cycle of each baud period
    // (will be used when sampling the data, in order to avoid glitches)
    logic baud_tick;
    logic half_baud_tick;
    assign baud_tick      = (baud_counter == (BAUD_DIV - 1));
    assign half_baud_tick = (baud_counter == (HALF_BAUD_DIV - 1));

    // =========================================================================
    // 1. Combinational FIFO Write & Output Handshake (Skid/Prefetch Buffer)
    // =========================================================================
    // Write assembled byte into FIFO at the center of a valid (high) stop bit
    assign fifo_write_en = (state == STOP) && baud_tick;

    // data_out interface logic:
    // data_out uses skid logic, to allows the module to send a new output 
    // in each cycle.
    // If there's a byte in FIFO, and data_out is emptying (ready_out and 
    // valid_out are high) / empty (valid_out is 0) - request a read from FIFO.
    // In the next cycle, if ready_out is 0, save the output byte in a skid reg,
    // and read data_out and valid_out from the skid logic.
    logic                  fifo_data_valid;
    logic                  skid_data_valid;
    logic [DATA_WIDTH-1:0] skid_data;
    assign valid_out    = fifo_data_valid || skid_data_valid;
    assign data_out     = skid_data_valid ? skid_data : fifo_data_out;
    assign fifo_read_en = !fifo_empty && (!valid_out || ready_out);

    // =========================================================================
    // 2. Sequential Logic — RX baud counter, FSM, datapath, and output buffer
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            baud_counter <= '0;
            bit_counter  <= '0;
            shift_reg    <= '0;
            error_reg    <= '0;
        end else begin
            case (state)

                IDLE: begin
                    baud_counter <= '0;
                    bit_counter  <= '0;
                    if (rx_falling_edge) begin
                        state <= START;
                    end
                end

                START: begin
                    // Sample at the middle of the start bit to filter glitches
                    if (half_baud_tick) begin
                        baud_counter <= '0;
                        bit_counter  <= '0;
                        if (!current_rx) begin
                            state <= DATA;
                        end else begin
                            // False start bit - line returned high before the middle of 
                            // the start-bit window.
                            state <= IDLE;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                DATA: begin
                    // Sample every middle of data-bit window - count a full baud period 
                    // from the middle of the start-bit window.
                    if (baud_tick) begin
                        baud_counter <= '0;
                        shift_reg    <= {current_rx, shift_reg[DATA_WIDTH-1:1]}; // shift LSB in
                        if (bit_counter == (DATA_WIDTH - 1)) begin
                            bit_counter <= '0;
                            state       <= STOP;
                        end else begin
                            bit_counter <= bit_counter + 1'b1;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                STOP: begin
                    // Sample at the middle of the stop bit 
                    // (fifo_write_en is high at this cycle)
                    if (baud_tick) begin
                        baud_counter <= '0;
                        state        <= IDLE;
                        if (!current_rx || fifo_full)
                            error_reg <= '1;
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // data_out interface sequential logic:
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_data_valid <= 1'b0;
            skid_data_valid <= 1'b0;
            skid_data       <= '0;
        end else begin
            // FIFO read was issued in previout cycle, data is valid now
            fifo_data_valid <= fifo_read_en; 

            if (fifo_data_valid && !ready_out) begin
                // Downstream stalled: hold the new data in skid reg
                skid_data_valid <= 1'b1;
                skid_data       <= fifo_data_out;
            end else if (ready_out) begin
                // Reset skid_data_valid when data is read
                skid_data_valid <= 1'b0;
            end
        end
    end

endmodule