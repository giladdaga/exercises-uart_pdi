`timescale 1ns / 1ps

// =============================================================================
// Module: serializer
// Description: Reads bytes over a Ready/Valid interface, store them in an
//              internal FIFO and transmits them over a standard UART TX line 
//              using 8N1 framing (1 start bit, 8 data bits LSB-first, no 
//              parity, 1 stop bit) at the configured baud rate. The TX line is 
//              idle-high.
// =============================================================================
module serializer #(
    parameter int DATA_WIDTH = 8,    // Must be 8 for standard UART
    parameter int FIFO_DEPTH = 16,   // Internal FIFO depth in bytes
    parameter int BAUD_DIV   = 868   // Clock cycles per baud period
                                     // e.g. 100 MHz / 115200 baud ≈ 868
) (
    input  logic                  clk,      // System clock
    input  logic                  rst_n,    // Active-low asynchronous reset

    // Parallel input interface (Ready/Valid handshake)
    input  logic [DATA_WIDTH-1:0] data_in,  // Byte to transmit
    input  logic                  valid_in, // High when data_in is valid
    output logic                  ready_in, // High when FIFO can accept a byte

    // UART serial output
    output logic                  tx        // UART TX line (idle = 1)
);

    localparam int BAUD_COUNTER_WIDTH  = $clog2(BAUD_DIV);
    localparam int BIT_COUNTER_WIDTH   = $clog2(DATA_WIDTH);

    // =========================================================================
    // Internal FIFO
    // =========================================================================
    logic                  fifo_full;
    logic                  fifo_empty;
    logic                  fifo_read_en;
    logic [DATA_WIDTH-1:0] fifo_data_out;

    fifo #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .FIFO_WIDTH (DATA_WIDTH)
    ) u_fifo (
        .clk      (clk),
        .reset    (!rst_n),
        .write_en (valid_in && !fifo_full),
        .data_in  (data_in),
        .empty    (fifo_empty),
        .read_en  (fifo_read_en),
        .full     (fifo_full),
        .data_out (fifo_data_out)
    );

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE  = 2'b00,   // TX idle (high), waiting for data
        START = 2'b01,   // Transmitting start bit (low)
        DATA  = 2'b10,   // Transmitting 8 data bits, LSB first
        STOP  = 2'b11    // Transmitting stop bit (high)
    } state_t;

    state_t                        state;
    logic [BAUD_COUNTER_WIDTH-1:0] baud_counter; // Counts down within a baud period
    logic [BIT_COUNTER_WIDTH-1:0]  bit_counter;  // Current data bit index (0-7)
    logic [DATA_WIDTH-1:0]         shift_reg;    // Byte being transmitted

    // baud_tick is high at the last clock cycle of each baud period
    logic baud_tick;
    assign baud_tick = (baud_counter == (BAUD_DIV - 1));

    // =========================================================================
    // 1. Combinational Outputs
    // =========================================================================
    assign ready_in  = !fifo_full;

    // TX is combinatorially driven from current state/shift_reg
    always_comb begin
        case (state)
            IDLE:    tx = 1'b1;
            START:   tx = 1'b0;
            DATA:    tx = shift_reg[0];
            STOP:    tx = 1'b1;
            default: tx = 1'b1;
        endcase
    end

    // FIFO read: issued in IDLE (for the first byte) or at the last baud cycle
    // of STOP when the next byte is already waiting (back-to-back frames).
    assign fifo_read_en = ((state == IDLE) && !fifo_empty) ||
                          ((state == STOP) && baud_tick && !fifo_empty);

    // =========================================================================
    // 2. Sequential Logic — baud counter, FSM, datapath
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            baud_counter <= '0;
            bit_counter  <= '0;
            shift_reg    <= '0;
        end else begin
            case (state)

                IDLE: begin
                    baud_counter <= '0;
                    bit_counter  <= '0;
                    if (!fifo_empty) begin
                        // FIFO read issued this cycle; data arrives next cycle
                        state <= START;
                    end
                end

                START: begin
                    // Load byte from FIFO on the first cycle of START
                    // (FIFO read was issued in IDLE, data valid now)
                    if (baud_counter == '0)
                        shift_reg <= fifo_data_out;

                    if (baud_tick) begin
                        // Finished transmission of start bit
                        baud_counter <= '0;
                        bit_counter  <= '0;
                        state        <= DATA;
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                DATA: begin
                    if (baud_tick) begin
                        baud_counter  <= '0;
                        if (bit_counter == (DATA_WIDTH - 1)) begin
                            bit_counter <= '0;
                            state       <= STOP;
                        end else begin
                            shift_reg   <= {1'b0, shift_reg[DATA_WIDTH-1:1]}; // shift LSB out
                            bit_counter <= bit_counter + 1'b1;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                STOP: begin
                    if (baud_tick) begin
                        baud_counter <= '0;
                        if (!fifo_empty) begin
                            // Back-to-back: FIFO read issued this cycle (via fifo_read_en),
                            // data will be valid next cycle — go to START to load it.
                            state <= START;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        baud_counter <= baud_counter + 1'b1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
