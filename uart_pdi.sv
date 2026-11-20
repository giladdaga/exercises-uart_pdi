`timescale 1ns / 1ps

// =============================================================================
// Module: uart_pdi
// Description: An interface that uses UART protocol for communication between 
//              two agents - UPF and external initiator.
//              The read and write operations protocol is as descripted below:

// Read op protocol:
// The module will serialise the requested number of bits, beginning from the requested bit-index, from the input din.
// LSB first within each byte, zero-padded if length is not a multiple of 8.
// The bytes will be transmitted over TX. The keyword 'F' will mark the end of the transaction.
// Input  (RX):     |   'R'   |  idx[3] |  idx[2] |  idx[1] |  idx[0] |  len[3] |  len[2] |  len[1] |  len[0] |
// Output (TX):     | data[0] | data[1] |  .....  | data[N] |   'F'   |

// Write op protocol:
// After receiving 'W', the module will transmit the received data over dout.
// The new data will begin at the requested bit-index. The rest of the data will retain the previous value.
// When reset = 1, dout will be set to 0.
// Input  (RX):     |   'W'   |  idx[3] |  idx[2] |  idx[1] |  idx[0] |  len[3] |  len[2] |  len[1] |  len[0] | 
//                  | data[0] | data[1] |  .....  | data[N] |   'F'   |

// 'R' = 0x52, 'W' = 0x57, 'F' = 0x46.
// The module is a synthesizeable, parameterised verilog code.
// The protocol uses big-endian encoding for the 4-byte length & index fields.
// If bit_index + length > DATA_WIDTH, the behavior is unspecified.
// Assume 8N1 format: 8 data bits, no parity, 1 stop bit. Baud rate is an implementation parameter.
// =============================================================================


module uart_pdi  // UART Parallel Data Interface
  #(parameter DATA_WIDTH = 1024,           // Parameterized bus width; max 2^16 = 65536 bits
    parameter BYTE_LENGTH = 8,             // Number of bits per byte
    parameter BYTES_FOR_LENGTH = 4,         // Number of bits for length field
    parameter BYTES_FOR_INDEX = 4,          // Number of bits for index field
    parameter BAUD_DIV = 868               // Clock cycles per baud period
                                           // e.g. 100 MHz / 115200 baud ≈ 868
) (
    input wire clk,                        // System clock
    input wire rst_n,                      // Active-low synchronous reset
    input wire rx,                         // UART RX (serial input from external initiator)
    output wire tx,                        // UART TX (serial output to external initiator)
    input logic [DATA_WIDTH-1:0] din,      // Wide parallel data input (read by UPF)
    output logic [DATA_WIDTH-1:0] dout     // Wide parallel data output (written by UPF)
);


    // =========================================================================
    // Wires declaration
    // =========================================================================

    logic [BYTE_LENGTH-1:0] rx_byte;
    logic rx_byte_valid;
    logic uart_ready;
    logic deserializer_error;

    logic [BYTE_LENGTH-1:0] tx_byte;
    logic tx_byte_valid;
    logic serializer_ready;
    logic serializer_error;

    logic uart_pdi_control_error;

    // =========================================================================
    // Modules instantiation
    // =========================================================================

    deserializer #(
        .DATA_WIDTH(BYTE_LENGTH),
        .BAUD_DIV(BAUD_DIV)
    ) u_deserializer (
        .clk(clk),
        .rst_n(rst_n),
        .rx(rx),
        .data_out(rx_byte),
        .valid_out(rx_byte_valid),
        .ready_out(uart_ready),
        .error_out(deserializer_error)
    );

    serializer #(
        .DATA_WIDTH(BYTE_LENGTH),
        .BAUD_DIV(BAUD_DIV)
    ) u_serializer (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(tx_byte),
        .valid_in(tx_byte_valid),
        .ready_in(serializer_ready),
        .tx(tx)
    );

    uart_pdi_control #(
        .DATA_WIDTH(DATA_WIDTH),
        .BYTE_LENGTH(BYTE_LENGTH),
        .BYTES_FOR_LENGTH(BYTES_FOR_LENGTH),
        .BYTES_FOR_INDEX(BYTES_FOR_INDEX)
    ) u_uart_pdi_control (
        .clk(clk),
        .rst_n(rst_n),
        .rx_byte(rx_byte),
        .rx_byte_valid(rx_byte_valid),
        .ready_to_receive(uart_ready),
        .tx_byte(tx_byte),
        .tx_byte_valid(tx_byte_valid),
        .ready_out(serializer_ready),
        .din(din),
        .dout(dout),
        .error_out(uart_pdi_control_error)
    );



endmodule
