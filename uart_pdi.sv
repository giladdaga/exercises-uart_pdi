`timescale 1ns / 1ps

module uart_pdi  // UART Parallel Data Interface
  #(parameter DATA_WIDTH = 1024,           // Parameterized bus width; max 2^16 = 65536 bits
    parameter BYTE_LENGTH = 8,             // Number of bits per byte
    parameter BITS_FOR_LENGTH = 4,         // Number of bits for length field
    parameter BITS_FOR_INDEX = 4           // Number of bits for index field
) (
    input wire clk,                        // System clock
    input wire rst_n,                      // Active-low synchronous reset
    input wire rx,                         // UART RX (serial input from external initiator)
    output wire tx,                        // UART TX (serial output to external initiator)
    input logic [DATA_WIDTH-1:0] din,      // Wide parallel data input (read by UPF)
    output logic [DATA_WIDTH-1:0] dout     // Wide parallel data output (written by UPF)
);

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



reg [BITS_FOR_INDEX-1:0] index [BYTE_LENGTH-1:0];
reg [BITS_FOR_LENGTH-1:0] length [BYTE_LENGTH-1:0];
reg [DATA_WIDTH-1:0] din_reg;
reg [DATA_WIDTH-1:0] dout_reg;

always @(posedge clk) begin
    if (!rst_n) begin
        dout_reg <= 0;
    end
    else begin
        din_reg <= din;
        dout_reg <= din;
    end
end

assign dout = dout_reg;


endmodule