# uart_pdi_exercise
UART_PDI exercise: module design and testbench


Assumptions:

- Read op protocol:
The module will serialise the requested number of bits, beginning from the requested bit-index, from the input din.
LSB first within each byte, zero-padded if length is not a multiple of 8.
The bytes will be transmitted over TX. The keyword 'F' will mark the end of the transaction.
Input  (RX):     |   'R'   |  idx[3] |  idx[2] |  idx[1] |  idx[0] |  len[3] |  len[2] |  len[1] |  len[0] |
Output (TX):     | data[0] | data[1] |  .....  | data[N] |   'F'   |

- Write op protocol:
After receiving 'W', the module will transmit the received data over dout.
The new data will begin at the requested bit-index. The rest of the data will retain the previous value.
When reset = 1, dout will be set to 0.
Input  (RX):     |   'W'   |  idx[3] |  idx[2] |  idx[1] |  idx[0] |  len[3] |  len[2] |  len[1] |  len[0] | 
                 | data[0] | data[1] |  .....  | data[N] |   'F'   |

- 'R' = 0x52, 'W' = 0x57, 'F' = 0x46.
- The module is a synthesizeable, parameterised verilog code.
- The protocol uses big-endian encoding for the 4-byte length & index fields.
- If bit_index + length > DATA_WIDTH, the behavior is unspecified.
- Assume 8N1 format: 8 data bits, no parity, 1 stop bit. Baud rate is an implementation parameter.