`timescale 1ns / 1ps

module uart_pdi_control  // uart_pdi control module
  #(parameter DATA_WIDTH = 1024,           // Parameterized bus width; max 2^16 = 65536 bits
    parameter BYTE_LENGTH = 8,             // Number of bits per byte
    parameter BITS_FOR_LENGTH = 4,         // Number of bits for length field
    parameter BITS_FOR_INDEX = 4,          // Number of bits for index field
) (
    input logic clk,                        // System clock
    input logic rst_n,                      // Active-low synchronous reset
    
    input logic rx_byte,                    // Input byte from deserializer
    input logic rx_byte_vailid              // High when rx_byte is vaild
    output logic ready_to_receive           // Ready to receive next byte from deserializer

    output logic tx,                        // Output byte to serializer
    output logic tx_byte_vaild              // High when tx_byte is vaild
    input logic ready_out                   // serializer is ready to receive next byte
    
    input logic [DATA_WIDTH-1:0] din,       // Wide parallel data input (read by UPF)
    output logic [DATA_WIDTH-1:0] dout      // Wide parallel data output (written by UPF)
);




endmodule