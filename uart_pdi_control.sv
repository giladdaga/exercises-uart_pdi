`timescale 1ns / 1ps

// =============================================================================
// Module: uart_pdi_control
// Description: Control FSM and datapath for UART Parallel Data Interface (PDI).
//              Coordinates command interpretation, wide bus reads from din to 
//              UART TX (serializer), and wide bus writes from UART RX 
//              (deserializer) to dout.
//
// Protocol:
//   READ:  RX receives 'R' | 4-byte index (BE) | 4-byte length (BE)
//          TX sends ceil(length/8) data bytes (LSB-first, zero-padded) | 'F'
//
//   WRITE: RX receives 'W' | 4-byte index (BE) | 4-byte length (BE)
//                      | ceil(length/8) data bytes (LSB-first) | 'F'
//          dout[index +: length] updated; rest retains previous value.
//          dout = 0 on reset.
// =============================================================================
module uart_pdi_control #(
    parameter int DATA_WIDTH      = 1024, // Wide parallel bus width; max 2^16 = 65536 bits
    parameter int BYTE_LENGTH     = 8,    // Number of bits per byte
    parameter int BYTES_FOR_LENGTH = 4,    // Number of bytes for length field
    parameter int BYTES_FOR_INDEX  = 4     // Number of bytes for index field
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // Deserializer interface (RX byte from external initiator)
    input  logic [BYTE_LENGTH-1:0] rx_byte,
    input  logic                  rx_byte_valid,
    output logic                  ready_to_receive,

    // Serializer interface (TX byte to external initiator)
    output logic [BYTE_LENGTH-1:0] tx_byte,
    output logic                  tx_byte_valid,
    input  logic                  ready_out,

    // Wide parallel bus interface
    input  logic [DATA_WIDTH-1:0] din,
    output logic [DATA_WIDTH-1:0] dout,

    // Protocol error flag
    output logic                  error_out
);

    // =========================================================================
    // Protocol Constants & Sizing Parameters
    // =========================================================================
    localparam logic [BYTE_LENGTH-1:0] CMD_R = 8'h52; // 'R' in ASCII
    localparam logic [BYTE_LENGTH-1:0] CMD_W = 8'h57; // 'W' in ASCII
    localparam logic [BYTE_LENGTH-1:0] CMD_F = 8'h46; // 'F' in ASCII

    localparam int BITS_FOR_INDEX   = BYTES_FOR_INDEX * BYTE_LENGTH;
    localparam int BITS_FOR_LENGTH  = BYTES_FOR_LENGTH * BYTE_LENGTH;
    localparam int BYTE_COUNTER_WIDTH = (BYTES_FOR_INDEX > BYTES_FOR_LENGTH) ? $clog2(BYTES_FOR_INDEX + 1) : $clog2(BYTES_FOR_LENGTH + 1);
    localparam int BIT_COUNTER_WIDTH  = ((BITS_FOR_LENGTH > 32) ? BITS_FOR_LENGTH : 32) + 1;

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [2:0] {
        ST_START        = 3'b000, // Idle, waiting for 'R' or 'W' opcode
        ST_GET_INDEX    = 3'b001, // Receiving index bytes (big-endian)
        ST_GET_LENGTH   = 3'b010, // Receiving length bytes (big-endian)
        ST_READ_DATA    = 3'b011, // Serializing din bits and transmitting to serializer
        ST_READ_FINISH  = 3'b100, // Transmitting 'F' to serializer
        ST_WRITE_DATA   = 3'b101, // Receiving write data bytes and updating dout
        ST_WRITE_FINISH = 3'b110  // Receiving 'F' from deserializer
    } state_t;

    typedef enum logic {
        OP_READ  = 1'b0,
        OP_WRITE = 1'b1
    } op_t;

    state_t                          state;
    op_t                             op;
    logic [BITS_FOR_INDEX-1:0]       index_reg;
    logic [BITS_FOR_LENGTH-1:0]      length_reg;
    logic [BYTE_COUNTER_WIDTH-1:0]   byte_counter; // Will be used when receiving index & length
    logic [BIT_COUNTER_WIDTH-1:0]    bit_counter; // Will be used when engaging din & dout
    logic [DATA_WIDTH-1:0]           din_latch;
    logic                            error_reg;

    // Handshake indicators
    logic rx_handshake;
    logic tx_handshake;
    assign rx_handshake = rx_byte_valid && ready_to_receive;
    assign tx_handshake = tx_byte_valid && ready_out;

    // Helper: evaluates full length on the last incoming length byte
    logic [BITS_FOR_LENGTH-1:0] incoming_length;
    always_comb begin
        incoming_length = length_reg;
        incoming_length[0 +: BYTE_LENGTH] = rx_byte;
    end

    // =========================================================================
    // 1. Combinational Outputs
    // =========================================================================
    assign error_out = error_reg;

    // Ready to receive a byte from deserializer in input states
    assign ready_to_receive = (state == ST_START)        ||
                              (state == ST_GET_INDEX)    ||
                              (state == ST_GET_LENGTH)   ||
                              (state == ST_WRITE_DATA)   ||
                              (state == ST_WRITE_FINISH);

    // TX byte valid in output states
    assign tx_byte_valid = (state == ST_READ_DATA) || (state == ST_READ_FINISH);

    // Construct TX byte: LSB-first with zero-padding on last byte
    always_comb begin
        tx_byte = '0; // Default TX

        // TX in ST_READ_DATA state is din[index+counter]:
        if (state == ST_READ_DATA) begin
            for (int i = 0; i < BYTE_LENGTH; i++) begin
                if ((bit_counter + i) < length_reg) begin
                    if ((index_reg + bit_counter + i) < DATA_WIDTH) begin
                        tx_byte[i] = din_latch[index_reg + bit_counter + i];
                    end else begin
                        tx_byte[i] = 1'b0; // Zero-pad if byte is end of din is reached
                    end
                end else begin
                    tx_byte[i] = 1'b0; // Zero-pad if final byte is incompleted
                end
            end

        // TX in ST_READ_FINISH state is 'F':
        end else if (state == ST_READ_FINISH) begin
            tx_byte = CMD_F;
        end
    end

    // =========================================================================
    // 2. Sequential Logic — FSM, Datapath, dout
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_START;
            op           <= OP_READ;
            index_reg    <= '0;
            length_reg   <= '0;
            byte_counter <= '0;
            bit_counter  <= '0;
            din_latch    <= '0;
            dout         <= '0;
            error_reg    <= 1'b0;
        end else begin
            case (state)

                // -------------------------------------------------------------
                // ST_START: Wait for 'R' or 'W' command opcode
                // -------------------------------------------------------------
                ST_START: begin
                    byte_counter <= '0;
                    bit_counter  <= '0;
                    if (rx_handshake) begin
                        state <= ST_GET_INDEX;
                        if (rx_byte == CMD_R)
                            op    <= OP_READ;
                        else if (rx_byte == CMD_W)
                            op    <= OP_WRITE;
                        else
                            error_reg <= 1'b1; // Invalid opcode: self-loop in ST_START & raise error flag
                    end
                end

                // -------------------------------------------------------------
                // ST_GET_INDEX: Receive 4-byte index field (big-endian)
                // -------------------------------------------------------------
                ST_GET_INDEX: begin
                    if (rx_handshake) begin
                        index_reg[(BYTES_FOR_INDEX - 1 - byte_counter)*BYTE_LENGTH +: BYTE_LENGTH] <= rx_byte;
                        if (byte_counter == (BYTES_FOR_INDEX - 1)) begin
                            // Index transactions finished
                            byte_counter <= '0;
                            state        <= ST_GET_LENGTH;
                        end else begin
                            byte_counter <= byte_counter + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // ST_GET_LENGTH: Receive 4-byte length field (big-endian)
                // -------------------------------------------------------------
                ST_GET_LENGTH: begin
                    if (rx_handshake) begin
                        length_reg[(BYTES_FOR_LENGTH - 1 - byte_counter)*BYTE_LENGTH +: BYTE_LENGTH] <= rx_byte;
                        if (byte_counter == (BYTES_FOR_LENGTH - 1)) begin
                            // Length transaction finished
                            byte_counter <= '0;
                            bit_counter  <= '0;
                            din_latch    <= din; // Atomically capture parallel input
                            if (incoming_length == '0) begin // If length == 0
                                state <= (op == OP_READ) ? ST_READ_FINISH : ST_WRITE_FINISH;
                            end else begin
                                state <= (op == OP_READ) ? ST_READ_DATA : ST_WRITE_DATA;
                            end
                        end else begin
                            byte_counter <= byte_counter + 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // ST_READ_DATA: Stream serialized bytes to serializer
                // -------------------------------------------------------------
                ST_READ_DATA: begin
                    if (tx_handshake) begin
                        if ((bit_counter + BYTE_LENGTH) >= length_reg) begin
                            // Current byte is last one
                            bit_counter <= '0;
                            state       <= ST_READ_FINISH;
                        end else begin
                            bit_counter <= bit_counter + BYTE_LENGTH;
                        end
                    end
                end

                // -------------------------------------------------------------
                // ST_READ_FINISH: Transmit 'F' delimiter to complete READ
                // -------------------------------------------------------------
                ST_READ_FINISH: begin
                    if (tx_handshake) begin
                        state <= ST_START;
                    end
                end

                // -------------------------------------------------------------
                // ST_WRITE_DATA: Receive data bytes and write into dout
                // -------------------------------------------------------------
                ST_WRITE_DATA: begin
                    if (rx_handshake) begin
                        for (int i = 0; i < BYTE_LENGTH; i++) begin
                            if ((bit_counter + i) < length_reg) begin
                                // Make sure to stay within length
                                if ((index_reg + bit_counter + i) < DATA_WIDTH) begin
                                    // Make sure to stay within dout width
                                    dout[index_reg + bit_counter + i] <= rx_byte[i];
                                end
                            end
                        end
                        if ((bit_counter + BYTE_LENGTH) >= length_reg) begin
                            // Current byte is last one
                            bit_counter <= '0;
                            state       <= ST_WRITE_FINISH;
                        end else begin
                            bit_counter <= bit_counter + BYTE_LENGTH;
                        end
                    end
                end

                // -------------------------------------------------------------
                // ST_WRITE_FINISH: Verify 'F' delimiter from initiator
                // -------------------------------------------------------------
                ST_WRITE_FINISH: begin
                    if (rx_handshake) begin
                        state <= ST_START;
                        if (rx_byte != CMD_F) begin
                            error_reg <= 1'b1; // Protocol error: missing 'F' delimiter
                        end
                    end
                end

                default: state <= ST_START;
            endcase
        end
    end

endmodule
