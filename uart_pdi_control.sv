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
    parameter int BITS_FOR_LENGTH = 4,    // Number of bits for length field
    parameter int BITS_FOR_INDEX  = 4     // Number of bits for index field
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
    localparam logic [BYTE_LENGTH-1:0] CMD_R = 8'h52; // 'R' = Read command
    localparam logic [BYTE_LENGTH-1:0] CMD_W = 8'h57; // 'W' = Write command
    localparam logic [BYTE_LENGTH-1:0] CMD_F = 8'h46; // 'F' = End of transaction

    localparam int TOTAL_INDEX_BITS   = BITS_FOR_INDEX * BYTE_LENGTH;
    localparam int TOTAL_LENGTH_BITS  = BITS_FOR_LENGTH * BYTE_LENGTH;
    localparam int MAX_FIELD_BYTES    = (BITS_FOR_INDEX > BITS_FOR_LENGTH) ? BITS_FOR_INDEX : BITS_FOR_LENGTH;
    localparam int BYTE_COUNTER_WIDTH = $clog2(MAX_FIELD_BYTES + 1);
    localparam int BIT_COUNTER_WIDTH  = (TOTAL_LENGTH_BITS > 32 ? TOTAL_LENGTH_BITS : 32) + 1;

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [2:0] {
        ST_START        = 3'b000, // Idle, waiting for 'R' or 'W' opcode
        ST_GET_INDEX    = 3'b001, // Receiving index bytes (big-endian)
        ST_GET_LENGTH   = 3'b010, // Receiving length bytes (big-endian)
        ST_READ_DATA    = 3'b011, // Serializing din bits and transmitting to serializer
        ST_READ_FINISH  = 3'b100, // Transmitting 'F' delimiter to serializer
        ST_WRITE_DATA   = 3'b101, // Receiving write data bytes and updating dout
        ST_WRITE_FINISH = 3'b110  // Awaiting 'F' delimiter from deserializer
    } state_t;

    typedef enum logic {
        OP_READ  = 1'b0,
        OP_WRITE = 1'b1
    } op_t;

    state_t                          state;
    op_t                             op;
    logic [TOTAL_INDEX_BITS-1:0]     index_reg;
    logic [TOTAL_LENGTH_BITS-1:0]    length_reg;
    logic [BYTE_COUNTER_WIDTH-1:0]   byte_counter;
    logic [BIT_COUNTER_WIDTH-1:0]    bit_counter;
    logic [DATA_WIDTH-1:0]           din_latch;
    logic                            error_reg;

    // Handshake indicators
    logic rx_handshake;
    logic tx_handshake;
    assign rx_handshake = rx_byte_valid && ready_to_receive;
    assign tx_handshake = tx_byte_valid && ready_out;

    // Helper: evaluates full length on the last incoming length byte
    logic [TOTAL_LENGTH_BITS-1:0] incoming_length;
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

    // TX byte valid when streaming data or end-of-frame marker to serializer
    assign tx_byte_valid = (state == ST_READ_DATA) || (state == ST_READ_FINISH);

    // Construct TX byte: LSB-first with zero-padding on last byte
    always_comb begin
        tx_byte = '0;
        if (state == ST_READ_DATA) begin
            for (int i = 0; i < BYTE_LENGTH; i++) begin
                if ((bit_counter + i) < length_reg) begin
                    if ((index_reg + bit_counter + i) < DATA_WIDTH) begin
                        tx_byte[i] = din_latch[index_reg + bit_counter + i];
                    end else begin
                        tx_byte[i] = 1'b0;
                    end
                end else begin
                    tx_byte[i] = 1'b0; // Zero-pad incomplete final byte
                end
            end
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
                        if (rx_byte == CMD_R) begin
                            op    <= OP_READ;
                            state <= ST_GET_INDEX;
                        end else if (rx_byte == CMD_W) begin
                            op    <= OP_WRITE;
                            state <= ST_GET_INDEX;
                        end
                        // Invalid opcode: self-loop in ST_START
                    end
                end

                // -------------------------------------------------------------
                // ST_GET_INDEX: Receive 4-byte index field (big-endian)
                // -------------------------------------------------------------
                ST_GET_INDEX: begin
                    if (rx_handshake) begin
                        index_reg[(BITS_FOR_INDEX - 1 - byte_counter)*BYTE_LENGTH +: BYTE_LENGTH] <= rx_byte;
                        if (byte_counter == (BITS_FOR_INDEX - 1)) begin
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
                        length_reg[(BITS_FOR_LENGTH - 1 - byte_counter)*BYTE_LENGTH +: BYTE_LENGTH] <= rx_byte;
                        if (byte_counter == (BITS_FOR_LENGTH - 1)) begin
                            byte_counter <= '0;
                            bit_counter  <= '0;
                            din_latch    <= din; // Atomically capture parallel input
                            if (incoming_length == '0) begin
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
                                if ((index_reg + bit_counter + i) < DATA_WIDTH) begin
                                    dout[index_reg + bit_counter + i] <= rx_byte[i];
                                end
                            end
                        end
                        if ((bit_counter + BYTE_LENGTH) >= length_reg) begin
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
