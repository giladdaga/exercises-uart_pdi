`timescale 1ns / 1ps

module serializer #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 16
) (
    input  logic                  clk,       // System clock
    input  logic                  rst_n,     // Active-low asynchronous reset
    
    // Parallel input interface
    input  logic [DATA_WIDTH-1:0] data_in,   // 8-bit parallel byte input
    input  logic                  valid_in,  // High when input byte is valid
    output logic                  ready_in,  // High when serializer FIFO can accept data (!full)
    
    // Serial output interface
    output logic                  data_out,  // Serial bit output
    output logic                  valid_out  // High when data_out contains a valid bit
);

    // =========================================================================
    // Internal FIFO Signals
    // =========================================================================
    logic                  fifo_full;
    logic                  fifo_empty;
    logic                  fifo_read_en;
    logic [DATA_WIDTH-1:0] fifo_data_out;

    // Ready to receive if internal FIFO is not full
    assign ready_in = !fifo_full;

    // FIFO Instantiation (From Projects/FIFO/FIFO.v)
    fifo #(
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_WIDTH(DATA_WIDTH)
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
    // State Machine & Shift Register Logic
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE      = 2'b00,
        FETCH     = 2'b01,
        SERIALIZE = 2'b10
    } state_t;

    state_t state;

    logic [DATA_WIDTH-1:0] shift_reg;
    logic [$clog2(DATA_WIDTH):0] bit_idx; // Counts from 0 to DATA_WIDTH-1

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            shift_reg    <= '0;
            bit_idx      <= '0;
            fifo_read_en <= 1'b0;
            data_out     <= 1'b0;
            valid_out    <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    valid_out <= 1'b0;
                    data_out  <= 1'b0;
                    
                    // If FIFO has data, request the first byte
                    if (!fifo_empty) begin
                        fifo_read_en <= 1'b1;
                        state        <= FETCH;
                    end else begin
                        fifo_read_en <= 1'b0;
                    end
                end

                FETCH: begin
                    // FIFO read_en was asserted in previous cycle,
                    // so fifo_data_out is available at this clock edge.
                    fifo_read_en <= 1'b0;
                    shift_reg    <= fifo_data_out;
                    data_out     <= fifo_data_out[0]; // Output LSB first
                    valid_out    <= 1'b1;
                    bit_idx      <= 1;
                    state        <= SERIALIZE;
                end

                SERIALIZE: begin
                    valid_out <= 1'b1;
                    data_out  <= shift_reg[bit_idx];

                    // Check if we are approaching the last bit (bit 6 out of 0..7)
                    // If FIFO has another byte, request a read in order to pre-fetch it,
                    // so there is no bubble between bytes
                    if ((bit_idx == DATA_WIDTH - 2) && !fifo_empty) begin
                        fifo_read_en <= 1'b1;
                    end else begin
                        fifo_read_en <= 1'b0;
                    end

                    // When last bit is reached (bit 7)
                    if (bit_idx == DATA_WIDTH - 1) begin
                        if (fifo_read_en) begin
                            // Pre fetch the new byte
                            shift_reg <= fifo_data_out;
                            data_out  <= fifo_data_out[0];
                            bit_idx   <= 1;
                            state     <= SERIALIZE;
                        end else if (!fifo_empty) begin
                            // If there wasn't a pre-fetch but there is data in the FIFO now
                            fifo_read_en <= 1'b1;
                            state        <= FETCH;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        bit_idx <= bit_idx + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

