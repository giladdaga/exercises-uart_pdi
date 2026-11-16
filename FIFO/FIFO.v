`timescale 1ns / 1ps

module fifo #(parameter FIFO_DEPTH=16 , FIFO_WIDTH=8) (

 input clk, // Clock
 input reset, // Reset

 // Write Port

 input write_en, // Write enable
 input [FIFO_WIDTH-1:0] data_in, // FIFO Input Data
 output empty, // FIFO empty indication

 // Read Port

 input read_en, // Read enable
 output full, // FIFO full indication
 output reg [FIFO_WIDTH-1:0] data_out // FIFO Output Data
);
    /*
    NOTES:
    - Reset is synchronic.
    - Reset overrides everything.
    - Read op can only read if fifo is not empty.
    - Write op can only write if fifo is not full, or if fifo is full and there's a simultaneous read op.
    - No overflow or underflow protection is implemented.
    - The FIFO won't output data that has been written in the same cycle.
    */

    localparam COUNTER_WIDTH = $clog2(FIFO_DEPTH); // Defines counter width

    reg [FIFO_WIDTH-1:0] memory [FIFO_DEPTH-1:0];
    reg [COUNTER_WIDTH:0] counter;

    // Logic expressions for full and empty
    assign empty = (counter == 0);
    assign full = (counter == FIFO_DEPTH);

    always @(posedge clk) begin
        // Reset operation - overrides everything. Synchronic reset.
        if (reset) begin
            data_out <= 0;
            counter  <= 0;
            for (int i = 0 ; i < FIFO_DEPTH ; i = i+1) memory[i] <= 0;
        end else begin
            // Read & Write operation
            if (read_en && write_en && !empty) begin
                for (int i = 1; i < counter; i = i+1) memory[i-1] <= memory[i];
                data_out          <= memory[0];
                memory[counter-1] <= data_in;
            end

            // Read operation
            else if (read_en && !empty) begin
                for (int i = 1; i < counter; i = i+1) memory[i-1] <= memory[i];
                data_out          <= memory[0];
                memory[counter-1] <= 0;
                counter           <= counter - 1;
            end


            // Write operation
            else if (write_en && !full) begin 
                counter         <= counter + 1;
                memory[counter] <= data_in;
            end


            // No operation
            else if (!read_en && !write_en) begin
                data_out <= 0;
            end
        end
    end

endmodule