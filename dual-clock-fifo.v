//============================================================================
// Dual Clock FIFO - For clock domain crossing between UART and system clocks
//============================================================================
module dual_clock_fifo #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 64,
    parameter ADDR_WIDTH = $clog2(FIFO_DEPTH)
)(
    // Write domain
    input wire wr_clk,
    input wire wr_rst,
    input wire [DATA_WIDTH-1:0] wr_data,
    input wire wr_en,
    output reg wr_full,

    // Read domain  
    input wire rd_clk,
    input wire rd_rst,
    output reg [DATA_WIDTH-1:0] rd_data,
    input wire rd_en,
    output reg rd_empty
);

    // Memory array
    reg [DATA_WIDTH-1:0] memory [0:FIFO_DEPTH-1];

    // Gray code pointers
    reg [ADDR_WIDTH:0] wr_ptr_gray, wr_ptr_gray_next;
    reg [ADDR_WIDTH:0] rd_ptr_gray, rd_ptr_gray_next;
    reg [ADDR_WIDTH:0] wr_ptr_bin, wr_ptr_bin_next;
    reg [ADDR_WIDTH:0] rd_ptr_bin, rd_ptr_bin_next;

    // Synchronized pointers
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;

    // Binary to Gray conversion function
    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] binary;
        begin
            bin2gray = binary ^ (binary >> 1);
        end
    endfunction

    // Write domain logic
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_ptr_bin <= 0;
            wr_ptr_gray <= 0;
            rd_ptr_gray_sync1 <= 0;
            rd_ptr_gray_sync2 <= 0;
        end else begin
            wr_ptr_bin <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    always @(*) begin
        wr_ptr_bin_next = wr_ptr_bin;
        if (wr_en && !wr_full) begin
            wr_ptr_bin_next = wr_ptr_bin + 1;
        end
        wr_ptr_gray_next = bin2gray(wr_ptr_bin_next);
    end

    // Write to memory
    always @(posedge wr_clk) begin
        if (wr_en && !wr_full) begin
            memory[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
        end
    end

    // Write full flag
    always @(*) begin
        wr_full = (wr_ptr_gray_next == rd_ptr_gray_sync2);
    end

    // Read domain logic
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_ptr_bin <= 0;
            rd_ptr_gray <= 0;
            wr_ptr_gray_sync1 <= 0;
            wr_ptr_gray_sync2 <= 0;
        end else begin
            rd_ptr_bin <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    always @(*) begin
        rd_ptr_bin_next = rd_ptr_bin;
        if (rd_en && !rd_empty) begin
            rd_ptr_bin_next = rd_ptr_bin + 1;
        end
        rd_ptr_gray_next = bin2gray(rd_ptr_bin_next);
    end

    // Read from memory
    always @(posedge rd_clk) begin
        rd_data <= memory[rd_ptr_bin[ADDR_WIDTH-1:0]];
    end

    // Read empty flag
    always @(*) begin
        rd_empty = (rd_ptr_gray == wr_ptr_gray_sync2);
    end

endmodule