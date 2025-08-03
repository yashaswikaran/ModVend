
// Dual Clock FIFO for Clock Domain Crossing
module dual_clock_fifo #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 16
)(
    // Write clock domain
    input wire wr_clk,
    input wire wr_rst,
    input wire [DATA_WIDTH-1:0] wr_data,
    input wire wr_en,
    output wire wr_full,

    // Read clock domain  
    input wire rd_clk,
    input wire rd_rst,
    output wire [DATA_WIDTH-1:0] rd_data,
    input wire rd_en,
    output wire rd_empty
);

    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

    // Memory array
    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    // Gray code pointers
    reg [ADDR_WIDTH:0] wr_ptr_gray, wr_ptr_gray_next;
    reg [ADDR_WIDTH:0] rd_ptr_gray, rd_ptr_gray_next;

    // Binary pointers
    reg [ADDR_WIDTH:0] wr_ptr_bin, rd_ptr_bin;

    // Synchronized pointers
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync, rd_ptr_gray_sync;

    // Write clock domain logic
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_ptr_bin <= 0;
            wr_ptr_gray <= 0;
        end else if (wr_en & ~wr_full) begin
            mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            wr_ptr_bin <= wr_ptr_bin + 1;
            wr_ptr_gray <= (wr_ptr_bin + 1) ^ ((wr_ptr_bin + 1) >> 1);
        end
    end

    // Read clock domain logic  
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_ptr_bin <= 0;
            rd_ptr_gray <= 0;
        end else if (rd_en & ~rd_empty) begin
            rd_ptr_bin <= rd_ptr_bin + 1;
            rd_ptr_gray <= (rd_ptr_bin + 1) ^ ((rd_ptr_bin + 1) >> 1);
        end
    end

    assign rd_data = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
    assign wr_full = (wr_ptr_gray_next == rd_ptr_gray_sync);
    assign rd_empty = (rd_ptr_gray == wr_ptr_gray_sync);

endmodule
