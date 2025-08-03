//============================================================================
// Dual Port RAM - For inventory and price storage with MODBUS access
//============================================================================
module dual_port_ram #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 8,
    parameter DEPTH = 256
)(
    // Port A
    input wire clk_a,
    input wire [ADDR_WIDTH-1:0] addr_a,
    input wire [DATA_WIDTH-1:0] data_a,
    input wire we_a,
    output reg [DATA_WIDTH-1:0] q_a,

    // Port B
    input wire clk_b,
    input wire [ADDR_WIDTH-1:0] addr_b,
    input wire [DATA_WIDTH-1:0] data_b,
    input wire we_b,
    output reg [DATA_WIDTH-1:0] q_b
);

    // Memory array
    reg [DATA_WIDTH-1:0] memory [0:DEPTH-1];

    // Port A operations
    always @(posedge clk_a) begin
        if (we_a) begin
            memory[addr_a] <= data_a;
        end
        q_a <= memory[addr_a];
    end

    // Port B operations
    always @(posedge clk_b) begin
        if (we_b) begin
            memory[addr_b] <= data_b;
        end
        q_b <= memory[addr_b];
    end

endmodule