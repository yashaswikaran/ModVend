
// Dual Port RAM for Inventory Management
module dual_port_ram #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 8,
    parameter DEPTH = 256
)(
    // Port A - System/MODBUS access
    input wire clk_a,
    input wire [ADDR_WIDTH-1:0] addr_a,
    input wire [DATA_WIDTH-1:0] data_a,
    input wire we_a,
    output reg [DATA_WIDTH-1:0] q_a,

    // Port B - Vending machine access
    input wire clk_b,
    input wire [ADDR_WIDTH-1:0] addr_b,
    input wire [DATA_WIDTH-1:0] data_b,
    input wire we_b,
    output reg [DATA_WIDTH-1:0] q_b
);

    // Memory array
    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];

    // Initialize inventory
    initial begin
        // Item 0: Coke - Stock: 50
        ram[0] = 16'd50;
        // Item 1: Pepsi - Stock: 30  
        ram[1] = 16'd30;
        // Item 2: Water - Stock: 100
        ram[2] = 16'd100;
        // Item 3: Chips - Stock: 25
        ram[3] = 16'd25;

        // Prices stored at offset 0x10
        ram[16] = 16'd150; // Coke price
        ram[17] = 16'd150; // Pepsi price
        ram[18] = 16'd100; // Water price
        ram[19] = 16'd200; // Chips price
    end

    // Port A operations
    always @(posedge clk_a) begin
        if (we_a) begin
            ram[addr_a] <= data_a;
        end
        q_a <= ram[addr_a];
    end

    // Port B operations
    always @(posedge clk_b) begin
        if (we_b) begin
            ram[addr_b] <= data_b;
        end
        q_b <= ram[addr_b];
    end

endmodule
