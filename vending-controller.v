//============================================================================
// Vending Machine Controller - Handles physical dispensing operations
//============================================================================
module vending_controller #(
    parameter NUM_ITEMS = 16,
    parameter DISPENSE_TIME = 50000000  // 1 second at 50MHz
)(
    input wire clk,
    input wire rst,

    // Command interface
    input wire dispense_cmd,
    input wire [3:0] item_select,

    // Inventory interface
    output reg [7:0] inv_addr,
    output reg [15:0] inv_data_out,
    input wire [15:0] inv_data_in,
    output reg inv_we,

    // Physical interface
    input wire [NUM_ITEMS-1:0] item_sensors,
    output reg [NUM_ITEMS-1:0] dispense_motors,

    // Status outputs
    output reg dispense_active,
    output reg [3:0] current_item,
    output reg error_state
);

    // State machine for dispensing
    localparam IDLE = 2'b00,
               CHECK_STOCK = 2'b01,
               DISPENSE = 2'b10,
               UPDATE_INV = 2'b11;

    reg [1:0] state, next_state;
    reg [31:0] dispense_timer;
    reg [15:0] current_stock;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            dispense_timer <= 0;
            dispense_motors <= 0;
            dispense_active <= 0;
            current_item <= 0;
            error_state <= 0;
            inv_addr <= 0;
            inv_data_out <= 0;
            inv_we <= 0;
        end else begin
            state <= next_state;

            case (state)
                IDLE: begin
                    dispense_active <= 0;
                    dispense_motors <= 0;
                    error_state <= 0;
                    dispense_timer <= 0;
                    if (dispense_cmd) begin
                        current_item <= item_select;
                        inv_addr <= item_select;
                        inv_we <= 0;
                    end
                end

                CHECK_STOCK: begin
                    current_stock <= inv_data_in;
                    if (inv_data_in == 0) begin
                        error_state <= 1;  // Out of stock
                    end else if (!item_sensors[item_select]) begin
                        error_state <= 1;  // Item not physically detected
                    end
                end

                DISPENSE: begin
                    dispense_active <= 1;
                    dispense_motors[current_item] <= 1;
                    if (dispense_timer < DISPENSE_TIME) begin
                        dispense_timer <= dispense_timer + 1;
                    end
                end

                UPDATE_INV: begin
                    dispense_motors <= 0;
                    inv_addr <= current_item;
                    inv_data_out <= current_stock - 1;
                    inv_we <= 1;
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (dispense_cmd) next_state = CHECK_STOCK;
            end
            CHECK_STOCK: begin
                if (error_state) next_state = IDLE;
                else next_state = DISPENSE;
            end
            DISPENSE: begin
                if (dispense_timer >= DISPENSE_TIME) next_state = UPDATE_INV;
            end
            UPDATE_INV: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule