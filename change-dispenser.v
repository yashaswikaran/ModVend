//============================================================================
// Change Dispenser - Handles returning change using multiple denominations
//============================================================================
module change_dispenser #(
    parameter PRICE_WIDTH = 16
)(
    input wire clk,
    input wire rst,
    input wire [PRICE_WIDTH-1:0] change_amount,
    input wire dispense_change,
    output reg [8:0] change_motors,        // Motors for each denomination
    output reg change_complete
);

    // Coin denomination values (same as payment controller)
    localparam [PRICE_WIDTH-1:0] COIN_VALUES [8:0] = {
        16'd1, 16'd2, 16'd5, 16'd10, 16'd20, 
        16'd50, 16'd100, 16'd500, 16'd2000
    };

    // State machine
    localparam IDLE = 2'b00,
               CALCULATE = 2'b01,
               DISPENSE = 2'b10,
               COMPLETE = 2'b11;

    reg [1:0] state, next_state;
    reg [PRICE_WIDTH-1:0] remaining_change;
    reg [3:0] current_denomination;
    reg [15:0] dispense_timer;
    reg [7:0] coins_to_dispense [8:0];  // Count of each denomination to dispense
    reg [7:0] current_coin_count;

    localparam DISPENSE_TIME = 50000;  // Time to dispense one coin at 50MHz

    // Initialize arrays properly
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            change_motors <= 9'b000000000;
            change_complete <= 0;
            remaining_change <= 0;
            current_denomination <= 8; // Start from highest denomination
            dispense_timer <= 0;
            current_coin_count <= 0;
            for (i = 0; i < 9; i = i + 1) begin
                coins_to_dispense[i] <= 0;
            end
        end else begin
            state <= next_state;

            case (state)
                IDLE: begin
                    change_complete <= 0;
                    change_motors <= 9'b000000000;
                    if (dispense_change && change_amount > 0) begin
                        remaining_change <= change_amount;
                        current_denomination <= 8; // Start from 2000
                        // Clear coin counts
                        for (i = 0; i < 9; i = i + 1) begin
                            coins_to_dispense[i] <= 0;
                        end
                    end
                end

                CALCULATE: begin
                    // Calculate how many coins of current denomination needed
                    if (remaining_change >= COIN_VALUES[current_denomination]) begin
                        coins_to_dispense[current_denomination] <= 
                            remaining_change / COIN_VALUES[current_denomination];
                        remaining_change <= 
                            remaining_change % COIN_VALUES[current_denomination];
                    end

                    if (current_denomination > 0) begin
                        current_denomination <= current_denomination - 1;
                    end
                end

                DISPENSE: begin
                    // Dispense coins starting from highest denomination
                    if (current_coin_count < coins_to_dispense[current_denomination]) begin
                        change_motors[current_denomination] <= 1;
                        if (dispense_timer < DISPENSE_TIME) begin
                            dispense_timer <= dispense_timer + 1;
                        end else begin
                            dispense_timer <= 0;
                            current_coin_count <= current_coin_count + 1;
                            change_motors[current_denomination] <= 0;
                        end
                    end else begin
                        // Move to next denomination
                        current_coin_count <= 0;
                        if (current_denomination > 0) begin
                            current_denomination <= current_denomination - 1;
                        end
                    end
                end

                COMPLETE: begin
                    change_complete <= 1;
                    change_motors <= 9'b000000000;
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (dispense_change && change_amount > 0)
                    next_state = CALCULATE;
            end

            CALCULATE: begin
                if (current_denomination == 0 && remaining_change == 0)
                    next_state = DISPENSE;
                else if (current_denomination == 0)
                    next_state = COMPLETE; // Can't make exact change
            end

            DISPENSE: begin
                if (current_denomination == 0 && 
                    current_coin_count >= coins_to_dispense[current_denomination])
                    next_state = COMPLETE;
            end

            COMPLETE: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule