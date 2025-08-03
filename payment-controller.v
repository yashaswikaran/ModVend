//============================================================================
// Payment Controller - Handles coin acceptance, price verification, and change
//============================================================================
module payment_controller #(
    parameter NUM_ITEMS = 16,
    parameter PRICE_WIDTH = 16
)(
    input wire clk,
    input wire rst,

    // Coin interface
    input wire [8:0] coin_inputs,       // Coin denominations: [1,2,5,10,20,50,100,500,2000]
    input wire coin_inserted,           // Pulse when coin is inserted
    input wire payment_complete,        // User signals payment completion
    input wire transaction_cancel,      // Cancel current transaction

    // Item selection
    input wire [3:0] selected_item,
    input wire start_transaction,

    // Price memory interface
    output reg [3:0] price_mem_addr,
    input wire [PRICE_WIDTH-1:0] price_mem_data,

    // Control outputs
    output reg dispense_cmd,
    output reg payment_accepted,
    output reg payment_rejected,
    output reg [PRICE_WIDTH-1:0] total_inserted,
    output reg [PRICE_WIDTH-1:0] change_amount,
    output reg change_ready
);

    // Coin denomination values
    localparam [PRICE_WIDTH-1:0] COIN_VALUES [8:0] = {
        16'd1, 16'd2, 16'd5, 16'd10, 16'd20, 
        16'd50, 16'd100, 16'd500, 16'd2000
    };

    // State machine states
    localparam IDLE = 3'b000,
               WAIT_COINS = 3'b001,
               CHECK_PRICE = 3'b010,
               VERIFY_PAYMENT = 3'b011,
               DISPENSE = 3'b100,
               RETURN_CHANGE = 3'b101,
               COMPLETE = 3'b110,
               CANCEL = 3'b111;

    reg [2:0] state, next_state;
    reg [PRICE_WIDTH-1:0] selected_price;
    reg [3:0] current_item;

    // Sequential logic
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            total_inserted <= 0;
            selected_price <= 0;
            current_item <= 0;
            dispense_cmd <= 0;
            payment_accepted <= 0;
            payment_rejected <= 0;
            change_amount <= 0;
            change_ready <= 0;
            price_mem_addr <= 0;
        end else begin
            state <= next_state;

            case (state)
                IDLE: begin
                    dispense_cmd <= 0;
                    payment_accepted <= 0;
                    payment_rejected <= 0;
                    change_ready <= 0;
                    if (start_transaction) begin
                        current_item <= selected_item;
                        price_mem_addr <= selected_item;
                        total_inserted <= 0;
                    end
                end

                WAIT_COINS: begin
                    if (coin_inserted) begin
                        // Add coin value based on which input is active
                        case (coin_inputs)
                            9'b000000001: total_inserted <= total_inserted + COIN_VALUES[0]; // 1
                            9'b000000010: total_inserted <= total_inserted + COIN_VALUES[1]; // 2
                            9'b000000100: total_inserted <= total_inserted + COIN_VALUES[2]; // 5
                            9'b000001000: total_inserted <= total_inserted + COIN_VALUES[3]; // 10
                            9'b000010000: total_inserted <= total_inserted + COIN_VALUES[4]; // 20
                            9'b000100000: total_inserted <= total_inserted + COIN_VALUES[5]; // 50
                            9'b001000000: total_inserted <= total_inserted + COIN_VALUES[6]; // 100
                            9'b010000000: total_inserted <= total_inserted + COIN_VALUES[7]; // 500
                            9'b100000000: total_inserted <= total_inserted + COIN_VALUES[8]; // 2000
                            default: begin
                                // Invalid coin combination - reject
                                payment_rejected <= 1;
                            end
                        endcase
                    end
                end

                CHECK_PRICE: begin
                    selected_price <= price_mem_data;
                end

                VERIFY_PAYMENT: begin
                    if (total_inserted >= selected_price) begin
                        payment_accepted <= 1;
                        change_amount <= total_inserted - selected_price;
                        dispense_cmd <= 1;
                    end else begin
                        payment_rejected <= 1;
                    end
                end

                DISPENSE: begin
                    dispense_cmd <= 0; // Pulse for one cycle
                end

                RETURN_CHANGE: begin
                    if (change_amount > 0) begin
                        change_ready <= 1;
                    end
                end

                COMPLETE: begin
                    // Transaction complete, reset for next
                    total_inserted <= 0;
                    change_amount <= 0;
                    change_ready <= 0;
                    payment_accepted <= 0;
                end

                CANCEL: begin
                    // Return all inserted money
                    change_amount <= total_inserted;
                    change_ready <= 1;
                    payment_rejected <= 1;
                    total_inserted <= 0;
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start_transaction)
                    next_state = WAIT_COINS;
            end

            WAIT_COINS: begin
                if (transaction_cancel)
                    next_state = CANCEL;
                else if (payment_complete)
                    next_state = CHECK_PRICE;
            end

            CHECK_PRICE: begin
                next_state = VERIFY_PAYMENT;
            end

            VERIFY_PAYMENT: begin
                if (payment_rejected)
                    next_state = CANCEL;
                else if (payment_accepted)
                    next_state = DISPENSE;
            end

            DISPENSE: begin
                next_state = RETURN_CHANGE;
            end

            RETURN_CHANGE: begin
                if (change_amount == 0 || change_ready)
                    next_state = COMPLETE;
            end

            COMPLETE: begin
                next_state = IDLE;
            end

            CANCEL: begin
                next_state = COMPLETE;
            end
        endcase
    end

endmodule