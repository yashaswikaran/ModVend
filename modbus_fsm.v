
// MODBUS RTU Finite State Machine
module modbus_fsm (
    input wire clk,
    input wire rst,

    // FIFO interface
    input wire [7:0] rx_data,
    input wire rx_valid,
    output reg [7:0] tx_data,
    output reg tx_valid,

    // Inventory interface
    output reg [15:0] inv_addr,
    output reg [15:0] inv_wr_data,
    input wire [15:0] inv_rd_data,
    output reg inv_wr_en,

    // Vending control
    output reg dispense_item,
    output reg [3:0] item_select
);

    // MODBUS RTU States
    localparam IDLE = 3'b000,
               ADDR_RX = 3'b001,
               FUNC_RX = 3'b010,
               DATA_RX = 3'b011,
               CRC_RX = 3'b100,
               PROCESS = 3'b101,
               RESPONSE = 3'b110;

    reg [2:0] state, next_state;
    reg [7:0] rx_buffer [0:255];
    reg [7:0] rx_count;
    reg [7:0] slave_addr;
    reg [7:0] func_code;
    reg [15:0] crc_received, crc_calculated;

    // Timer for 3.5 character silence detection
    reg [15:0] silence_timer;
    localparam SILENCE_3_5_CHAR = 16'd1750; // Adjust based on baud rate

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            rx_count <= 0;
            silence_timer <= 0;
        end else begin
            state <= next_state;

            // Silence timer for frame detection
            if (rx_valid) begin
                silence_timer <= 0;
            end else if (silence_timer < SILENCE_3_5_CHAR) begin
                silence_timer <= silence_timer + 1;
            end
        end
    end

    // State machine logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (rx_valid && (silence_timer >= SILENCE_3_5_CHAR)) begin
                    next_state = ADDR_RX;
                end
            end

            ADDR_RX: begin
                if (rx_valid) begin
                    next_state = FUNC_RX;
                end
            end

            FUNC_RX: begin
                if (rx_valid) begin
                    next_state = DATA_RX;
                end
            end

            DATA_RX: begin
                if (silence_timer >= SILENCE_3_5_CHAR) begin
                    next_state = PROCESS;
                end
            end

            PROCESS: begin
                next_state = RESPONSE;
            end

            RESPONSE: begin
                // Send response and return to IDLE
                next_state = IDLE;
            end
        endcase
    end

    // Function code processing
    always @(posedge clk) begin
        case (func_code)
            8'h03: begin // Read Holding Registers
                // Read inventory data
                inv_addr <= {rx_buffer[2], rx_buffer[3]};
            end

            8'h06: begin // Write Single Register  
                // Update inventory or dispense item
                inv_addr <= {rx_buffer[2], rx_buffer[3]};
                inv_wr_data <= {rx_buffer[4], rx_buffer[5]};
                inv_wr_en <= 1'b1;

                // Trigger item dispensing if address matches
                if (inv_addr == 16'h0100) begin
                    dispense_item <= 1'b1;
                    item_select <= inv_wr_data[3:0];
                end
            end
        endcase
    end

endmodule
