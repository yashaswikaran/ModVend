//============================================================================
// MODBUS RTU Finite State Machine - Handles MODBUS protocol
//============================================================================
module modbus_fsm #(
    parameter SLAVE_ADDR = 8'h01
)(
    input wire clk,
    input wire rst,

    // FIFO interface
    input wire [7:0] rx_data,
    input wire rx_valid,
    output reg rx_rd_en,

    // UART TX interface
    output reg [7:0] tx_data,
    output reg tx_valid,
    input wire tx_ready,

    // Register interface
    output reg [15:0] reg_addr,
    output reg [15:0] reg_wr_data,
    input wire [15:0] reg_rd_data,
    output reg reg_wr_en,
    output reg reg_rd_en,

    // Control outputs
    output reg [7:0] func_code,
    output reg frame_error,
    output reg [3:0] dispense_item,
    output reg dispense_cmd,
    output reg [3:0] selected_item_out
);

    // MODBUS state machine
    localparam IDLE = 4'b0000,
               RX_ADDR = 4'b0001,
               RX_FUNC = 4'b0010,
               RX_REG_HI = 4'b0011,
               RX_REG_LO = 4'b0100,
               RX_DATA_HI = 4'b0101,
               RX_DATA_LO = 4'b0110,
               RX_CRC_HI = 4'b0111,
               RX_CRC_LO = 4'b1000,
               PROCESS = 4'b1001,
               TX_RESPONSE = 4'b1010,
               TX_ERROR = 4'b1011;

    reg [3:0] state, next_state;
    reg [7:0] rx_buffer [0:15];
    reg [7:0] rx_count;
    reg [7:0] slave_addr_rx;
    reg [15:0] data_addr, data_value;
    reg [7:0] tx_count;
    reg [15:0] crc_received, crc_calculated;

    // Special command register addresses
    localparam CMD_DISPENSE = 16'hFF10;
    localparam CMD_SELECT_ITEM = 16'hFF11;
    localparam STATUS_REG = 16'hFF00;

    // CRC calculation (simplified)
    function [15:0] calc_crc;
        input [7:0] data;
        input [15:0] crc_in;
        reg [15:0] crc_temp;
        integer i;
        begin
            crc_temp = crc_in;
            for (i = 0; i < 8; i = i + 1) begin
                if ((crc_temp[0] ^ data[i]) == 1'b1) begin
                    crc_temp = (crc_temp >> 1) ^ 16'hA001;
                end else begin
                    crc_temp = crc_temp >> 1;
                end
            end
            calc_crc = crc_temp;
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            rx_count <= 0;
            frame_error <= 0;
            dispense_cmd <= 0;
            func_code <= 0;
            reg_wr_en <= 0;
            reg_rd_en <= 0;
            tx_valid <= 0;
            tx_count <= 0;
            selected_item_out <= 0;
            crc_calculated <= 16'hFFFF;
        end else begin
            state <= next_state;
            dispense_cmd <= 0; // Pulse signal
            reg_wr_en <= 0;
            reg_rd_en <= 0;
            tx_valid <= 0;

            case (state)
                IDLE: begin
                    rx_count <= 0;
                    frame_error <= 0;
                    tx_count <= 0;
                    crc_calculated <= 16'hFFFF;
                    if (rx_valid) begin
                        rx_buffer[0] <= rx_data;
                        slave_addr_rx <= rx_data;
                        crc_calculated <= calc_crc(rx_data, 16'hFFFF);
                        rx_count <= 1;
                    end
                end

                RX_ADDR: begin
                    if (slave_addr_rx != SLAVE_ADDR && slave_addr_rx != 8'h00) begin
                        frame_error <= 1; // Not for us
                    end
                end

                RX_FUNC: begin
                    if (rx_valid) begin
                        func_code <= rx_data;
                        rx_buffer[rx_count] <= rx_data;
                        crc_calculated <= calc_crc(rx_data, crc_calculated);
                        rx_count <= rx_count + 1;
                    end
                end

                RX_REG_HI: begin
                    if (rx_valid) begin
                        rx_buffer[rx_count] <= rx_data;
                        data_addr[15:8] <= rx_data;
                        crc_calculated <= calc_crc(rx_data, crc_calculated);
                        rx_count <= rx_count + 1;
                    end
                end

                RX_REG_LO: begin
                    if (rx_valid) begin
                        rx_buffer[rx_count] <= rx_data;
                        data_addr[7:0] <= rx_data;
                        crc_calculated <= calc_crc(rx_data, crc_calculated);
                        rx_count <= rx_count + 1;
                    end
                end

                RX_DATA_HI: begin
                    if (rx_valid) begin
                        rx_buffer[rx_count] <= rx_data;
                        data_value[15:8] <= rx_data;
                        crc_calculated <= calc_crc(rx_data, crc_calculated);
                        rx_count <= rx_count + 1;
                    end
                end

                RX_DATA_LO: begin
                    if (rx_valid) begin
                        rx_buffer[rx_count] <= rx_data;
                        data_value[7:0] <= rx_data;
                        crc_calculated <= calc_crc(rx_data, crc_calculated);
                        rx_count <= rx_count + 1;
                    end
                end

                RX_CRC_HI: begin
                    if (rx_valid) begin
                        crc_received[7:0] <= rx_data; // CRC is sent LSB first
                        rx_count <= rx_count + 1;
                    end
                end

                RX_CRC_LO: begin
                    if (rx_valid) begin
                        crc_received[15:8] <= rx_data;
                        rx_count <= rx_count + 1;
                    end
                end

                PROCESS: begin
                    // Check CRC
                    if (crc_received != crc_calculated) begin
                        frame_error <= 1;
                    end else begin
                        case (func_code)
                            8'h03: begin // Read holding registers
                                reg_addr <= data_addr;
                                reg_rd_en <= 1;
                            end
                            
                            8'h06: begin // Write single register
                                reg_addr <= data_addr;
                                reg_wr_data <= data_value;
                                reg_wr_en <= 1;
                                
                                // Special commands
                                if (data_addr == CMD_SELECT_ITEM) begin
                                    selected_item_out <= data_value[3:0];
                                end else if (data_addr == CMD_DISPENSE) begin
                                    dispense_item <= selected_item_out;
                                    dispense_cmd <= 1;
                                end
                            end
                            
                            default: begin
                                frame_error <= 1; // Unsupported function
                            end
                        endcase
                    end
                end

                TX_RESPONSE: begin
                    if (tx_ready && !frame_error) begin
                        case (tx_count)
                            0: begin
                                tx_data <= slave_addr_rx;
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            1: begin
                                tx_data <= func_code;
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            2: begin
                                if (func_code == 8'h03) begin
                                    tx_data <= reg_rd_data[15:8]; // Data high byte
                                end else begin
                                    tx_data <= data_addr[15:8]; // Echo register address
                                end
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            3: begin
                                if (func_code == 8'h03) begin
                                    tx_data <= reg_rd_data[7:0]; // Data low byte
                                end else begin
                                    tx_data <= data_addr[7:0]; // Echo register address
                                end
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            4: begin
                                if (func_code == 8'h06) begin
                                    tx_data <= data_value[15:8]; // Echo data high
                                    tx_valid <= 1;
                                    tx_count <= tx_count + 1;
                                end else begin
                                    // CRC calculation and transmission would go here
                                    tx_count <= 0;
                                end
                            end
                            5: begin
                                tx_data <= data_value[7:0]; // Echo data low
                                tx_valid <= 1;
                                tx_count <= 0;
                            end
                            default: tx_count <= 0;
                        endcase
                    end
                end

                TX_ERROR: begin
                    if (tx_ready) begin
                        case (tx_count)
                            0: begin
                                tx_data <= slave_addr_rx;
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            1: begin
                                tx_data <= func_code | 8'h80; // Error response
                                tx_valid <= 1;
                                tx_count <= tx_count + 1;
                            end
                            2: begin
                                tx_data <= 8'h01; // Illegal function error
                                tx_valid <= 1;
                                tx_count <= 0;
                            end
                            default: tx_count <= 0;
                        endcase
                    end
                end
            endcase
        end
    end

    // State transition logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (rx_valid) next_state = RX_ADDR;
            end
            RX_ADDR: begin
                if (frame_error) next_state = IDLE;
                else next_state = RX_FUNC;
            end
            RX_FUNC: begin
                if (rx_valid) next_state = RX_REG_HI;
            end
            RX_REG_HI: begin
                if (rx_valid) next_state = RX_REG_LO;
            end
            RX_REG_LO: begin
                if (rx_valid) begin
                    if (func_code == 8'h06) // Write single register
                        next_state = RX_DATA_HI;
                    else
                        next_state = RX_CRC_HI;
                end
            end
            RX_DATA_HI: begin
                if (rx_valid) next_state = RX_DATA_LO;
            end
            RX_DATA_LO: begin
                if (rx_valid) next_state = RX_CRC_HI;
            end
            RX_CRC_HI: begin
                if (rx_valid) next_state = RX_CRC_LO;
            end
            RX_CRC_LO: begin
                if (rx_valid) next_state = PROCESS;
            end
            PROCESS: begin
                if (frame_error) next_state = TX_ERROR;
                else next_state = TX_RESPONSE;
            end
            TX_RESPONSE: begin
                if (tx_count == 0 && tx_ready) next_state = IDLE;
            end
            TX_ERROR: begin
                if (tx_count == 0 && tx_ready) next_state = IDLE;
            end
        endcase
    end

    assign rx_rd_en = rx_valid; // Simple flow control

endmodule