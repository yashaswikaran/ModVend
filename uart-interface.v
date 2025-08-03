//============================================================================
// UART Interface Module - Handles serial communication
//============================================================================
module uart_interface #(
    parameter BAUD_RATE = 19200,
    parameter CLK_FREQ = 50_000_000
)(
    input wire clk,
    input wire rst,
    
    // Physical UART lines
    input wire uart_rx,
    output wire uart_tx,
    
    // Internal interface
    output reg [7:0] rx_data,
    output reg rx_valid,
    input wire [7:0] tx_data,
    input wire tx_valid,
    output reg tx_ready
);

    // Baud rate generation
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;
    reg [15:0] baud_counter;
    reg baud_tick;

    always @(posedge clk) begin
        if (rst) begin
            baud_counter <= 0;
            baud_tick <= 0;
        end else begin
            if (baud_counter >= BAUD_DIV - 1) begin
                baud_counter <= 0;
                baud_tick <= 1;
            end else begin
                baud_counter <= baud_counter + 1;
                baud_tick <= 0;
            end
        end
    end

    // UART Receiver
    reg [3:0] rx_state;
    reg [3:0] rx_bit_count;
    reg [7:0] rx_shift_reg;
    reg rx_sync1, rx_sync2;

    localparam RX_IDLE = 4'd0,
               RX_START = 4'd1,
               RX_DATA = 4'd2,
               RX_STOP = 4'd3;

    always @(posedge clk) begin
        if (rst) begin
            rx_state <= RX_IDLE;
            rx_data <= 0;
            rx_valid <= 0;
            rx_bit_count <= 0;
            rx_shift_reg <= 0;
            rx_sync1 <= 1;
            rx_sync2 <= 1;
        end else begin
            // Synchronize input
            rx_sync1 <= uart_rx;
            rx_sync2 <= rx_sync1;
            rx_valid <= 0;

            if (baud_tick) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (!rx_sync2) begin // Start bit detected
                            rx_state <= RX_START;
                        end
                    end
                    
                    RX_START: begin
                        if (!rx_sync2) begin // Confirm start bit
                            rx_state <= RX_DATA;
                            rx_bit_count <= 0;
                        end else begin
                            rx_state <= RX_IDLE; // False start
                        end
                    end
                    
                    RX_DATA: begin
                        rx_shift_reg <= {rx_sync2, rx_shift_reg[7:1]};
                        rx_bit_count <= rx_bit_count + 1;
                        if (rx_bit_count == 7) begin
                            rx_state <= RX_STOP;
                        end
                    end
                    
                    RX_STOP: begin
                        if (rx_sync2) begin // Valid stop bit
                            rx_data <= rx_shift_reg;
                            rx_valid <= 1;
                        end
                        rx_state <= RX_IDLE;
                    end
                endcase
            end
        end
    end

    // UART Transmitter
    reg [3:0] tx_state;
    reg [3:0] tx_bit_count;
    reg [9:0] tx_shift_reg; // Start + 8 data + stop
    reg tx_line;

    localparam TX_IDLE = 4'd0,
               TX_TRANSMIT = 4'd1;

    always @(posedge clk) begin
        if (rst) begin
            tx_state <= TX_IDLE;
            tx_ready <= 1;
            tx_bit_count <= 0;
            tx_shift_reg <= 10'h3FF; // All ones (idle)
            tx_line <= 1;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_ready <= 1;
                    tx_line <= 1;
                    if (tx_valid) begin
                        tx_shift_reg <= {1'b1, tx_data, 1'b0}; // Stop + data + start
                        tx_state <= TX_TRANSMIT;
                        tx_bit_count <= 0;
                        tx_ready <= 0;
                    end
                end
                
                TX_TRANSMIT: begin
                    if (baud_tick) begin
                        tx_line <= tx_shift_reg[0];
                        tx_shift_reg <= {1'b1, tx_shift_reg[9:1]};
                        tx_bit_count <= tx_bit_count + 1;
                        if (tx_bit_count == 9) begin // Transmitted all bits
                            tx_state <= TX_IDLE;
                        end
                    end
                end
            endcase
        end
    end

    assign uart_tx = tx_line;

endmodule