
// UART Interface Module for MODBUS RTU Communication
module uart_interface #(
    parameter BAUD_RATE = 9600,
    parameter CLK_FREQ = 50_000_000
)(
    input wire clk,
    input wire rst,
    input wire uart_rx,
    output wire uart_tx,

    // Interface to FIFO
    output wire [7:0] rx_data,
    output wire rx_valid,
    input wire [7:0] tx_data,
    input wire tx_valid,
    output wire tx_ready
);

    // Baud rate generator
    localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;
    reg [$clog2(BAUD_DIV)-1:0] baud_counter;
    reg baud_tick;

    always @(posedge clk) begin
        if (rst) begin
            baud_counter <= 0;
            baud_tick <= 0;
        end else begin
            baud_tick <= 0;
            if (baud_counter == BAUD_DIV - 1) begin
                baud_counter <= 0;
                baud_tick <= 1;
            end else begin
                baud_counter <= baud_counter + 1;
            end
        end
    end

    // UART TX and RX modules instantiated here
    // Implementation details omitted for brevity

endmodule
