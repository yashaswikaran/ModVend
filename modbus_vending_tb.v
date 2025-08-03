
// Comprehensive Testbench for MODBUS-Controlled Smart Vending Machine
`timescale 1ns / 1ps

module modbus_vending_tb;

    // Clock and reset
    reg clk_50m;
    reg clk_uart;
    reg rst;

    // UART signals
    reg uart_rx;
    wire uart_tx;

    // Test control
    reg [7:0] test_frame [0:7];
    integer frame_index;
    integer bit_index;
    reg tx_active;

    // DUT instantiation
    modbus_vending_machine dut (
        .clk_sys(clk_50m),
        .clk_uart(clk_uart),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .item_dispensed(),
        .item_select(),
        .machine_status()
    );

    // Clock generation
    initial begin
        clk_50m = 0;
        forever #10 clk_50m = ~clk_50m; // 50MHz system clock
    end

    initial begin
        clk_uart = 0;
        forever #26041 clk_uart = ~clk_uart; // 19200 baud clock
    end

    // Test stimulus
    initial begin
        // Initialize
        rst = 1;
        uart_rx = 1; // UART idle high
        tx_active = 0;
        frame_index = 0;
        bit_index = 0;

        #1000;
        rst = 0;

        // Wait for system to stabilize
        #10000;

        $display("Starting MODBUS RTU Tests...");

        // Test Case 1: Read Holding Register (Function Code 03)
        $display("Test 1: Read item inventory (Function Code 03)");
        test_frame[0] = 8'h01; // Slave address
        test_frame[1] = 8'h03; // Function code - Read Holding Registers
        test_frame[2] = 8'h00; // Starting address high byte
        test_frame[3] = 8'h01; // Starting address low byte (item 1)
        test_frame[4] = 8'h00; // Quantity high byte
        test_frame[5] = 8'h01; // Quantity low byte (1 register)
        test_frame[6] = 8'hD5; // CRC low byte
        test_frame[7] = 8'hCA; // CRC high byte

        send_modbus_frame(8);
        wait_for_response();

        #50000; // Inter-frame delay

        // Test Case 2: Write Single Register (Function Code 06) - Dispense Item
        $display("Test 2: Dispense item (Function Code 06)");
        test_frame[0] = 8'h01; // Slave address
        test_frame[1] = 8'h06; // Function code - Write Single Register
        test_frame[2] = 8'h01; // Register address high byte
        test_frame[3] = 8'h00; // Register address low byte (dispense command)
        test_frame[4] = 8'h00; // Data high byte
        test_frame[5] = 8'h01; // Data low byte (dispense item 1)
        test_frame[6] = 8'h08; // CRC low byte
        test_frame[7] = 8'h0A; // CRC high byte

        send_modbus_frame(8);
        wait_for_response();

        #50000;

        // Test Case 3: Invalid Function Code (Exception Test)
        $display("Test 3: Invalid function code (Exception handling)");
        test_frame[0] = 8'h01; // Slave address
        test_frame[1] = 8'h99; // Invalid function code
        test_frame[2] = 8'h00; // Dummy data
        test_frame[3] = 8'h01;
        test_frame[4] = 8'h00;
        test_frame[5] = 8'h01;
        test_frame[6] = 8'hAA; // Dummy CRC
        test_frame[7] = 8'hBB;

        send_modbus_frame(8);
        wait_for_response();

        #100000;
        $display("All tests completed!");
        $finish;
    end

    // Task to send MODBUS frame
    task send_modbus_frame(input integer frame_len);
        integer i, j;
        begin
            // 3.5 character silence before frame
            #91666; // 3.5 * (11 bits * 52083ns) at 19200 baud

            for (i = 0; i < frame_len; i = i + 1) begin
                send_uart_byte(test_frame[i]);

                // Ensure gap between bytes is less than 1.5 characters
                if (i < frame_len - 1) begin
                    #26041; // Small gap between bytes
                end
            end

            $display("Sent MODBUS frame: %h %h %h %h %h %h %h %h", 
                     test_frame[0], test_frame[1], test_frame[2], test_frame[3],
                     test_frame[4], test_frame[5], test_frame[6], test_frame[7]);
        end
    endtask

    // Task to send single UART byte
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            // Start bit
            uart_rx = 0;
            #52083; // Bit time at 19200 baud

            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i];
                #52083;
            end

            // Parity bit (even parity)
            uart_rx = ^data; // XOR of all data bits
            #52083;

            // Stop bit
            uart_rx = 1;
            #52083;
        end
    endtask

    // Task to wait for response
    task wait_for_response();
        begin
            // Wait for potential response
            #200000;
            $display("Response wait completed");
        end
    endtask

    // Monitor for debugging
    initial begin
        $monitor("Time: %0t, UART_RX: %b, RST: %b", $time, uart_rx, rst);
    end

    // VCD dump for waveform analysis
    initial begin
        $dumpfile("modbus_vending_tb.vcd");
        $dumpvars(0, modbus_vending_tb);
    end

endmodule

// Top-level MODBUS Vending Machine Module (skeleton)
module modbus_vending_machine (
    input wire clk_sys,
    input wire clk_uart, 
    input wire rst,
    input wire uart_rx,
    output wire uart_tx,
    output wire [3:0] item_dispensed,
    output wire [3:0] item_select,
    output wire [7:0] machine_status
);

    // Internal signals
    wire [7:0] rx_data, tx_data;
    wire rx_valid, tx_valid, tx_ready;
    wire fifo_full, fifo_empty;
    wire [15:0] inv_addr, inv_data_in, inv_data_out;
    wire inv_we;

    // UART Interface
    uart_interface uart_inst (
        .clk(clk_uart),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .rx_data(rx_data),
        .rx_valid(rx_valid),
        .tx_data(tx_data),
        .tx_valid(tx_valid),
        .tx_ready(tx_ready)
    );

    // Dual Clock FIFO
    dual_clock_fifo fifo_inst (
        .wr_clk(clk_uart),
        .wr_rst(rst),
        .wr_data(rx_data),
        .wr_en(rx_valid),
        .wr_full(fifo_full),
        .rd_clk(clk_sys),
        .rd_rst(rst),
        .rd_data(rx_data_sync),
        .rd_en(fifo_rd_en),
        .rd_empty(fifo_empty)
    );

    // MODBUS FSM
    modbus_fsm fsm_inst (
        .clk(clk_sys),
        .rst(rst),
        .rx_data(rx_data_sync),
        .rx_valid(~fifo_empty),
        .tx_data(tx_data),
        .tx_valid(tx_valid),
        .inv_addr(inv_addr),
        .inv_wr_data(inv_data_in),
        .inv_rd_data(inv_data_out),
        .inv_wr_en(inv_we),
        .dispense_item(item_dispensed[0]),
        .item_select(item_select)
    );

    // Dual Port RAM for inventory
    dual_port_ram inventory_ram (
        .clk_a(clk_sys),
        .addr_a(inv_addr[7:0]),
        .data_a(inv_data_in),
        .we_a(inv_we),
        .q_a(inv_data_out),
        .clk_b(clk_sys),
        .addr_b(8'h00),
        .data_b(16'h0000),
        .we_b(1'b0),
        .q_b()
    );

    assign machine_status = {4'b0000, item_select};

endmodule
