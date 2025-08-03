`timescale 1ns / 1ps

// ----------------------------------------------------
// MODBUS-Controlled Smart Vending Machine Testbench
// ----------------------------------------------------
module modbus_vending_tb;

    // Clock and Reset
    reg clk_50m    = 0;
    reg clk_uart   = 0;
    reg rst        = 1;

    // UART signals
    reg uart_rx = 1;     // Idle = 1
    wire uart_tx;

    // DUT outputs
    wire [3:0] item_dispensed;
    wire [3:0] item_select;
    wire [7:0] machine_status;

    // Instantiate DUT (top-level system)
    modbus_vending_machine dut (
        .clk_sys       (clk_50m),
        .clk_uart      (clk_uart),
        .rst           (rst),
        .uart_rx       (uart_rx),
        .uart_tx       (uart_tx),
        .item_dispensed(item_dispensed),
        .item_select   (item_select),
        .machine_status(machine_status)
    );

    // Generate 50MHz system clock
    always #10 clk_50m = ~clk_50m;    // 20ns period

    // Generate UART baud clock approximating 19200 baud
    always #26041 clk_uart = ~clk_uart; // ~52us per full cycle (26us half cycle)

    // --------------------------------
    // Test Sequencer
    // --------------------------------
    initial begin
        // Reset system
        rst = 1;
        #1000;
        rst = 0;
        #10000;  // Wait for system init

        $display("==== MODBUS Vending Machine Testbench START ====");

        // ---- TEST 1: Read Holding Register (Function 03, Item 1) ----
        $display("TEST1: MODBUS Read (Inventory Item 1)");
        send_modbus_frame(8'h01, 8'h03, 16'h0001, 16'h0001); // slave=1, fc=3, addr=0001, qty=1
        wait_for_uart_response();

        #60000; // Inter-frame silent period

        // ---- TEST 2: Dispense Command (Function 06) ----
        $display("TEST2: MODBUS Write Single Register (Dispense Item)");
        send_modbus_frame(8'h01, 8'h06, 16'h0100, 16'h0001); // reg=0x0100 (dispense), value=1
        wait_for_uart_response();

        #60000;

        // ---- TEST 3: Invalid Function (should return exception) ----
        $display("TEST3: MODBUS Invalid Function Code");
        send_modbus_frame(8'h01, 8'h99, 16'h0001, 16'h0001); // Invalid FC=0x99
        wait_for_uart_response();

        #100000;

        $display("==== All MODBUS test cases completed ====");
        $finish;
    end

    // --------------------
    // Utility TASKS
    // --------------------

    // Task: Send MODBUS RTU frame (8N1, LSB first, no parity)
    task send_modbus_frame(input [7:0] slave, input [7:0] fcode, input [15:0] addr, input [15:0] val);
        reg [7:0] frame[0:7];
        reg [15:0] crc;
        integer i;
        begin
            // Assemble frame (MODBUS RTU spec)
            frame[0] = slave;                          // Slave address
            frame[1] = fcode;                          // Function code
            frame[2] = addr[15:8];                     // Address (hi)
            frame[3] = addr[7:0];                      // Address (lo)
            frame[4] = val[15:8];                      // Data/qty hi
            frame[5] = val[7:0];                       // Data/qty lo

            // --- Compute MODBUS CRC
            calc_crc16_modbus(frame, 6, crc);
            frame[6] = crc[7:0];
            frame[7] = crc[15:8];

            // Inter-frame gap (3.5 char = ~183 us for 19200 baud)
            uart_rx = 1; #190000; // 3.5 * (1/19200)*11*1e9 ≈ 200us

            // Send frame bytes, one at a time
            for (i = 0; i < 8; i = i + 1)
                send_uart_byte(frame[i]);

            $display("SENT: MODBUS [%02h %02h %02h %02h %02h %02h] CRC=%04h (Frame: %h %h %h %h %h %h %h %h)",
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], crc, 
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]);
        end
    endtask

    // Task: Send one byte on UART (8N1, LSB first, no parity)
    task send_uart_byte(input [7:0] dat);
        integer k;
        begin
            // Start bit (0)
            uart_rx = 0; #52083;
            // Data
            for (k=0; k<8; k=k+1) begin
                uart_rx = dat[k]; #52083;
            end
            // Stop bit (1)
            uart_rx = 1; #52083;
        end
    endtask

    // Task: Wait for UART TX activity window
    task wait_for_uart_response();
        integer t;
        begin
            // Wait for TX (you may enhance by actively monitoring uart_tx for real-time response capture)
            #300000; // ~300us nominal
            $display("   ...Response wait window elapsed");
        end
    endtask

    // ---------------------------
    // MODBUS CRC-16 TASK
    // ---------------------------
    // (Standard poly=0xA001, init=0xFFFF, LSB first)
    task calc_crc16_modbus(input [7:0] dat[0:5], input integer len, output reg [15:0] crc);
        integer i, j;
        reg [7:0] b;
        begin
            crc = 16'hFFFF;
            for (i = 0; i < len; i = i + 1) begin
                b = dat[i];
                crc = crc ^ b;
                for (j = 0; j < 8; j = j + 1) begin
                    if (crc[0])
                        crc = (crc >> 1) ^ 16'hA001;
                    else
                        crc = crc >> 1;
                end
            end
        end
    endtask

    // --------------------------------
    // VCD Waveform Dump
    // --------------------------------
    initial begin
        $dumpfile("modbus_vending_tb.vcd");
        $dumpvars(0, modbus_vending_tb);
    end

    // Monitor
    initial begin
        $monitor("T=%0t | RX=%b, TX=%b | DISP=%b, SEL=%b, STAT=%h",
            $time, uart_rx, uart_tx, item_dispensed, item_select, machine_status);
    end

endmodule
