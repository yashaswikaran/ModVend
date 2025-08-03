//============================================================================
// TESTBENCH for MODBUS Smart Vending Machine System
//============================================================================
`timescale 1ns / 1ps

module tb_modbus_smart_vending_machine;

    // Parameters
    parameter BAUD_RATE = 19200;
    parameter CLK_FREQ = 50_000_000;
    parameter SLAVE_ADDR = 8'h01;
    parameter NUM_ITEMS = 16;
    parameter PRICE_WIDTH = 16;

    // Clock and reset
    reg clk_sys;
    reg clk_uart;
    reg rst;

    // UART interface
    reg uart_rx;
    wire uart_tx;

    // Vending machine outputs
    wire [3:0] item_select;
    wire dispense_trigger;
    wire [7:0] machine_status;

    // Physical interface
    reg [NUM_ITEMS-1:0] item_sensors;
    wire [NUM_ITEMS-1:0] dispense_motors;

    // Payment interface
    reg [8:0] coin_inputs;
    reg coin_inserted;
    reg payment_complete;
    reg transaction_cancel;
    wire [8:0] change_motors;

    // LED indicators
    wire status_led;
    wire error_led;
    wire comm_led;
    wire payment_accepted_led;
    wire payment_rejected_led;

    // Test variables
    reg [7:0] test_data;
    integer i;

    // Instantiate the DUT (Device Under Test)
    modbus_smart_vending_machine #(
        .BAUD_RATE(BAUD_RATE),
        .CLK_FREQ(CLK_FREQ),
        .SLAVE_ADDR(SLAVE_ADDR),
        .NUM_ITEMS(NUM_ITEMS),
        .PRICE_WIDTH(PRICE_WIDTH)
    ) dut (
        .clk_sys(clk_sys),
        .clk_uart(clk_uart),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .item_select(item_select),
        .dispense_trigger(dispense_trigger),
        .machine_status(machine_status),
        .item_sensors(item_sensors),
        .dispense_motors(dispense_motors),
        .coin_inputs(coin_inputs),
        .coin_inserted(coin_inserted),
        .payment_complete(payment_complete),
        .transaction_cancel(transaction_cancel),
        .change_motors(change_motors),
        .status_led(status_led),
        .error_led(error_led),
        .comm_led(comm_led),
        .payment_accepted_led(payment_accepted_led),
        .payment_rejected_led(payment_rejected_led)
    );

    // Clock generation
    initial begin
        clk_sys = 0;
        forever #10 clk_sys = ~clk_sys; // 50MHz system clock
    end

    initial begin
        clk_uart = 0;
        forever #20 clk_uart = ~clk_uart; // 25MHz UART clock
    end

    // Reset generation
    initial begin
        rst = 1;
        #100;
        rst = 0;
        $display("Reset released at time %0t", $time);
    end

    // Initialize signals
    initial begin
        uart_rx = 1; // UART idle state
        item_sensors = 16'hFFFF; // All items present
        coin_inputs = 9'b000000000;
        coin_inserted = 0;
        payment_complete = 0;
        transaction_cancel = 0;
    end

    // Test sequence
    initial begin
        $display("Starting MODBUS Smart Vending Machine Test");
        
        // Wait for reset to complete
        wait(!rst);
        #1000;

        // Test 1: Set price for item 0 (50 units)
        $display("\n=== Test 1: Set price for item 0 ===");
        set_item_price(4'h0, 16'd50);
        #5000;

        // Test 2: Set inventory for item 0 (10 items)
        $display("\n=== Test 2: Set inventory for item 0 ===");
        set_item_inventory(4'h0, 16'd10);
        #5000;

        // Test 3: Select item 0
        $display("\n=== Test 3: Select item 0 ===");
        select_item(4'h0);
        #5000;

        // Test 4: Insert coins (total 75 units - should get 25 change)
        $display("\n=== Test 4: Insert coins ===");
        insert_coin(9'b001000000); // 100 unit coin
        #2000;
        signal_payment_complete();
        #5000;

        // Test 5: Trigger dispense
        $display("\n=== Test 5: Trigger dispense ===");
        trigger_dispense();
        #10000;

        // Test 6: Read status register
        $display("\n=== Test 6: Read status register ===");
        read_register(16'hFF00);
        #5000;

        // Test 7: Test insufficient payment
        $display("\n=== Test 7: Test insufficient payment ===");
        select_item(4'h1);
        set_item_price(4'h1, 16'd100);
        #2000;
        insert_coin(9'b000100000); // 50 unit coin (insufficient)
        #2000;
        signal_payment_complete();
        #5000;
        trigger_dispense();
        #10000;

        $display("\n=== All tests completed ===");
        #10000;
        $finish;
    end

    // Task to send MODBUS command to set item price
    task set_item_price;
        input [3:0] item_num;
        input [15:0] price;
        begin
            $display("Setting price for item %0d to %0d units", item_num, price);
            send_modbus_write(16'h0100 + item_num, price);
        end
    endtask

    // Task to send MODBUS command to set item inventory
    task set_item_inventory;
        input [3:0] item_num;
        input [15:0] quantity;
        begin
            $display("Setting inventory for item %0d to %0d items", item_num, quantity);
            send_modbus_write(16'h0000 + item_num, quantity);
        end
    endtask

    // Task to select item
    task select_item;
        input [3:0] item_num;
        begin
            $display("Selecting item %0d", item_num);
            send_modbus_write(16'hFF11, {12'h000, item_num});
        end
    endtask

    // Task to trigger dispense
    task trigger_dispense;
        begin
            $display("Triggering dispense command");
            send_modbus_write(16'hFF10, 16'h0001);
        end
    endtask

    // Task to read register
    task read_register;
        input [15:0] reg_addr;
        begin
            $display("Reading register 0x%04h", reg_addr);
            send_modbus_read(reg_addr);
        end
    endtask

    // Task to simulate coin insertion
    task insert_coin;
        input [8:0] coin_type;
        begin
            case (coin_type)
                9'b000000001: $display("Inserting 1 unit coin");
                9'b000000010: $display("Inserting 2 unit coin");
                9'b000000100: $display("Inserting 5 unit coin");
                9'b000001000: $display("Inserting 10 unit coin");
                9'b000010000: $display("Inserting 20 unit coin");
                9'b000100000: $display("Inserting 50 unit coin");
                9'b001000000: $display("Inserting 100 unit coin");
                9'b010000000: $display("Inserting 500 unit coin");
                9'b100000000: $display("Inserting 2000 unit coin");
                default: $display("Invalid coin type");
            endcase
            
            coin_inputs = coin_type;
            #100;
            coin_inserted = 1;
            #200;
            coin_inserted = 0;
            coin_inputs = 9'b000000000;
        end
    endtask

    // Task to signal payment completion
    task signal_payment_complete;
        begin
            $display("Payment complete signal");
            payment_complete = 1;
            #200;
            payment_complete = 0;
        end
    endtask

    // Simplified MODBUS frame sending tasks
    task send_modbus_write;
        input [15:0] reg_addr;
        input [15:0] data;
        begin
            // Simplified MODBUS write frame
            send_uart_byte(SLAVE_ADDR);     // Slave address
            send_uart_byte(8'h06);          // Function code (write single register)
            send_uart_byte(reg_addr[15:8]); // Register address high
            send_uart_byte(reg_addr[7:0]);  // Register address low
            send_uart_byte(data[15:8]);     // Data high
            send_uart_byte(data[7:0]);      // Data low
            send_uart_byte(8'h00);          // CRC low (simplified)
            send_uart_byte(8'h00);          // CRC high (simplified)
        end
    endtask

    task send_modbus_read;
        input [15:0] reg_addr;
        begin
            // Simplified MODBUS read frame
            send_uart_byte(SLAVE_ADDR);     // Slave address
            send_uart_byte(8'h03);          // Function code (read holding registers)
            send_uart_byte(reg_addr[15:8]); // Register address high
            send_uart_byte(reg_addr[7:0]);  // Register address low
            send_uart_byte(8'h00);          // Quantity high (1 register)
            send_uart_byte(8'h01);          // Quantity low
            send_uart_byte(8'h00);          // CRC low (simplified)
            send_uart_byte(8'h00);          // CRC high (simplified)
        end
    endtask

    // Task to send a byte via UART
    task send_uart_byte;
        input [7:0] data_byte;
        integer bit_count;
        begin
            // Start bit
            uart_rx = 0;
            #(1000000000/BAUD_RATE); // Bit period in ns
            
            // Data bits (LSB first)
            for (bit_count = 0; bit_count < 8; bit_count = bit_count + 1) begin
                uart_rx = data_byte[bit_count];
                #(1000000000/BAUD_RATE);
            end
            
            // Stop bit
            uart_rx = 1;
            #(1000000000/BAUD_RATE);
        end
    endtask

    // Monitor outputs
    initial begin
        $monitor("Time=%0t | Status LED=%b | Error LED=%b | Payment Accepted=%b | Payment Rejected=%b | Dispense Active=%b | Item=%0d | Machine Status=0x%02h", 
                 $time, status_led, error_led, payment_accepted_led, payment_rejected_led, dispense_trigger, item_select, machine_status);
    end

    // Monitor change motors
    always @(change_motors) begin
        if (change_motors != 9'b000000000) begin
            $display("Time=%0t | Change dispensing: motors=0b%09b", $time, change_motors);
        end
    end

    // Monitor dispense motors
    always @(dispense_motors) begin
        if (dispense_motors != 16'h0000) begin
            $display("Time=%0t | Item dispensing: motors=0x%04h", $time, dispense_motors);
        end
    end

endmodule