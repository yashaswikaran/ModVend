//============================================================================
// TOP MODULE: modbus_smart_vending_machine
// Complete system integration with all components
//============================================================================
module modbus_smart_vending_machine #(
    parameter BAUD_RATE = 19200,
    parameter CLK_FREQ = 50_000_000,
    parameter SLAVE_ADDR = 8'h01,
    parameter NUM_ITEMS = 16,
    parameter PRICE_WIDTH = 16
)(
    // System clocks and reset
    input wire clk_sys,          // 50MHz system clock
    input wire clk_uart,         // UART baud clock
    input wire rst,              // Active high reset

    // UART interface
    input wire uart_rx,          // UART receive
    output wire uart_tx,         // UART transmit

    // Vending machine outputs
    output wire [3:0] item_select,      // Selected item (0-15)
    output wire dispense_trigger,       // Item dispense signal
    output wire [7:0] machine_status,   // Status register

    // Physical interface
    input wire [NUM_ITEMS-1:0] item_sensors,  // Item presence sensors
    output wire [NUM_ITEMS-1:0] dispense_motors, // Motor control

    // Payment interface
    input wire [8:0] coin_inputs,       // Coin denomination inputs (1,2,5,10,20,50,100,500,2000)
    input wire coin_inserted,           // Coin insertion event
    input wire payment_complete,        // User finished inserting coins
    input wire transaction_cancel,      // Cancel transaction
    output wire [8:0] change_motors,    // Change dispensing motors

    // LED indicators
    output wire status_led,             // System status
    output wire error_led,              // Error indicator
    output wire comm_led,               // Communication activity
    output wire payment_accepted_led,   // Payment successful
    output wire payment_rejected_led    // Payment rejected
);

    // Internal signal declarations
    wire [7:0] uart_rx_data, uart_tx_data;
    wire uart_rx_valid, uart_tx_valid, uart_tx_ready;

    // FIFO signals
    wire [7:0] fifo_rd_data;
    wire fifo_rd_empty, fifo_wr_full;
    wire fifo_rd_en;

    // MODBUS FSM signals
    wire [15:0] modbus_reg_addr;
    wire [15:0] modbus_wr_data, modbus_rd_data;
    wire modbus_wr_en, modbus_rd_en;
    wire [7:0] modbus_func_code;
    wire modbus_frame_error;
    wire dispense_cmd_modbus;

    // Inventory RAM signals (Port A - MODBUS, Port B - System)
    wire [7:0] inv_addr_a, inv_addr_b;
    wire [15:0] inv_data_a, inv_data_b;
    wire [15:0] inv_q_a, inv_q_b;
    wire inv_we_a, inv_we_b;

    // Price RAM signals (Port A - MODBUS, Port B - Payment)
    wire [7:0] price_addr_a, price_addr_b;
    wire [15:0] price_data_a, price_data_b;
    wire [15:0] price_q_a, price_q_b;
    wire price_we_a, price_we_b;

    // Payment controller signals
    wire [3:0] selected_item;
    wire [PRICE_WIDTH-1:0] total_inserted;
    wire payment_start;
    wire dispense_cmd_payment;
    wire payment_accepted, payment_rejected;
    wire [PRICE_WIDTH-1:0] change_amount;
    wire change_ready;
    wire change_dispensed;
    wire vending_error;

    // Control signals
    wire dispense_cmd_final;
    wire [3:0] dispense_item;
    reg [15:0] status_register;

    // Activity indicators
    reg [23:0] comm_timeout;
    reg [15:0] error_timeout;

    //========================================================================
    // UART Interface Module
    //========================================================================
    uart_interface #(
        .BAUD_RATE(BAUD_RATE),
        .CLK_FREQ(CLK_FREQ)
    ) uart_inst (
        .clk(clk_uart),
        .rst(rst),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .rx_data(uart_rx_data),
        .rx_valid(uart_rx_valid),
        .tx_data(uart_tx_data),
        .tx_valid(uart_tx_valid),
        .tx_ready(uart_tx_ready)
    );

    //========================================================================
    // Dual Clock FIFO for Clock Domain Crossing
    //========================================================================
    dual_clock_fifo #(
        .DATA_WIDTH(8),
        .FIFO_DEPTH(64)
    ) rx_fifo (
        // Write domain (UART clock)
        .wr_clk(clk_uart),
        .wr_rst(rst),
        .wr_data(uart_rx_data),
        .wr_en(uart_rx_valid),
        .wr_full(fifo_wr_full),

        // Read domain (System clock)
        .rd_clk(clk_sys),
        .rd_rst(rst),
        .rd_data(fifo_rd_data),
        .rd_en(fifo_rd_en),
        .rd_empty(fifo_rd_empty)
    );

    //========================================================================
    // MODBUS RTU Finite State Machine
    //========================================================================
    modbus_fsm #(
        .SLAVE_ADDR(SLAVE_ADDR)
    ) modbus_controller (
        .clk(clk_sys),
        .rst(rst),

        // FIFO interface
        .rx_data(fifo_rd_data),
        .rx_valid(~fifo_rd_empty),
        .rx_rd_en(fifo_rd_en),

        // UART TX interface
        .tx_data(uart_tx_data),
        .tx_valid(uart_tx_valid),
        .tx_ready(uart_tx_ready),

        // Register interface
        .reg_addr(modbus_reg_addr),
        .reg_wr_data(modbus_wr_data),
        .reg_rd_data(modbus_rd_data),
        .reg_wr_en(modbus_wr_en),
        .reg_rd_en(modbus_rd_en),

        // Control outputs
        .func_code(modbus_func_code),
        .frame_error(modbus_frame_error),
        .dispense_item(dispense_item),
        .dispense_cmd(dispense_cmd_modbus),
        .selected_item_out(selected_item)
    );

    //========================================================================
    // Dual Port RAM for Inventory Management
    //========================================================================
    dual_port_ram #(
        .DATA_WIDTH(16),
        .ADDR_WIDTH(8),
        .DEPTH(256)
    ) inventory_memory (
        // Port A - MODBUS access
        .clk_a(clk_sys),
        .addr_a(modbus_reg_addr[7:0]),
        .data_a(modbus_wr_data),
        .we_a(modbus_wr_en & (modbus_reg_addr[15:8] == 8'h00)), // Inventory at 0x00xx
        .q_a(inv_q_a),

        // Port B - System access
        .clk_b(clk_sys),
        .addr_b(inv_addr_b),
        .data_b(inv_data_b),
        .we_b(inv_we_b),
        .q_b(inv_q_b)
    );

    //========================================================================
    // Dual Port RAM for Price Storage
    //========================================================================
    dual_port_ram #(
        .DATA_WIDTH(16),
        .ADDR_WIDTH(8),
        .DEPTH(256)
    ) price_memory (
        // Port A - MODBUS access for price setting
        .clk_a(clk_sys),
        .addr_a(modbus_reg_addr[7:0]),
        .data_a(modbus_wr_data),
        .we_a(modbus_wr_en & (modbus_reg_addr[15:8] == 8'h01)), // Prices at 0x01xx
        .q_a(price_q_a),

        // Port B - Payment controller access
        .clk_b(clk_sys),
        .addr_b(price_addr_b),
        .data_b(16'h0000),
        .we_b(1'b0),
        .q_b(price_q_b)
    );

    //========================================================================
    // Payment Controller with Multi-Denomination Support
    //========================================================================
    payment_controller #(
        .NUM_ITEMS(NUM_ITEMS),
        .PRICE_WIDTH(PRICE_WIDTH)
    ) payment_ctrl (
        .clk(clk_sys),
        .rst(rst),

        // Coin interface
        .coin_inputs(coin_inputs),
        .coin_inserted(coin_inserted),
        .payment_complete(payment_complete),
        .transaction_cancel(transaction_cancel),

        // Item selection interface
        .selected_item(selected_item),
        .start_transaction(payment_start),

        // Price memory interface
        .price_mem_addr(price_addr_b),
        .price_mem_data(price_q_b),

        // Outputs
        .dispense_cmd(dispense_cmd_payment),
        .payment_accepted(payment_accepted),
        .payment_rejected(payment_rejected),
        .total_inserted(total_inserted),
        .change_amount(change_amount),
        .change_ready(change_ready)
    );

    //========================================================================
    // Change Dispensing Controller
    //========================================================================
    change_dispenser #(
        .PRICE_WIDTH(PRICE_WIDTH)
    ) change_ctrl (
        .clk(clk_sys),  
        .rst(rst),
        .change_amount(change_amount),
        .dispense_change(change_ready),
        .change_motors(change_motors),
        .change_complete(change_dispensed)
    );

    //========================================================================
    // Vending Machine Control Logic
    //========================================================================
    vending_controller #(
        .NUM_ITEMS(NUM_ITEMS)
    ) vend_ctrl (
        .clk(clk_sys),
        .rst(rst),

        // Dispense commands (priority to payment controller)
        .dispense_cmd(dispense_cmd_final),
        .item_select(selected_item),

        // Inventory interface
        .inv_addr(inv_addr_b),
        .inv_data_out(inv_data_b),
        .inv_data_in(inv_q_b),
        .inv_we(inv_we_b),

        // Physical interface
        .item_sensors(item_sensors),
        .dispense_motors(dispense_motors),

        // Status
        .dispense_active(dispense_trigger),
        .current_item(item_select),
        .error_state(vending_error)
    );

    // Logic to determine dispense command source
    assign payment_start = dispense_cmd_modbus; // MODBUS triggers payment check
    assign dispense_cmd_final = payment_accepted & dispense_cmd_payment;

    // Status register assembly for MODBUS read access
    assign modbus_rd_data = (modbus_reg_addr[15:8] == 8'h00) ? inv_q_a :
                           (modbus_reg_addr[15:8] == 8'h01) ? price_q_a :
                           (modbus_reg_addr == 16'hFF00) ? status_register : 16'h0000;

    //========================================================================
    // Status and Indicator Logic
    //========================================================================
    always @(posedge clk_sys) begin
        if (rst) begin
            status_register <= 16'h0000;
            comm_timeout <= 24'h000000;
            error_timeout <= 16'h0000;
        end else begin
            // Communication activity indicator
            if (uart_rx_valid || uart_tx_valid) begin
                comm_timeout <= 24'hFFFFFF;  // ~335ms at 50MHz
            end else if (comm_timeout > 0) begin
                comm_timeout <= comm_timeout - 1;
            end

            // Error indicator timeout
            if (modbus_frame_error || vending_error || payment_rejected) begin
                error_timeout <= 16'hFFFF;   // ~1.3ms at 50MHz
            end else if (error_timeout > 0) begin
                error_timeout <= error_timeout - 1;
            end

            // Status register assembly (readable via MODBUS at 0xFF00)
            status_register <= {
                change_dispensed,      // Bit 15: Change dispensing complete
                payment_accepted,      // Bit 14: Payment accepted
                payment_rejected,      // Bit 13: Payment rejected
                dispense_trigger,      // Bit 12: Dispensing active
                vending_error,         // Bit 11: Vending error
                comm_led,              // Bit 10: Communication active
                status_led,            // Bit 9: System ready
                1'b0,                  // Bit 8: Reserved
                item_select,           // Bits 7-4: Current item
                modbus_func_code[3:0]  // Bits 3-0: Last function code
            };
        end
    end

    // Output assignments
    assign machine_status = status_register[7:0];
    assign status_led = ~rst & ~vending_error;
    assign comm_led = (comm_timeout > 0);
    assign error_led = (error_timeout > 0) | modbus_frame_error;
    assign payment_accepted_led = payment_accepted;
    assign payment_rejected_led = payment_rejected;

endmodule