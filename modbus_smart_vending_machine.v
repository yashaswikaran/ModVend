
// Top-Level MODBUS-Controlled Smart Vending Machine
// Complete system integration with all components
module modbus_smart_vending_machine #(
    parameter BAUD_RATE = 19200,
    parameter CLK_FREQ = 50_000_000,
    parameter SLAVE_ADDR = 8'h01,
    parameter NUM_ITEMS = 16
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

    // LED indicators
    output wire status_led,             // System status
    output wire error_led,              // Error indicator
    output wire comm_led                // Communication activity
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

    // Inventory RAM signals
    wire [7:0] inv_addr_a, inv_addr_b;
    wire [15:0] inv_data_a, inv_data_b;
    wire [15:0] inv_q_a, inv_q_b;
    wire inv_we_a, inv_we_b;

    // Control signals
    wire dispense_cmd;
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
        .dispense_cmd(dispense_cmd)
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
        .we_a(modbus_wr_en),
        .q_a(modbus_rd_data),

        // Port B - System access
        .clk_b(clk_sys),
        .addr_b(inv_addr_b),
        .data_b(inv_data_b),
        .we_b(inv_we_b),
        .q_b(inv_q_b)
    );

    //========================================================================
    // Vending Machine Control Logic
    //========================================================================
    vending_controller vend_ctrl (
        .clk(clk_sys),
        .rst(rst),

        // Dispense commands
        .dispense_cmd(dispense_cmd),
        .item_select(dispense_item),

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
        .error_state(error_led)
    );

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
            if (modbus_frame_error) begin
                error_timeout <= 16'hFFFF;   // ~1.3ms at 50MHz
            end else if (error_timeout > 0) begin
                error_timeout <= error_timeout - 1;
            end

            // Status register assembly
            status_register <= {
                4'b0000,           // Reserved
                dispense_trigger,  // Bit 11: Dispensing active
                error_led,         // Bit 10: Error state
                comm_led,          // Bit 9: Communication active
                status_led,        // Bit 8: System ready
                item_select,       // Bits 7-4: Current item
                modbus_func_code[3:0] // Bits 3-0: Last function code
            };
        end
    end

    // Output assignments
    assign machine_status = status_register[7:0];
    assign status_led = ~rst & ~error_led;
    assign comm_led = (comm_timeout > 0);
    assign error_led = (error_timeout > 0) | modbus_frame_error;

endmodule

//============================================================================
// Vending Machine Controller - Handles physical dispensing operations
//============================================================================
module vending_controller #(
    parameter NUM_ITEMS = 16,
    parameter DISPENSE_TIME = 50000000  // 1 second at 50MHz
)(
    input wire clk,
    input wire rst,

    // Command interface
    input wire dispense_cmd,
    input wire [3:0] item_select,

    // Inventory interface
    output reg [7:0] inv_addr,
    output reg [15:0] inv_data_out,
    input wire [15:0] inv_data_in,
    output reg inv_we,

    // Physical interface
    input wire [NUM_ITEMS-1:0] item_sensors,
    output reg [NUM_ITEMS-1:0] dispense_motors,

    // Status outputs
    output reg dispense_active,
    output reg [3:0] current_item,
    output reg error_state
);

    // State machine for dispensing
    localparam IDLE = 2'b00,
               CHECK_STOCK = 2'b01,
               DISPENSE = 2'b10,
               UPDATE_INV = 2'b11;

    reg [1:0] state, next_state;
    reg [31:0] dispense_timer;
    reg [15:0] current_stock;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            dispense_timer <= 0;
            dispense_motors <= 0;
            dispense_active <= 0;
            current_item <= 0;
            error_state <= 0;
            inv_addr <= 0;
            inv_data_out <= 0;
            inv_we <= 0;
        end else begin
            state <= next_state;

            case (state)
                IDLE: begin
                    dispense_active <= 0;
                    dispense_motors <= 0;
                    if (dispense_cmd) begin
                        current_item <= item_select;
                        inv_addr <= item_select;
                        inv_we <= 0;
                    end
                end

                CHECK_STOCK: begin
                    current_stock <= inv_data_in;
                    if (inv_data_in == 0) begin
                        error_state <= 1;  // Out of stock
                    end else if (!item_sensors[item_select]) begin
                        error_state <= 1;  // Item not detected
                    end
                end

                DISPENSE: begin
                    dispense_active <= 1;
                    dispense_motors[current_item] <= 1;
                    if (dispense_timer < DISPENSE_TIME) begin
                        dispense_timer <= dispense_timer + 1;
                    end
                end

                UPDATE_INV: begin
                    dispense_motors <= 0;
                    inv_addr <= current_item;
                    inv_data_out <= current_stock - 1;
                    inv_we <= 1;
                end
            endcase
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (dispense_cmd) next_state = CHECK_STOCK;
            end
            CHECK_STOCK: begin
                if (error_state) next_state = IDLE;
                else next_state = DISPENSE;
            end
            DISPENSE: begin
                if (dispense_timer >= DISPENSE_TIME) next_state = UPDATE_INV;
            end
            UPDATE_INV: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule
