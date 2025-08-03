
`timescale 1ns / 1ps

//============================================================================
// MODBUS RTU Finite State Machine - Verilog-2005 Compatible
// Industrial grade implementation without unpacked arrays in ports
//============================================================================
module modbus_fsm #(
    parameter SLAVE_ADDR = 8'h01
)(
    input  wire        clk,
    input  wire        rst,

    // FIFO interface (from dual-clock FIFO)
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,        // ~fifo_rd_empty from top level
    output reg         rx_rd_en,        // fifo_rd_en to top level

    // UART TX interface
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,

    // Register interface (to dual-port RAM)
    output reg  [15:0] reg_addr,
    output reg  [15:0] reg_wr_data,
    input  wire [15:0] reg_rd_data,
    output reg         reg_wr_en,
    output reg         reg_rd_en,

    // Control outputs (to vending controller)
    output reg  [7:0]  func_code,
    output reg         frame_error,
    output reg  [3:0]  dispense_item,
    output reg         dispense_cmd
);

    //========================================================================
    // MODBUS RTU State Machine States
    //========================================================================
    localparam [2:0] IDLE        = 3'b000,
                     RECEIVE     = 3'b001,
                     VALIDATE    = 3'b010,
                     PROCESS     = 3'b011,
                     RESPONSE    = 3'b100,
                     TRANSMIT    = 3'b101,
                     ERROR       = 3'b110;

    //========================================================================
    // Timing Parameters (adjust for 50MHz system clock, 19200 baud)
    //========================================================================
    localparam [15:0] SILENCE_3_5_CHAR = 16'd9114;  // 3.5 chars * 11 bits * (50MHz/19200)
    localparam [15:0] SILENCE_1_5_CHAR = 16'd3906;  // 1.5 chars * 11 bits * (50MHz/19200)

    //========================================================================
    // Internal Registers
    //========================================================================
    reg [2:0]  state, next_state;
    reg [15:0] silence_timer;

    // Frame buffer and counters
    reg [7:0]  rx_frame_0, rx_frame_1, rx_frame_2, rx_frame_3;
    reg [7:0]  rx_frame_4, rx_frame_5, rx_frame_6, rx_frame_7;
    reg [7:0]  rx_count;
    reg [7:0]  expected_length;

    // Response buffer
    reg [7:0]  tx_frame_0, tx_frame_1, tx_frame_2, tx_frame_3;
    reg [7:0]  tx_frame_4, tx_frame_5, tx_frame_6, tx_frame_7;
    reg [7:0]  tx_index;
    reg [7:0]  tx_length;

    // MODBUS frame fields
    reg [7:0]  slave_address;
    reg [7:0]  function_code;
    reg [15:0] register_address;
    reg [15:0] register_count;
    reg [15:0] register_value;
    reg [15:0] frame_crc;
    reg [15:0] calculated_crc;

    // Control flags
    reg frame_complete;
    reg crc_valid;
    reg address_match;
    reg response_ready;

    //========================================================================
    // CRC-16 MODBUS Calculation Task
    //========================================================================
    task crc16_modbus;
        input [7:0] length;
        output [15:0] crc_out;
        integer i, j;
        reg [15:0] crc;
        reg [7:0] data_byte;
        begin
            crc = 16'hFFFF;
            for (i = 0; i < 8; i = i + 1) begin
                if (i < length) begin
                    case (i)
                        0: data_byte = rx_frame_0;
                        1: data_byte = rx_frame_1;
                        2: data_byte = rx_frame_2;
                        3: data_byte = rx_frame_3;
                        4: data_byte = rx_frame_4;
                        5: data_byte = rx_frame_5;
                        6: data_byte = rx_frame_6;
                        7: data_byte = rx_frame_7;
                        default: data_byte = 8'h00;
                    endcase

                    crc = crc ^ data_byte;
                    for (j = 0; j < 8; j = j + 1) begin
                        if (crc[0])
                            crc = (crc >> 1) ^ 16'hA001;
                        else
                            crc = crc >> 1;
                    end
                end
            end
            crc_out = crc;
        end
    endtask

    //========================================================================
    // CRC calculation for response frames
    //========================================================================
    task crc16_response;
        input [7:0] length;
        output [15:0] crc_out;
        integer i, j;
        reg [15:0] crc;
        reg [7:0] data_byte;
        begin
            crc = 16'hFFFF;
            for (i = 0; i < 8; i = i + 1) begin
                if (i < length) begin
                    case (i)
                        0: data_byte = tx_frame_0;
                        1: data_byte = tx_frame_1;
                        2: data_byte = tx_frame_2;
                        3: data_byte = tx_frame_3;
                        4: data_byte = tx_frame_4;
                        5: data_byte = tx_frame_5;
                        6: data_byte = tx_frame_6;
                        7: data_byte = tx_frame_7;
                        default: data_byte = 8'h00;
                    endcase

                    crc = crc ^ data_byte;
                    for (j = 0; j < 8; j = j + 1) begin
                        if (crc[0])
                            crc = (crc >> 1) ^ 16'hA001;
                        else
                            crc = crc >> 1;
                    end
                end
            end
            crc_out = crc;
        end
    endtask

    //========================================================================
    // Main State Machine - Sequential Logic
    //========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            silence_timer <= 0;
            rx_count <= 0;
            tx_index <= 0;
            frame_complete <= 0;
            response_ready <= 0;

            // Clear all outputs
            rx_rd_en <= 0;
            tx_valid <= 0;
            reg_wr_en <= 0;
            reg_rd_en <= 0;
            dispense_cmd <= 0;
            frame_error <= 0;
            func_code <= 0;
            dispense_item <= 0;
            reg_addr <= 0;
            reg_wr_data <= 0;
            tx_data <= 0;

            // Clear frame buffers
            rx_frame_0 <= 0; rx_frame_1 <= 0; rx_frame_2 <= 0; rx_frame_3 <= 0;
            rx_frame_4 <= 0; rx_frame_5 <= 0; rx_frame_6 <= 0; rx_frame_7 <= 0;
            tx_frame_0 <= 0; tx_frame_1 <= 0; tx_frame_2 <= 0; tx_frame_3 <= 0;
            tx_frame_4 <= 0; tx_frame_5 <= 0; tx_frame_6 <= 0; tx_frame_7 <= 0;
        end else begin
            state <= next_state;

            // Silence timer management
            if (rx_valid && (state == IDLE || state == RECEIVE))
                silence_timer <= 0;
            else if (silence_timer < SILENCE_3_5_CHAR)
                silence_timer <= silence_timer + 1;

            // Default output states
            rx_rd_en <= 0;
            tx_valid <= 0;
            reg_wr_en <= 0;
            reg_rd_en <= 0;
            dispense_cmd <= 0;

            case (state)
                //====================================================
                IDLE: begin
                    rx_count <= 0;
                    tx_index <= 0;
                    frame_complete <= 0;
                    response_ready <= 0;
                    frame_error <= 0;

                    // Start receiving when silence period ends and data arrives
                    if (rx_valid && silence_timer >= SILENCE_3_5_CHAR) begin
                        rx_frame_0 <= rx_data;
                        rx_rd_en <= 1;
                        rx_count <= 1;
                        slave_address <= rx_data;
                    end
                end

                //====================================================
                RECEIVE: begin
                    if (rx_valid) begin
                        case (rx_count)
                            1: rx_frame_1 <= rx_data;
                            2: rx_frame_2 <= rx_data;
                            3: rx_frame_3 <= rx_data;
                            4: rx_frame_4 <= rx_data;
                            5: rx_frame_5 <= rx_data;
                            6: rx_frame_6 <= rx_data;
                            7: rx_frame_7 <= rx_data;
                        endcase

                        rx_rd_en <= 1;
                        rx_count <= rx_count + 1;

                        // Extract function code
                        if (rx_count == 1)
                            function_code <= rx_data;

                        // Determine expected frame length
                        if (rx_count == 1) begin
                            case (rx_data)
                                8'h03: expected_length <= 8;  // Read
                                8'h06: expected_length <= 8;  // Write
                                default: expected_length <= 8;
                            endcase
                        end
                    end

                    // Check for frame completion
                    if (silence_timer >= SILENCE_1_5_CHAR || rx_count >= expected_length) begin
                        frame_complete <= 1;
                    end
                end

                //====================================================
                VALIDATE: begin
                    // Extract frame fields
                    slave_address <= rx_frame_0;
                    function_code <= rx_frame_1;
                    register_address <= {rx_frame_2, rx_frame_3};

                    if (function_code == 8'h03) begin
                        register_count <= {rx_frame_4, rx_frame_5};
                    end else if (function_code == 8'h06) begin
                        register_value <= {rx_frame_4, rx_frame_5};
                    end

                    // Calculate and verify CRC
                    crc16_modbus(rx_count - 2, calculated_crc);
                    frame_crc <= {rx_frame_7, rx_frame_6}; // CRC is low byte first

                    // Validation checks
                    address_match <= (slave_address == SLAVE_ADDR);
                    func_code <= function_code;
                end

                //====================================================
                PROCESS: begin
                    crc_valid <= (calculated_crc == frame_crc);

                    if (!address_match) begin
                        // Ignore frame - not for this slave
                        frame_error <= 0;
                        response_ready <= 0;
                    end else if (!crc_valid) begin
                        // CRC error - send exception response
                        frame_error <= 1;
                        tx_frame_0 <= SLAVE_ADDR;
                        tx_frame_1 <= function_code | 8'h80;
                        tx_frame_2 <= 8'h04; // Server device failure
                        crc16_response(3, calculated_crc);
                        tx_frame_3 <= calculated_crc[7:0];
                        tx_frame_4 <= calculated_crc[15:8];
                        tx_length <= 5;
                        response_ready <= 1;
                    end else begin
                        // Valid frame - process command
                        frame_error <= 0;
                        reg_addr <= register_address;

                        case (function_code)
                            8'h03: begin // Read Holding Registers
                                reg_rd_en <= 1;

                                // Prepare response frame
                                tx_frame_0 <= SLAVE_ADDR;
                                tx_frame_1 <= 8'h03;
                                tx_frame_2 <= 8'h02; // Byte count
                                tx_frame_3 <= reg_rd_data[15:8];
                                tx_frame_4 <= reg_rd_data[7:0];
                                crc16_response(5, calculated_crc);
                                tx_frame_5 <= calculated_crc[7:0];
                                tx_frame_6 <= calculated_crc[15:8];
                                tx_length <= 7;
                                response_ready <= 1;
                            end

                            8'h06: begin // Write Single Register
                                reg_wr_data <= register_value;
                                reg_wr_en <= 1;

                                // Check for dispense command
                                if (register_address == 16'h0100) begin
                                    dispense_item <= register_value[3:0];
                                    dispense_cmd <= 1;
                                end

                                // Echo request as response
                                tx_frame_0 <= rx_frame_0;
                                tx_frame_1 <= rx_frame_1;
                                tx_frame_2 <= rx_frame_2;
                                tx_frame_3 <= rx_frame_3;
                                tx_frame_4 <= rx_frame_4;
                                tx_frame_5 <= rx_frame_5;
                                crc16_response(6, calculated_crc);
                                tx_frame_6 <= calculated_crc[7:0];
                                tx_frame_7 <= calculated_crc[15:8];
                                tx_length <= 8;
                                response_ready <= 1;
                            end

                            default: begin // Unsupported function
                                tx_frame_0 <= SLAVE_ADDR;
                                tx_frame_1 <= function_code | 8'h80;
                                tx_frame_2 <= 8'h01; // Illegal function
                                crc16_response(3, calculated_crc);
                                tx_frame_3 <= calculated_crc[7:0];
                                tx_frame_4 <= calculated_crc[15:8];
                                tx_length <= 5;
                                response_ready <= 1;
                            end
                        endcase
                    end
                end

                //====================================================
                RESPONSE: begin
                    // Wait for 3.5 character silence before responding
                    if (silence_timer >= SILENCE_3_5_CHAR) begin
                        tx_index <= 0;
                    end
                end

                //====================================================
                TRANSMIT: begin
                    if (tx_ready && !tx_valid && tx_index < tx_length) begin
                        case (tx_index)
                            0: tx_data <= tx_frame_0;
                            1: tx_data <= tx_frame_1;
                            2: tx_data <= tx_frame_2;
                            3: tx_data <= tx_frame_3;
                            4: tx_data <= tx_frame_4;
                            5: tx_data <= tx_frame_5;
                            6: tx_data <= tx_frame_6;
                            7: tx_data <= tx_frame_7;
                            default: tx_data <= 8'h00;
                        endcase
                        tx_valid <= 1;
                        tx_index <= tx_index + 1;
                    end else if (!tx_ready && tx_valid) begin
                        tx_valid <= 1; // Hold valid until accepted
                    end else begin
                        tx_valid <= 0;
                    end
                end

                //====================================================
                ERROR: begin
                    frame_error <= 1;
                    // Error handling - return to IDLE after timeout
                end

                default: begin
                    // Default case - return to IDLE
                end
            endcase
        end
    end

    //========================================================================
    // Next State Logic - Combinational
    //========================================================================
    always @(*) begin
        next_state = state;

        case (state)
            IDLE: begin
                if (rx_valid && silence_timer >= SILENCE_3_5_CHAR)
                    next_state = RECEIVE;
            end

            RECEIVE: begin
                if (frame_complete)
                    next_state = VALIDATE;
                else if (rx_count >= 8) // Maximum frame size protection
                    next_state = ERROR;
            end

            VALIDATE: begin
                next_state = PROCESS;
            end

            PROCESS: begin
                if (response_ready && address_match)
                    next_state = RESPONSE;
                else
                    next_state = IDLE;
            end

            RESPONSE: begin
                if (silence_timer >= SILENCE_3_5_CHAR)
                    next_state = TRANSMIT;
            end

            TRANSMIT: begin
                if (tx_index >= tx_length)
                    next_state = IDLE;
            end

            ERROR: begin
                if (silence_timer >= SILENCE_3_5_CHAR)
                    next_state = IDLE;
            end

            default: begin
                next_state = IDLE;
            end
        endcase
    end

endmodule
