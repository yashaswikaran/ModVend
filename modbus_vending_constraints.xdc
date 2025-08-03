
# Timing Constraints for MODBUS Smart Vending Machine
# Target: Xilinx Artix-7 XC7A35T

# Clock definitions
create_clock -period 20.000 -name clk_sys [get_ports clk_sys]
create_clock -period 52083.333 -name clk_uart [get_ports clk_uart]

# Clock domain crossing constraints
set_clock_groups -asynchronous -group [get_clocks clk_sys] -group [get_clocks clk_uart]

# Input/Output delays
set_input_delay -clock clk_sys 2.0 [get_ports {uart_rx item_sensors[*]}]
set_output_delay -clock clk_sys 2.0 [get_ports {uart_tx dispense_motors[*] *_led}]

# Pin assignments (example for Basys 3 board)
set_property PACKAGE_PIN W5 [get_ports clk_sys]
set_property IOSTANDARD LVCMOS33 [get_ports clk_sys]

set_property PACKAGE_PIN B18 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]

set_property PACKAGE_PIN A18 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

# LED assignments
set_property PACKAGE_PIN U16 [get_ports status_led]
set_property IOSTANDARD LVCMOS33 [get_ports status_led]

set_property PACKAGE_PIN E19 [get_ports error_led]
set_property IOSTANDARD LVCMOS33 [get_ports error_led]

set_property PACKAGE_PIN U19 [get_ports comm_led]
set_property IOSTANDARD LVCMOS33 [get_ports comm_led]

# Switch assignments for item sensors (simulation)
set_property PACKAGE_PIN V17 [get_ports {item_sensors[0]}]
set_property PACKAGE_PIN V16 [get_ports {item_sensors[1]}]
set_property PACKAGE_PIN W16 [get_ports {item_sensors[2]}]
set_property PACKAGE_PIN W17 [get_ports {item_sensors[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {item_sensors[*]}]

# Reset
set_property PACKAGE_PIN T18 [get_ports rst]
set_property IOSTANDARD LVCMOS33 [get_ports rst]

# Timing exceptions for reset
set_false_path -from [get_ports rst]

# FIFO timing constraints
set_max_delay -from [get_pins -hierarchical *fifo*/wr_ptr_gray_reg*/C] -to [get_pins -hierarchical *fifo*/rd_ptr_gray_sync_reg*/D] 20.0
set_max_delay -from [get_pins -hierarchical *fifo*/rd_ptr_gray_reg*/C] -to [get_pins -hierarchical *fifo*/wr_ptr_gray_sync_reg*/D] 52083.333

# UART timing
set_input_delay -clock clk_uart -max 5000.0 [get_ports uart_rx]
set_input_delay -clock clk_uart -min -5000.0 [get_ports uart_rx]
set_output_delay -clock clk_uart -max 5000.0 [get_ports uart_tx]
set_output_delay -clock clk_uart -min -5000.0 [get_ports uart_tx]
