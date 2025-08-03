
# Synthesis Script for MODBUS Smart Vending Machine
# Compatible with Xilinx Vivado and Intel Quartus

# Xilinx Vivado TCL Script
puts "Starting MODBUS Vending Machine Synthesis..."

# Create project
create_project modbus_vending ./modbus_vending_project -part xc7a35tcpg236-1

# Add source files
add_files -norecurse {
    uart_interface.v
    dual_clock_fifo.v
    modbus_fsm.v
    dual_port_ram.v
    modbus_smart_vending_machine.v
}

# Add testbench
add_files -fileset sim_1 -norecurse modbus_vending_tb.v

# Add constraints
add_files -fileset constrs_1 -norecurse modbus_vending_constraints.xdc

# Set top module
set_property top modbus_smart_vending_machine [current_fileset]

# Synthesis settings
set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
set_property strategy Performance_ExtraTimingOpt [get_runs impl_1]

# Run synthesis
launch_runs synth_1 -jobs 4
wait_on_run synth_1

# Check timing
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

puts "Synthesis completed successfully"

# Run implementation
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

puts "Implementation completed - bitstream generated"
