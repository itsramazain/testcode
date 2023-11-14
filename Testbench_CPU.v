`timescale 1ns/1ps

module Testbench_CPU;

// Parameters
parameter CLK_PERIOD = 1; // Clock period in ps

// Signals
reg MAX10_CLK1_50;          // clock signal
reg reset;                  // reset signal
wire [7:0] pc_next_test;     // program counter value
wire pc_increment_test;      // program counter enable (for future purposes)
wire [31:0] instruction_test;  // instruction to be fetched
wire [15:0] BT_immediate_test;
wire [7:0] BT_test;           // branch target value
wire [7:0] BT_or_next_pc_test;  // max result (branch target or program counter)
wire branch_or_not_test;         // branch or program counter selection signal
wire [4:0] selected_write_register_test;  // register address selected to be written on
wire [31:0] write_data_test;    // data to be written to the selected register
wire [4:0] register_1_read_address_test;
wire [31:0] read_data_1_test;    // output data from registers (output 1)
wire [4:0] register_2_read_address_test;
wire [31:0] read_data_2_test;    // output data from registers (output 2)
wire [3:0] alu_control_test;

// Instantiate CPU module
CPU my_cpu (
    .MAX10_CLK1_50(MAX10_CLK1_50),
    .reset(reset),
	 .pc_next_test(pc_next_test),
	 .pc_increment_test(pc_increment_test),
	 .instruction_test(instruction_test),
	 .BT_immediate_test(BT_immediate_test),
	 .BT_test(BT_test),
	 .BT_or_next_pc_test(BT_or_next_pc_test),
	 .branch_or_not_test(branch_or_not_test),
	 .selected_write_register_test(selected_register_test),
	 .write_data_test(write_data_test),
	 .register_1_read_address_test(register_1_read_address_test),
	 .read_data_1_test(read_data_1_test),
	 .register_2_read_address_test(register_2_read_address_test),
	 .read_data_2_test(read_data_2_test),
	 .alu_control_test(alu_control_test)
	 );


// Clock generation
initial begin
			MAX10_CLK1_50 <= 1;
		end 
		
		always @ (*)
		begin : Clock_Generator
			#(CLK_PERIOD) MAX10_CLK1_50 <= ~MAX10_CLK1_50;
		end
		
		
// Test scenario
initial begin
    // Initialize signals
    reset = 1;
    #2; // Reset for 500 cycles

    // Release reset
    reset = 0;

    // Monitor signals
    $monitor("Time=%0t: MAX10_CLK1_50=%b, reset=%b", $time, MAX10_CLK1_50, reset);

    // Continue simulation
    #1000; // Run for additional cycles

    // Add more test scenarios here

    // End simulation
    $stop;
end

endmodule