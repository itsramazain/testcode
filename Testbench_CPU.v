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
wire [15:0] BT_immediate_test;  // first 16 bits of the instruction to calculate branch target
wire [7:0] BT_test;           // branch target value
wire [7:0] BT_or_next_pc_test;  // max result (branch target or program counter)
wire branch_or_not_test;         // branch or program counter selection signal


wire register_write_enable_test;  // register write enable signal from control unit
wire [4:0] selected_write_register_address_test;  // register address selected to be written on
wire [31:0] write_data_test;    // data to be written to the selected register
wire [4:0] register_1_read_address_test;  // register 1 address (output 1)
wire [4:0] register_2_read_address_test; // register 2 address (output 2)


wire [3:0] alu_control_test;
wire [31:0] Alu_Operand_A_from_reg_test;    // output data from registers (output 1)
wire [31:0] Alu_Operand_B_from_reg_or_RAM_test;    // output data from registers (output 2)
wire [31:0] immediate_data_from_instruction_operand_B_test; //immediate data from instruction
wire [31:0] ALU_operand_B_result_test; // mux result for ALU operand B - between register output or immediate data from instruction
wire [4:0] shift_amount_test; // shift amount for ALU
wire [31:0] ALU_result_test;  // ALU result test
wire [9:0] RAM_address_test;   // Address to access RAM from ALU
wire zero_flag_test;
wire less_flag_test;
wire [31:0] RAM_result_test;


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
	 
	 .register_write_enable_test(register_write_enable_test),
	 .selected_write_register_address_test(selected_write_register_address_test),
	 .write_data_test(write_data_test),
	 .register_1_read_address_test(register_1_read_address_test),
	 .register_2_read_address_test(register_2_read_address_test),
	 
	 .alu_control_test(alu_control_test),
	 .Alu_Operand_A_from_reg_test(Alu_Operand_A_from_reg_test),
	 .Alu_Operand_B_from_reg_or_RAM_test( Alu_Operand_B_from_reg_or_RAM_test),
	 .immediate_data_from_instruction_operand_B_test(immediate_data_from_instruction_operand_B_test),
	 .ALU_operand_B_result_test(ALU_operand_B_result_test),
	 .shift_amount_test(shift_amount_test),
	 .ALU_result_test(ALU_result_test),
	 .RAM_address_test(RAM_address_test),
	 .zero_flag_test(zero_flag_test),
	 .less_flag_test(less_flag_test),
	 .RAM_result_test(RAM_result_test)
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