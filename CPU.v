module CPU (
    MAX10_CLK1_50,        // Clock input from DE-10 Lite FPGA (50MHZ)
    reset,                 // Reset signal (active high)
	 pc_next_test,            // PC Output (ROM address)
	 pc_increment_test,
	 instruction_test,
	 BT_immediate_test,
	 BT_test,
	 BT_or_next_pc_test,
	 branch_or_not_test,
	 register_write_enable_test,
	 selected_write_register_address_test,
	 write_data_test,
	 register_1_read_address_test,
	 register_2_read_address_test,
	 alu_control_test,
	 Alu_Operand_A_from_reg_test,
	 Alu_Operand_B_from_reg_or_RAM_test,
	 immediate_data_from_instruction_operand_B_test,
	 ALU_operand_B_result_test,
	 ALU_src_selector_signal,
	 shift_amount_test,
	 ALU_result_test,
	 RAM_address_test,
	 zero_flag_test,
	 less_flag_test,
	 RAM_result_test,
	 ALU_or_RAM_result_test,
	 ALU_or_RAM_select_signal_test,
	 RAM_write_enable_signal_test,
	 RAM_read_enable_signal_test
	 );

/* ******************* inputs for CPU ******************** */

input MAX10_CLK1_50;        
input reset;



/* **************** Program Counter (PC) and Instruction test Signals  ************** */

wire [7:0] pc_next;            // PC Output (ROM address)
output [7:0] pc_next_test;
assign pc_next_test = pc_next;
 
wire pc_increment;             // Fetch instruction control signal
output pc_increment_test;
assign pc_increment_test = pc_increment;

wire [31:0] instruction;       // Instruction fetched from ROM (sent to Control unit to decode)
output [31:0] instruction_test;
assign instruction_test = instruction;

output [15:0] BT_immediate_test;  // the value that should be added to PC to calculate the branch target
assign BT_immediate_test = instruction;



/* **************** Branch Target Calculation test Signals ********************* */

wire [7:0] BT;                // Branch Target
output [7:0] BT_test;
assign BT_test = BT;

wire [7:0] BT_or_next_pc;     // MUX Output (Branch Target or Next PC)
output [7:0] BT_or_next_pc_test;
assign BT_or_next_pc_test = BT_or_next_pc;

wire branch_or_not;            // Branch or Not Signal
output branch_or_not_test;
assign branch_or_not_test = branch_or_not;



/* ********************** Register File test Signals ************************ */

// register write enable signal 
wire write_en;
output register_write_enable_test;
assign register_write_enable_test = write_en;

// Selected Register address for Write - 5 bits
wire [4:0] write_register;     // Write Register   
output [4:0] selected_write_register_address_test;
assign selected_write_register_address_test = write_register;

// Data to Write in Register - 32 bits
wire [31:0] reg_or_mem_or_ra;  
output [31:0] write_data_test;
assign write_data_test = reg_or_mem_or_ra;

// register 1 address to be read from (ALU Operand A)
wire [4:0] read_register_1;   
output [4:0] register_1_read_address_test;
assign register_1_read_address_test = read_register_1;


// register 2 address to be read from (ALU Operand b or write to ram)
wire [4:0] read_register_2; 
output [4:0] register_2_read_address_test;
assign register_2_read_address_test = read_register_2;




/* **************** ALU Control test Signals ********************** */

// ALU operation
wire [3:0] alu_control;        
output [3:0] alu_control_test;
assign alu_control_test = alu_control;


// Data output from Register 1 choosen above (ALU Operand A & jump register address mux)
wire [31:0] alu_operand_A;     
output [31:0] Alu_Operand_A_from_reg_test;
assign Alu_Operand_A_from_reg_test = alu_operand_A;

// Data output from Register 2 choosen above (ALU Operand b or write to ram)
wire [31:0] alu_operand_B;     
output [31:0] Alu_Operand_B_from_reg_or_RAM_test;
assign Alu_Operand_B_from_reg_or_RAM_test = alu_operand_B;

// immediate data from instruction [15:0] - choose immediate data or data output from reg to ALU operand B
wire [31:0] sign_extended_imm;
output [31:0] immediate_data_from_instruction_operand_B_test;
assign immediate_data_from_instruction_operand_B_test = sign_extended_imm;

// mux result for ALU operand B - between register output or immediate data from instruction
wire [31:0] operand_B;
output [31:0] ALU_operand_B_result_test;
assign ALU_operand_B_result_test = operand_B;

// Control signal to choose input (0 for sign-extended_imm, 1 for read_register_2)
wire alu_src;    
output ALU_src_selector_signal;
assign ALU_src_selector_signal = alu_src;

// shift amount for ALU - instruction [10:6]
output [4:0] shift_amount_test;
assign shift_amount_test = instruction[15:0];

// ALU result 
wire [31:0] alu_result;
output [31:0] ALU_result_test;
assign ALU_result_test = alu_result;

// Address to access RAM from ALU
wire [9:0] ram_address;        
output [9:0] RAM_address_test;
assign RAM_address_test = ram_address;

// Zero Signal from ALU
wire zero;                     
output zero_flag_test;
assign zero_flag_test = zero;

// Less Signal from ALU
wire less;                     
output less_flag_test;
assign less_flag_test = less;


/* ********************** RAM test Signals ************************ */

wire [31:0] ram_result;        // Data from RAM
output [31:0] RAM_result_test;
assign RAM_result_test = ram_result;

// result when choose between ALU OR RAM result
wire [31:0] mem_reg_result;
output [31:0] ALU_or_RAM_result_test;
assign ALU_or_RAM_result_test = mem_reg_result;

// signal to select between RAM or ALU result
wire mem_reg_selector;         // Memory or Register Selector
output ALU_or_RAM_select_signal_test;
assign ALU_or_RAM_select_signal_test = mem_reg_selector;

// RAM Write Enable Signal
wire ram_write_enable;         
output RAM_write_enable_signal_test;
assign RAM_write_enable_signal_test = ram_write_enable;

// RAM read enable signal
wire ram_read_enable;       
output RAM_read_enable_signal_test;
assign RAM_read_enable_signal_test = ram_read_enable;





wire jumptwocon;               // Jump to CONtrol unit
wire jump;                     // Jump Instruction Signal
wire [31:0] jump_or_next_pc_or_branch; // MUX Output (Jump or Next PC or Branch)
wire branch;                   // Branch Instruction Signal
wire jr;                       // Jump to Register Signal
wire [31:0] jump_or_next_pc_or_branch_or_jr; // MUX Output (Jump or Next PC or Branch or JR)
wire jal;                      // Jump and Link Instruction Signal

// Branch Condition Signals
wire branchnotequal;           // Branch Not Equal Signal
wire brachlessthat;            // Branch Less Than Signal
wire branchgreaterthan;        // Branch Greater Than Signal
wire branchlessthanorequal;    // Branch Less Than or Equal Signal
wire branchgreaterthanorequal; // Branch Greater Than or Equal Signal
wire brancheq;                 // Branch Equal Signal


// Instantiate the Program Counter module
ProgramCounter program_counter (
    .clk(MAX10_CLK1_50),               // input for clock
    .reset(reset),                     // input for reset
	 .enable_increment(pc_increment),   // input for enable increment
    .pc(pc_next)                       // output - 8 bits address for ROM
);


// Instantiate a 2x1 multiplexer to select the branch target or next program counter
Mux2x1 Branch_target_OR_PC (
    .i0(pc_next),              // Input 0: Next program counter value
    .i1(BT),                   // Input 1: Branch target value
    .sel(branch_or_not),       // Select signal to choose between inputs
    .out(BT_or_next_pc)        // Output signal, either the branch target or next program counter
);



// Instantiate the branch target calculator module
Branch_Target_Calculator branch_calc(
		.immediate(instruction[15:0]),               //immedate filed from the instruction
		.program_counter(pc_next),                   //nest instruction
		.BT(BT)
		);
		
// Choose between branch or next instruction logic				
BranchLogic branch_or_next_instruction(
    .brancheq(brancheq),
    .branchnotequal(branchnotequal),
    .brachlessthat(brachlessthat),
    .branchgreaterthanorequal(branchgreaterthanorequal),
    .branchlessthanorequal(branchlessthanorequal),
    .jal(jal),
    .zero(zero),
    .less(less),
    .branch_or_not(branch_or_not)
);

	
// Instantiate a 2x1 multiplexer to select the jump address if it's a jump instruction
Mux2x1 chooseJUMP (
    .i0(BT_or_next_pc),                       // Input 0: Branch target or next program counter
    .i1({{pc_next}, {instruction[25:0]}}),    // Input 1: Concatenation of next program counter and instruction bits[25:0]
    .sel(jump),                               // Select signal to choose between inputs
    .out(jump_or_next_pc_or_branch)           // Output signal, either the jump address or next program counter or branch target
);


// Instantiate a 2x1 multiplexer to select the register address if it's a JR (Jump Register) instruction
Mux2x1 jumptoregister (
    .i0(jump_or_next_pc_or_branch),         // Input 0: Output of the previous multiplexer (jump address, next PC, or branch target)
    .i1(alu_operand_A),                     // Input 1: Operand A value from ALU
    .sel(jr),                               // Select signal to choose between inputs
    .out(jump_or_next_pc_or_branch_or_jr)   // Output signal, either the selected address or ALU operand A
);


// Instantiate the ROM module     -- containts 256 words each word is 32 bits
ROM32x256 ROM (
    .clock(MAX10_CLK1_50),                             // input for clock
	 .address(jump_or_next_pc_or_branch_or_jr),         // input - 8 bits address from PC
    .q(instruction)                                    // ROM output - 32 bits instruction
);



// Instantiate the ControlUnit module
ControlUnit control_unit (
    .clk(MAX10_CLK1_50),                               // Clock input
    .reset(reset),                                     // Reset signal
    .instruction(instruction),                         // Input instruction
    .alu_op(alu_control),                              // ALU control signal
    .alu_src(alu_src),                                 // ALU source selector
    .reg_write_enable(write_en),                       // Register file write enable
    .pc_increment(pc_increment),                       // Program counter increment signal
    .mem_to_reg(mem_reg_selector),                     // Memory-to-register selector
    .read_register_1(read_register_1),                 // Read register 1 address
    .read_register_2(read_register_2),                 // Read register 2 address
    .write_register(write_register),                   // Write register address
    // .rom_read_enable(rom_read_enable),              // Uncomment this if needed  *for funture*
    .ram_read_enable(ram_read_enable),                 // RAM read enable signal
    .ram_write_enable(ram_write_enable),               // RAM write enable signal
    .jump(jump),                                       // Jump instruction signal
    .branchnotequal(branchnotequal),                   // Branch not equal instruction signal
    .brachlessthat(brachlessthat),                     // Branch less than instruction signal
    .branchgreaterthan(branchgreaterthan),             // Branch greater than instruction signal
    .branchlessthanorequal(branchlessthanorequal),     // Branch less than or equal instruction signal
    .branchgreaterthanorequal(branchgreaterthanorequal),      // Branch greater than or equal instruction signal
    .brancheq(brancheq),                               // Branch equal instruction signal
    .jr(jr),                                           // Jump register instruction signal
    .jal(jal)                                          // Jump and link instruction signal
);



// Connect RegisterFile's read_data_1 and read_data_2 to read_register_1 and read_register_2
Mux2x1 ALU_or_JAL_selector (
    .i0(mem_reg_result),        // Input 0: Data from ALU or RAM result
    .i1(pc_next + 32'd2),       // Input 1: PC + 2 for the jal instruction
    .sel(jal),                  // Selector: Determines the output
    .out(reg_or_mem_or_ra)      // Output: Selected data for the register or ra
);


RegisterFile Register_File (
    .clock(MAX10_CLK1_50),                       // Clock input
    .Reset(reset),                               // Reset signal
    .read_register_1(read_register_1),           // Read register 1 address
    .read_register_2(read_register_2),           // Read register 2 address
    .reg_write_address(write_register),       // Register address for write (if used)
    .reg_write_enable(write_en),                 // Register write enable signal
    .write_data(reg_or_mem_or_ra),               // Data to be written to the register file
    .read_data_1(alu_operand_A),                 // Data read from read_register_1
    .read_data_2(alu_operand_B)                  // Data read from read_register_2
);

// Instantiate the ALU module and connect the ALU control signal
ALU Alu (
    .clk(MAX10_CLK1_50),                     // Clock input
    .reset(reset),                           // Reset signal
    .operand_A(alu_operand_A),               // ALU input A
    .operand_B(operand_B),                   // ALU input B
    .alu_control(alu_control),               // ALU control signal
    .alu_result(alu_result),                 // ALU result
    .ram_address(ram_address),               // RAM address output
    .shmant(instruction[15:0]),              // Shift amount from instruction
    .zero(zero),                             // Zero signal
    .less(less)                              // Less signal
);


// ALU Operand Mux: Selects between sign-extended immediate value and ALU operand B.
Mux2x1 ALU_operand_selector (
    .i0(alu_operand_B),   // Input 0: Sign-extended immediate value
    .i1(sign_extended_imm),       // Input 1: ALU operand B (read_register_2)
    .sel(alu_src),            // Selector: Control signal to choose input (0 for sign-extended_imm, 1 for read_register_2)
    .out(operand_B)           // Output: Selected ALU operand B
);




SignExtendImmediate sign_extension (
    .clk(MAX10_CLK1_50),                      // Clock input
    .reset(reset),                            // Reset signal
    .instruction(instruction[15:0]),          // Instruction immediate field
    .sign_extended_imm(sign_extended_imm)     // Sign-extended immediate value
);

RAM32x1024 RAM (
    .address(ram_address),                    // RAM address
    .clock(MAX10_CLK1_50),                    // Clock input
    .data(alu_operand_B),                     // Data input to RAM
    .rden(ram_read_enable),                   // RAM read enable
    .wren(ram_write_enable),                  // RAM write enable
    .q(ram_result)                            // RAM data output
);

// Memory Register Select Mux: Selects between data from RAM and ALU result.
Mux2x1 ALU_or_RAM_selector (
    .i0(alu_result),          // Input 0: Data from RAM
    .i1(ram_result),          // Input 1: Data from ALU result
    .sel(mem_reg_selector),   // Selector: MUX selection control
    .out(mem_reg_result)      // Output: Selected data for memory register
);



endmodule
