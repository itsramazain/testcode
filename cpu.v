module CPU (
     MAX10_CLK1_50,        // Clock input from DE-10 Lite FPGA (50MHZ)
    reset,pc_next0,pc_increment0,BT0,BT_or_next_pc0,rom_address0,alu_src0,write_en0
  	 // Reset signal (active high)
	 ,jump0, alu_result0,write_register0,read_register_20,read_register_10,read_data_20,
	 read_data_10,branch0,selected_register0,write_data0,mem_reg_selector0 ,branch_or_not0,alu_control0,
	 less0,reg_or_mem_or_ra0,jr0,zero0,
	 operand_B0,alu_operand_A0,zero_flag0,branchnotequal0,brachlessthat0,branchgreaterthan0,
	 branchlessthanorequal0,branchgreaterthanorequal0,
	 brancheq0,
	 ram_result0,mem_reg_selector0,jump_or_next_pc_or_branch_or_jr0,ram_read_enable0,
	 ram_write_enable0,
	 jump_or_next_pc_or_branch_or_jr0,instruction0,jump_or_next_pc_or_branch0,jal0,ro,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31,r32,alu_operand_B0
);

input reset;
wire [7:0]ram_address;
input MAX10_CLK1_50;
// Program Counter (PC) and Instruction Signals
wire [7:0] pc_next;            // PC Output (ROM address)
output [7:0] pc_next0;
assign pc_next0=pc_next;
wire [31:0]alu_operand_B;
output [31:0]alu_operand_B0;
assign alu_operand_B0=alu_operand_B;

wire pc_increment;             // Fetch instruction control signal
output pc_increment0;
assign pc_increment0=pc_increment;

wire [31:0] instruction;       // Instruction fetched from ROM (sent to Control unit to decode)
output [31:0] instruction0;
assign instruction0=instruction;

// Branch Target Calculation Signals
wire [7:0] BT;                // Branch Target
output [7:0] BT0;
assign BT0=BT;
wire [7:0] BT_or_next_pc;     // MUX Output (Branch Target or Next PC)
output[7:0] BT_or_next_pc0;
assign BT_or_next_pc0=BT_or_next_pc;

wire [4:0] rom_address;        // Address to access ROM
output [4:0]rom_address0;
assign rom_address0=rom_address;

wire alu_src;                  // ALU Source Selector
output alu_src0;
assign alu_src0=alu_src;

         
wire write_en;                 // Register Write Enable
output write_en0;
assign write_en0=write_en;

wire jump;                     // Jump Instruction Signal
output jump0;
assign jump0=jump;

wire [7:0] jump_or_next_pc_or_branch; // MUX Output (Jump or Next PC or Branch)
output [7:0]jump_or_next_pc_or_branch0;
assign jump_or_next_pc_or_branch0=jump_or_next_pc_or_branch;
 
wire branch;                   // Branch Instruction Signal

output branch0;
assign branch0=branch;

// Register File Signals
wire [4:0] selected_register;   // Selected Register for Write
output [4:0]selected_register0;

assign selected_register0=selected_register;

wire [31:0] write_data;         // Data to Write in Register File
output [31:0] write_data0 ;


assign write_data0=write_data;
wire [31:0] read_data_1;        // Data from Register File (Read Data 1)
output [31:0] read_data_10  ;
assign read_data_10=read_data_1;

wire [31:0] read_data_2;        // Data from Register File (Read Data 2)
output [31:0] read_data_20 ;
assign read_data_20=read_data_2;

// ALU and Control Signals
wire [4:0] read_register_1;    // Read Register 1 (ALU Operand)

output [4:0]read_register_10;
assign read_register_10=read_register_1;

wire [4:0] read_register_2;    // Read Register 2 (ALU Operand)
output [4:0]read_register_20;
assign read_register_20=read_register_2;

wire [4:0] write_register;     // Write Register
output [4:0]write_register0;
assign write_register0=write_register;

wire [31:0] alu_result;       // ALU Result
output [31:0] alu_result0;
assign alu_result0=alu_result;

wire zero_flag;                // Zero Flag Signal
output zero_flag0;
assign zero_flag0=zero_flag;

wire [31:0] alu_operand_A;     // ALU Operand A
output [31:0] alu_operand_A0;
assign alu_operand_A0=alu_operand_A;

wire [31:0] operand_B;         // Operand B for ALU
output [31:0] operand_B0;
assign operand_B0=operand_B;

wire zero;                     // Zero Signal
output zero0;
assign zer0=zero;

wire jr;                       // Jump to Register Signal

output jr0;
assign jr0=jr;

// Additional Control Signals
wire [31:0] reg_or_mem_or_ra;  // MUX Output (Register File or Memory or RA)
output [31:0] reg_or_mem_or_ra0;
assign  reg_or_mem_or_ra0= reg_or_mem_or_ra;

wire less;                     // Less Signal
output less0;
assign less0=less;

wire [3:0] alu_control;        // ALU Control Signals
output [3:0] alu_control0;
assign  alu_control0=alu_control;

wire [31:0] ram_result;        // Data from RAM
output [31:0]ram_result0;
assign  ram_result0= ram_result;

wire branch_or_not;            // Branch or Not Signal
output branch_or_not0;
assign branch_or_not0=branch_or_not;

wire mem_reg_selector;         // Memory or Register Selector

output mem_reg_selector0;
assign mem_reg_selector0=mem_reg_selector;

wire [7:0] jump_or_next_pc_or_branch_or_jr; // MUX Output (Jump or Next PC or Branch or JR)
output [7:0] jump_or_next_pc_or_branch_or_jr0;
assign jump_or_next_pc_or_branch_or_jr0=jump_or_next_pc_or_branch_or_jr;

wire [31:0] mem_reg_result;    // Data Output from Memory or Register File
wire ram_read_enable;          // RAM Read Enable Signal
output ram_read_enable0;
assign ram_read_enable0=ram_read_enable;
wire jal;                      // Jump and Link Instruction Signal
output jal0;
assign jal0=jal;

wire ram_write_enable;         // RAM Write Enable Signal
output ram_write_enable0;
assign ram_write_enable0=ram_write_enable;

// Branch Condition Signals
wire branchnotequal;           // Branch Not Equal Signal

output branchnotequal0;
assign branchnotequal0=branchnotequal;
output [31:0]ro,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31,r32;
wire brachlessthat;            // Branch Less Than Signal
output brachlessthat0;
assign brachlessthat0=brachlessthat;

wire branchgreaterthan;        // Branch Greater Than Signal
output branchgreaterthan0;
assign branchgreaterthan0=branchgreaterthan;

wire branchlessthanorequal;    // Branch Less Than or Equal Signal
output branchlessthanorequal0;
assign branchlessthanorequal0=branchlessthanorequal;

wire branchgreaterthanorequal; // Branch Greater Than or Equal Signal
output branchgreaterthanorequal0;
assign branchgreaterthanorequal0=branchgreaterthanorequal;

wire brancheq;                 // Branch Equal Signal

output brancheq0;
assign brancheq0=brancheq;




// Instantiate the Program Counter module
ProgramCounter program_counter (
    .clk(MAX10_CLK1_50),               // input for clock
    .reset(reset),                     // input for reset
	 .enable_increment(pc_increment),   // input for enable increment
    .pc(pc_next)                       // output - 8 bits address for ROM
);



// Instantiate the branch target calculator module
Branch_Target_Calculator tar(
		.immediate(instruction[15:0]),               //immedate filed from the instruction
		.program_counter(pc_next),                   //nest instruction
		.BT(BT)
		);
		
// Choose between branch or next instruction logic				
BranchLogic branch_or_no(
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

										
// Instantiate a 2x1 multiplexer to select the branch target or next program counter
Mux2x1 BT_OR_nextPC (
    .i0(pc_next),              // Input 0: Next program counter value
    .i1(BT),                   // Input 1: Branch target value
    .sel(branch_or_not),       // Select signal to choose between inputs
    .out(BT_or_next_pc)        // Output signal, either the branch target or next program counter
);

 defparam BT_OR_nextPC.n=8;
	
// Instantiate a 2x1 multiplexer to select the jump address if it's a jump instruction
Mux2x1 chooseJUMP (
    .i0(BT_or_next_pc),                       // Input 0: Branch target or next program counter
    .i1(instruction[7:0]),    // Input 1: Concatenation of next program counter and instruction bits[25:0]
    .sel(jump),                               // Select signal to choose between inputs
    .out(jump_or_next_pc_or_branch)           // Output signal, either the jump address or next program counter or branch target
);

 defparam chooseJUMP.n=8;


// Instantiate a 2x1 multiplexer to select the register address if it's a JR (Jump Register) instruction
Mux2x1 jumptoregister (
    .i0(jump_or_next_pc_or_branch),         // Input 0: Output of the previous multiplexer (jump address, next PC, or branch target)
    .i1(alu_operand_A[7:0]),                     // Input 1: Operand A value from ALU
    .sel(jr),                               // Select signal to choose between inputs
    .out(jump_or_next_pc_or_branch_or_jr)   // Output signal, either the selected address or ALU operand A
);

 defparam jumptoregister.n=8;

// Instantiate the ROM module     -- containts 256 words each word is 32 bits
ROM32x256 rom(
    .clock(MAX10_CLK1_50),                             // input for clock
	 .address(jump_or_next_pc_or_branch_or_jr),         // input - 8 bits address from PC
    .q(instruction) 	 // ROM output - 32 bits instruction
	 ,.rden ( 1'b1)
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
Mux2x1 reg_or_ra_selector (
    .i0(mem_reg_result),        // Input 0: Data from ALU result
    .i1({24'b0,pc_next} + 8'd32),       // Input 1: PC + 2 for the jal instruction
    .sel(jal),                  // Selector: Determines the output
    .out(reg_or_mem_or_ra)      // Output: Selected data for the register or ra
);



RegisterFile register_file (
    .clock(MAX10_CLK1_50),                       // Clock input
    .Reset(reset),                               // Reset signal
    .read_register_1(read_register_1),           // Read register 1 address
    .read_register_2(read_register_2),           // Read register 2 address
    .reg_write_address(write_register),       // Register address for write (if used)
    .reg_write_enable(write_en),                 // Register write enable signal
    .write_data(reg_or_mem_or_ra),               // Data to be written to the register file
    .read_data_1(alu_operand_A),                 // Data read from read_register_1
    .read_data_2(alu_operand_B),	 // Data read from read_register_2
	 .ro(ro),.r1(r1),.r2(r2),.r3(r3),.r4(r4),.r5(r5),.r6(r6),.r7(r7),.r8(r8),.r10(r10),.r9(r9),.r11(r11),.r12(r12),.r13(r13),.r14(r14),.r15(r15),.r16(r16),.r17(r17),.r18(r18),.r19(r19),.r20(r20),.r21(r21),.r22(r22),.r23(r23),.r24(r24),.r25(r25),.r26(r26),.r27(r27),.r28(r28),.r29(r29),.r30(r30),.r31(r31),.r32(r32)
);
wire overflow;


// Instantiate the ALU module and connect the ALU control signal
ALU my_alu (
    .clk(MAX10_CLK1_50),                     // Clock input
    .reset(reset),                           // Reset signal
    .operand_A(alu_operand_A),               // ALU input A
    .operand_B(operand_B),                   // ALU input B
    .alu_control(alu_control),               // ALU control signal
    .alu_result(alu_result),                 // ALU result
    .zero_flag(zero_flag),                   // Zero flag output
    .ram_address(ram_address),               // RAM address output
    .shmant(instruction[10:6]),              // Shift amount from instruction
    .zero(zero),                             // Zero signal
    .less(less) ,	 // Less signal
	 .overflow(overflow)
	 
	 
	 
);
wire [31:0]sign_extended_imm;

// ALU Operand Mux: Selects between sign-extended immediate value and ALU operand B.
Mux2x1 alu_operand (
    .i0(sign_extended_imm),   // Input 0: Sign-extended immediate value
    .i1(alu_operand_B),       // Input 1: ALU operand B (read_register_2)
    .sel(alu_src),            // Selector: Control signal to choose input (0 for sign-extended_imm, 1 for read_register_2)
    .out(operand_B)           // Output: Selected ALU operand B
);




SignExtendImmediate sign_extend (
    .clk(MAX10_CLK1_50),                      // Clock input
    .reset(reset),                            // Reset signal
    .instruction(instruction[15:0]),          // Instruction immediate field
    .sign_extended_imm(sign_extended_imm)     // Sign-extended immediate value
);

RAM32x1024 ram (
    .address(ram_address),                    // RAM address
    .clock(MAX10_CLK1_50),                    // Clock input
    .data(alu_operand_B),                     // Data input to RAM
    .rden(ram_read_enable),                   // RAM read enable
    .wren(ram_write_enable),                  // RAM write enable
    .q(ram_result)                            // RAM data output
);

// Memory Register Select Mux: Selects between data from RAM and ALU result.
Mux2x1 mem_reg_select (
    .i0(alu_result),          // Input 0: Data from RAM
    .i1(ram_result),          // Input 1: Data from ALU result
    .sel(mem_reg_selector),   // Selector: MUX selection control
    .out(mem_reg_result)      // Output: Selected data for memory register
);



endmodule