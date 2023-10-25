`timescale 1ns / 1ps
module test();
	reg reset;
	reg CLOCK_50;
	
	wire [7:0]jump_or_next_pc_or_branch0,jump_or_next_pc_or_branch_or_jr0,pc_next0,BT0,BT_or_next_pc0;
	wire pc_increment0,zero_flag0,alu_src0,write_en0,jump0,branch_or_not0,less0,jr0,zero0,branchnotequal0,brachlessthat0,branchgreaterthan0,
	brancheq0,mem_reg_selector0,ram_read_enable0,
	 ram_write_enable0,jal0;
	wire [4:0]write_register0,read_register_10,read_register_20,selected_register0,rom_address0;
	wire [3:0]alu_control0;
	wire [31:0]instruction0,alu_result0,read_data_20,read_data_10,write_data0,operand_B0,alu_operand_A0,ram_result0;
	wire [31:0]alu_operand_B0,reg_or_mem_or_ra0,ro,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31,r32;
CPU DUT (
     CLOCK_50,        // Clock input from DE-10 Lite FPGA (50MHZ)
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
	parameter CLOCK_PERIOD=2;
	
	
	
	initial begin
			CLOCK_50 <= 1'b1;
		end // initial
		always @ (*)
		begin : Clock_Generator
			#((CLOCK_PERIOD) / 2) 
			CLOCK_50 <= ~CLOCK_50;
		end
	
	
	initial
	begin
	
	reset<=1;
	 #0.05;reset<=0;
	end
	
	

	
	
	
	initial
	begin

	

	

		
	
	
	
	
	
	
	
	
	
	end
	
	
	
	initial
	begin
	#1000 $stop;
	end
	initial
  begin
    $dumpfile("dump.vcd");
    $dumpvars(1);
  end
	
	

endmodule