//read after write memory 4294967296 locations each location has an adress
///for now its eather read form memory or wite to memeory will work on it in the hazzared part
//i will have it as 1 k memrylocation cuxz it cant synthisise it !!
//big endian high-->low  //low-->high
///its always reading 
module RAM32x1024(adress,data_in,data_out ,Readmem,Writemem);
		input [31:0]data_in;
		input [7:0]adress;
		integer i;
		input Readmem,Writemem;
		output  reg[31:0]data_out ;
		reg [7:0]ram [0:1023];
		
		
		
		initial
		begin
			for(i=0;i<1024;i=i+1)
				ram[i]=8'b0;
		end 
		
		
		always@(*)
		
			if (Readmem)
			
				data_out={ram[adress],ram[adress+1],ram[adress+2],ram[adress+3]};
			
			
			else if (Writemem)
			begin
			
				ram[adress]=data_in[31:23];
				
				ram[adress+1]=data_in[23:16];
				
				ram[adress+2]=data_in[15:8];
				
				ram[adress+3]=data_in[7:0];
			end 
		
			
		
		


endmodule 
