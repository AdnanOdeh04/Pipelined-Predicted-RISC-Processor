module mux2x1(
	input [31:0]A,
	input [31:0]B,
	input Sel,
	output reg [31:0]out);
	
	always @(*)begin
		if(Sel == 0)
			out = A;
		else if(Sel == 1)
			out = B;
	end	 
	
endmodule