module mux2x1_5bit(
	input [4:0]A,
	input [4:0]B,
	input Sel,
	output reg [4:0]out);
	
	always @(*)begin
		if(Sel == 0)
			out = A;
		else
			out = B;
	end	 
	
endmodule