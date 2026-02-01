module FWmux4x1( 
	input [31:0]A,
	input [31:0]B,
	input [31:0]C,
	input [31:0]D,
	input [1:0]Sel,
	output reg [31:0]out);
	
	always @(*)begin
		if(Sel == 2'b00)
			out = A;
		else if(Sel == 2'b01)
			out = B;
		else if(Sel == 2'b10)
			out = C;
		else if (Sel == 2'b11)
			out = D;
		else
			out = 32'b0;
	end
	
endmodule