module Pc_control(
    input  [4:0] OPCode,
    input        Predicate,
    output reg [1:0] PCSrc
);

    wire ADD   = (OPCode == 5'd0);
    wire SUB   = (OPCode == 5'd1);
    wire OR   = (OPCode == 5'd2);
    wire NOR  = (OPCode == 5'd3);
    wire AND  = (OPCode == 5'd4);
    wire ADDI  = (OPCode == 5'd5);
    wire ORI   = (OPCode == 5'd6);
    wire NORI  = (OPCode == 5'd7);
    wire ANDI  = (OPCode == 5'd8);
    wire LW    = (OPCode == 5'd9);
    wire SW    = (OPCode == 5'd10);
    wire J     = (OPCode == 5'd11);
    wire CALL  = (OPCode == 5'd12);
    wire JR    = (OPCode == 5'd13);

    always @(*) begin
        if ((J || CALL) && Predicate)
            PCSrc = 2'b01;    // jump / call
        else if (JR && Predicate)
            PCSrc = 2'b10;    // jump register
        else
	            PCSrc = 2'b00;    // normal PC + 4
    end

endmodule
