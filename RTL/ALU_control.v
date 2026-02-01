module ALU_control(
    input  [4:0] Opcode,
    input Stall,
    output reg [2:0] ALUOP
);
    wire ADD   = (Opcode == 5'd0);
    wire SUB   = (Opcode == 5'd1);
    wire OR    = (Opcode == 5'd2);
    wire NOR   = (Opcode == 5'd3);
    wire AND   = (Opcode == 5'd4);
    wire ADDI  = (Opcode == 5'd5);
    wire ORI   = (Opcode == 5'd6);
    wire NORI  = (Opcode == 5'd7);
    wire ANDI  = (Opcode == 5'd8);
    wire LW    = (Opcode == 5'd9);
    wire SW    = (Opcode == 5'd10);

    always @(*) begin
        if ((ADD | ADDI | LW | SW) && ~Stall)
            ALUOP = 3'b000;
        else if (SUB && ~Stall)
            ALUOP = 3'b001;
        else if ((OR | ORI) && ~Stall)
            ALUOP = 3'b010;
        else if ((NOR | NORI) && ~Stall)
            ALUOP = 3'b011;
        else if ((AND | ANDI) && ~Stall)
            ALUOP = 3'b100;
        else
            ALUOP = 3'b000;
    end
endmodule