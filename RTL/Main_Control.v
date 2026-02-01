module Main_Control(
    input [4:0] opcode,
    input Stall,
    output RegR2,
    output ExtOp,
    output RegWrite,
    output ALUSrc,
    output MemRd,
    output MemWr,
    output RegSel,
    output reg [1:0] WB
);			
    wire ADD   = (opcode == 5'd0);
    wire SUB   = (opcode == 5'd1);
    wire OR    = (opcode == 5'd2);
    wire NOR   = (opcode == 5'd3);
    wire AND   = (opcode == 5'd4);
    wire ADDI  = (opcode == 5'd5);
    wire ORI   = (opcode == 5'd6);
    wire NORI  = (opcode == 5'd7);
    wire ANDI  = (opcode == 5'd8);
    wire LW    = (opcode == 5'd9);
    wire SW    = (opcode == 5'd10);
    wire J     = (opcode == 5'd11);
    wire CALL  = (opcode == 5'd12);
    wire JR    = (opcode == 5'd13);
    
    assign RegR2 = SW;
    assign ExtOp = (ADDI | LW | SW);
    assign RegWrite = (ADD | SUB | OR | NOR | AND | ADDI | ORI | NORI | ANDI | LW | CALL) & ~Stall;
    assign ALUSrc = (ADDI | ORI | NORI | ANDI | LW | SW) & ~Stall;
    assign MemRd = LW & ~Stall;	
    assign MemWr = SW & ~Stall;
    assign RegSel = CALL ;
    
    always @(*) begin
        if (CALL == 1 && ~Stall)
            WB = 2'b10;
        else if (LW == 1 && ~Stall)
            WB = 2'b01;
        else 
            WB = 2'b00; 
    end
endmodule