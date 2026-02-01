module ALU(
    input  [2:0]  ALUOP,
    input  [31:0] BusA,
    input  [31:0] BusB,
    output        Zero,
    output reg [31:0] RES
);

always @(*) begin
    case (ALUOP)
        3'b000: RES = BusA + BusB;        
        3'b001: RES = BusA - BusB;        
        3'b010: RES = BusA | BusB;        
        3'b011: RES = ~(BusA | BusB);        
        3'b100: RES = BusA & BusB;     
        default: RES = 32'b0;
    endcase
end

assign Zero = (RES == 32'b0);

endmodule 



