module MEM_TO_WB_reg (
    input clk,
    input reset,
    input  wire Predicate_mem,
    input  wire [31:0] Res_mem,
    input  wire [4:0]  RW_mem,
    input  wire RegWrite_mem,
	input wire 	RegSel_mem,
    output reg Predicate_WB,
    output reg  [31:0] Data_WB,
    output reg  [4:0]  RW_WB,
    output reg RegWrite_WB,
	output reg RegSel_WB
);
    // FIXED: Removed "or posedge reset" from sensitivity list for consistency
    always @(posedge clk) begin
        if (reset) begin
            Predicate_WB <= 1'b0;
            Data_WB <= 32'b0;
            RW_WB <= 5'b0;
            RegWrite_WB <= 1'b0;
        end
        else begin
            Predicate_WB <= Predicate_mem;
            Data_WB <= Res_mem;
            RW_WB <= RW_mem;
            RegWrite_WB <= RegWrite_mem & Predicate_mem;
			RegSel_WB <= RegSel_mem;
        end
    end
endmodule