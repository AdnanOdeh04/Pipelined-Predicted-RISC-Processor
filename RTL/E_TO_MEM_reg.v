module E_TO_MEM_reg (
    input clk,
	input reset,
    input  wire Predicate_E,
    input  wire [31:0] Res_E,
    input  wire [31:0] Data_E,
    input  wire [4:0]  RW_E,
    input  wire MEMRd_E,
    input  wire MEMWr_E,
    input  wire RegWrite_E,
    input  wire [1:0] WB_data_E,
	input wire [31:0] PCPLUS_E,
	input wire RegSel_E,
    output reg Predicate_mem,
    output reg [31:0] Res_mem,
    output reg [31:0] Data_mem,
    output reg [4:0]  RW_mem,
    output reg MEMRd_mem,
    output reg MEMWr_mem,
    output reg RegWrite_mem,
    output reg [1:0] WB_data_mem,
	output reg [31:0] PCPLUS_mem,
	output reg RegSel_mem
);

	always @(posedge clk) begin
		if (reset) begin
	        Predicate_mem <= 1'b0;
	        Res_mem <= 32'b0;
	        Data_mem <= 32'b0;
	        RW_mem <= 5'b0;
	        MEMRd_mem <= 1'b0;
	        MEMWr_mem <= 1'b0;
	        RegWrite_mem <= 1'b0;
	        WB_data_mem <= 2'b0;
	        PCPLUS_mem <= 32'b0;
    	end
		else begin
	        Predicate_mem <= Predicate_E;
	        Res_mem       <= Res_E;
	        Data_mem      <= Data_E;
	        RW_mem        <= RW_E;
	        MEMRd_mem     <= MEMRd_E;
	        MEMWr_mem     <= MEMWr_E;
	        RegWrite_mem  <= RegWrite_E;
	        WB_data_mem   <= WB_data_E;
			PCPLUS_mem <= PCPLUS_E;
			RegSel_mem <= RegSel_E;
		end
    end

endmodule
