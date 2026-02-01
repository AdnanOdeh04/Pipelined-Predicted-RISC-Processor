module D_TO_E_reg(
    input clk,
    input reset,
    input wire Predicate_D,
    input wire [31:0] BUSA_D,
    input wire [31:0] BUSB_D,
    input wire [31:0] Imm_D,
    input wire [4:0] RW_D,
    input wire [2:0] ALUOP_D,      
    input wire ALUSrc_D,
    input wire MEMRd_D,
    input wire MEMWr_D,
    input wire [1:0] WB_data_D,    
    input wire RegWrite_D,
    input wire [31:0] PCPLUS_D,	
    input wire stall,
    input wire flush,  // NOTE: flush should NOT affect D->E for control flow instructions
	input wire RegSel_D,
    output reg Predicate_E,
    output reg [31:0] BUSA_E,
    output reg [31:0] BUSB_E,
    output reg [31:0] Imm_E,
    output reg [4:0] RW_E,
    output reg [2:0] ALUOP_E,      
    output reg ALUSrc_E,
    output reg RegWrite_E,
    output reg MEMRd_E,
    output reg MEMWr_E,
    output reg [1:0] WB_data_E,    
    output reg [31:0] PCPLUS_E,
	output reg RegSel_E
);
    
    always @(posedge clk) begin
        if (reset) begin 
            RegWrite_E <= 1'b0;
            MEMRd_E <= 1'b0;
            MEMWr_E <= 1'b0;
            Predicate_E <= 1'b0;
            BUSA_E <= 32'b0;
            BUSB_E <= 32'b0;
            Imm_E <= 32'b0;
            RW_E <= 5'b0;
            ALUOP_E <= 3'b0;
            ALUSrc_E <= 1'b0;
            WB_data_E <= 2'b0;
            PCPLUS_E <= 32'b0;
        end
        // REMOVED: flush condition from D->E
        // The D->E register should NOT be flushed when control flow changes
        // Only the F->D register gets flushed
        else if (stall) begin
            // Insert complete bubble (NOP)
            RegWrite_E <= 1'b0;
            MEMRd_E <= 1'b0;
            MEMWr_E <= 1'b0;
            Predicate_E <= 1'b0;
            BUSA_E <= 32'b0;
            BUSB_E <= 32'b0;
            Imm_E <= 32'b0;
            RW_E <= 5'b0;
            ALUOP_E <= 3'b0;
            ALUSrc_E <= 1'b0;
            WB_data_E <= 2'b0;
            PCPLUS_E <= 32'b0;
        end
        else begin
            Predicate_E <= Predicate_D;
            BUSA_E <= BUSA_D;
            BUSB_E <= BUSB_D;
            Imm_E <= Imm_D;
            RW_E <= RW_D;
            ALUOP_E <= ALUOP_D;
            ALUSrc_E <= ALUSrc_D;
            MEMRd_E <= MEMRd_D;
            MEMWr_E <= MEMWr_D;
            RegWrite_E <= RegWrite_D;
            WB_data_E <= WB_data_D;
            PCPLUS_E <= PCPLUS_D;
			RegSel_E <= RegSel_D;
        end
    end
endmodule