module F_TO_D_reg(
    input clk,
    input reset,
    input wire [31:0] Inst_F,
    input wire [31:0] Pc_F,
    input wire [31:0] PcPlus_F,
    input wire stall,
    input wire flush,
    output reg [31:0] Inst_D,
    output reg [31:0] Pc_D,
    output reg [31:0] PcPlus_D
);
    // FIXED: Removed "or posedge reset" from sensitivity list for consistency
    always @(posedge clk) begin
        if (reset) begin
            Inst_D <= 32'b0;
            Pc_D <= 32'b0;
            PcPlus_D <= 32'b0;
        end
        else if (flush) begin
            Inst_D <= 32'b0;
            Pc_D <= 32'b0;
            PcPlus_D <= 32'b0;
        end
        else if (stall) begin
            // Hold current values
        end
        else begin
            Inst_D <= Inst_F;
            Pc_D <= Pc_F;
            PcPlus_D <= PcPlus_F;
        end
    end
endmodule