module Hazard_Detect_And_Stall(
    input wire [4:0] RW_EX,
    input wire [4:0] RW_mem,
    input wire [4:0] RW_WB,
    input wire mem_RD_EX,
    input wire Regwrite_EX,
    input wire Regwrite_mem,
    input wire Regwrite_WB,
    input wire [4:0] Rs,
    input wire [4:0] Rt,
    input wire [4:0] Rp,
    input wire RegSel_WB,
    output reg [1:0] FWA,
    output reg [1:0] FWB,
    output reg [1:0] FWP,       
    output reg Stall
);
    
    // Forwarding for Rs (BusA)
    always @(*) begin
        if ((Rs != 5'd0) && (Rs != 5'd30) && (RW_EX != 5'd0) && (RW_EX != 5'd30) && (Rs == RW_EX) && Regwrite_EX)
            FWA = 2'b01;  // Forward from EX stage
        else if ((Rs != 5'd0) && (Rs != 5'd30) && (RW_mem != 5'd0) && (RW_mem != 5'd30) && (Rs == RW_mem) && Regwrite_mem)
            FWA = 2'b10;  // Forward from MEM stage
        else if ((Rs != 5'd0) && (Rs != 5'd30) && (RW_WB != 5'd0) && (RW_WB != 5'd30) && (Rs == RW_WB) && Regwrite_WB)
            FWA = 2'b11;  // Forward from WB stage
        // Special case: CALL writing to R31 in WB, and JR reading R31 in decode
        else if (RegSel_WB && (Rs == 5'd31))
            FWA = 2'b11;  // Forward from WB stage for R31
        else 
            FWA = 2'b00;  // No forwarding
    end
    
    // Forwarding for Rt (BusB)
    always @(*) begin
        if ((Rt != 5'd0) && (Rt != 5'd30) && (RW_EX != 5'd0) && (RW_EX != 5'd30) && (Rt == RW_EX) && Regwrite_EX)
            FWB = 2'b01;
        else if ((Rt != 5'd0) && (Rt != 5'd30) && (RW_mem != 5'd0) && (RW_mem != 5'd30) && (Rt == RW_mem) && Regwrite_mem)
            FWB = 2'b10;
        else if ((Rt != 5'd0) && (Rt != 5'd30) && (RW_WB != 5'd0) && (RW_WB != 5'd30) && (Rt == RW_WB) && Regwrite_WB)
            FWB = 2'b11;
        // Special case for CALL/R31
        else if (RegSel_WB && (Rt == 5'd31))
            FWB = 2'b11;
        else 
            FWB = 2'b00;
    end
    
    // Forwarding for Rp (Predicate)
    always @(*) begin
        if ((Rp != 5'd0) && (Rp != 5'd30) && (RW_EX != 5'd0) && (RW_EX != 5'd30) && (Rp == RW_EX) && Regwrite_EX)
            FWP = 2'b01;
        else if ((Rp != 5'd0) && (Rp != 5'd30) && (RW_mem != 5'd0) && (RW_mem != 5'd30) && (Rp == RW_mem) && Regwrite_mem)
            FWP = 2'b10;
        else if ((Rp != 5'd0) && (Rp != 5'd30) && (RW_WB != 5'd0) && (RW_WB != 5'd30) && (Rp == RW_WB) && Regwrite_WB)
            FWP = 2'b11;
        // Special case for CALL/R31
        else if (RegSel_WB && (Rp == 5'd31))
            FWP = 2'b11;
        else 
            FWP = 2'b00;
    end
    
    // Stall detection for load-use hazard
    always @(*) begin
        if (mem_RD_EX && (RW_EX != 5'd0) && (RW_EX != 5'd30) &&
            ((Rs == RW_EX && Rs != 5'd0 && Rs != 5'd30) || 
             (Rt == RW_EX && Rt != 5'd0 && Rt != 5'd30) ||
             (Rp == RW_EX && Rp != 5'd0 && Rp != 5'd30)))
            Stall = 1'b1;
        else
            Stall = 1'b0;
    end
endmodule