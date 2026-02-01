module REG_file(
    input clk,
    input reset,
    input RegW,
    input RegPRes,
    input [4:0] Rp,
    input [4:0] Rd,
    input [4:0] Rs,
    input [4:0] Rt,
    input [31:0] BusW,
    input [31:0] input_mux_pc,	
    input wire stall,
    output [31:0] BusA,
    output [31:0] BusB,
    output [31:0] BusP,
    output reg [31:0] PC
);
    localparam REG_NUM = 32;
    localparam DATA_WIDTH = 32;

    reg [DATA_WIDTH-1:0] Register [0:REG_NUM-1];

    // Combinational reads
    assign BusA = Register[Rs];
    assign BusB = Register[Rt];
    assign BusP = Register[Rp];

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            PC <= 32'd0;
            for (i = 0; i < REG_NUM; i = i + 1)
                Register[i] <= 32'b0;
            
            Register[0]  <= 32'd0;    // Hardwired zero
            Register[1]  <= 32'd100;  // Test value 1
            Register[2]  <= 32'd150;  // Test value 2
            Register[3]  <= 32'd10;
            Register[4]  <= 32'd20;
            Register[5]  <= 32'd255;
            Register[6]  <= 32'd240;
            Register[10] <= 32'd0;    // Zero predicate
            Register[20] <= 32'd1;    // Non-zero predicate
            Register[30] <= 32'd0;    // PC register
            Register[31] <= 32'd0;    // Return address register
        end
        else begin
            // R0 always zero
            Register[0] <= 32'b0;
            
            // PC updates only when not stalled
            if (!stall) begin
                PC <= input_mux_pc;
                Register[30] <= input_mux_pc;
            end
            
            // CRITICAL FIX: Register writes happen even during stall
            // because writeback stage is independent of fetch/decode stalls
            if (RegW && (Rp == 5'd0 || RegPRes) && (Rd != 5'd0) && (Rd != 5'd30))
                Register[Rd] <= BusW;
        end
    end
endmodule