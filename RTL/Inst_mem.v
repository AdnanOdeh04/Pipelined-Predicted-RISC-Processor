module Inst_mem(
    input [31:0] PC,
    output [31:0] data_out
);
    reg [31:0] ram [0:255];
    assign data_out = ram[PC[7:0]];
    
    initial begin
        integer i;
        for (i = 0; i < 256; i = i + 1)
            ram[i] = 32'b0;
    end
endmodule