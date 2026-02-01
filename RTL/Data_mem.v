module Data_mem(
    input reset,
    input clk,
    input [31:0] data_in,
    input wr,
    input rd,
    input [31:0] addr,
    output [31:0] data_out
);
    reg [31:0] ram [0:31];
    
    always @(posedge clk) begin
        if (reset) begin 
            integer i;
            for (i = 0; i < 32; i = i + 1)
                ram[i] <= 32'h0;
            
            // Initialize for test
            ram[0]  <= 32'd100;  // For load-use test
            ram[8]  <= 32'h000000AA;
            ram[12] <= 32'h000000BB;
        end
        else if (wr) begin
            ram[addr[4:0]] <= data_in;	   
        end
    end

    assign data_out = ram[addr[4:0]];
endmodule