module CPU(
    input CLK,
    input reset,
	input  resetMemory,
    output [31:0] Instruction_top
);
    wire [31:0] PC_in;
    wire [31:0] PC_out;
    wire [1:0]  PC_control;
    wire [31:0] PC_plus1;
    wire [31:0] PC_JumpTarget;
    wire [31:0] PC_JR;

    wire [31:0] Instruction;
    wire [4:0]  opcode;
    wire [4:0]  Rp;
    wire [4:0]  Rd;
    wire [4:0]  Rs;
    wire [4:0]  Rt;
    wire [11:0] imm;
    wire [21:0] offset;

    wire RegR2, ExtOp, RegWrite, ALUSrc, MemRd, MemWr, RegSel;
    wire [1:0] WB;

    wire [31:0] BusA, BusB, BusP, BusW;
    wire [4:0] OutputRegR2Mux;
    wire [4:0] OutputRegSelMux;

    wire [2:0] ALUOP;
    wire Zero;
    wire [31:0] RES;
    wire [31:0] ALUSrcMux;

    wire [31:0] Data_out;
    wire Predicate;
    reg [31:0] valueAfterExtended;

    assign Instruction_top = Instruction;
    assign PC_plus1 = PC_out + 32'd1;  
	
	
	wire flush;
	//=========Fetch To Decode Pipline=============	 
	wire [31:0] intruction_FD; 
	wire [31:0] PC_FD; 
	wire [31:0] PcPlus_D;
	//=====Decode to Execute========
	wire  Predicate_E;
	wire [31:0] BUSA_E;
	wire [31:0] BUSB_E;
	wire [31:0] Imm_E;
	wire [4:0] RW_E;
	wire [2:0] ALUOP_E;
	wire ALUSrc_E;
	wire RegWrite_E;
	wire MEMRd_E;
	wire MEMWr_E;
	wire [1:0] WB_data_E;
	wire [31:0] PcPlus_E;
	//=========== Execute to Memory ===========	 
	wire Predicate_mem;
	wire [31:0] Res_mem;
	wire [31:0] Data_mem;
	wire [4:0] RW_mem;
	wire MEMRd_mem;
	wire MEMWr_mem;
	wire RegWrite_mem;
	wire [1:0] WB_data_mem;
	wire [31:0] PCPLUS_mem;
	// ========== Memory To WriteBack ===========
	wire Predicate_WB;
	wire [31:0] Data_WB;
	wire [4:0] RW_WB;
	wire RegWrite_WB;  
	// ========== Hazard & Forwarding Unit =========
	
	//=========== Forwarding ==========
	wire [31:0] BusA_outputmux_Forwarding;
	wire [31:0] BusB_outputmux_Forwarding; 
	wire [31:0] BusP_outputmux_Forwarding;
	wire [1:0] FWA;
	wire [1:0] FWB;
	wire [1:0] FWP;
	wire Stall;
	Hazard_Detect_And_Stall HDAS(
		.RW_EX(RW_E),
		.RW_mem(RW_mem),
		.RW_WB(RW_WB),
		.mem_RD_EX(MEMRd_E),
		.Regwrite_EX(RegWrite_E),
		.Regwrite_mem(RegWrite_mem),
		.Regwrite_WB(RegWrite_WB),
		.Rs(Rs),
		.Rt(Rt),  
		.Rp(Rp),
		.RegSel_WB(RegSel_WB),
		.FWA(FWA),
		.FWB(FWB),
		.FWP(FWP),
		.Stall(Stall)
	);
	
	FWmux4x1 FWAMux (
		.A(BusA),
		.B(RES),
		.C(BusW),
		.D(Data_WB),
		.Sel(FWA),
		.out(BusA_outputmux_Forwarding)
	); 
	
	FWmux4x1 FWBMux (
		.A(BusB),
		.B(RES),
		.C(BusW),
		.D(Data_WB),
		.Sel(FWB),
		.out(BusB_outputmux_Forwarding)
	);
	

	FWmux4x1 FWPMux (
	    .A(BusP),
	    .B(RES),
	    .C(BusW),
	    .D(Data_WB),
	    .Sel(FWP),
	    .out(BusP_outputmux_Forwarding)
	);

    // ========== FETCH ==========
    Inst_mem IM(
        .PC(PC_out),
        .data_out(Instruction)
    );	 
	//=========Fetch To Decode Pipline=============	 
	F_TO_D_reg FDREG(
    .clk(CLK),
    .reset(reset),  // ADD THIS LINE
    .Inst_F(Instruction), 
    .PcPlus_F(PC_plus1),
    .Pc_F(PC_out),
    .stall(Stall),
    .flush(flush),
    .Inst_D(intruction_FD),
    .Pc_D(PC_FD),
    .PcPlus_D(PcPlus_D)
);

    // ========== DECODE ==========
    assign opcode = intruction_FD[31:27];
    assign Rp = intruction_FD[26:22];
    assign Rd = intruction_FD[21:17];
    assign Rs = intruction_FD[16:12];
    assign Rt = intruction_FD[11:7];
    assign imm = intruction_FD[11:0];
    assign offset = intruction_FD[21:0];

    wire [31:0] sign_extended_offset = {{10{offset[21]}}, offset};
    assign PC_JumpTarget = PC_FD + sign_extended_offset;
    assign PC_JR = BusA_outputmux_Forwarding;

    assign Predicate = (Rp == 5'd0) ? 1'b1 : |BusP_outputmux_Forwarding;

    Main_Control MC(
        .opcode(opcode),
        .Stall(Stall),
        .RegR2(RegR2),
        .ExtOp(ExtOp),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .RegSel(RegSel),
        .WB(WB)
    );

    Pc_control PControl(
        .OPCode(opcode),
        .Predicate(Predicate),
        .PCSrc(PC_control)
    );
	assign flush = (PC_control != 2'b00);

    mux4x1 pc_mux(
        .A(PC_plus1),
        .B(PC_JumpTarget),
        .C(PC_JR),
        .Sel(PC_control),
        .out(PC_in)
    );

    mux2x1_5bit m1(
        .A(Rt),
        .B(Rd),
        .Sel(RegR2),
        .out(OutputRegR2Mux)
    );

    mux2x1_5bit m2(
        .A(Rd),
        .B(5'd31),
        .Sel(RegSel),
        .out(OutputRegSelMux)
    );

    REG_file RF(
        .clk(CLK),
        .reset(reset),
        .RegW(RegWrite_WB),
        .RegPRes(Predicate_WB),
        .Rp(Rp),
        .Rd(RW_WB),
        .Rs(Rs),
		.stall(Stall),
        .Rt(OutputRegR2Mux),
        .BusW(Data_WB),
        .input_mux_pc(PC_in),
        .BusA(BusA),
        .BusB(BusB),
        .BusP(BusP),
        .PC(PC_out)
    );
	//=====Decode to Execute========
	wire RegSel_E;
	D_TO_E_reg DEREG(
		.clk(CLK),
		.reset(reset),
        .Predicate_D(Predicate),
        .BUSA_D(BusA_outputmux_Forwarding),
        .BUSB_D(BusB_outputmux_Forwarding),
        .Imm_D(valueAfterExtended),
        .RW_D(OutputRegSelMux),
        .ALUOP_D(ALUOP_D),  // ? Use ALUOP_D from control
        .ALUSrc_D(ALUSrc),
        .MEMRd_D(MemRd),
        .MEMWr_D(MemWr),
        .WB_data_D(WB),
        .RegWrite_D(RegWrite),
        .PCPLUS_D(PcPlus_D),
        .stall(Stall),
        .flush(flush),
		.RegSel_D(RegSel),
        .Predicate_E(Predicate_E), 
        .BUSA_E(BUSA_E),
        .BUSB_E(BUSB_E),  
        .Imm_E(Imm_E),
        .RW_E(RW_E),
        .ALUOP_E(ALUOP_E),  // ? Output to Execute stage
        .ALUSrc_E(ALUSrc_E), 
        .RegWrite_E(RegWrite_E),
        .MEMRd_E(MEMRd_E),
        .MEMWr_E(MEMWr_E),
        .WB_data_E(WB_data_E),
        .PCPLUS_E(PcPlus_E),
		.RegSel_E(RegSel_E)
    );
		
		
	
    // ========== EXECUTE ==========
    always @(*) begin
        if (ExtOp)
            valueAfterExtended = {{20{imm[11]}}, imm};  // Sign extend
        else
            valueAfterExtended = {20'b0, imm};           // Zero extend
    end

    mux2x1 m3(
        .A(BUSB_E),
        .B(Imm_E),
        .Sel(ALUSrc_E),
        .out(ALUSrcMux)
    );

    wire [2:0] ALUOP_D;  // Add this wire
    ALU_control AC(
        .Opcode(opcode),
        .Stall(Stall),
        .ALUOP(ALUOP_D)  // Generate for Decode stage
    );
	

    ALU AU(
        .ALUOP(ALUOP_E),  // ? Correct pipeline stage
        .BusA(BUSA_E),
        .BusB(ALUSrcMux),
        .Zero(Zero),
        .RES(RES)
    );
	//=========== Execute to Memory ===========
	wire RegSel_mem;
	E_TO_MEM_reg EMREG(
		.clk(CLK),
		.reset(reset),
		.Predicate_E(Predicate_E),
		.Res_E(RES),
		.Data_E(BUSB_E),
		.RW_E(RW_E),
		.MEMRd_E(MEMRd_E),
		.MEMWr_E(MEMWr_E),
		.RegWrite_E(RegWrite_E),
		.WB_data_E(WB_data_E),
		.PCPLUS_E(PcPlus_E),
		.RegSel_E(RegSel_E),
		.Predicate_mem(Predicate_mem),
		.Res_mem(Res_mem),
		.Data_mem(Data_mem),
		.RW_mem(RW_mem),
		.MEMRd_mem(MEMRd_mem),
		.MEMWr_mem(MEMWr_mem),
		.RegWrite_mem(RegWrite_mem),
		.WB_data_mem(WB_data_mem),
		.PCPLUS_mem(PCPLUS_mem),
		.RegSel_mem(RegSel_mem));
    // ========== MEMORY ==========
    Data_mem DM( 
		.reset(resetMemory),
        .clk(CLK),
        .data_in(Data_mem),
        .wr(MEMWr_mem & Predicate_mem),
        .rd(MEMRd_mem),
        .addr(Res_mem),
        .data_out(Data_out)
    );
	
    // ========== WRITE-BACK ==========
    mux4x1 m4(
        .A(Res_mem),
        .B(Data_out),
        .C(PCPLUS_mem),
        .Sel(WB_data_mem),
        .out(BusW)
    ); 
	// ========== Memory To WriteBack ===========
	MEM_TO_WB_reg MEMWBREG(
    .clk(CLK),
    .reset(reset),  // ADD THIS LINE
    .Predicate_mem(Predicate_mem),
    .Res_mem(BusW),
    .RW_mem(RW_mem),
    .RegWrite_mem(RegWrite_mem),
	.RegSel_mem(RegSel_mem),
    .Predicate_WB(Predicate_WB),
    .Data_WB(Data_WB),
    .RW_WB(RW_WB),
    .RegWrite_WB(RegWrite_WB),
	.RegSel_WB(RegSel_WB)
);

endmodule  

`timescale 1ns/1ps

module CPU_tb_complete_all_instructions;

    reg CLK;
    reg reset;
    reg resetMemory;
    wire [31:0] Instruction_top;
    
    integer cycle_count;
    integer total_tests;
    integer passed_tests;
    integer failed_tests;

    // Instantiate CPU
    CPU dut (
        .CLK(CLK),
        .reset(reset),
        .resetMemory(resetMemory),
        .Instruction_top(Instruction_top)
    );

    // Clock generation
    initial begin
        CLK = 0;
        forever #50 CLK = ~CLK;
    end

    // Cycle counter
    always @(posedge CLK) begin
        if (!reset && !resetMemory)
            cycle_count = cycle_count + 1;
    end

    // Decode and display instruction
    task decode_and_display_instruction;
        input [31:0] inst;
        input [31:0] pc;
        reg [4:0] opcode, rp, rd, rs, rt;
        reg [11:0] imm;
        reg signed [21:0] offset;
        reg signed [31:0] target;
        begin
            opcode = inst[31:27];
            rp = inst[26:22];
            rd = inst[21:17];
            rs = inst[16:12];
            rt = inst[11:7];
            imm = inst[11:0];
            offset = inst[21:0];
            target = pc + offset;
            
            case(opcode)
                5'd0:  $write("ADD R%0d,R%0d,R%0d,R%0d", rd, rs, rt, rp);
                5'd1:  $write("SUB R%0d,R%0d,R%0d,R%0d", rd, rs, rt, rp);
                5'd2:  $write("OR R%0d,R%0d,R%0d,R%0d", rd, rs, rt, rp);
                5'd3:  $write("NOR R%0d,R%0d,R%0d,R%0d", rd, rs, rt, rp);
                5'd4:  $write("AND R%0d,R%0d,R%0d,R%0d", rd, rs, rt, rp);
                5'd5:  $write("ADDI R%0d,R%0d,%0d,R%0d", rd, rs, imm, rp);
                5'd6:  $write("ORI R%0d,R%0d,%0d,R%0d", rd, rs, imm, rp);
                5'd7:  $write("NORI R%0d,R%0d,%0d,R%0d", rd, rs, imm, rp);
                5'd8:  $write("ANDI R%0d,R%0d,%0d,R%0d", rd, rs, imm, rp);
                5'd9:  $write("LW R%0d,%0d(R%0d),R%0d", rd, imm, rs, rp);
                5'd10: $write("SW R%0d,%0d(R%0d),R%0d", rd, imm, rs, rp);
                5'd11: $write("J +%0d,R%0d (->PC=%0d)", offset, rp, target);
                5'd12: $write("CALL +%0d,R%0d (->PC=%0d)", offset, rp, target);
                5'd13: $write("JR R%0d,R%0d", rs, rp);
                default: $write("UNKNOWN");
            endcase
        end
    endtask

    // Monitor all signals at each clock cycle
    always @(posedge CLK) begin
        if (!reset && !resetMemory) begin
            $display("\n================================================================================");
            $display("CYCLE %0d @ time=%0t ns", cycle_count, $time);
            $display("================================================================================");
            
            // === FETCH STAGE ===
            $display("\n--- FETCH STAGE ---");
            $display("  PC             = %0d (0x%08h)", dut.PC_out, dut.PC_out);
            $display("  PC_next        = %0d (0x%08h)", dut.PC_in, dut.PC_in);
            $display("  PC+1           = %0d (0x%08h)", dut.PC_plus1, dut.PC_plus1);
            $display("  Instruction    = 0x%08h", Instruction_top);
            $write  ("  Decoded        = ");
            decode_and_display_instruction(Instruction_top, dut.PC_out);
            $display("");
            $display("  PC_control     = %b (0=seq, 1=jump, 2=jr)", dut.PC_control);
            $display("  Flush          = %b", dut.flush);
            $display("  Stall          = %b", dut.Stall);
            
            // === DECODE STAGE ===
            $display("\n--- DECODE STAGE ---");
            $display("  Inst_FD        = 0x%08h", dut.intruction_FD);
            $write  ("  Decoded        = ");
            decode_and_display_instruction(dut.intruction_FD, dut.PC_FD);
            $display("");
            $display("  PC_FD          = %0d", dut.PC_FD);
            $display("  Opcode         = %0d", dut.opcode);
            $display("  Registers      = Rp=%0d Rd=%0d Rs=%0d Rt=%0d", dut.Rp, dut.Rd, dut.Rs, dut.Rt);
            $display("  BusA (R%0d)     = %0d (0x%08h)", dut.Rs, dut.BusA, dut.BusA);
            $display("  BusB (R%0d)     = %0d (0x%08h)", dut.Rt, dut.BusB, dut.BusB);
            $display("  BusP (R%0d)     = %0d (0x%08h)", dut.Rp, dut.BusP, dut.BusP);
            $display("  Predicate      = %b", dut.Predicate);
            $display("  Control Sigs   = RegW=%b ALUSrc=%b MemRd=%b MemWr=%b WB=%b RegSel=%b DestReg=R%0d",
                     dut.RegWrite, dut.ALUSrc, dut.MemRd, dut.MemWr, dut.WB, dut.RegSel, dut.OutputRegSelMux);
            
            // === FORWARDING ===
            $display("\n--- FORWARDING ---");
            $display("  FWA=%b FWB=%b FWP=%b (0=none, 1=EX, 2=MEM, 3=WB)", 
                     dut.FWA, dut.FWB, dut.FWP);
            $display("  BusA_FW        = %0d (0x%08h)", 
                     dut.BusA_outputmux_Forwarding, dut.BusA_outputmux_Forwarding);
            $display("  BusB_FW        = %0d (0x%08h)", 
                     dut.BusB_outputmux_Forwarding, dut.BusB_outputmux_Forwarding);
            $display("  BusP_FW        = %0d (0x%08h)", 
                     dut.BusP_outputmux_Forwarding, dut.BusP_outputmux_Forwarding);
            
            // === EXECUTE STAGE ===
            $display("\n--- EXECUTE STAGE ---");
            $display("  Predicate_E    = %b", dut.Predicate_E);
            $display("  BUSA_E         = %0d (0x%08h)", dut.BUSA_E, dut.BUSA_E);
            $display("  BUSB_E         = %0d (0x%08h)", dut.BUSB_E, dut.BUSB_E);
            $display("  Imm_E          = %0d (0x%08h)", dut.Imm_E, dut.Imm_E);
            $display("  ALUSrcMux      = %0d (0x%08h)", dut.ALUSrcMux, dut.ALUSrcMux);
            $display("  ALUOP_E        = %b", dut.ALUOP_E);
            $display("  ALU_Result     = %0d (0x%08h)", dut.RES, dut.RES);
            $display("  Zero Flag      = %b", dut.Zero);
            $display("  Dest Reg       = R%0d", dut.RW_E);
            $display("  RegWrite_E     = %b", dut.RegWrite_E);
            
            // === MEMORY STAGE ===
            $display("\n--- MEMORY STAGE ---");
            $display("  Predicate_mem  = %b", dut.Predicate_mem);
            $display("  Address        = %0d (0x%08h)", dut.Res_mem, dut.Res_mem);
            $display("  WriteData      = %0d (0x%08h)", dut.Data_mem, dut.Data_mem);
            $display("  ReadData       = %0d (0x%08h)", dut.Data_out, dut.Data_out);
            $display("  Dest Reg       = R%0d", dut.RW_mem);
            $display("  MemRd          = %b", dut.MEMRd_mem);
            $display("  MemWr          = %b (actual=%b)", dut.MEMWr_mem, dut.MEMWr_mem & dut.Predicate_mem);
            $display("  RegWrite_mem   = %b", dut.RegWrite_mem);
            
            // === WRITE-BACK STAGE ===
            $display("\n--- WRITE-BACK STAGE ---");
            $display("  Predicate_WB   = %b", dut.Predicate_WB);
            $display("  WriteData      = %0d (0x%08h)", dut.Data_WB, dut.Data_WB);
            $display("  Dest Reg       = R%0d", dut.RW_WB);
            $display("  RegWrite_WB    = %b", dut.RegWrite_WB);
            if (dut.RegWrite_WB)
                $display("  >>> WRITING R%0d = %0d <<<", dut.RW_WB, dut.Data_WB);
            
            // === REGISTER FILE ===
            print_register_changes();
            
            $display("");
        end
    end

    // Track previous register values
    reg [31:0] prev_regs [0:31];
    integer init_done;
    integer i;
    
    initial begin
        init_done = 0;
        for (i = 0; i < 32; i = i + 1)
            prev_regs[i] = 0;
    end

    // Print only registers that changed
    task print_register_changes;
        integer j;
        integer changed_count;
        begin
            changed_count = 0;
            
            for (j = 0; j < 32; j = j + 1) begin
                if (dut.RF.Register[j] != prev_regs[j])
                    changed_count = changed_count + 1;
            end
            
            if (changed_count > 0) begin
                $display("\n--- REGISTER CHANGES ---");
                for (j = 0; j < 32; j = j + 1) begin
                    if (dut.RF.Register[j] != prev_regs[j]) begin
                        $display("  R%0d: %0d -> %0d", j, prev_regs[j], dut.RF.Register[j]);
                    end
                end
            end
            
            for (j = 0; j < 32; j = j + 1)
                prev_regs[j] = dut.RF.Register[j];
            init_done = 1;
        end
    endtask

    // Verify instruction result
    task verify_instruction;
        input [255:0] inst_name;
        input [4:0] reg_num;
        input [31:0] expected;
        reg [31:0] actual;
        begin
            actual = dut.RF.Register[reg_num];
            total_tests = total_tests + 1;
            
            $display("\nTEST #%0d: %s", total_tests, inst_name);
            $display("  R%0d: Expected=%0d, Actual=%0d -> %s", 
                     reg_num, expected, actual, 
                     (actual == expected) ? "PASS" : "FAIL");
            
            if (actual == expected)
                passed_tests = passed_tests + 1;
            else
                failed_tests = failed_tests + 1;
        end
    endtask

    // Main test sequence
    initial begin
        cycle_count = 0;
        total_tests = 0;
        passed_tests = 0;
        failed_tests = 0;
        
        $display("\n========================================================================");
        $display("   COMPREHENSIVE TESTBENCH - ALL INSTRUCTIONS & HAZARD SCENARIOS");
        $display("========================================================================");
        
        reset = 1;
        resetMemory = 1;
        #200;
        @(posedge CLK);
        #1;
        reset = 0;
        resetMemory = 0;
        
        $display("\n>>> LOADING COMPREHENSIVE TEST PROGRAM <<<\n");
        
        // ========== SECTION 1: BASIC ARITHMETIC (RAW Hazards) ==========
        $display("=== SECTION 1: BASIC ARITHMETIC & RAW HAZARDS ===");
        
        // PC=0: ADD R7, R1, R2, R0 (R7 = 100 + 150 = 250)
        dut.IM.ram[0] = {5'd0, 5'd0, 5'd7, 5'd1, 5'd2, 7'b0};
        $display("PC=0:  ADD R7,R1,R2,R0    ; R7 = 100 + 150 = 250");
        
        // PC=1: SUB R8, R7, R1, R0 (RAW: R7 from EX stage) (R8 = 250 - 100 = 150)
        dut.IM.ram[1] = {5'd1, 5'd0, 5'd8, 5'd7, 5'd1, 7'b0};
        $display("PC=1:  SUB R8,R7,R1,R0    ; R8 = 250 - 100 = 150 [RAW: R7 from EX]");
        
        // PC=2: OR R9, R8, R2, R0 (RAW: R8 from MEM stage) (R9 = 150 | 150 = 150)
        dut.IM.ram[2] = {5'd2, 5'd0, 5'd9, 5'd8, 5'd2, 7'b0};
        $display("PC=2:  OR R9,R8,R2,R0     ; R9 = 150 | 150 = 150 [RAW: R8 from MEM]");
        
        // PC=3: AND R10, R9, R7, R0 (RAW: R9 from WB stage) (R10 = 150 & 250 = 146)
        dut.IM.ram[3] = {5'd4, 5'd0, 5'd10, 5'd9, 5'd7, 7'b0};
        $display("PC=3:  AND R10,R9,R7,R0   ; R10 = 150 & 250 = 146 [RAW: R9 from WB]");
        
        // ========== SECTION 2: IMMEDIATE INSTRUCTIONS ==========
        $display("\n=== SECTION 2: IMMEDIATE INSTRUCTIONS ===");
        
        // PC=4: ADDI R11, R1, 50, R0 (R11 = 100 + 50 = 150)
        dut.IM.ram[4] = {5'd5, 5'd0, 5'd11, 5'd1, 12'd50};
        $display("PC=4:  ADDI R11,R1,50,R0  ; R11 = 100 + 50 = 150");
        
        // PC=5: ORI R12, R3, 5, R0 (R12 = 10 | 5 = 15)
        dut.IM.ram[5] = {5'd6, 5'd0, 5'd12, 5'd3, 12'd5};
        $display("PC=5:  ORI R12,R3,5,R0    ; R12 = 10 | 5 = 15");
        
        // PC=6: ANDI R13, R5, 240, R0 (R13 = 255 & 240 = 240)
        dut.IM.ram[6] = {5'd8, 5'd0, 5'd13, 5'd5, 12'd240};
        $display("PC=6:  ANDI R13,R5,240,R0 ; R13 = 255 & 240 = 240");
        
        // PC=7: NORI R14, R3, R0, R0 (R14 = ~(10 | 0) = 0xFFFFFFF5)
        dut.IM.ram[7] = {5'd7, 5'd0, 5'd14, 5'd3, 12'd0};
        $display("PC=7:  NORI R14,R3,0,R0   ; R14 = ~(10 | 0)");
        
        // ========== SECTION 3: LOAD-USE HAZARD (STALL) ==========
        $display("\n=== SECTION 3: LOAD-USE HAZARD (STALL) ===");
        
        // PC=8: SW R1, 10(R0), R0 (MEM[10] = 100)
        dut.IM.ram[8] = {5'd10, 5'd0, 5'd1, 5'd0, 12'd10};
        $display("PC=8:  SW R1,10(R0),R0    ; MEM[10] = 100");
        
        // PC=9: LW R15, 10(R0), R0 (R15 = MEM[10] = 100)
        dut.IM.ram[9] = {5'd9, 5'd0, 5'd15, 5'd0, 12'd10};
        $display("PC=9:  LW R15,10(R0),R0   ; R15 = MEM[10] = 100");
        
        // PC=10: ADD R16, R15, R2, R0 (LOAD-USE HAZARD: R15, STALL)
        dut.IM.ram[10] = {5'd0, 5'd0, 5'd16, 5'd15, 5'd2, 7'b0};
        $display("PC=10: ADD R16,R15,R2,R0  ; R16 = 100 + 150 = 250 [STALL: Load-Use]");
        
        // PC=11: SUB R17, R16, R1, R0 (R17 = 250 - 100 = 150)
        dut.IM.ram[11] = {5'd1, 5'd0, 5'd17, 5'd16, 5'd1, 7'b0};
        $display("PC=11: SUB R17,R16,R1,R0  ; R17 = 250 - 100 = 150");
        
        // ========== SECTION 4: PREDICATION ==========
        $display("\n=== SECTION 4: PREDICATION ===");
        
        // PC=12: ADD R18, R1, R2, R20 (R20=1, executed: R18 = 250)
        dut.IM.ram[12] = {5'd0, 5'd20, 5'd18, 5'd1, 5'd2, 7'b0};
        $display("PC=12: ADD R18,R1,R2,R20  ; R18 = 250 [Predicate R20=1, EXEC]");
        
        // PC=13: ADD R19, R1, R2, R21 (R21=0, NOT executed: R19 = 0)
        dut.IM.ram[13] = {5'd0, 5'd21, 5'd19, 5'd1, 5'd2, 7'b0};
        $display("PC=13: ADD R19,R1,R2,R21  ; R19 = 0 [Predicate R21=0, NOT EXEC]");
        
        // PC=14: SW R2, 15(R0), R21 (R21=0, NOT executed)
        dut.IM.ram[14] = {5'd10, 5'd21, 5'd2, 5'd0, 12'd15};
        $display("PC=14: SW R2,15(R0),R21   ; No write [Predicate R21=0, NOT EXEC]");
        
        // ========== SECTION 5: CONTROL HAZARDS - JUMP ==========
        $display("\n=== SECTION 5: CONTROL HAZARDS - JUMP ===");
        
        // PC=15: J +5, R0 (Jump to PC=20)
        dut.IM.ram[15] = {5'd11, 5'd0, 22'd5};
        $display("PC=15: J +5,R0            ; Jump to PC=20 [CONTROL HAZARD]");
        
        // PC=16-19: SHOULD BE SKIPPED (flushed)
        dut.IM.ram[16] = {5'd0, 5'd0, 5'd25, 5'd1, 5'd1, 7'b0};
        dut.IM.ram[17] = {5'd0, 5'd0, 5'd25, 5'd1, 5'd1, 7'b0};
        dut.IM.ram[18] = {5'd0, 5'd0, 5'd25, 5'd1, 5'd1, 7'b0};
        dut.IM.ram[19] = {5'd0, 5'd0, 5'd25, 5'd1, 5'd1, 7'b0};
        $display("PC=16-19: [SKIPPED]");
        
        // PC=20: ADD R22, R3, R4, R0 (R22 = 10 + 20 = 30)
        dut.IM.ram[20] = {5'd0, 5'd0, 5'd22, 5'd3, 5'd4, 7'b0};
        $display("PC=20: ADD R22,R3,R4,R0   ; R22 = 10 + 20 = 30");
        
        // ========== SECTION 6: PREDICATED JUMP (NOT TAKEN) ==========
        $display("\n=== SECTION 6: PREDICATED JUMP (NOT TAKEN) ===");
        
        // PC=21: J +10, R21 (R21=0, Jump NOT taken)
        dut.IM.ram[21] = {5'd11, 5'd21, 22'd10};
        $display("PC=21: J +10,R21          ; NO Jump [Predicate R21=0]");
        
        // PC=22: ADD R23, R2, R3, R0 (Should execute: R23 = 150 + 10 = 160)
        dut.IM.ram[22] = {5'd0, 5'd0, 5'd23, 5'd2, 5'd3, 7'b0};
        $display("PC=22: ADD R23,R2,R3,R0   ; R23 = 150 + 10 = 160 [Jump not taken]");
        
        // ========== SECTION 7: CALL & RETURN ==========
        $display("\n=== SECTION 7: CALL & RETURN ===");
        
        // PC=23: CALL +3, R0 (Call to PC=26, R31 = 24)
        dut.IM.ram[23] = {5'd12, 5'd0, 22'd3};
        $display("PC=23: CALL +3,R0         ; Call to PC=26, R31=24");
        
        // PC=24: J +4, R0 (Jump to PC=28)
        dut.IM.ram[24] = {5'd11, 5'd0, 22'd4};
        $display("PC=24: J +4,R0            ; Jump to PC=28 [After return from CALL]");
        
        // PC=25: SHOULD BE SKIPPED
        dut.IM.ram[25] = {5'd0, 5'd0, 5'd26, 5'd1, 5'd1, 7'b0};
        $display("PC=25: [SKIPPED]");
        
        // PC=26: ADD R24, R1, R3, R0 (In function: R24 = 100 + 10 = 110)
        dut.IM.ram[26] = {5'd0, 5'd0, 5'd24, 5'd1, 5'd3, 7'b0};
        $display("PC=26: ADD R24,R1,R3,R0   ; R24 = 100 + 10 = 110 [In function]");
        
        // PC=27: JR R31, R0 (Return to PC=24)
        dut.IM.ram[27] = {5'd13, 5'd0, 5'd0, 5'd31, 5'd0, 7'b0};
        $display("PC=27: JR R31,R0          ; Return to PC=24 [JR with R31]");
        
        // PC=28: ADD R25, R1, R4, R0 (After return: R25 = 100 + 20 = 120)
        dut.IM.ram[28] = {5'd0, 5'd0, 5'd25, 5'd1, 5'd4, 7'b0};
        $display("PC=28: ADD R25,R1,R4,R0   ; R25 = 100 + 20 = 120 [After return]");
        
        // ========== SECTION 8: MULTIPLE CONSECUTIVE LOADS (STALLS) ==========
        $display("\n=== SECTION 8: MULTIPLE LOADS (MULTIPLE STALLS) ===");
        
        // PC=29: LW R26, 0(R0), R0 (R26 = MEM[0] = 100)
        dut.IM.ram[29] = {5'd9, 5'd0, 5'd26, 5'd0, 12'd0};
        $display("PC=29: LW R26,0(R0),R0    ; R26 = MEM[0] = 100");
        
        // PC=30: LW R27, 8(R0), R0 (R27 = MEM[8] = 0xAA)
        dut.IM.ram[30] = {5'd9, 5'd0, 5'd27, 5'd0, 12'd8};
        $display("PC=30: LW R27,8(R0),R0    ; R27 = MEM[8] = 0xAA");
        
        // PC=31: ADD R28, R26, R27, R0 (STALL: R26 & R27)
        dut.IM.ram[31] = {5'd0, 5'd0, 5'd28, 5'd26, 5'd27, 7'b0};
        $display("PC=31: ADD R28,R26,R27,R0 ; R28 = 100 + 170 = 270 [STALL: R27]");
        
        // ========== SECTION 9: NOR INSTRUCTION ==========
        $display("\n=== SECTION 9: NOR INSTRUCTION ===");
        
        // PC=32: NOR R29, R3, R4, R0 (R29 = ~(10 | 20) = 0xFFFFFFE5)
        dut.IM.ram[32] = {5'd3, 5'd0, 5'd29, 5'd3, 5'd4, 7'b0};
        $display("PC=32: NOR R29,R3,R4,R0   ; R29 = ~(10 | 20)");
        
        $display("\n>>> STARTING SIMULATION <<<\n");
        
        // Run simulation
        repeat(39) @(posedge CLK);
        
        $display("\n========================================================================");
        $display("                    VERIFICATION COMPLETE");
        $display("========================================================================");
        
        #10;
        
        // Verify all results
        verify_instruction("ADD R7 (basic)", 7, 32'd250);
        verify_instruction("SUB R8 (RAW: EX)", 8, 32'd150);
        verify_instruction("OR R9 (RAW: MEM)", 9, 32'd150);
        verify_instruction("AND R10 (RAW: WB)", 10, 32'd146);
        verify_instruction("ADDI R11", 11, 32'd150);
        verify_instruction("ORI R12", 12, 32'd15);
        verify_instruction("ANDI R13", 13, 32'd240);
        verify_instruction("NORI R14", 14, 32'hFFFFFFF5);
        verify_instruction("LW R15", 15, 32'd100);
        verify_instruction("ADD R16 (Load-Use Stall)", 16, 32'd250);
        verify_instruction("SUB R17", 17, 32'd150);
        verify_instruction("ADD R18 (Pred=1)", 18, 32'd250);
        verify_instruction("ADD R19 (Pred=0)", 19, 32'd0);
        verify_instruction("ADD R22 (after Jump)", 22, 32'd30);
        verify_instruction("ADD R23 (Jump not taken)", 23, 32'd160);
        verify_instruction("R31 (Return address)", 31, 32'd24);
        verify_instruction("ADD R24 (in function)", 24, 32'd110);
        verify_instruction("ADD R25 (after return)", 25, 32'd120);
        verify_instruction("LW R26", 26, 32'd100);
        verify_instruction("LW R27", 27, 32'd170);
        verify_instruction("ADD R28 (Multiple Stalls)", 28, 32'd270);
        verify_instruction("NOR R29", 29, 32'hFFFFFFE1);
        
        $display("\n========================================================================");
        $display("  Total Tests: %0d | Passed: %0d | Failed: %0d", 
                 total_tests, passed_tests, failed_tests);
        if (failed_tests == 0)
            $display("  STATUS: ALL TESTS PASSED");
        else
            $display("  STATUS: %0d TEST(S) FAILED", failed_tests);
        $display("========================================================================");
        
        $display("");
        #100;
        $finish;
    end
endmodule