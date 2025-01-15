module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteDataM, DataAdrM;
  logic        MemWriteM;

  // instantiate device to be tested
  top dut(clk, reset, WriteDataM, DataAdrM, MemWriteM);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWriteM) begin
        if(DataAdrM === 104 & WriteDataM === 25) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdrM !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule



module top(input logic clk, reset, 
				output logic [31:0] WriteDataM, DataAdrM,
				output logic MemWriteM);
	logic [31:0] PCF, InstrF, ReadDataM;
	
	//instantiate processor and memories
	riscv riscv(clk, reset, PCF, InstrF, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
	imem imem(PCF, InstrF);
	dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~riscv~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

module riscv(input logic clk, reset,
					output logic [31:0] PCF,
					input logic [31:0] InstrF,
					output logic MemWriteM,
					output logic [31:0] DataAdrM, WriteDataM,
					input logic [31:0] ReadDataM);
	
	logic PCSrcE;
	logic [31:0] InstrD;
	
	//Hazard signal
	logic StallF, FlushD, StallD, FlushE;
	logic [1:0] ForwardAE, ForwardBE;
	logic [4:0] Rs1E, Rs2E, RdE, RdM, RdW;
	logic [1:0] ResultSrcE; //need idx 0 for hazard from the controller
	
	
	logic RegWriteM, RegWriteW;
	logic ZeroE, lt_zero; 
	logic ALUSrcE;
	logic [1:0] ResultSrcW; 
	logic [2:0] ImmSrcD;
	logic [3:0] ALUControlE;
	
	controller c(InstrD[6:0], InstrD[14:12], InstrD[30], clk, reset, RegWriteM, RegWriteW, MemWriteM, PCSrcE, 
						ResultSrcE, ResultSrcW, ZeroE, lt_zero, ALUControlE, ALUSrcE, ImmSrcD);
	datapath dp(clk, reset, PCF, InstrF, InstrD, PCSrcE, WriteDataM, ReadDataM, RegWriteW, ALUSrcE,
					ResultSrcW, ImmSrcD, ALUControlE, ZeroE, lt_zero, StallF, FlushD, StallD, FlushE, 
					ForwardAE, ForwardBE, Rs1E, Rs2E, RdE, RdM, RdW, DataAdrM); 
	hazard h(InstrD[19:15], InstrD[24:20], Rs1E, Rs2E, RdE, RdM, RdW, PCSrcE, ResultSrcE[0], RegWriteM, RegWriteW,
					StallF, FlushD, StallD, FlushE, ForwardAE, ForwardBE);

endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Memory~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Controller~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

module controller(input logic [6:0] op,
						input logic [2:0] funct3,
                  input logic funct7b5, clk, reset,
						output logic RegWriteM, RegWriteW, 
						output logic MemWriteM, PCSrcE, 
						output logic [1:0] ResultSrcE, ResultSrcW, 
						input logic ZeroE, lt_zero, 
						output logic [3:0] ALUControlE,
						output logic ALUSrcE,
						output logic [2:0] ImmSrcD);
						
	
	logic RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcD; 
	logic [1:0] ResultSrcD; 
	logic [3:0] ALUControlD;
	
	
	logic [1:0] ALUOp;
	logic RegWriteE;
	logic [1:0] ResultSrcM;
	logic MemWriteE, JumpE, BranchE;
	
	maindec md(op, ResultSrcD, MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOp);
	aludec ad(op[5], funct3, funct7b5, ALUOp, ALUControlD);
	
	//Decode stage to Excute stage
	flopclr #(1) RWE(clk, reset, FlushE, RegWriteD, RegWriteE);
	flopclr #(2) RSE(clk, reset, FlushE, ResultSrcD, ResultSrcE);
	flopclr #(1) MWE(clk, reset, FlushE, MemWriteD, MemWriteE);
	flopclr #(1) JE(clk, reset, FlushE, JumpD, JumpE);
	flopclr #(1) BE(clk, reset, FlushE, BranchD, BranchE);
	flopclr #(4) ALUConE(clk, reset, FlushE, ALUControlD, ALUControlE);
	flopclr #(1) ASrcE(clk, reset, FlushE, ALUSrcD, ALUSrcE);
	
	//Execute stage to Memory Stage
	flopr #(1) RWM(clk, reset, RegWriteE, RegWriteM);
	flopr #(2) RSM(clk, reset, ResultSrcE, ResultSrcM);
	flopr #(1) MWM(clk, reset, MemWriteE, MemWriteM);
	
	assign PCSrcE = BranchE & ((funct3 == 3'b000) ? ZeroE : lt_zero) | JumpE;
	
	//Memory stage to WriteBack stage
	flopr #(1) RWWB(clk, reset, RegWriteM, RegWriteW);
	flopr #(2) RSWB(clk, reset, ResultSrcM, ResultSrcW);
	
endmodule


module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrcD,
               output logic       MemWriteD,
               output logic       BranchD, ALUSrcD,
               output logic       RegWriteD, JumpD,
               output logic [2:0] ImmSrcD,
               output logic [1:0] ALUOp);

  logic [11:0] controls;

  assign {RegWriteD, ImmSrcD, ALUSrcD, MemWriteD,
          ResultSrcD, BranchD, ALUOp, JumpD} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
	   7'b0110111: controls = 12'b1_100_0_0_11_0_11_0; // lui
      7'b0000011: controls = 12'b1_000_1_0_01_0_00_0; // lw
		7'b0100011: controls = 12'b0_001_1_1_00_0_00_0; // sw
		7'b0110011: controls = 12'b1_000_0_0_00_0_10_0; // R-type
		7'b1100011: controls = 12'b0_010_0_0_00_1_01_0; // beq or blt
		7'b0010011: controls = 12'b1_000_1_0_00_0_10_0; // I-type ALU
		7'b1101111: controls = 12'b1_011_0_0_10_0_00_1; // jal
      default:    controls = 12'b0_000_0_0_00_0_00_0; // non-implemented instruction
    endcase
endmodule


module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [3:0] ALUControlD);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControlD = 4'b0000; // addition
      2'b01:                ALUControlD = 4'b0001; // subtraction
		2'b11: 					 ALUControlD = 4'b1010; //lui
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControlD = 4'b0001; // sub
                          else          
                            ALUControlD = 4'b0000; // add, addi
                 3'b010:    ALUControlD = 4'b0101; // slt, slti
                 3'b110:    ALUControlD = 4'b0011; // or, ori
                 3'b111:    ALUControlD = 4'b0010; // and, andi
					  3'b001:	 ALUControlD = 4'b1001; //SRA
					  3'b100:    ALUControlD = 4'b0101; // BLT (signed comparison)
                 default:   ALUControlD = 4'b0000; // ???
               endcase
    endcase
endmodule

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Datapath~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
module datapath(input logic clk, reset, 
						output logic [31:0] PCF,
						input logic [31:0] InstrF,
						output logic [31:0] InstrD,
						input logic PCSrcE,					
						output logic [31:0] WriteDataM, 
						input logic [31:0] ReadDataM,
						input logic RegWriteW, 					
						input logic ALUSrcE, 					
						input logic [1:0] ResultSrcW, 
						input logic [2:0] ImmSrcD,		
						input logic [3:0] ALUControlE,			
						output logic ZeroE, lt_zero, 
						input logic StallF, FlushD, StallD, FlushE, 
						input logic [1:0] ForwardAE, ForwardBE,		
						output logic [4:0] Rs1E, Rs2E, RdE, RdM, RdW, 
						output logic [31:0] DataAdrM);
	
	logic [31:0] PCPlus4F, PCTargetE, PCFNext, PCD, PCPlus4D, ResultW, ImmExtD, PCE, ImmExtE;
	logic [31:0] PCPlus4E, SrcAE, WriteDataE, SrcBE, ALUResultE, PCPlus4M, ALUResultWB; 
	logic [31:0] ReadDataW, PCPlus4W; 
	
	logic [31:0] RD1, RD2, RD1E, RD2E;	
	
	//Fetch Stage
	mux2 #(32) pcfmux(PCPlus4F, PCTargetE, PCSrcE, PCFNext);
	flopenr #(32) pcfflopen(clk, reset, ~StallF, PCFNext, PCF);
	//imem imem_inst(PCF, InstrF);
	adder pcfadd4(PCF, 32'd4, PCPlus4F);
	
	//F-stage to M-stage
	flopenclr #(32) InstrDflopenclr(clk, reset, FlushD, ~StallD, InstrF, InstrD);
	flopenclr #(32) PCDflopenclr(clk, reset, FlushD, ~StallD, PCF, PCD);
	flopenclr #(32) PCPlus4Dflopenclr(clk, reset, FlushD, ~StallD, PCPlus4F, PCPlus4D);
	
	
	//Decode stage
	regfile rf(~clk, RegWriteW, InstrD[19:15], InstrD[24:20], RdW, ResultW, RD1, RD2); //check RdW datatype
	extend  ext(InstrD[31:7], ImmSrcD, ImmExtD);
	
	//D-stage to E-stage
	flopclr #(32) RD1Eflopclr(clk, reset, FlushE, RD1, RD1E); 
	flopclr #(32) RD2Eflopclr(clk, reset, FlushE, RD2, RD2E);
	flopclr #(32) PCEflopclr(clk, reset, FlushE, PCD, PCE);
	flopclr #(5) Rs1Eflopclr(clk, reset, FlushE, InstrD[19:15], Rs1E); //come here idk the correct data type
	flopclr #(5) Rs2Eflopclr(clk, reset, FlushE, InstrD[24:20], Rs2E); //come here idk the correct data type
	flopclr #(5) RdEflopclr(clk, reset, FlushE, InstrD[11:7], RdE); //come here idk the correct data type
	flopclr #(32) ImmExtEflopclr(clk, reset, FlushE, ImmExtD, ImmExtE);
	flopclr #(32) PCPlus4Eflopclr(clk, reset, FlushE, PCPlus4D, PCPlus4E);
	
	
	//Execute stage
	mux3 #(32)  SrcAEMux(RD1E, ResultW, DataAdrM, ForwardAE, SrcAE); //ALUResultM is DataAdrM
	mux3 #(32)  WDEMux(RD2E, ResultW, DataAdrM, ForwardBE, WriteDataE);
	mux2 #(32)  SrcBEmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);
	adder PCTargetEadder(PCE, ImmExtE, PCTargetE);
	alu  aluresultEalu(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE, lt_zero); //define ALUResultE for signal naming
	
	//E-stage to M-stage
	flopr #(32) ALUReMflop(clk, reset, ALUResultE, DataAdrM); 
	flopr #(32) WMMflop(clk, reset, WriteDataE, WriteDataM);
	flopr #(5) RdMflop(clk, reset, RdE, RdM); //come here idk the correct data type
	flopr #(32) PCPlus4Mflop(clk, reset, PCPlus4E, PCPlus4M);
	
	//Memory stage
	//dmem dmem_inst(clk, MemWriteM, ALUResultM, WriteDataM, rd);
	
	//M-stage to WB-stage
	flopr #(32) ALUReWBflop(clk, reset, DataAdrM, ALUResultWB); //add ALUResultWB since there's no signal name
	flopr #(32) rdWflop(clk, reset, ReadDataM, ReadDataW);
	flopr #(5) RdWflop(clk, reset, RdM, RdW); //RdW might cause an issue since its datatype 11:7 to 31:0
	flopr #(32) PCPlus4Wflop(clk, reset, PCPlus4M, PCPlus4W);
	
	//WriteBack stage
	mux3 #(32)  ResultWMux(ALUResultWB, ReadDataW, PCPlus4W, ResultSrcW, ResultW);
		
	
endmodule



module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [2:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      3'b000:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      3'b001:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
               // B-type (branches)
      3'b010:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
               // J-type (jal)
      3'b011:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
		         // U-type (lui)
      3'b100:   immext = {instr[31:12], 12'b0};
      default: immext = 32'b0; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
		(input logic clk, reset, en, input logic [WIDTH-1:0] d, output logic [WIDTH-1:0] q);
	
	always_ff @(posedge clk, posedge reset)
		if (reset) q <= 0;
		else if (en) q <= d;

endmodule

module flopclr #(parameter WIDTH = 8)
                (input  logic             clk, reset, clr,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk or posedge reset)
    if (reset) q <= 0;
    else if (clr) q <= 0;
    else q <= d;
endmodule

module flopenclr #(parameter WIDTH = 8)
                  (input logic clk, reset, clr, en, 
                   input logic [WIDTH-1:0] d, 
                   output logic [WIDTH-1:0] q);

  always_ff @(posedge clk or posedge reset)
    if (reset) q <= 0;
    else if (clr) q <= 0;
    else if (en) q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [3:0]  alucontrol,
           output logic [31:0] result,
           output logic        Zero,
			  output logic			 lt_zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      4'b0000:  result = sum;                 // add
      4'b0001:  result = sum;                 // subtract
      4'b0010:  result = a & b;               // and
      4'b0011:  result = a | b;       			 // or
      4'b0100:  result = a ^ b;       			 // xor
      4'b0101:  result = (a < b) ? 32'd1 : 32'd0;       // slt
      4'b0110:  result = a << b;       		 // sll
      4'b0111:  result = a >> b;       		 // srl
		4'b1001:  result = a >> b[4:0]; 			 //sra
		4'b1010:  result = b << 12;				 //lui
      default: result = 32'b0;
    endcase

  assign Zero = (result == 32'b0);
  assign lt_zero = result[31];
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Hazard~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
module hazard(
	input logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
	input logic PCSrcE, 
	input logic ResultSrcE0, RegWriteM, RegWriteW,
	output logic StallF, FlushD, StallD, FlushE, 
	output logic [1:0] ForwardAE, ForwardBE);
	
	logic lwStall;	
	
	//Data hazrd logic (shown for SrcA of ALU)
	always_comb begin
		if (((Rs1E == RdM) && RegWriteM) && (Rs1E != 0)) ForwardAE = 2'b10;  //Case 1
		else if (((Rs1E == RdW) && RegWriteW) && (Rs1E != 0)) ForwardAE = 2'b01; //Case 2
		else ForwardAE = 2'b00; //Case 3
		
		if (((Rs2E == RdM) && RegWriteM) && (Rs2E != 0)) ForwardBE = 2'b10;  //Case 1
		else if (((Rs2E == RdW) && RegWriteW) && (Rs2E != 0)) ForwardBE = 2'b01; //Case 2
		else ForwardBE = 2'b00; //Case 3
	end
	 
	 //load word stall logic
	 assign lwStall = ((Rs1D == RdE) || (Rs2D == RdE)) && ResultSrcE0;
	 assign StallF = lwStall;
	 assign StallD = lwStall;
	 
	 //Control hazard flush
	 assign FlushD = PCSrcE;
	 assign FlushE = lwStall || PCSrcE;
	 
endmodule

