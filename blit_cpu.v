module blit_cpu(
	input wire clk,
	
	output reg cpu_req,
	output reg [23:0] cpu_addr,
	output reg [15:0] cpu_wdata,
	output reg [1:0] cpu_wstrb,
	output reg cpu_we,
	input wire cpu_ack,
	input wire [15:0] cpu_rdata,
	input wire cpu_err,
	
	input wire [7:1] irq
);

	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	wire		ASn;			// From fx68k_i of fx68k.v
	wire		BGn;			// From fx68k_i of fx68k.v
	logic		E;			// From fx68k_i of fx68k.v
	wire		FC0;			// From fx68k_i of fx68k.v
	wire		FC1;			// From fx68k_i of fx68k.v
	wire		FC2;			// From fx68k_i of fx68k.v
	wire		LDSn;			// From fx68k_i of fx68k.v
	wire		UDSn;			// From fx68k_i of fx68k.v
	wire		VMAn;			// From fx68k_i of fx68k.v
	wire		eRWn;			// From fx68k_i of fx68k.v
	wire [23:1]	eab;			// From fx68k_i of fx68k.v
	wire [15:0]	oEdb;			// From fx68k_i of fx68k.v
	wire		oHALTEDn;		// From fx68k_i of fx68k.v
	wire		oRESETn;		// From fx68k_i of fx68k.v
	// End of automatics
	
	reg extReset;
	wire pwrUp = extReset;
	reg enPhi1;
	reg enPhi2;
	reg [3:0] resctr;
	
	initial begin
		extReset = 1'b1;
		enPhi1 = 1'b1;
		enPhi2 = 1'b0;
		resctr = 4'd15;
	end
	always @(posedge clk) begin
		enPhi1 <= !enPhi1;
		enPhi2 <= !enPhi2;
		if(resctr > 0)
			resctr <= resctr - 1;
		else
			extReset <= 1'b0;
	end
	
	reg DTACKn = 1'b1;
	reg BERRn = 1'b1;
	reg VPAn = 1'b1;
	wire BRn = 1'b1;
	wire BGACKn = 1'b1;
	
	reg [15:0] iEdb;
	reg [3:0] state;
	reg [2:0] IPLn = 3'b111;
	wire IPL0n = IPLn[0];
	wire IPL1n = IPLn[1];
	wire IPL2n = IPLn[2];
	
	always @(posedge clk) begin
		case(state)
		0: if(!extReset && (!UDSn || !LDSn)) begin
			if(FC0 && FC1 && FC2) begin
				VPAn <= 1'b0;
				state <= 2;
			end else begin
				cpu_req <= 1'b1;
				cpu_addr <= {eab, 1'b0};
				cpu_wdata <= oEdb;
				cpu_we <= !eRWn;
				cpu_wstrb <= {!UDSn, !LDSn};
				state <= 1;
			end
		end
		1: begin
			cpu_req <= 1'b0;
			if(cpu_ack) begin
				if(cpu_err)
					BERRn <= 1'b0;
				else
					DTACKn <= 1'b0;
				iEdb <= cpu_rdata;
				state <= 2;
			end
		end
		2: begin
			if(UDSn && LDSn) begin
				DTACKn <= 1'b1;
				BERRn <= 1'b1;
				VPAn <= 1'b1;
				state <= 0;
			end
		end
		endcase
	end
	
	always @(posedge clk) begin
		case(1'b1)
		irq[7]: IPLn <= 3'b000;
		irq[6]: IPLn <= 3'b001;
		irq[5]: IPLn <= 3'b010;
		irq[4]: IPLn <= 3'b011;
		irq[3]: IPLn <= 3'b100;
		irq[2]: IPLn <= 3'b101;
		irq[1]: IPLn <= 3'b110;
		default: IPLn <= 3'b111;
		endcase
	end

	fx68k fx68k_i(
		/*AUTOINST*/
		      // Outputs
		      .eRWn		(eRWn),
		      .ASn		(ASn),
		      .LDSn		(LDSn),
		      .UDSn		(UDSn),
		      .E		(E),
		      .VMAn		(VMAn),
		      .FC0		(FC0),
		      .FC1		(FC1),
		      .FC2		(FC2),
		      .BGn		(BGn),
		      .oRESETn		(oRESETn),
		      .oHALTEDn		(oHALTEDn),
		      .oEdb		(oEdb[15:0]),
		      .eab		(eab[23:1]),
		      // Inputs
		      .clk		(clk),
		      .extReset		(extReset),
		      .pwrUp		(pwrUp),
		      .enPhi1		(enPhi1),
		      .enPhi2		(enPhi2),
		      .DTACKn		(DTACKn),
		      .VPAn		(VPAn),
		      .BERRn		(BERRn),
		      .BRn		(BRn),
		      .BGACKn		(BGACKn),
		      .IPL0n		(IPL0n),
		      .IPL1n		(IPL1n),
		      .IPL2n		(IPL2n),
		      .iEdb		(iEdb[15:0]));

endmodule

// Local Variables:
// verilog-library-directories:("." "fx68k")
// End:
