`include "dport.vh"

module dport_phy(
	input wire dpclk,
	input wire [1:0] phymode,
	input wire [15:0] scrdat0,
	input wire [15:0] scrdat1,
	input wire [1:0] scrisk0,
	input wire [1:0] scrisk1,
	output reg [15:0] txdat0,
	output reg [15:0] txdat1,
	output reg [1:0] txisk0,
	output reg [1:0] txisk1
);

	reg [2:0] ctr;
	reg [15:0] txdat1_0, txdat0_, txdat1_;
	reg [1:0] txisk1_0, txisk0_, txisk1_;
	always @(posedge dpclk) begin
		txdat0 <= txdat0_;
		txisk0 <= txisk0_;
		txdat1 <= txdat1_0;
		txisk1 <= txisk1_0;
		txdat1_0 <= txdat1_;
		txisk1_0 <= txisk1_;
		if(ctr == 4)
			ctr <= 0;
		else
			ctr <= ctr + 1;
	end

	always @(*) begin
		txdat0_ = 0;
		txdat1_ = 0;
		txisk0_ = 0;
		txisk1_ = 0;
		case(phymode)
		1: begin
			txdat0_ = scrdat0;
			txdat1_ = scrdat1;
			txisk0_ = scrisk0;
			txisk1_ = scrisk1;
		end
		2: begin
			txdat0_ = 16'h4A4A;
			txdat1_ = 16'h4A4A;
		end
		3: begin
			case(ctr)
			0, 1: begin
				txdat0_ = 16'hcbbc;
				txisk0_ = 2'b01;
			end
			2, 3, 4: txdat0_ = 16'h4a4a;
			endcase
			txdat1_ = txdat0_;
			txisk1_ = txisk0_;
		end
		endcase
	end

endmodule
