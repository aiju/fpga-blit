`include "dport.vh"

module dport_regs(
	input wire clk,
	
	input wire [7:0] reg_addr,
	output reg [31:0] reg_rdata,
	input wire [31:0] reg_wdata,
	input wire reg_wr,
	input wire reg_req,
	output reg reg_ack,
	input wire [3:0] reg_wstrb,
	output reg reg_err,
	
	output reg [`ATTRMAX:0] attr,
	
	output reg [31:0] phyctl,
	
	output reg [31:0] auxctrl,
	input wire [31:0] auxstat
);

	initial begin
		attr = 0;
	end
	always @(posedge clk) begin
		reg_ack <= reg_req;
		auxctrl[31] <= 1'b0;
		if(reg_req) begin
			reg_err <= 1'b0;
			if(reg_wr)
				casez(reg_addr)
				'h00: phyctl <= reg_wdata;
				'h08: auxctrl <= reg_wdata;
				'h40: attr[31:0] <= reg_wdata;
				'h44: attr[63:32] <= reg_wdata;
				'h48: attr[95:64] <= reg_wdata;
				'h4c: attr[127:96] <= reg_wdata;
				'h50: attr[143:128] <= reg_wdata[15:0];
				'h54: attr[167:144] <= reg_wdata[23:0];
				'h58: attr[191:168] <= reg_wdata[23:0];
				'h5c: attr[208:192] <= reg_wdata[16:0];
				'h60: attr[232:209] <= reg_wdata[23:0];
				'h64: attr[256:233] <= reg_wdata[23:0];
				'h68: attr[273:257] <= reg_wdata[16:0];
				default: reg_err <= 1'b1;
				endcase
			else
				case(reg_addr)
				'h00: reg_rdata <= phyctl;
				'h08: reg_rdata <= auxctrl;
				'h0c: reg_rdata <= auxstat;
				default: reg_err <= 1'b1;
				endcase
		end
	end
endmodule
