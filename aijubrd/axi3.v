`default_nettype none
/* limitation in vlt -- to be replaced by a parameter */
`define ID 12

module axi3 #(
	parameter TIMEOUT = 1048575
) (
	input wire clk,
	input wire rstn,
	
	output wire axiaclk,
	
	input wire axiarvalid,
	output reg axiarready,
	input wire [31:0] axiaraddr,
	input wire [1:0] axiarburst, axiarlock,
	input wire [2:0] axiarsize, axiarprot,
	input wire [3:0] axiarlen, axiarcache, axiarqos,
	input wire [`ID-1:0] axiarid,
	
	output reg axirvalid,
	input wire axirready,
	output reg [31:0] axirdata,
	output reg [`ID-1:0] axirid,
	output reg [1:0] axirresp,
	output reg axirlast,

	input wire axiawvalid,
	output reg axiawready,
	input wire [31:0] axiawaddr,
	input wire [1:0] axiawburst, axiawlock,
	input wire [2:0] axiawsize, axiawprot,
	input wire [3:0] axiawlen, axiawcache, axiawqos,
	input wire [`ID-1:0] axiawid,
	
	input wire axiwvalid,
	output reg axiwready,
	input wire [31:0] axiwdata,
	input wire [3:0] axiwstrb,
	input wire [`ID-1:0] axiwid,
	input wire axiwlast,
	
	output reg axibvalid,
	input wire axibready,
	output reg [1:0] axibresp,
	output reg [`ID-1:0] axibid,
	
	output reg [31:0] outaddr,
	input wire [31:0] outrdata,
	output reg [31:0] outwdata,
	output reg outwr,
	output reg outreq,
	input wire outack,
	input wire outerr,
	output reg [3:0] outwstrb
);

	localparam OKAY = 0;
//	localparam EXOKAY = 1;
	localparam SLVERR = 2;
	localparam DECERR = 3;
//	localparam FIXED = 0;
	localparam INCR = 1;
//	localparam WRAP = 2;
	
	reg [2:0] state;
	localparam IDLE = 0;
	localparam READOUT = 1;
	localparam READREPLY = 2;
	localparam WAITWRDATA = 3;
	localparam WRITEOUT = 4;
	localparam WRRESP = 5;
	
	reg rpend;
	reg [31:0] axiawaddr0, axiaraddr0;
	reg [1:0] axiawburst0, axiarburst0;
	reg [1:0] axiawsize0, axiarsize0;
	reg [3:0] axiawlen0, axiarlen0;
	reg [31:0] timer;
	
	assign axiaclk = clk;
	
	function [31:0] nextaddr(input [31:0] addr, input [1:0] axiburst, input [1:0] size);
		case(axiburst)
		default: nextaddr = addr;
		INCR: nextaddr = addr + (1<<size);
		endcase
	endfunction
	
	initial begin
		state = IDLE;
		axiarready = 1'b1;
		axiawready = 1'b1;
		axirvalid = 1'b0;
		axibvalid = 1'b0;
		axiwready = 1'b0;
		outreq = 1'b0;
	end
	
	always @(posedge clk or negedge rstn)
		if(!rstn) begin
			state <= IDLE;
			rpend <= 0;
			timer <= 32'bx;
			axiarready <= 1'b1;
			axirvalid <= 1'b0;
			axirdata <= 32'bx;
			axirid <= {`ID{1'bx}};
			axirresp <= 2'bx;
			axirlast <= 1'bx;
			axiawready <= 1'b1;
			axiwready <= 1'b0;
			axibvalid <= 1'b0;
			axibresp <= 2'bx;
			axibid <= {`ID{1'bx}};
			outaddr <= 32'bx;
			outwdata <= 32'bx;
			outwr <= 1'bx;
			outreq <= 1'b0;
			outwstrb <= 4'bx;
			axiaraddr0 <= 32'bx;
			axiarburst0 <= 2'bx;
			axiarlen0 <= 4'bx;
			axiawaddr0 <= 32'bx;
			axiawburst0 <= 2'bx;
			axiawlen0 <= 4'bx;
		end else begin
			outreq <= 0;
			case(state)
			IDLE: begin
				if(axiarvalid || axiawvalid) begin
					axiarready <= 0;
					axiawready <= 0;
				end
				if(axiarvalid) begin
					axiarburst0 <= axiarburst;
					axiarlen0 <= axiarlen;
					axiarsize0 <= axiarsize;
					axirid <= axiarid;
					if(axiawvalid) begin
						axiaraddr0 <= axiaraddr;
						rpend <= 1'b1;
					end else begin
						axiaraddr0 <= nextaddr(axiaraddr, axiarburst, axiarsize);
						outreq <= 1;
						outaddr <= axiaraddr;
						outwr <= 0;
						state <= READOUT;
						timer <= TIMEOUT;
					end
				end
				if(axiawvalid) begin
					axibresp <= OKAY;
					axiawaddr0 <= axiawaddr;
					axiawburst0 <= axiawburst;
					axiawlen0 <= axiawlen;
					axiawsize0 <= axiawsize;
					axiwready <= 1;
					axibid <= axiawid;
					state <= WAITWRDATA;
				end
			end
			READOUT: begin
				timer <= timer - 1;
				if(outack || timer == 0) begin
					axirvalid <= 1;
					axirdata <= outrdata;
					axirresp <= !outack ? DECERR : outerr ? SLVERR : OKAY;
					axirlast <= axiarlen0 == 0;
					state <= READREPLY;
				end
			end
			READREPLY:
				if(axirready) begin
					axirvalid <= 0;
					if(axirlast) begin
						state <= IDLE;
						axiarready <= 1'b1;
						axiawready <= 1'b1;
					end else begin
						state <= READOUT;
						outreq <= 1;
						outaddr <= axiaraddr0;
						axiaraddr0 <= nextaddr(axiaraddr0, axiarburst0, axiarsize0);
						axiarlen0 <= axiarlen0 - 1;
						timer <= TIMEOUT;
					end
				end
			WAITWRDATA:
				if(axiwvalid) begin
					axiwready <= 0;
					outreq <= 1;
					outaddr <= axiawaddr0;
					outwr <= 1;
					outwdata <= axiwdata;
					outwstrb <= axiwstrb;
					state <= WRITEOUT;
					axiawaddr0 <= nextaddr(axiawaddr0, axiawburst0, axiawsize0);
					timer <= TIMEOUT;
				end
			WRITEOUT: begin
				timer <= timer - 1;
				if(outack || timer == 0) begin
					if(axiwlast) begin
						axibvalid <= 1;
						state <= WRRESP;
					end else begin
						axiwready <= 1;
						state <= WAITWRDATA;
					end
					if(!outack)
						axibresp <= DECERR;
					else if(outerr)
						axibresp <= SLVERR;
				end
			end
			WRRESP:
				if(axibready) begin
					axibvalid <= 0;
					if(rpend) begin
						rpend <= 0;
						state <= READOUT;
						outreq <= 1;
						outaddr <= axiaraddr0;
						outwr <= 0;
						timer <= TIMEOUT;
					end else begin
						state <= IDLE;
						axiarready <= 1'b1;
						axiawready <= 1'b1;
					end
				end
			endcase
		end

endmodule
