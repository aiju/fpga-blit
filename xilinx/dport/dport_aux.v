`include "dport.vh"

module dport_aux(
	input wire clk,
	
	input wire [31:0] auxctrl,
	output wire [31:0] auxstat,
	
	input wire [4:0] aux_addr,
	input wire [31:0] aux_wdata,
	input wire aux_req,
	input wire aux_wr,
	output reg aux_ack,
	output wire [31:0] aux_rdata,
	
	input wire auxi,
	output reg auxo,
	output reg auxd
);

	reg auxidle = 1'b1;
	wire [4:0] auxwlen = auxctrl[4:0];
	wire auxgo = auxctrl[31] && auxidle;
	
	reg [4:0] auxrlen;
	reg auxdone;
	reg auxtimeout;

	assign auxstat = {auxidle, 22'b0, auxtimeout, 3'b0, auxrlen};
	always @(posedge clk) begin
		auxidle <= auxidle && !auxgo || auxdone;
	end
	
	reg [7:0] mem[0:31];
	
	wire memce0, memce1;
	wire memwr0, memwr1;
	wire [4:0] memaddr0, memaddr1;
	wire [7:0] memwdata0, memwdata1;
	reg [7:0] memrdata0, memrdata1;
	
	always @(posedge clk) begin
		if(memce0) begin
			if(memwr0)
				mem[memaddr0] <= memwdata0;
			else
				memrdata0 <= mem[memaddr0];
		end
	end
	always @(posedge clk) begin
		if(memce1) begin
			if(memwr1)
				mem[memaddr1] <= memwdata1;
			else
				memrdata1 <= mem[memaddr1];
		end
	end
	
	assign memce0 = aux_req;
	assign memwr0 = aux_wr;
	assign memaddr0 = aux_addr;
	assign memwdata0 = aux_wdata[{aux_addr[1:0], 3'b0} +: 8];
	assign aux_rdata = {4{memrdata0}};
	always @(posedge clk)
		aux_ack <= aux_req;

	localparam TIMEOUT = 40000;
	localparam CLKDIV = 100;
	reg [6:0] auxdiv;
	wire auxclk = auxdiv < CLKDIV/2;
	wire auxtick = auxdiv == CLKDIV-1;
	initial auxdiv = 0;
	always @(posedge clk)
		if(auxtick)
			auxdiv <= 0;
		else
			auxdiv <= auxdiv + 1;

	parameter NCO = 20;
	parameter SYSMHZ = 100;
	parameter MAXKHZ = 1500;
	parameter MINKHZ = 750;
	localparam [NCO-1:0] MAXFREQ = 4.0 * MAXKHZ * (1<<NCO) / (SYSMHZ * 1000);
	localparam [NCO-1:0] MINFREQ = 4.0 * MINKHZ * (1<<NCO) / (SYSMHZ * 1000);
	reg [NCO-1:0] rxdiv, rxdiv_, fctr, fctr_, freq, alpha, beta;
	reg carry, rxclk, rxclkup, rxclkdn, rxa, rxb;

	initial begin
		rxdiv = 0;
		rxdiv_ = 0;
		fctr = (MAXKHZ+MINKHZ)/2;
		fctr_ = (MAXKHZ+MINKHZ)/2;
		rxclk = 0;
		rxa = 0;
		rxb = 0;
		alpha = 2500;
		beta = 50;
	end
	always @(posedge clk) begin
		if(rxclkdn)
			rxa <= rxb;
		if(rxclkup)
			rxb <= auxi;
	end
	wire up = auxi ^ rxb;
	wire down = rxa ^ rxb;
	always @(posedge clk) begin
		fctr <= fctr_;
		rxdiv <= rxdiv_;
	end
	always @(*) begin
		case({up,down})
		default: begin
			fctr_ = fctr;
			freq = fctr;
		end
		2'b10: begin
			freq = fctr + alpha;
			fctr_ = fctr + beta;
			if(fctr_ > MAXFREQ) fctr_ = MAXFREQ;
		end
		2'b01: begin
			freq = fctr - alpha;
			fctr_ = fctr - beta;
			if(fctr_ < MINFREQ) fctr_ = MINFREQ;
		end
		endcase
		{carry, rxdiv_} = {1'b0, rxdiv} + {1'b0, freq};
	end
	always @(posedge clk) begin
		rxclkup <= 0;
		rxclkdn <= 0;
		if(carry) begin
			rxclk <= !rxclk;
			if(rxclk)
				rxclkdn <= 1;
			else
				rxclkup <= 1;
		end
	end
	reg [7:0] rxd;
	reg rxdok;
	wire sync = rxd == 8'b11110000;
	always @(posedge clk) begin
		rxdok <= 0;
		if(rxclkup) begin
			rxd <= {rxd[6:0], auxi};
			rxdok <= 1;
		end
	end

	reg [2:0] txstate, txstate_, rxstate;
	reg rxstart, clrctr, clridx, incidx, loadsr, loadsr0;
	reg [3:0] ctr;
	reg [4:0] idx;
	reg [7:0] sr;
	
	localparam IDLE = 0;
	localparam TXPREC = 1;
	localparam TXSYNC = 2;
	localparam TXDATA = 3;
	localparam TXEND = 4;
	localparam TXWAIT = 5;
	initial txstate = IDLE;
	reg auxgo0;
	always @(posedge clk) begin
		loadsr0 <= auxtick && loadsr;
		if(auxtick) begin
			txstate <= txstate_;
			ctr <= clrctr ? 0 : ctr + 1;
			sr <= {sr[6:0], 1'b0};
			if(clridx)
				idx <= 0;
			if(incidx)
				idx <= idx + 1;
			auxgo0 <= 1'b0;
		end
		if(auxgo)
			auxgo0 <= 1'b1;
		if(loadsr0)
			sr <= memrdata1;
	end

	always @(*) begin
		txstate_ = txstate;
		clrctr = 0;
		clridx = 0;
		incidx = 0;
		loadsr = 0;
		auxo = 0;
		auxd = 1;
		rxstart = 0;
		case(txstate)
		IDLE:
			if(auxgo0) begin
				txstate_ = TXPREC;
				clrctr = 1;
				clridx = 1;
			end
		TXPREC: begin
			auxo = !auxclk;
			auxd = 0;
			if(ctr == 15) begin
				txstate_ = TXSYNC;
				clrctr = 1;
			end
		end
		TXSYNC: begin
			auxo = !ctr[1];
			auxd = 0;
			if(ctr == 3) begin
				txstate_ = TXDATA;
				clrctr = 1;
				loadsr = 1;
			end
		end
		TXDATA: begin
			auxo = !(auxclk ^ sr[7]);
			auxd = 0;
			if(ctr == 7) begin
				loadsr = 1;
				incidx = 1;
				clrctr = 1;
				if(idx == auxwlen)
					txstate_ = TXEND;
			end
		end
		TXEND: begin
			auxo = !ctr[1];
			auxd = 0;
			if(ctr == 3) begin
				clrctr = 1;
				txstate_ = TXWAIT;
				rxstart = 1;
			end
		end
		TXWAIT:
			if(rxstate == IDLE)
				txstate_ = IDLE;
		endcase
	end
	
	reg [3:0] inv, rxctr;
	initial rxstate = IDLE;
	reg [7:0] rxsr;
	reg [4:0] rxbctr;
	reg rxbyte;
	localparam RXDELAY = 1;
	localparam RXWAIT = 2;
	localparam RXDATA = 3;
	localparam RXPARK = 4;
	reg [15:0] rxtimer;
	
	always @(posedge clk) begin
		auxdone <= 0;
		rxbyte <= 0;
		if(rxdok)
			rxctr <= sync ? 0 : rxctr + 1;
		case(rxstate)
		IDLE:
			if(rxstart) begin
				rxstate <= RXDELAY;
				rxbctr <= 0;
			end
		RXDELAY:
			if(rxdok) begin
				rxbctr <= rxbctr + 1;
				if(rxbctr == 7) begin
					rxtimer <= TIMEOUT;
					rxstate <= RXWAIT;
				end
			end
		RXWAIT:
			if(rxtimer == 0) begin
				rxstate <= IDLE;
				auxdone <= 1;
				auxtimeout <= 1;
			end else begin
				rxtimer <= rxtimer - 1;
				if(rxdok && sync) begin
					rxstate <= RXDATA;
					inv <= 0;
					rxbctr <= 0;
				end
			end
		RXDATA:
			if(rxdok) begin
				if(rxctr[0]) begin
					rxsr <= {rxsr[6:0], rxd[1]};
					if(rxd[0] == rxd[1])
						inv <= inv + 1;
				end
				if(rxctr == 15)
					rxbyte <= 1;
				if(sync) begin
					rxstate <= RXPARK;
					auxrlen <= rxbctr;
					auxdone <= 1;
					auxtimeout <= 0;
				end
			end
		RXPARK:
			if(rxdok && rxctr == 15)
				rxstate <= IDLE;
		endcase
		if(rxbyte)
			rxbctr <= rxbctr + 1;
	end
	
	assign memce1 = rxbyte || auxtick && loadsr;
	assign memwr1 = rxbyte;
	assign memaddr1 = rxbyte ? rxbctr : incidx ? idx + 1 : idx;
	assign memwdata1 = rxsr;

endmodule
