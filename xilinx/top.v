module top(
	input wire [1:0]  refclk,
	inout wire  auxn,
	inout wire  auxp,
	output wire [3:0]  tx
);

	wire [15:0]  mouse_x;
	wire  _outerr;
	wire [15:0]  mouse_y;
	wire  _gp0aclk;
	wire [2:0]  _gp0awprot;
	wire [11:0]  _gp0arid;
	wire [11:0]  _gp0awid;
	wire  dport_reg_wr;
	wire [15:0]  pixel_data;
	wire  dport_aux_wr;
	wire [3:0]  _gp0arcache;
	wire [31:0]  dport_reg_rdata;
	wire [3:0]  _gp0awcache;
	wire [31:0]  dport_reg_wdata;
	wire [7:0]  reg_addr;
	wire [3:0]  _fclkclk;
	wire  clk = _fclkclk[0];
	wire [31:0]  dport_aux_rdata;
	wire [31:0]  dport_aux_wdata;
	wire  _gp0arvalid;
	wire  _gp0arready;
	wire  _gp0awvalid;
	wire [31:0]  _outaddr;
	wire  _gp0awready;
	wire  dport_reg_ack;
	wire  kbd_out_valid;
	wire [3:0]  dport_reg_wstrb;
	wire  kbd_out_ready;
	wire [7:0]  kbd_in_data;
	wire  uart_in_valid;
	wire  dport_aux_ack;
	wire  uart_in_ready;
	wire [1:0]  _gp0arburst;
	wire [7:0]  uart_out_data;
	wire  dport_reg_req;
	wire  dport_reg_err;
	wire [1:0]  _gp0awburst;
	wire [31:0]  _gp0rdata;
	wire [31:0]  _gp0wdata;
	wire [3:0]  _gp0arlen;
	wire  dport_aux_req;
	wire [3:0]  _gp0awlen;
	wire  vblank;
	wire [1:0]  _gp0bresp;
	wire  reg_wr;
	wire [3:0]  _gp0arqos;
	wire  _gp0rlast;
	wire [3:0]  _fclkresetn;
	wire  pixel_valid;
	wire [3:0]  _gp0awqos;
	wire  _gp0wlast;
	wire [1:0]  _gp0rresp;
	wire [11:0]  _gp0bid;
	wire [3:0]  _gp0wstrb;
	wire  _outwr;
	wire [11:0]  _gp0rid;
	wire [31:0]  reg_rdata;
	wire [11:0]  _gp0wid;
	wire [31:0]  reg_wdata;
	wire [7:0]  dport_reg_addr;
	wire [31:0]  _outrdata;
	wire  rstn = _fclkresetn[0];
	wire  dmahstart;
	wire [31:0]  _outwdata;
	wire  reg_ack;
	wire [4:0]  dport_aux_addr;
	wire [2:0]  mouse_but;
	wire [31:0]  _gp0araddr;
	wire  kbd_in_valid;
	wire  _gp0bvalid;
	wire [31:0]  _gp0awaddr;
	wire  kbd_in_ready;
	wire  _gp0bready;
	wire [7:0]  kbd_out_data;
	wire  uart_out_valid;
	wire [1:0]  _gp0arlock;
	wire  reg_req;
	wire  reg_err;
	wire  _outack;
	wire  uart_out_ready;
	wire [1:0]  _gp0awlock;
	wire  _gp0rvalid;
	wire [3:0]  _outwstrb;
	wire [7:0]  uart_in_data;
	wire  _gp0rready;
	wire  _gp0wvalid;
	wire  _gp0wready;
	wire [1:0]  _gp0arsize;
	wire [1:0]  _gp0awsize;
	wire [2:0]  _gp0arprot;
	wire  _outreq;

	regs regs0(
		.reg_req(reg_req),
		.reg_wr(reg_wr),
		.reg_addr(reg_addr),
		.reg_wdata(reg_wdata),
		.reg_ack(reg_ack),
		.reg_rdata(reg_rdata),
		.reg_err(reg_err),
		.clk(clk),
		.uart_in_valid(uart_in_valid),
		.uart_in_data(uart_in_data),
		.uart_in_ready(uart_in_ready),
		.uart_out_valid(uart_out_valid),
		.uart_out_data(uart_out_data),
		.uart_out_ready(uart_out_ready),
		.kbd_in_valid(kbd_in_valid),
		.kbd_in_data(kbd_in_data),
		.kbd_in_ready(kbd_in_ready),
		.kbd_out_valid(kbd_out_valid),
		.kbd_out_data(kbd_out_data),
		.kbd_out_ready(kbd_out_ready),
		.mouse_x(mouse_x),
		.mouse_y(mouse_y),
		.mouse_but(mouse_but)
	);
	blit blit0(
		.clk(clk),
		.uart_in_valid(uart_in_valid),
		.uart_in_data(uart_in_data),
		.uart_in_ready(uart_in_ready),
		.uart_out_valid(uart_out_valid),
		.uart_out_data(uart_out_data),
		.uart_out_ready(uart_out_ready),
		.kbd_in_valid(kbd_in_valid),
		.kbd_in_data(kbd_in_data),
		.kbd_in_ready(kbd_in_ready),
		.kbd_out_valid(kbd_out_valid),
		.kbd_out_data(kbd_out_data),
		.kbd_out_ready(kbd_out_ready),
		.pixel_valid(pixel_valid),
		.pixel_data(pixel_data),
		.dmahstart(dmahstart),
		.vblank(vblank),
		.mouse_x(mouse_x),
		.mouse_y(mouse_y),
		.mouse_but(mouse_but)
	);
	dport dport0(
		.reg_addr(dport_reg_addr),
		.reg_rdata(dport_reg_rdata),
		.reg_wdata(dport_reg_wdata),
		.reg_wr(dport_reg_wr),
		.reg_req(dport_reg_req),
		.reg_ack(dport_reg_ack),
		.reg_wstrb(dport_reg_wstrb),
		.reg_err(dport_reg_err),
		.aux_addr(dport_aux_addr),
		.aux_wdata(dport_aux_wdata),
		.aux_req(dport_aux_req),
		.aux_wr(dport_aux_wr),
		.aux_ack(dport_aux_ack),
		.aux_rdata(dport_aux_rdata),
		.refclk(refclk),
		.auxp(auxp),
		.auxn(auxn),
		.tx(tx),
		.clk(clk),
		.pixel_valid(pixel_valid),
		.pixel_data(pixel_data),
		.dmahstart(dmahstart),
		.vblank(vblank)
	);
	_intercon _intercon(
		.clk(clk),
		.rstn(rstn),
		.reg_req(reg_req),
		.reg_ack(reg_ack),
		.reg_addr(reg_addr),
		.reg_rdata(reg_rdata),
		.reg_wdata(reg_wdata),
		.reg_wr(reg_wr),
		.reg_err(reg_err),
		.dport_reg_req(dport_reg_req),
		.dport_reg_ack(dport_reg_ack),
		.dport_reg_addr(dport_reg_addr),
		.dport_reg_rdata(dport_reg_rdata),
		.dport_reg_wdata(dport_reg_wdata),
		.dport_reg_wr(dport_reg_wr),
		.dport_reg_err(dport_reg_err),
		.dport_reg_wstrb(dport_reg_wstrb),
		.dport_aux_req(dport_aux_req),
		.dport_aux_ack(dport_aux_ack),
		.dport_aux_addr(dport_aux_addr),
		.dport_aux_rdata(dport_aux_rdata),
		.dport_aux_wdata(dport_aux_wdata),
		.dport_aux_wr(dport_aux_wr),
		._outaddr(_outaddr),
		._outrdata(_outrdata),
		._outwdata(_outwdata),
		._outwr(_outwr),
		._outreq(_outreq),
		._outack(_outack),
		._outerr(_outerr),
		._outwstrb(_outwstrb)
	);
	axi3 _axi3(
		.axiaclk(_gp0aclk),
		.axiarvalid(_gp0arvalid),
		.axiarready(_gp0arready),
		.axiaraddr(_gp0araddr),
		.axiarburst(_gp0arburst),
		.axiarlock(_gp0arlock),
		.axiarsize(_gp0arsize),
		.axiarprot(_gp0arprot),
		.axiarlen(_gp0arlen),
		.axiarcache(_gp0arcache),
		.axiarqos(_gp0arqos),
		.axiarid(_gp0arid),
		.axirvalid(_gp0rvalid),
		.axirready(_gp0rready),
		.axirdata(_gp0rdata),
		.axirid(_gp0rid),
		.axirresp(_gp0rresp),
		.axirlast(_gp0rlast),
		.axiawvalid(_gp0awvalid),
		.axiawready(_gp0awready),
		.axiawaddr(_gp0awaddr),
		.axiawburst(_gp0awburst),
		.axiawlock(_gp0awlock),
		.axiawsize(_gp0awsize),
		.axiawprot(_gp0awprot),
		.axiawlen(_gp0awlen),
		.axiawcache(_gp0awcache),
		.axiawqos(_gp0awqos),
		.axiawid(_gp0awid),
		.axiwvalid(_gp0wvalid),
		.axiwready(_gp0wready),
		.axiwdata(_gp0wdata),
		.axiwstrb(_gp0wstrb),
		.axiwid(_gp0wid),
		.axiwlast(_gp0wlast),
		.axibvalid(_gp0bvalid),
		.axibready(_gp0bready),
		.axibresp(_gp0bresp),
		.axibid(_gp0bid),
		.outaddr(_outaddr),
		.outrdata(_outrdata),
		.outwdata(_outwdata),
		.outwr(_outwr),
		.outreq(_outreq),
		.outack(_outack),
		.outerr(_outerr),
		.outwstrb(_outwstrb),
		.clk(clk),
		.rstn(rstn)
	);
	(* DONT_TOUCH="YES" *)PS7 _PS7(
		.MAXIGP0ACLK(_gp0aclk),
		.MAXIGP0AWPROT(_gp0awprot),
		.MAXIGP0ARID(_gp0arid),
		.MAXIGP0AWID(_gp0awid),
		.MAXIGP0ARCACHE(_gp0arcache),
		.MAXIGP0AWCACHE(_gp0awcache),
		.FCLKCLK(_fclkclk),
		.MAXIGP0ARVALID(_gp0arvalid),
		.MAXIGP0ARREADY(_gp0arready),
		.MAXIGP0AWVALID(_gp0awvalid),
		.MAXIGP0AWREADY(_gp0awready),
		.MAXIGP0ARBURST(_gp0arburst),
		.MAXIGP0AWBURST(_gp0awburst),
		.MAXIGP0RDATA(_gp0rdata),
		.MAXIGP0WDATA(_gp0wdata),
		.MAXIGP0ARLEN(_gp0arlen),
		.MAXIGP0AWLEN(_gp0awlen),
		.MAXIGP0BRESP(_gp0bresp),
		.MAXIGP0ARQOS(_gp0arqos),
		.MAXIGP0RLAST(_gp0rlast),
		.FCLKRESETN(_fclkresetn),
		.MAXIGP0AWQOS(_gp0awqos),
		.MAXIGP0WLAST(_gp0wlast),
		.MAXIGP0RRESP(_gp0rresp),
		.MAXIGP0BID(_gp0bid),
		.MAXIGP0WSTRB(_gp0wstrb),
		.MAXIGP0RID(_gp0rid),
		.MAXIGP0WID(_gp0wid),
		.MAXIGP0ARADDR(_gp0araddr),
		.MAXIGP0BVALID(_gp0bvalid),
		.MAXIGP0AWADDR(_gp0awaddr),
		.MAXIGP0BREADY(_gp0bready),
		.MAXIGP0ARLOCK(_gp0arlock),
		.MAXIGP0AWLOCK(_gp0awlock),
		.MAXIGP0RVALID(_gp0rvalid),
		.MAXIGP0RREADY(_gp0rready),
		.MAXIGP0WVALID(_gp0wvalid),
		.MAXIGP0WREADY(_gp0wready),
		.MAXIGP0ARSIZE(_gp0arsize),
		.MAXIGP0AWSIZE(_gp0awsize),
		.MAXIGP0ARPROT(_gp0arprot)
	);
endmodule

module _intercon(
	input wire clk,
	input wire rstn,
	input wire _outreq,
	input wire _outwr,
	output reg _outack,
	output reg _outerr,
	input wire [31:0] _outaddr,
	input wire [3:0] _outwstrb,
	input wire [31:0] _outwdata,
	output reg [31:0] _outrdata,
	output reg reg_req,
	input wire reg_ack,
	input wire reg_err,
	output wire [31:0] reg_addr,
	input wire [31:0] reg_rdata,
	output wire [31:0] reg_wdata,
	output wire reg_wr,
	output reg dport_reg_req,
	input wire dport_reg_ack,
	input wire dport_reg_err,
	output wire [31:0] dport_reg_addr,
	input wire [31:0] dport_reg_rdata,
	output wire [31:0] dport_reg_wdata,
	output wire dport_reg_wr,
	output wire [3:0] dport_reg_wstrb,
	output reg dport_aux_req,
	input wire dport_aux_ack,
	output wire [31:0] dport_aux_addr,
	input wire [31:0] dport_aux_rdata,
	output wire [31:0] dport_aux_wdata,
	output wire dport_aux_wr
);

	localparam IDLE = 0;
	localparam WAIT_reg_ = 1;
	localparam WAIT_dport_reg_ = 2;
	localparam WAIT_dport_aux_ = 3;
	reg [2:0] state;

	always @(posedge clk or negedge rstn)
		if(!rstn) begin
			state <= IDLE;
			_outack <= 1'b0;
			_outerr <= 1'b0;
			_outrdata <= 32'bx;
			reg_req <= 1'b0;
			dport_reg_req <= 1'b0;
			dport_aux_req <= 1'b0;
		end else begin
			_outack <= 1'b0;
			reg_req <= 1'b0;
			dport_reg_req <= 1'b0;
			dport_aux_req <= 1'b0;
			case(state)
			IDLE:
				if(_outreq)
					casez(_outaddr[29:0])
					default: begin
						_outack <= 1'b1;
						_outerr <= 1'b1;
					end
					30'b00000000000000000000000zzzzzzzz: begin
						state <= WAIT_reg_;
						reg_req <= 1'b1;
					end
					30'b00000000000000000000001zzzzzzzz: begin
						state <= WAIT_dport_reg_;
						dport_reg_req <= 1'b1;
					end
					30'b00000000000000000000010000zzzzz: begin
						state <= WAIT_dport_aux_;
						dport_aux_req <= 1'b1;
					end
					endcase
			WAIT_reg_:
				if(reg_ack) begin
					state <= IDLE;
					_outack <= 1'b1;
					_outerr <= reg_err;
					_outrdata <= reg_rdata;
				end
			WAIT_dport_reg_:
				if(dport_reg_ack) begin
					state <= IDLE;
					_outack <= 1'b1;
					_outerr <= dport_reg_err;
					_outrdata <= dport_reg_rdata;
				end
			WAIT_dport_aux_:
				if(dport_aux_ack) begin
					state <= IDLE;
					_outack <= 1'b1;
					_outerr <= 1'b0;
					_outrdata <= dport_aux_rdata;
				end
			endcase
		end

	assign reg_addr = _outaddr;
	assign reg_wr = _outwr;
	assign reg_wdata = _outwdata;
	assign dport_reg_addr = _outaddr;
	assign dport_reg_wr = _outwr;
	assign dport_reg_wdata = _outwdata;
	assign dport_reg_wstrb = _outwstrb;
	assign dport_aux_addr = _outaddr;
	assign dport_aux_wr = _outwr;
	assign dport_aux_wdata = _outwdata;
endmodule
