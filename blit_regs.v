module blit_regs
#(parameter HZ = 100_000_000)
(
	input wire clk,
	
	input wire regs_req,
	input wire [7:0] regs_addr,
	input wire [15:0] regs_wdata,
	input wire [1:0] regs_wstrb,
	input wire regs_we,
	output reg regs_ack,
	output reg [15:0] regs_rdata,
	
	output wire [7:1] irq,
	
	output reg bootup,
	
	output reg [17:0] daddr,
	output reg [15:0] dstat,
	
	input wire uart_in_valid,
	input wire [7:0] uart_in_data,
	output reg uart_in_ready,
	
	output reg uart_out_valid,
	output reg [7:0] uart_out_data,
	input wire uart_out_ready,
	
	input wire kbd_in_valid,
	input wire [7:0] kbd_in_data,
	output reg kbd_in_ready,
	
	output reg kbd_out_valid,
	output reg [7:0] kbd_out_data,
	input wire kbd_out_ready,
	
	input wire vblank,
	
	input wire [15:0] mouse_x,
	input wire [15:0] mouse_y,
	input wire [2:0] mouse_but
);

	localparam BAUD = 40000;
	/* verilator lint_off WIDTH */
	localparam [15:0] UARTTIM = HZ / (BAUD * 11);
	/* verilator lint_on WIDTH */
	reg [15:0] uartrxctr;
	wire uartblock = uartrxctr > 0;
	always @(posedge clk) begin
		if(uartrxctr > 0)
			uartrxctr <= uartrxctr - 1;
		if(uart_in_valid && uart_in_ready)
			uartrxctr <= UARTTIM;
	end

	wire [15:0] mask = {{8{regs_wstrb[1]}}, {8{regs_wstrb[0]}}};
	
	reg [7:0] uartctrl;
	wire uart_rx_irq_en = uartctrl[7];
	wire uart_tx_irq_en = uartctrl[6:5] == 2'b01;
	wire [7:0] uartstat = {6'b0, !uart_out_valid || uart_out_ready, uart_in_valid && !uartblock};

	reg [7:0] kbdctrl;
	wire kbd_rx_irq_en = kbdctrl[7];
	wire kbd_tx_irq_en = kbdctrl[6:5] == 2'b01;
	wire [7:0] kbdstat = {6'b0, !kbd_out_valid || kbd_out_ready, kbd_in_valid};
	
	reg vblank_irq;
	reg mouse_irq;
	assign irq = {
		1'b0,
		1'b0,
		uart_tx_irq_en && (!uart_out_valid || uart_out_ready) || uart_rx_irq_en && uart_in_valid && !uartblock,
		mouse_irq,
		1'b0,
		kbd_tx_irq_en && (!kbd_out_valid || kbd_out_ready) || kbd_rx_irq_en && kbd_in_valid,
		vblank_irq
	};
	
	initial bootup = 1'b1;

	reg [2:0] mouse_but_last;
	always @(posedge clk) begin
		if(uart_out_valid && uart_out_ready)
			uart_out_valid <= 1'b0;
		if(kbd_out_valid && kbd_out_ready)
			kbd_out_valid <= 1'b0;
		regs_ack <= regs_req;
		uart_in_ready <= 1'b0;
		kbd_in_ready <= 1'b0;
		
		if(regs_req && regs_we) begin
			case(regs_addr)
			8'o10: uartctrl <= regs_wdata[7:0];
			8'o12: begin
				if(!uart_out_valid || uart_out_ready) begin
					uart_out_valid <= 1'b1;
					uart_out_data <= regs_wdata[7:0];
				end
			end
			8'o30: daddr[17:2] <= daddr[17:2] & ~mask | regs_wdata & mask;
			8'o40: dstat <= dstat & ~mask | dstat & mask;
			8'o24, 8'o26: begin end
			8'o60: kbdctrl <= regs_wdata[7:0];
			8'o62: begin
				if(!kbd_out_valid || kbd_out_ready) begin
					kbd_out_valid <= 1'b1;
					kbd_out_data <= regs_wdata[7:0];
				end
			end
			8'o70: vblank_irq <= 1'b0;
			default: $display("write to %o = %h", regs_addr, regs_wdata);
			endcase
		end else if(regs_req && !regs_we) begin
			case(regs_addr)
			8'o0: regs_rdata <= mouse_y;
			8'o2: regs_rdata <= mouse_x;
			8'o10: regs_rdata <= {2{uartstat}};
			8'o12: begin
				regs_rdata <= {2{uart_in_data}};
				uart_in_ready <= !uartblock;
			end
			8'o20, 8'o24: begin
				mouse_irq <= 1'b0;
				regs_rdata <= {2{{5'b0, mouse_but}}};
			end
			8'o30: regs_rdata <= daddr[17:2];
			8'o40: regs_rdata <= dstat;
			8'o60: regs_rdata <= {2{kbdstat}};
			8'o62: begin
				regs_rdata <= {2{kbd_in_data}};
				kbd_in_ready <= 1'b1;
			end
			8'o26: begin end
			default: $display("read from %o", regs_addr);
			endcase
		end
		if(vblank) vblank_irq <= 1'b1;
		if(mouse_but != mouse_but_last) mouse_irq <= 1'b1;
		mouse_but_last <= mouse_but;
	end

endmodule
