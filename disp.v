module disp
#(parameter HZ = 100_000_000)
(
	input wire clk,
	
	input wire [17:0] daddr,
	input wire [15:0] dstat,
	
	output reg dma_req,
	output reg [17:0] dma_addr,
	input wire dma_ack,
	input wire [15:0] dma_rdata,
	
	output wire vblank,
	
	output reg pixel_valid,
	output reg pixel
);

	localparam HTOT = 880;
	localparam HACT = 800;
	localparam HSTART = 40;
	localparam VTOT = 1026;
	localparam VACT = 1024;
	localparam VSTART = 1;
	localparam FPS = 60;
	localparam PXHZ = HTOT * VTOT * FPS;
	/* verilator lint_off WIDTH */
	/* verilator lint_off WIDTHCONCAT */
	localparam [15:0] PXDIV = {PXHZ, 16'b0} / HZ;
	/* verilator lint_on WIDTH */
	/* verilator lint_on WIDTHCONCAT */

	reg dma_active;
	reg dma_issued;
	reg [17:0] dma_ctr;

	localparam FIFOLEN = 4;
	localparam FIFOBITS = $clog2(FIFOLEN);
	reg [15:0] fifo[0:FIFOLEN - 1];
	reg fifo_full = 1'b0;
	reg fifo_empty = 1'b1;
	reg [FIFOBITS-1:0] fifo_al_ptr = 0;
	reg [FIFOBITS-1:0] fifo_wr_ptr = 0;
	reg [FIFOBITS-1:0] fifo_rd_ptr = 0;
	
	wire fifo_alloc = !fifo_full && dma_active && !dma_issued;
	wire fifo_write = dma_ack;
	wire fifo_read;
	
	wire [FIFOBITS-1:0] fifo_al_ptr_inc = fifo_al_ptr + 1;
	wire [FIFOBITS-1:0] fifo_wr_ptr_inc = fifo_wr_ptr + 1;
	wire [FIFOBITS-1:0] fifo_rd_ptr_inc = fifo_rd_ptr + 1;
	
	always @(posedge clk) begin
		if(fifo_alloc) begin
			fifo_al_ptr <= fifo_al_ptr_inc;
			if(!fifo_read && fifo_al_ptr_inc == fifo_rd_ptr)
				fifo_full <= 1'b1;
		end
		if(fifo_write) begin
			fifo_wr_ptr <= fifo_wr_ptr_inc;
			fifo_empty <= 1'b0;
		end
		if(fifo_read) begin
			fifo_rd_ptr <= fifo_rd_ptr_inc;
			if(!fifo_write && fifo_rd_ptr_inc == fifo_wr_ptr)
				fifo_empty <= 1'b1;
			fifo_full <= 1'b0;
		end
	end
	
	always @(posedge clk)
		if(fifo_write)
			fifo[fifo_wr_ptr] <= dma_rdata;
	
	reg [15:0] pxdiv;
	reg [15:0] x, y, x_nxt, y_nxt;
	reg pxclk;
	
	reg [15:0] sr;
	reg [4:0] sr_rem;
	reg [15:0] sr_ctr;
	
	reg hactive, vactive;
	wire active = hactive && vactive;
	
	always @(*) begin
		if(x == HTOT - 1) begin
			x_nxt = 0;
			if(y == VTOT - 1)
				y_nxt = 0;
			else
				y_nxt = y + 1;
		end else begin
			x_nxt = x + 1;
			y_nxt = y;
		end
	end

	always @(posedge clk) begin
		vblank <= 1'b0;
		{pxclk, pxdiv} <= pxdiv + PXDIV;
		if(pxclk) begin
			if(y == VSTART + VACT && x == 0) begin
				vblank <= 1'b1;
			end
			x <= x_nxt;
			y <= y_nxt;
			if(y_nxt == VSTART) vactive <= 1'b1;
			if(y_nxt == VSTART + VACT) vactive <= 1'b0;
			if(x_nxt == HSTART) hactive <= 1'b1;
			if(x_nxt == HSTART + HACT) hactive <= 1'b0;
		end
	end

	always @(posedge clk) begin
		dma_req <= 1'b0;
		if(fifo_alloc) begin
			dma_req <= 1'b1;
			dma_issued <= 1'b1;
		end 
		if(dma_ack) begin
			dma_issued <= 1'b0;
			dma_addr <= dma_addr + 2;
			dma_ctr <= dma_ctr - 1;
			if(dma_ctr == 1)
				dma_active <= 0;
		end
		if(pxclk) begin
			if(x == 0 && y == 0) begin
				dma_addr <= daddr;
			end
			if(x == 0 && vactive) begin
				dma_active <= 1'b1;
				dma_ctr <= HACT / 16;
				sr_ctr <= HACT / 16;
			end
		end
	end
	


	assign fifo_read = (sr_rem == 0 || sr_rem == 1 && pxclk && active) && sr_ctr > 0 && !fifo_empty;
	always @(posedge clk) begin
		pixel_valid <= 1'b0;
		if(sr_rem > 0 && pxclk && active) begin
			sr_rem <= sr_rem - 1;
			sr <= sr << 1;
			pixel_valid <= 1'b1;
			pixel <= sr[15];
		end
		if(fifo_read) begin
			sr <= fifo[fifo_rd_ptr];
			sr_rem <= 16;
			sr_ctr <= sr_ctr - 1;
		end
	end
	
endmodule
