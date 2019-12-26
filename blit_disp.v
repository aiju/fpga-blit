module blit_disp
(
	input wire clk,
	
	input wire [17:0] daddr,
	input wire [15:0] dstat,
	
	output reg dma_req,
	output reg [17:0] dma_addr,
	input wire dma_ack,
	input wire [15:0] dma_rdata,
	
	input wire dmahstart,
	input wire vblank,
	
	output reg pixel_valid,
	output reg [15:0] pixel_data
);

	reg [7:0] ctr;
	reg dma_issued;

	always @(posedge clk) begin
		if(vblank) begin
			dma_addr <= daddr;
		end
		if(dmahstart) begin
			ctr <= 50;
		end
		dma_req <= 1'b0;
		pixel_valid <= 1'b0;
		if(!dma_issued) begin
			if(ctr > 0) begin
				dma_req <= 1'b1;
				dma_issued <= 1'b1;
			end
		end else begin
			if(dma_ack) begin
				dma_issued <= 1'b0;
				pixel_valid <= 1'b1;
				pixel_data <= dma_rdata;
				dma_addr <= dma_addr + 2;
				ctr <= ctr - 1;
			end
		end
	end
	
endmodule
