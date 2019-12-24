module blit_ramarb(
	input wire clk,
	
	input wire cpu_ram_req,
	input wire [17:0] cpu_ram_addr,
	input wire [15:0] cpu_ram_wdata,
	input wire [1:0] cpu_ram_wstrb,
	input wire cpu_ram_we,
	output wire cpu_ram_ack,
	output wire [15:0] cpu_ram_rdata,

	input wire dma_req,
	input wire [17:0] dma_addr,
	output reg dma_ack,
	output reg [15:0] dma_rdata,

	output reg ram_req,
	output reg [17:0] ram_addr,
	output wire [15:0] ram_wdata,
	output wire [1:0] ram_wstrb,
	output reg ram_we,
	input wire ram_ack,
	input wire [15:0] ram_rdata
);

	assign ram_wdata = cpu_ram_wdata;
	assign ram_wstrb = cpu_ram_wstrb;

	reg cpu_ram_pending, cpu_ram_issued;
	reg dma_pending, dma_issued;
	
	always @(posedge clk) begin
		ram_req <= 1'b0;
		cpu_ram_ack <= 1'b0;
		dma_ack <= 1'b0;

		if(dma_req || dma_pending) begin
			if(cpu_ram_issued || dma_issued) begin
				dma_pending <= 1'b1;
			end else begin
				ram_req <= 1'b1;
				ram_addr <= dma_addr;
				ram_we <= 1'b0;
				dma_issued <= 1'b1;
				dma_pending <= 1'b0;
			end
		end
		if(cpu_ram_req || cpu_ram_pending) begin
			if(dma_req || dma_pending || cpu_ram_issued || dma_issued) begin
				cpu_ram_pending <= 1'b1;
			end else begin
				ram_req <= 1'b1;
				ram_addr <= cpu_ram_addr;
				ram_we <= cpu_ram_we;
				cpu_ram_issued <= 1'b1;
				cpu_ram_pending <= 1'b0;
			end
		end
		if(ram_ack) begin
			if(cpu_ram_issued) begin
				cpu_ram_ack <= 1'b1;
				cpu_ram_rdata <= ram_rdata;
				cpu_ram_issued <= 1'b0;
			end
			if(dma_issued) begin
				dma_ack <= 1'b1;
				dma_rdata <= ram_rdata;
				dma_issued <= 1'b0;
			end
		end
	end

endmodule
