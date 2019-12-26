module dport_pxconv_tb;
	
	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	wire [17:0]	dma_addr;		// From blit_disp_i of blit_disp.v
	wire		dma_req;		// From blit_disp_i of blit_disp.v
	wire		dmastart;		// From dport_pxclk_i of dport_pxclk.v
	wire [47:0]	dp_pixel_data;		// From dport_pxconv_i of dport_pxconv.v
	wire		dp_pixel_ready;		// From dport_stuff_i of dport_stuff.v
	wire		dp_pixel_valid;		// From dport_pxconv_i of dport_pxconv.v
	wire [15:0]	dpdat0;			// From dport_stuff_i of dport_stuff.v
	wire [15:0]	dpdat1;			// From dport_stuff_i of dport_stuff.v
	wire		dpdmahstart;		// From dport_pxclk_i of dport_pxclk.v
	wire		dphstart;		// From dport_pxclk_i of dport_pxclk.v
	wire [1:0]	dpisk0;			// From dport_stuff_i of dport_stuff.v
	wire [1:0]	dpisk1;			// From dport_stuff_i of dport_stuff.v
	wire		dpvblank;		// From dport_pxclk_i of dport_pxclk.v
	wire		dpvstart;		// From dport_pxclk_i of dport_pxclk.v
	wire [15:0]	pixel_data;		// From blit_disp_i of blit_disp.v
	wire		pixel_valid;		// From blit_disp_i of blit_disp.v
	wire [15:0]	raw_pixel_data;		// From fifo_i of fifo.v
	wire		raw_pixel_ready;	// From dport_pxconv_i of dport_pxconv.v
	wire		raw_pixel_valid;	// From fifo_i of fifo.v
	// End of automatics
	/*AUTOREGINPUT*/
	// Beginning of automatic reg inputs (for undeclared instantiated-module inputs)
	reg [17:0]	daddr;			// To blit_disp_i of blit_disp.v
	reg		dma_ack;		// To blit_disp_i of blit_disp.v
	reg [15:0]	dma_rdata;		// To blit_disp_i of blit_disp.v
	reg [15:0]	dstat;			// To blit_disp_i of blit_disp.v
	reg		reset;			// To dport_pxclk_i of dport_pxclk.v, ...
	// End of automatics
	
	wire twolane = 1'b1;
	wire speed = 1'b1;
	wire [`ATTRMAX:0] attr = {
		17'd26426, 24'd62, 24'd25,
		17'd44049, 24'd61, 24'd41,
		16'h21,
		16'd352, 16'd35,
		16'd136, 16'd3,
		16'd1712, 16'd1060,
		16'd1280, 16'd1024
	};
	
	reg dpclk;
	initial dpclk = 1'b0;
	always #3.7 dpclk <= !dpclk;
	
	reg clk;
	initial clk = 1'b0;
	always #5 clk <= !clk;
	
	initial #300000 $finish;
	
	initial begin
		$dumpfile("test.vcd");
		$dumpvars;
	end
	
	initial begin
		reset = 1'b1;
		#1000 reset = 1'b0;
		#80000 reset = 1'b1;
		#90000 reset = 1'b0;
	end
	
	always @(posedge clk) begin
		dma_ack <= dma_req;
		dma_rdata <= dma_addr;
	end
	
	wire dmahstart;
	pulse_sync sync_dmahstart(dpclk, clk, dpdmahstart, dmahstart);
	wire vblank;
	pulse_sync sync_dmavblank(dpclk, clk, dpvblank, vblank);
	blit_disp blit_disp_i(/*AUTOINST*/
			      // Outputs
			      .dma_req		(dma_req),
			      .dma_addr		(dma_addr[17:0]),
			      .pixel_valid	(pixel_valid),
			      .pixel_data	(pixel_data[15:0]),
			      // Inputs
			      .clk		(clk),
			      .daddr		(daddr[17:0]),
			      .dstat		(dstat[15:0]),
			      .dma_ack		(dma_ack),
			      .dma_rdata	(dma_rdata[15:0]),
			      .dmahstart	(dmahstart),
			      .vblank		(vblank));
	fifo fifo_i(/*AUTOINST*/
		    // Outputs
		    .raw_pixel_valid	(raw_pixel_valid),
		    .raw_pixel_data	(raw_pixel_data[15:0]),
		    // Inputs
		    .clk		(clk),
		    .dpclk		(dpclk),
		    .pixel_valid	(pixel_valid),
		    .pixel_data		(pixel_data[15:0]),
		    .raw_pixel_ready	(raw_pixel_ready));
	dport_pxclk dport_pxclk_i(/*AUTOINST*/
				  // Outputs
				  .dphstart		(dphstart),
				  .dpvstart		(dpvstart),
				  .dmastart		(dmastart),
				  .dpvblank		(dpvblank),
				  .dpdmahstart		(dpdmahstart),
				  // Inputs
				  .dpclk		(dpclk),
				  .attr			(attr[`ATTRMAX:0]),
				  .speed		(speed),
				  .reset		(reset));
	dport_pxconv dport_pxconv_i(/*AUTOINST*/
				    // Outputs
				    .raw_pixel_ready	(raw_pixel_ready),
				    .dp_pixel_valid	(dp_pixel_valid),
				    .dp_pixel_data	(dp_pixel_data[47:0]),
				    // Inputs
				    .dpclk		(dpclk),
				    .reset		(reset),
				    .raw_pixel_valid	(raw_pixel_valid),
				    .raw_pixel_data	(raw_pixel_data[15:0]),
				    .dp_pixel_ready	(dp_pixel_ready),
				    .dpdmahstart	(dpdmahstart));

	dport_stuff dport_stuff_i(/*AUTOINST*/
				  // Outputs
				  .dp_pixel_ready	(dp_pixel_ready),
				  .dpdat0		(dpdat0[15:0]),
				  .dpdat1		(dpdat1[15:0]),
				  .dpisk0		(dpisk0[1:0]),
				  .dpisk1		(dpisk1[1:0]),
				  // Inputs
				  .dpclk		(dpclk),
				  .dp_pixel_valid	(dp_pixel_valid),
				  .dp_pixel_data	(dp_pixel_data[47:0]),
				  .dphstart		(dphstart),
				  .dpvstart		(dpvstart),
				  .dmastart		(dmastart),
				  .attr			(attr[`ATTRMAX:0]),
				  .twolane		(twolane),
				  .speed		(speed),
				  .reset		(reset));

endmodule

module fifo(
	input wire clk,
	input wire dpclk,
	
	input wire pixel_valid,
	input wire [15:0] pixel_data,
	
	output wire raw_pixel_valid,
	output wire [15:0] raw_pixel_data,
	input wire raw_pixel_ready
);

	localparam FIFOLEN = 512;
	
	reg [15:0] fifo[0:FIFOLEN-1];
	reg [31:0] wrptr, rdptr;
	
	always @(posedge clk) begin
		if(pixel_valid && (wrptr + 1) % FIFOLEN != rdptr) begin
			fifo[wrptr] <= pixel_data;
			wrptr <= (wrptr + 1) % FIFOLEN;
		end
	end
	
	assign raw_pixel_valid = rdptr != wrptr;
	assign raw_pixel_data = fifo[rdptr];
	always @(posedge dpclk) begin
		if(raw_pixel_valid && raw_pixel_ready) begin
			rdptr <= (rdptr + 1) % FIFOLEN;
		end
	end

endmodule

// Local Variables:
// verilog-library-directories:("." "../..")
// End:
