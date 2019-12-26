`include "dport.vh"

module dport(
	input wire clk,
	
	input wire pixel_valid,
	input wire [15:0] pixel_data,

	input wire [7:0] reg_addr,
	output wire [31:0] reg_rdata,
	input wire [31:0] reg_wdata,
	input wire reg_wr,
	input wire reg_req,
	output wire reg_ack,
	input wire [3:0] reg_wstrb,
	output wire reg_err,

	input wire [4:0] aux_addr,
	input wire [31:0] aux_wdata,
	input wire aux_req,
	input wire aux_wr,
	output reg aux_ack,
	output reg [31:0] aux_rdata,

	input wire [1:0] refclk,
	output wire [3:0] tx,
	inout wire auxp,
	inout wire auxn,
	
	output wire dmahstart,
	output wire vblank
);

	wire auxi;

	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	wire [`ATTRMAX:0] attr;			// From regs_i of dport_regs.v
	wire [31:0]	auxctrl;		// From regs_i of dport_regs.v
	wire		auxd;			// From aux_i of dport_aux.v
	wire		auxo;			// From aux_i of dport_aux.v
	wire [31:0]	auxstat;		// From aux_i of dport_aux.v
	wire		dmastart;		// From pxclk_i of dport_pxclk.v
	wire [47:0]	dp_pixel_data;		// From pxconv_i of dport_pxconv.v
	wire		dp_pixel_ready;		// From stuff_i of dport_stuff.v
	wire		dp_pixel_valid;		// From pxconv_i of dport_pxconv.v
	wire		dpclk;			// From gtp_i of dport_gtp.v
	wire [15:0]	dpdat0;			// From stuff_i of dport_stuff.v
	wire [15:0]	dpdat1;			// From stuff_i of dport_stuff.v
	wire		dpdmahstart;		// From pxclk_i of dport_pxclk.v
	wire		dphstart;		// From pxclk_i of dport_pxclk.v
	wire [1:0]	dpisk0;			// From stuff_i of dport_stuff.v
	wire [1:0]	dpisk1;			// From stuff_i of dport_stuff.v
	wire		dpvblank;		// From pxclk_i of dport_pxclk.v
	wire		dpvstart;		// From pxclk_i of dport_pxclk.v
	wire		gtpready;		// From gtp_i of dport_gtp.v
	wire [31:0]	phyctl;			// From regs_i of dport_regs.v
	wire		raw_pixel_ready;	// From pxconv_i of dport_pxconv.v
	wire [15:0]	txdat0;			// From phy_i of dport_phy.v
	wire [15:0]	txdat1;			// From phy_i of dport_phy.v
	wire [1:0]	txisk0;			// From phy_i of dport_phy.v
	wire [1:0]	txisk1;			// From phy_i of dport_phy.v
	// End of automatics
	
	wire speed = phyctl[0];
	wire twolane = phyctl[1];
	wire [1:0] preemph0 = phyctl[5:4];
	wire [1:0] preemph1 = phyctl[7:6];
	wire [1:0] swing0 = phyctl[9:8];
	wire [1:0] swing1 = phyctl[11:10];
	wire [1:0] phymode = phyctl[13:12];
	wire [2:0] prbssel = phyctl[16:14];
	wire reset = phyctl[17];

	dport_regs regs_i(/*AUTOINST*/
			  // Outputs
			  .reg_rdata		(reg_rdata[31:0]),
			  .reg_ack		(reg_ack),
			  .reg_err		(reg_err),
			  .attr			(attr[`ATTRMAX:0]),
			  .phyctl		(phyctl[31:0]),
			  .auxctrl		(auxctrl[31:0]),
			  // Inputs
			  .clk			(clk),
			  .reg_addr		(reg_addr[7:0]),
			  .reg_wdata		(reg_wdata[31:0]),
			  .reg_wr		(reg_wr),
			  .reg_req		(reg_req),
			  .reg_wstrb		(reg_wstrb[3:0]),
			  .auxstat		(auxstat[31:0]));
	dport_aux aux_i(/*AUTOINST*/
			// Outputs
			.auxstat	(auxstat[31:0]),
			.aux_ack	(aux_ack),
			.aux_rdata	(aux_rdata[31:0]),
			.auxo		(auxo),
			.auxd		(auxd),
			// Inputs
			.clk		(clk),
			.auxctrl	(auxctrl[31:0]),
			.aux_addr	(aux_addr[4:0]),
			.aux_wdata	(aux_wdata[31:0]),
			.aux_req	(aux_req),
			.aux_wr		(aux_wr),
			.auxi		(auxi));
	
	reg fiforeset = 1'b1;
	reg [7:0] rstctr;
	always @(posedge clk) begin
		if(rstctr != 255)
			rstctr <= rstctr + 1;
		else
			fiforeset <= 1'b0;
	end
	
	wire [63:0] fifodo;
	wire fifoempty;
	wire fiforden;
	FIFO36E1 #(
		.DATA_WIDTH("18"),
		.SIM_DEVICE("7SERIES"),
		.FIFO_MODE("FIFO36_72"),
		.ALMOST_FULL_OFFSET(32),
		.FIRST_WORD_FALL_THROUGH("TRUE")
	) fifo(
		.WRCLK(clk),
		.WREN(pixel_valid),
		.DI({48'b0, pixel_data}),
		
		.RDCLK(dpclk),
		.DO(fifodo),
		.RDEN(fiforden),
		.EMPTY(fifoempty),
		.RST(fiforeset)
	);
	wire [15:0] raw_pixel_data = fifodo[15:0];
	wire raw_pixel_valid = !fifoempty && !fiforeset;
	assign fiforden = raw_pixel_valid && raw_pixel_ready;
	dport_pxconv pxconv_i(/*AUTOINST*/
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
	dport_pxclk pxclk_i(/*AUTOINST*/
			    // Outputs
			    .dphstart		(dphstart),
			    .dpvstart		(dpvstart),
			    .dmastart		(dmastart),
			    .dpvblank		(dpvblank),
			    .dpdmahstart	(dpdmahstart),
			    // Inputs
			    .dpclk		(dpclk),
			    .attr		(attr[`ATTRMAX:0]),
			    .speed		(speed),
			    .reset		(reset));
	pulse_sync sync_hstart(dpclk, clk, dpdmahstart, dmahstart);
	pulse_sync sync_vblank(dpclk, clk, dpvblank, vblank);
	dport_stuff stuff_i(/*AUTOINST*/
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
			    .attr		(attr[`ATTRMAX:0]),
			    .twolane		(twolane),
			    .speed		(speed),
			    .reset		(reset));
	wire [15:0] scrdat0, scrdat1;
	wire [1:0] scrisk0, scrisk1;
	dport_scrambler scr0(dpclk, dpdat0, dpisk0, scrdat0, scrisk0);
	dport_scrambler scr1(dpclk, dpdat1, dpisk1, scrdat1, scrisk1);
	dport_phy phy_i(/*AUTOINST*/
			// Outputs
			.txdat0		(txdat0[15:0]),
			.txdat1		(txdat1[15:0]),
			.txisk0		(txisk0[1:0]),
			.txisk1		(txisk1[1:0]),
			// Inputs
			.dpclk		(dpclk),
			.phymode	(phymode[1:0]),
			.scrdat0	(scrdat0[15:0]),
			.scrdat1	(scrdat1[15:0]),
			.scrisk0	(scrisk0[1:0]),
			.scrisk1	(scrisk1[1:0]));
	dport_gtp gtp_i(
		.refclkp(refclk),
		.tx0data(txdat0),
		.tx1data(txdat1),
		.tx0isk(txisk0),
		.tx1isk(txisk1),
		/*AUTOINST*/
			// Outputs
			.dpclk		(dpclk),
			.gtpready	(gtpready),
			.tx		(tx[3:0]),
			// Inputs
			.clk		(clk),
			.prbssel	(prbssel[2:0]),
			.speed		(speed),
			.preemph0	(preemph0[1:0]),
			.swing0		(swing0[1:0]),
			.preemph1	(preemph1[1:0]),
			.swing1		(swing1[1:0]));

	wire auxi0;
	sync auxsync(clk, !auxi0, auxi);
	PULLUP p0(.O(auxp));
	PULLDOWN p1(.O(auxn));
	IOBUFDS #(.DIFF_TERM("false"), .IOSTANDARD("BLVDS_25")) io_1(.I(!auxo), .O(auxi0), .T(auxd), .IO(auxp), .IOB(auxn));

endmodule
