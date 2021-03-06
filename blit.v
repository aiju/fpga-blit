module blit
#(parameter HZ = 100_000_000)
(
	input wire clk,

	input wire uart_in_valid,
	input wire [7:0] uart_in_data,
	output wire uart_in_ready,
	
	output wire uart_out_valid,
	output wire [7:0] uart_out_data,
	input wire uart_out_ready,
	
	input wire kbd_in_valid,
	input wire [7:0] kbd_in_data,
	output wire kbd_in_ready,
	
	output wire kbd_out_valid,
	output wire [7:0] kbd_out_data,
	input wire kbd_out_ready,
	
	output wire pixel_valid,
	output wire [15:0] pixel_data,
	
	input wire dmahstart,
	input wire vblank,
	
	input wire [15:0] mouse_x,
	input wire [15:0] mouse_y,
	input wire [2:0] mouse_but
);

	/*AUTOWIRE*/
	// Beginning of automatic wires (for undeclared instantiated-module outputs)
	wire		bootup;			// From regs_i of blit_regs.v
	wire		cpu_ack;		// From bus_i of blit_bus.v
	wire [23:0]	cpu_addr;		// From cpu_i of blit_cpu.v
	wire		cpu_err;		// From bus_i of blit_bus.v
	wire		cpu_ram_ack;		// From ramarb_i of blit_ramarb.v
	wire [17:0]	cpu_ram_addr;		// From bus_i of blit_bus.v
	wire [15:0]	cpu_ram_rdata;		// From ramarb_i of blit_ramarb.v
	wire		cpu_ram_req;		// From bus_i of blit_bus.v
	wire [15:0]	cpu_ram_wdata;		// From bus_i of blit_bus.v
	wire		cpu_ram_we;		// From bus_i of blit_bus.v
	wire [1:0]	cpu_ram_wstrb;		// From bus_i of blit_bus.v
	wire [15:0]	cpu_rdata;		// From bus_i of blit_bus.v
	wire		cpu_req;		// From cpu_i of blit_cpu.v
	wire [15:0]	cpu_wdata;		// From cpu_i of blit_cpu.v
	wire		cpu_we;			// From cpu_i of blit_cpu.v
	wire [1:0]	cpu_wstrb;		// From cpu_i of blit_cpu.v
	wire [17:0]	daddr;			// From regs_i of blit_regs.v
	wire		dma_ack;		// From ramarb_i of blit_ramarb.v
	wire [17:0]	dma_addr;		// From disp_i of blit_disp.v
	wire [15:0]	dma_rdata;		// From ramarb_i of blit_ramarb.v
	wire		dma_req;		// From disp_i of blit_disp.v
	wire [15:0]	dstat;			// From regs_i of blit_regs.v
	wire [7:1]	irq;			// From regs_i of blit_regs.v
	wire		ram_ack;		// From ram_i of blit_ram.v
	wire [17:0]	ram_addr;		// From ramarb_i of blit_ramarb.v
	wire [15:0]	ram_rdata;		// From ram_i of blit_ram.v
	wire		ram_req;		// From ramarb_i of blit_ramarb.v
	wire [15:0]	ram_wdata;		// From ramarb_i of blit_ramarb.v
	wire		ram_we;			// From ramarb_i of blit_ramarb.v
	wire [1:0]	ram_wstrb;		// From ramarb_i of blit_ramarb.v
	wire		regs_ack;		// From regs_i of blit_regs.v
	wire [7:0]	regs_addr;		// From bus_i of blit_bus.v
	wire [15:0]	regs_rdata;		// From regs_i of blit_regs.v
	wire		regs_req;		// From bus_i of blit_bus.v
	wire [15:0]	regs_wdata;		// From bus_i of blit_bus.v
	wire		regs_we;		// From bus_i of blit_bus.v
	wire [1:0]	regs_wstrb;		// From bus_i of blit_bus.v
	wire		rom_ack;		// From rom_i of blit_rom.v
	wire [15:0]	rom_addr;		// From bus_i of blit_bus.v
	wire [15:0]	rom_rdata;		// From rom_i of blit_rom.v
	wire		rom_req;		// From bus_i of blit_bus.v
	// End of automatics

	blit_cpu #(.HZ(HZ)) cpu_i(/*AUTOINST*/
				  // Outputs
				  .cpu_req		(cpu_req),
				  .cpu_addr		(cpu_addr[23:0]),
				  .cpu_wdata		(cpu_wdata[15:0]),
				  .cpu_wstrb		(cpu_wstrb[1:0]),
				  .cpu_we		(cpu_we),
				  // Inputs
				  .clk			(clk),
				  .cpu_ack		(cpu_ack),
				  .cpu_rdata		(cpu_rdata[15:0]),
				  .cpu_err		(cpu_err),
				  .irq			(irq[7:1]));
		 
	blit_bus bus_i(/*AUTOINST*/
		       // Outputs
		       .cpu_ack		(cpu_ack),
		       .cpu_rdata	(cpu_rdata[15:0]),
		       .cpu_err		(cpu_err),
		       .rom_req		(rom_req),
		       .rom_addr	(rom_addr[15:0]),
		       .cpu_ram_req	(cpu_ram_req),
		       .cpu_ram_addr	(cpu_ram_addr[17:0]),
		       .cpu_ram_wdata	(cpu_ram_wdata[15:0]),
		       .cpu_ram_wstrb	(cpu_ram_wstrb[1:0]),
		       .cpu_ram_we	(cpu_ram_we),
		       .regs_req	(regs_req),
		       .regs_addr	(regs_addr[7:0]),
		       .regs_wdata	(regs_wdata[15:0]),
		       .regs_wstrb	(regs_wstrb[1:0]),
		       .regs_we		(regs_we),
		       // Inputs
		       .clk		(clk),
		       .cpu_req		(cpu_req),
		       .cpu_addr	(cpu_addr[23:0]),
		       .cpu_wdata	(cpu_wdata[15:0]),
		       .cpu_wstrb	(cpu_wstrb[1:0]),
		       .cpu_we		(cpu_we),
		       .rom_ack		(rom_ack),
		       .rom_rdata	(rom_rdata[15:0]),
		       .cpu_ram_ack	(cpu_ram_ack),
		       .cpu_ram_rdata	(cpu_ram_rdata[15:0]),
		       .regs_ack	(regs_ack),
		       .regs_rdata	(regs_rdata[15:0]),
		       .bootup		(bootup));

	blit_rom rom_i(/*AUTOINST*/
		       // Outputs
		       .rom_ack		(rom_ack),
		       .rom_rdata	(rom_rdata[15:0]),
		       // Inputs
		       .clk		(clk),
		       .rom_req		(rom_req),
		       .rom_addr	(rom_addr[15:0]));

	blit_ram ram_i(/*AUTOINST*/
		       // Outputs
		       .ram_ack		(ram_ack),
		       .ram_rdata	(ram_rdata[15:0]),
		       // Inputs
		       .clk		(clk),
		       .ram_req		(ram_req),
		       .ram_addr	(ram_addr[17:0]),
		       .ram_wdata	(ram_wdata[15:0]),
		       .ram_wstrb	(ram_wstrb[1:0]),
		       .ram_we		(ram_we));

	blit_regs #(.HZ(HZ)) regs_i(/*AUTOINST*/
				    // Outputs
				    .regs_ack		(regs_ack),
				    .regs_rdata		(regs_rdata[15:0]),
				    .irq		(irq[7:1]),
				    .bootup		(bootup),
				    .daddr		(daddr[17:0]),
				    .dstat		(dstat[15:0]),
				    .uart_in_ready	(uart_in_ready),
				    .uart_out_valid	(uart_out_valid),
				    .uart_out_data	(uart_out_data[7:0]),
				    .kbd_in_ready	(kbd_in_ready),
				    .kbd_out_valid	(kbd_out_valid),
				    .kbd_out_data	(kbd_out_data[7:0]),
				    // Inputs
				    .clk		(clk),
				    .regs_req		(regs_req),
				    .regs_addr		(regs_addr[7:0]),
				    .regs_wdata		(regs_wdata[15:0]),
				    .regs_wstrb		(regs_wstrb[1:0]),
				    .regs_we		(regs_we),
				    .uart_in_valid	(uart_in_valid),
				    .uart_in_data	(uart_in_data[7:0]),
				    .uart_out_ready	(uart_out_ready),
				    .kbd_in_valid	(kbd_in_valid),
				    .kbd_in_data	(kbd_in_data[7:0]),
				    .kbd_out_ready	(kbd_out_ready),
				    .vblank		(vblank),
				    .mouse_x		(mouse_x[15:0]),
				    .mouse_y		(mouse_y[15:0]),
				    .mouse_but		(mouse_but[2:0]));

	blit_ramarb ramarb_i(/*AUTOINST*/
			     // Outputs
			     .cpu_ram_ack	(cpu_ram_ack),
			     .cpu_ram_rdata	(cpu_ram_rdata[15:0]),
			     .dma_ack		(dma_ack),
			     .dma_rdata		(dma_rdata[15:0]),
			     .ram_req		(ram_req),
			     .ram_addr		(ram_addr[17:0]),
			     .ram_wdata		(ram_wdata[15:0]),
			     .ram_wstrb		(ram_wstrb[1:0]),
			     .ram_we		(ram_we),
			     // Inputs
			     .clk		(clk),
			     .cpu_ram_req	(cpu_ram_req),
			     .cpu_ram_addr	(cpu_ram_addr[17:0]),
			     .cpu_ram_wdata	(cpu_ram_wdata[15:0]),
			     .cpu_ram_wstrb	(cpu_ram_wstrb[1:0]),
			     .cpu_ram_we	(cpu_ram_we),
			     .dma_req		(dma_req),
			     .dma_addr		(dma_addr[17:0]),
			     .ram_ack		(ram_ack),
			     .ram_rdata		(ram_rdata[15:0]));

	blit_disp disp_i(/*AUTOINST*/
			 // Outputs
			 .dma_req		(dma_req),
			 .dma_addr		(dma_addr[17:0]),
			 .pixel_valid		(pixel_valid),
			 .pixel_data		(pixel_data[15:0]),
			 // Inputs
			 .clk			(clk),
			 .daddr			(daddr[17:0]),
			 .dstat			(dstat[15:0]),
			 .dma_ack		(dma_ack),
			 .dma_rdata		(dma_rdata[15:0]),
			 .dmahstart		(dmahstart),
			 .vblank		(vblank));

endmodule
