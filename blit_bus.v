module blit_bus(
	input wire clk,
	
	input wire cpu_req,
	input wire [23:0] cpu_addr,
	input wire [15:0] cpu_wdata,
	input wire [1:0] cpu_wstrb,
	input wire cpu_we,
	output reg cpu_ack,
	output reg [15:0] cpu_rdata,
	output cpu_err,
	
	output reg rom_req,
	output wire [15:0] rom_addr,
	input wire rom_ack,
	input wire [15:0] rom_rdata,
	
	output reg cpu_ram_req,
	output reg [17:0] cpu_ram_addr,
	output wire [15:0] cpu_ram_wdata,
	output wire [1:0] cpu_ram_wstrb,
	output reg cpu_ram_we,
	input wire cpu_ram_ack,
	input wire [15:0] cpu_ram_rdata,

	output reg regs_req,
	output wire [7:0] regs_addr,
	output wire [15:0] regs_wdata,
	output wire [1:0] regs_wstrb,
	output wire regs_we,
	input wire regs_ack,
	input wire [15:0] regs_rdata,
	
	input wire bootup
);

	assign rom_addr = cpu_addr[15:0];

	assign cpu_ram_addr = cpu_addr[17:0];
	assign cpu_ram_wdata = cpu_wdata;
	assign cpu_ram_wstrb = cpu_wstrb;
	assign cpu_ram_we = cpu_we;
	
	assign regs_addr = cpu_addr[7:0];
	assign regs_wdata = cpu_wdata;
	assign regs_wstrb = cpu_wstrb;
	assign regs_we = cpu_we;
	
	reg [31:0] state;
	localparam IDLE = 0;
	localparam ROM = 1;
	localparam RAM = 2;
	localparam REGS = 3;

	always @(posedge clk) begin
		cpu_ack <= 1'b0;
		cpu_err <= 1'b0;
		rom_req <= 1'b0;
		cpu_ram_req <= 1'b0;
		regs_req <= 1'b0;
		case(state)
		IDLE:
			if(cpu_req) begin
				casez(cpu_addr)
				24'b0000_00zz_zzzz_zzzz_zzzz_zzzz:
					if(cpu_addr < 8) begin
						if(bootup) begin
							rom_req <= 1'b1;
							state <= ROM;
						end else begin
							cpu_ack <= 1'b1;
							cpu_err <= 1'b1;
						end
					end else begin
						cpu_ram_req <= 1'b1;
						state <= RAM;
					end
				24'h4zzzz: begin
					rom_req <= 1'b1;
					state <= ROM;
				end
				24'h600zz: begin
					regs_req <= 1'b1;
					state <= REGS;
				end
				default: begin
					$display("error %o", cpu_addr);
					cpu_ack <= 1'b1;
					//cpu_err <= 1'b1;
				end
				endcase
			end
		ROM:
			if(rom_ack) begin
				cpu_ack <= 1'b1;
				cpu_rdata <= rom_rdata;
				state <= IDLE;
			end
		RAM:
			if(cpu_ram_ack) begin
				cpu_ack <= 1'b1;
				cpu_rdata <= cpu_ram_rdata;
				state <= IDLE;
			end
		REGS:
			if(regs_ack) begin
				cpu_ack <= 1'b1;
				cpu_rdata <= regs_rdata;
				state <= IDLE;
			end
		endcase
	end

endmodule
