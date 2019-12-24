module rom(
	input wire clk,
	
	input wire rom_req,
	input wire [15:0] rom_addr,
	output reg rom_ack,
	output reg [15:0] rom_rdata
);

	reg [15:0] rom[0:24575];
	
	initial begin
		$readmemh("rom.mem", rom);
		rom[109] = 1;
	end
	
	always @(posedge clk) begin
		rom_ack <= rom_req;
		if(rom_req) begin
			rom_rdata <= rom[rom_addr[15:1]];
		end
	end

endmodule
