module ram(
	input wire clk,
	
	input wire ram_req,
	input wire [17:0] ram_addr,
	input wire [15:0] ram_wdata,
	input wire [1:0] ram_wstrb,
	input wire ram_we,
	output reg ram_ack,
	output reg [15:0] ram_rdata
);

	reg [7:0] raml[0:131071];
	reg [7:0] ramh[0:131071];

	always @(posedge clk) begin
		ram_ack <= 1'b0;
		if(ram_req) begin
			ram_ack <= 1'b1;
			if(ram_we) begin
				if(ram_wstrb[0])
					raml[ram_addr[17:1]] <= ram_wdata[7:0];
				if(ram_wstrb[1])
					ramh[ram_addr[17:1]] <= ram_wdata[15:8];
			end else begin
				ram_rdata <= {ramh[ram_addr[17:1]], raml[ram_addr[17:1]]};
			end
		end
	end

endmodule
