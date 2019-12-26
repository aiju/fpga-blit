module dport_pxconv(
	input wire dpclk,
	input wire reset,

	input wire raw_pixel_valid,
	input wire [15:0] raw_pixel_data,
	output wire raw_pixel_ready,
	
	output reg dp_pixel_valid,
	output reg [47:0] dp_pixel_data,
	input wire dp_pixel_ready,
	
	input wire dpdmahstart
);

	localparam PAD = 240;
	localparam BLITH = 800;
	localparam TOTH = BLITH + 2 * PAD;
	
	localparam [24:0] BRIGHT = 24'h00_F0_00;
	localparam [24:0] DARK = 24'h00_21_00;
	localparam [24:0] PADCOL = 24'h00_00_00;
	
	reg [15:0] sr;
	reg sr_valid;
	reg [4:0] sr_ctr;
	wire sr_ready;
	assign raw_pixel_ready = (sr_ctr == 0 || sr_ready && sr_ctr <= 1) || reset;
	always @(posedge dpclk) begin
		if(sr_valid && sr_ready) begin
			sr <= sr << 2;
			sr_ctr <= sr_ctr - 1;
			if(sr_ctr == 1)
				sr_valid <= 1'b0;
		end
		if(raw_pixel_valid && raw_pixel_ready) begin
			sr <= raw_pixel_data;
			sr_valid <= 1'b1;
			sr_ctr <= 8;
		end
		if(reset) begin
			sr_valid <= 1'b0;
			sr_ctr <= 0;
		end
	end
	wire [47:0] sr_data = {sr[14] ? BRIGHT : DARK, sr[15] ? BRIGHT : DARK};
	
	reg active;
	reg [15:0] x;
	
	assign sr_ready = (!dp_pixel_valid || dp_pixel_ready) && (x >= PAD && x < PAD + BLITH);
	always @(posedge dpclk) begin
		if(dp_pixel_valid && dp_pixel_ready)
			dp_pixel_valid <= 1'b0;
		if(dpdmahstart) begin
			active <= 1'b1;
			x <= 0;
		end
		if(active && (!dp_pixel_valid || dp_pixel_ready)) begin
			if(x < PAD) begin
				dp_pixel_valid <= 1'b1;
				dp_pixel_data <= {2{PADCOL}};
				x <= x + 2;
			end else if(x < PAD + BLITH) begin
				if(sr_valid) begin
					dp_pixel_valid <= 1'b1;
					dp_pixel_data <= sr_data;
					x <= x + 2;
				end
			end else begin
				dp_pixel_valid <= 1'b1;
				dp_pixel_data <= {2{PADCOL}};
				if(x == TOTH - 2) begin
					x <= 0;
					active <= 1'b0;
				end else
					x <= x + 2;
			end
		end
		if(reset) begin
			dp_pixel_valid <= 1'b0;
			x <= 0;
			active <= 1'b0;
		end
	end

endmodule
