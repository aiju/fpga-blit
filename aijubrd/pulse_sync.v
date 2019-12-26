module pulse_sync(
	input wire clk_a,
	input wire clk_b,
	
	input wire in,
	output reg out
);

	reg ta;
	initial ta = 1'b0;
	always @(posedge clk_a) begin
		ta <= ta ^ in;
	end
	wire tb;
	reg tb0;
	initial tb0 = 1'b0;
	sync sync_i(clk_b, ta, tb);
	always @(posedge clk_b) begin
		tb0 <= tb;
		out <= tb ^ tb0;
	end

endmodule
