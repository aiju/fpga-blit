`include "dport.vh"

module dport_scrambler(
	input wire clk,
	input wire [15:0] indata,
	input wire [1:0] inisk,
	output reg [15:0] outdata,
	output reg [1:0] outisk
);

	reg [15:0] lfsr, lfsr_;
	reg [8:0] bsctr;
	reg [15:0] key;
	
	initial begin
		lfsr = 16'hffff;
		bsctr = 0;
	end
	wire [1:0] bs, reset;
	assign bs[0] = inisk[0] && indata[7:0] == `symBS;
	assign bs[1] = inisk[1] && indata[15:8] == `symBS;
	assign reset = bsctr == 0 ? bs : 0;
	always @(posedge clk) begin
		outisk <= inisk;
		outdata[7:0] <= reset[0] ? `symSR : indata[7:0] ^ (!inisk[0] ? key[7:0] : 0);
		outdata[15:8] <= reset[1] ? `symSR : indata[15:8] ^ (!inisk[1] ? key[15:8] : 0);
		lfsr <= lfsr_;
		if(reset[0])
			lfsr <= 'he817;
		if(reset[1])
			lfsr <= 'hffff;
		if(|bs)
			bsctr <= bsctr + 1;
	end
	
	always @(*) begin
		lfsr_[0] = lfsr[0] ^ lfsr[11] ^ lfsr[12] ^ lfsr[13];
		lfsr_[1] = lfsr[1] ^ lfsr[12] ^ lfsr[13] ^ lfsr[14];
		lfsr_[2] = lfsr[2] ^ lfsr[13] ^ lfsr[14] ^ lfsr[15];
		lfsr_[3] = lfsr[0] ^ lfsr[3] ^ lfsr[11] ^ lfsr[12] ^ lfsr[13] ^ lfsr[14] ^ lfsr[15];
		lfsr_[4] = lfsr[0] ^ lfsr[1] ^ lfsr[4] ^ lfsr[11] ^ lfsr[14] ^ lfsr[15];
		lfsr_[5] = lfsr[0] ^ lfsr[1] ^ lfsr[2] ^ lfsr[5] ^ lfsr[11] ^ lfsr[13] ^ lfsr[15];
		lfsr_[6] = lfsr[1] ^ lfsr[2] ^ lfsr[3] ^ lfsr[6] ^ lfsr[12] ^ lfsr[14];
		lfsr_[7] = lfsr[2] ^ lfsr[3] ^ lfsr[4] ^ lfsr[7] ^ lfsr[13] ^ lfsr[15];
		lfsr_[8] = lfsr[3] ^ lfsr[4] ^ lfsr[5] ^ lfsr[8] ^ lfsr[14];
		lfsr_[9] = lfsr[4] ^ lfsr[5] ^ lfsr[6] ^ lfsr[9] ^ lfsr[15];
		lfsr_[10] = lfsr[5] ^ lfsr[6] ^ lfsr[7] ^ lfsr[10];
		lfsr_[11] = lfsr[6] ^ lfsr[7] ^ lfsr[8] ^ lfsr[11];
		lfsr_[12] = lfsr[7] ^ lfsr[8] ^ lfsr[9] ^ lfsr[12];
		lfsr_[13] = lfsr[8] ^ lfsr[9] ^ lfsr[10] ^ lfsr[13];
		lfsr_[14] = lfsr[9] ^ lfsr[10] ^ lfsr[11] ^ lfsr[14];
		lfsr_[15] = lfsr[10] ^ lfsr[11] ^ lfsr[12] ^ lfsr[15];
		
		key[0] = lfsr[15];
		key[1] = lfsr[14];
		key[2] = lfsr[13];
		key[3] = lfsr[12];
		key[4] = lfsr[11];
		key[5] = lfsr[10];
		key[6] = lfsr[9];
		key[7] = lfsr[8];
		key[8] = lfsr[7];
		key[9] = lfsr[6];
		key[10] = lfsr[5];
		key[11] = lfsr[4] ^ lfsr[15];
		key[12] = lfsr[3] ^ lfsr[14] ^ lfsr[15];
		key[13] = lfsr[2] ^ lfsr[13] ^ lfsr[14] ^ lfsr[15];
		key[14] = lfsr[1] ^ lfsr[12] ^ lfsr[13] ^ lfsr[14];
		key[15] = lfsr[0] ^ lfsr[11] ^ lfsr[12] ^ lfsr[13];
		
		if(reset[0])
			key[15:8] = 'hff;
	end
endmodule
