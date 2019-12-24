module regs(
	input wire clk,
	
	input wire reg_req,
	input wire reg_wr,
	input wire [7:0] reg_addr,
	input wire [31:0] reg_wdata,
	output reg reg_ack,
	output reg [31:0] reg_rdata,

	output reg uart_in_valid,
	output reg [7:0] uart_in_data,
	input wire uart_in_ready,
	
	input wire uart_out_valid,
	input wire [7:0] uart_out_data,
	output reg uart_out_ready,
	
	output reg kbd_in_valid,
	output reg [7:0] kbd_in_data,
	input wire kbd_in_ready,
	
	input wire kbd_out_valid,
	input wire [7:0] kbd_out_data,
	output wire kbd_out_ready,
	
	input wire pixel_valid,
	input wire pixel,
	
	output reg [15:0] mouse_x,
	output reg [15:0] mouse_y
);

	assign kbd_out_ready = 1'b1;
	always @(posedge clk) begin
		if(uart_in_valid && uart_in_ready)
			uart_in_valid <= 1'b0;
		reg_ack <= reg_req;
		uart_out_ready <= 1'b0;
		if(reg_req) begin
			if(reg_wr)
				case(reg_addr)
				8'h0: begin
					uart_in_valid <= 1'b1;
					uart_in_data <= reg_wdata[7:0];
				end
				8'h4: begin
					kbd_in_valid <= 1'b1;
					kbd_in_data <= reg_wdata[7:0];
				end
				8'h14: {mouse_y, mouse_x} <= reg_wdata;
				default: begin end
				endcase
			else
				case(reg_addr)
				8'h4: reg_rdata <= {31'b0, uart_in_valid};
				8'hc: reg_rdata <= {31'b0, kbd_in_valid};
				8'h10: begin
					reg_rdata <= {!uart_out_valid, 23'b0, uart_out_data};
					uart_out_ready <= uart_out_valid;
				end
				8'h18: reg_rdata <= {23'b0, pixel_valid, pixel};
				default: reg_rdata <= 0;
				endcase
		end
	end

endmodule