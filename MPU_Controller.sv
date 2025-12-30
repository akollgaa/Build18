`default_nettype none

/**
*
*	Takes the data from the MPU and translates it into degrees
* 
* Created by: Adam Kollgaard
* Date: 12/21/25
* */

module MPU_Controller(
	input  logic       clock, reset,
	inout  tri         scl, sda,
	output logic [8:0] x, y, z	
);

logic [7:0] addr;
logic addr_in;
logic [7:0] data;
logic data_done;
logic [15:0] data_x, data_y, data_z;
logic start;
logic running;
logic data_collected;

logic x_in, y_in, z_in;

I2C_interface i2c(.clock(clock),
									.reset(reset),
									.address(addr),
									.addr_in(addr_in),
									.scl(scl), .sda(sda),
									.data(data),
									.done(data_done));

TwoByte_Reg(.data_in(x_in), .in(data), .out(data_x));
TwoByte_Reg(.data_in(y_in), .in(data), .out(data_y));
TwoByte_Reg(.data_in(z_in), .in(data), .out(data_z));

enum logic [3:0] {INIT = 4'd0, XONE = 4'd1, XTWO = 4'd2,
									YONE = 4'd3, YTWO = 4'd4, ZONE = 4'd5,
									ZTWO = 4'd6} state, nexstate;

always_ff @(posedge clock, posedge reset) begin
	if(reset | start) begin
		start <= 1'd0;
	end
	else if(data_collected) begin
		// TODO: After the data is collected from the sensor
		// here you can calculate the actual angles needed
		
	end
	else if(~running) begin
		start <= 1'd1;
	end
end

// Nextstate logic
always_comb begin
	case(state)
		INIT: begin
			nextState = (start) ? XONE : INIT;
		end
		XONE: begin
			nextState = (data_done) ? XTWO : XONE;
		end
		XTWO: begin
			nextState = (data_done) ? YONE : XTWO;
		end
		YONE: begin
			nextState = (data_done) ? YTWO : YONE;
		end
		YTWO: begin
			nextState = (data_done) ? ZONE : YTWO;
		end
		ZONE: begin
			nextState = (data_done) ? ZTWO : ZONE;
		end
		ZTWO: begin
			nextState = (data_done) ? ZTWO : INIT;
		end
	endcase
end

// Output logic
always_comb begin
	x_in = 1'd0;
	y_in = 1'd0;
	z_in = 1'd0;
	addr_in = 1'd0;
	running = 1'd0;
	data_collected = 1'd0;
	case(state)
		INIT: begin
			if(start) begin
				addr_in = 1'd1;
				addr = 8'b
			end
		end
		XONE: begin
			running = 1'd1;
			if(data_done) begin
				addr_in = 1'd1;
				addr = 8'b;
				x_in = 1'd1;	
			end
		end
		XTWO: begin
			running = 1'd1;
			if(data_done) begin
				addr_in = 1'd1;
				x_in = 1'd1;	
			end
		end
		YONE: begin
			running = 1'd1;
			if(data_done) begin
				addr_in = 1'd1;
				y_in = 1'd1;	
			end
		end
		YTWO: begin
			running = 1'd1;
			if(data_done) begin
				addr_in = 1'd1;
				y_in = 1'd1;	
			end
		end
		ZONE: begin
			running = 1'd1;
			if(data_done) begin
				addr_in = 1'd1;
				z_in = 1'd1;	
			end
		end
		ZTWO: begin
			running = 1'd1;
			data_collected = 1'd1;
			if(data_done) begin
				z_in = 1'd1;	
			end
		end
	endcase
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		state = INIT;
	end
	else begin
		state = nextState;
	end
end

endmodule: MPU_Controller

module TwoByte_Reg(
	input  logic clock, reset,
	input  logic data_in,
	input  logic [7:0] in,
	output logic [15:0] out
);

logic first_half;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		first_half <= 1'd0;
		out <= 16'd0;
	end
	else if(data_in) begin
		if(first_half) begin
			out[7:0] <= in;
			first_half <= ~first_half;
		end
		else begin
			out[15:8] <= in;
			first_half <= ~first_half;
		end
	end
end

endmodule: TwoByte_Reg
