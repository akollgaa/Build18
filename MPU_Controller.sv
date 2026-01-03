`default_nettype none

/**
*
*	Takes the data from the MPU and translates it into degrees
*
* Created by: Adam Kollgaard
* Date: 12/21/25
* */

/**
  * Initializes the MPU controller based on typical settings
  * Gathers the accelerometer/gyroscrope values and calculates
  * the orientation of the MPU in degrees.
  *
  * */
module MPU_Controller(
	input  logic       clock, reset, initialize,
	inout  tri         scl, sda,
	output logic [8:0] x, y, z
);

logic [7:0] addr;
logic [7:0] setup_addr;
logic [7:0] data_addr;
logic begin_trans;
logic begin_trans_setup;
logic [7:0] data;
logic [7:0] w_data;
logic data_done;
logic [15:0] data_x, data_y, data_z;
logic [15:0] gyro_x, gyro_y, gyro_z;
logic re, we, we_success;
logic start;
logic running;
logic data_collected;
logic collected_gyro;
logic initialize_done;
logic finish_setup;

logic x_in, y_in, z_in;

I2C_Interface i2c(.clock(clock),
									.reset(reset),
                  .re(re),
                  .we(we),
									.start(begin_trans | begin_trans_setup),
									.address(addr),
									.we_data(w_data),
									.scl(scl), .sda(sda),
									.re_data(data),
									.we_success(we_success),
									.done(data_done));

TwoByte_Reg  xReg(.data_in(x_in), .in(data), .out(data_x), .*);
TwoByte_Reg  yReg(.data_in(y_in), .in(data), .out(data_y), .*);
TwoByte_Reg  zReg(.data_in(z_in), .in(data), .out(data_z), .*);

enum logic [3:0] {INIT = 4'd0, XONE = 4'd1, XTWO = 4'd2,
									YONE = 4'd3, YTWO = 4'd4, ZONE = 4'd5,
									ZTWO = 4'd6, SETUP=4'd7} state, nextState;

enum logic [2:0] {PWR = 3'd1, SMPLE = 3'd2,
									GYRO = 3'd3, ACCEL = 3'd4} s, ns;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		start <= 1'd0;
		collected_gyro <= 1'd0;
	end
	else if(start) begin
		start <= 1'd0;
	end
	else if(data_collected) begin
		if(collected_gyro) begin
			collected_gyro <= 1'd0;
		// TODO: After the data is collected from the sensor
		// here you can calculate the actual angles needed
		end
		else begin
			collected_gyro <= 1'd1;
			gyro_x <= data_x;
			gyro_y <= data_y;
			gyro_z <= data_z;
		end
	end
	else if(~running) begin
		start <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		initialize_done <= 1'd0;
	end
	else if(finish_setup) begin
		initialize_done <= 1'd1;
	end
end

always_comb begin
	if(initalize_done) begin
		addr = data_addr;
	end
	else begin
		addr = setup_addr;
	end
end

// Nextstate logic
always_comb begin
	case(state)
		SETUP: begin
			nextState = (initialize_done) ? INIT : SETUP;
		end
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
	re = 1'd0;
	we = 1'd1;
	data_addr = 1'd0;
	begin_trans = 1'd0;
	running = 1'd0;
	data_collected = 1'd0;
	case(state)
		SETUP: begin
			we = 1'd1;
			running = 1'd1;
			if(initialize_done) begin
			end
		end
		INIT: begin
			re = 1'd1;
			if(start) begin
				begin_trans = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h3B;
				end
				else begin
					data_addr = 8'h43;
				end
			end
		end
		XONE: begin
			re = 1'd1;
			running = 1'd1;
			if(data_done) begin
				begin_trans = 1'd1;
				x_in = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h3C;
				end
				else begin
					data_addr = 8'h44;
				end
			end
		end
		XTWO: begin
			re = 1'd1;
			running = 1'd1;
			if(data_done) begin
				begin_trans = 1'd1;
				x_in = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h3D;
				end
				else begin
					data_addr = 8'h45;
				end
			end
		end
		YONE: begin
			re = 1'd1;
			running = 1'd1;
			if(data_done) begin
				begin_trans = 1'd1;
				y_in = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h3E;
				end
				else begin
					data_addr = 8'h46;
				end
			end
		end
		YTWO: begin
			re = 1'd1;
			running = 1'd1;
			if(data_done) begin
				begin_trans = 1'd1;
				y_in = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h3F;
				end
				else begin
					data_addr = 8'h47;
				end
			end
		end
		ZONE: begin
			re = 1'd1;
			running = 1'd1;
			if(data_done) begin
				begin_trans = 1'd1;
				z_in = 1'd1;
				if(collected_gyro) begin
					data_addr = 8'h40;
				end
				else begin
					data_addr = 8'h48;
				end
			end
		end
		ZTWO: begin
			re = 1'd1;
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

// STD for the initalization case
// Next state logic
always_comb begin
	case(s)
		PWR: begin
			ns = (data_done & ~initialize_done) ? SMPLE : PWR;
		end
		SMPLE: begin
			ns = (data_done) ? GYRO : SMPLE;
		end
		GYRO: begin
			ns = (data_done) ? ACCEL : GYRO;
		end
		ACCEL: begin
			ns = (data_done) ? PWR : ACCEL;
		end
	endcase
end

// Output logic
always_comb begin
	setup_addr = 8'b0000_0000;
	begin_trans_setup = 1'd0;
	finish_setup = 1'd0;
	w_data = 8'd0;
	case(s)
		PWR: begin
			if(initialize) begin
				begin_trans_setup = 1'd1;
				setup_addr = 8'h6B;
				w_data = 8'h00;
			end
			if(data_done) begin
				begin_trans_setup = 1'd1;
				setup_addr = 8'h19;	
				w_data = 8'h07;
			end
		end
		SMPLE: begin
			if(data_done) begin
				begin_trans_setup = 1'd1;
				setup_addr = 8'h1B;	
				w_data = 8'h00;
			end
		end
		GYRO: begin
			if(data_done) begin
				begin_trans_setup = 1'd1;
				setup_addr = 8'h1C;
				w_data = 8'h00;
			end
		end
		ACCEL: begin
			if(data_done) begin
				finish_setup = 1'd1;
			end
		end
	endcase
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		s <= WHO;
	end
	else begin
		s <= ns;
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
