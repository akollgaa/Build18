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
	output logic [8:0] roll, pitch, yaw
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
logic calc_roll, calc_pitch;
logic [8:0] bad_roll, bad_pitch, roll_out, pitch_out;
logic roll_done, pitch_done;
logic update_pitch;
logic update_roll;
logic calculate_new_data;
logic [15:0] gyro_x_deg, gyro_y_deg;

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

calculate_accel_roll accel_roll(.clock(clock),
															 .reset(reset),
															 .start(calc_roll),
															 .y(data_y), .z(data_z),
															 .roll(roll_out),
															 .done(roll_done));

calculate_accel_pitch accel_pitch(.clock(clock),
																 .reset(reset),
																 .start(calc_pitch),
																 .x(data_x), .y(data_y), .z(data_z),
																 .pitch(pitch_out), .done(pitch_done));

complementary_filter #(.delta_t(1000)) fil_pitch(.clock(clock),
																								 .reset(reset),
																								 .update(update_pitch),
																								 .alpha(7'd60),
																								 .gyro(gyro_y_deg), // The y part
																								 .accel(bad_pitch),
																								 .angle(pitch));

complementary_filter #(.delta_t(1000)) fil_roll(.clock(clock),
																							 .reset(reset),
																							 .update(update_roll),
																							 .alpha(7'd60),
																							 .gyro(gyro_x_deg), // The x part
																							 .accel(bad_roll),
																							 .angle(roll));

TwoByte_Reg  xReg(.data_in(x_in), .in(data), .out(data_x), .*);
TwoByte_Reg  yReg(.data_in(y_in), .in(data), .out(data_y), .*);
TwoByte_Reg  zReg(.data_in(z_in), .in(data), .out(data_z), .*);

enum logic [3:0] {INIT = 4'd0, XONE = 4'd1, XTWO = 4'd2,
									YONE = 4'd3, YTWO = 4'd4, ZONE = 4'd5,
									ZTWO = 4'd6, SETUP=4'd7} state, nextState;

enum logic [2:0] {PWR = 3'd1, SMPLE = 3'd2,
									GYRO = 3'd3, ACCEL = 3'd4} s, ns;

always_ff @(posedge clock, posedge reset) begin
	if(reset | update_roll) begin
		bad_roll <= 1'd0;
		update_roll <= 1'd0;
	end
	else if(roll_done) begin
		bad_roll <= roll_out;
		update_roll <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | update_pitch) begin
		bad_pitch <= 1'd0;
		update_pitch <= 1'd0;
	end
	else if(pitch_done) begin
		bad_pitch <= pitch_out;
		update_pitch <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		calc_roll <= 1'd0;
		calc_pitch <= 1'd0;
		gyro_x_deg <= 16'd0;
		gyro_y_deg <= 16'd0;
	end
	else if (calc_roll) begin
		calc_roll <= 1'd0;
	else if (calc_pitch) begin
		calc_pitch <= 1'd0;
	end
	else if(calculate_new_data) begin
		calc_roll <= 1'd1;
		calc_pitch <= 1'd1;
		gyro_x_deg <= gyro_x / 16'd16384;
		gyro_y_deg <= gyro_y / 16'd16384;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		start <= 1'd0;
		collected_gyro <= 1'd0;
		calculate_new_data <= 1'd0;
	end
	else if(start) begin
		start <= 1'd0;
	end
	else if(data_collected) begin
		if(collected_gyro) begin
			collected_gyro <= 1'd0;
			calculate_new_data <= 1'd1;
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
		calculate_new_data <= 1'd0; // Reset value
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
			if(data_done) begin
				z_in = 1'd1;
				data_collected = 1'd1;
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

/**
*	Implements a complementary filter to combine information
*	from both the accelerometer and gyroscope to calculate
*	more accurate roll, pitch, and yaw data.
* Courtesy of: https://seanboe.com/blog/complementary-filters
**/
module complementary_filter
#(parameter logic delta_t = 1000)
(input  logic clock, reset,
 input  logic update,
 input  logic [6:0] alpha,
 input  logic [15:0] gyro,
 input  logic [15:0] accel,
 output logic [8:0] angle,
);

	logic [8:0] prev_angle;

	always_ff @(posedge clock, posedge reset) begin
		if(reset) begin
			prev_angle <= 9'd0;
		end
		else if(update) begin
			angle <= ((alpha * (prev_angle + (gyro * delta_t))) + ((100 - alpha) * accel))/100;
			prev_angle <= angle;
		end
	end

endmodule: complementary_filter
/**
* Wrapper function
*	Calculates the Euler's roll angle based on the y and z
*	values from the accelerometer. Takes about 8 cycles to complete
*
* */
module calculate_accel_roll(
	input  logic clock, reset,
  input  logic start,
	input  logic [15:0] y, z,
	output logic [8:0] roll,
	output logic done
);

	atan2 func(.clock(clock),
						 .reset(reset),
						 .start(start),
						 .x(y), .y(z),
						 .angle(roll),
						 .done(done));

endmodule: calculate_accel_roll

/**
*	Calculates the Euler's pitch angle based on the
*	x, y and z values from the accelerometer.
* Takes about 16 cycles to complete
* */
module calculate_accel_pitch(
	input  logic clock, reset,
  input  logic start,
  input  logic [15:0] x, y z,
	output logic [8:0] pitch,
  output logic done	
);

	logic [15:0] denom;
	logic [15:0] num;
	logic sqrt_done, tan_start;
	logic [15:0] sqrt_in, sqrt_out;

	assign num = {1'd1, x[14:0]}; // Negate
	assign sqrt_in = (y * y) + (z * z);
	
	atan2 func1(.clock(clock),
						  .reset(reset),
						  .start(tan_start),
						  .x(num), .y(denom),
						  .angle(pitch),
						  .done(done));

	SquareRoot func2(.clock(clock),
									 .reset(reset),
									 .start(start),
									 .x(sqrt_in), .y(sqrt_out),
									 .done(sqrt_done));

	always_ff @(posedge clock, posedge reset) begin
		if(reset | done) begin
			sqrt_out <= 16'd0;
			tan_start <= 1'd0;
		end
		else if(sqrt_done) begin
			denom <= sqrt_out;
			tan_start <= 1'd1;
		end
	end

endmodule: calculate_accel_pitch

/**
* Implementation of a CORDIC algorithm for square roots.
* Completes in about 8 clock cycles. Assumes x will stay on the
* input during the entire duration of the calculation. Once done
* is asserted y will only stay for a single clock cycle.
* Courtsey of: https://www.convict.lu/Jeunes/Math/square_root_CORDIC.htm
* */
module SquareRoot(
	input  logic clock, reset,
	input  logic start,
	input  logic [15:0] x,
	output logic [15:0] y,
	output logic done
);

	logic [15:0] base;
	logic [3:0] iterations;
	logic calculating;

	assign done = iterations >= 4'd8;

	always_ff @(posedge clock, posedge reset) begin
		if(reset | done) begin
			base <= 16'd128;
			y <= 16'd0;
			calculating <= 1'd0;
			iterations <= 1'd0;
		end
		else if(calculating) begin
			// Here we implement the cordic algorithm
			if(((y + base) * (y + base)) <= x) begin
				y <= y + base;
				base <= base >> 1;	
			end
			else begin
				base <= base >> 1;
			end
			iterations <= iterations + 4'd1;
		end
		else if(start) begin
			calculating <= 1'd1;
		end
	end

endmodule: SquareRoot

/**
* Implementation of the CORDIC algorithm for the atan2 function
* It takes around 8 clock cycles to complete the computation
* There is some round but in general it accurate with +/-1 degree
* Courtesy of: http://signal-processing.net/PDF/Doc%2051%20-%20Atan2%20Cordic%20Algorithm.pdf
* */
module atan2(
	input  logic clock, reset,
  input  logic start,
  input  logic [15:0] x, y,
	output logic [8:0] angle,
  output logic done	
);

	logic [15:0] x1, y1, x2, y2;
	logic [8:0] additional_angle, a, cordic_angle;
	logic calculating;
	logic iterations;

	assign angle = additional_angle + a;

	always_comb begin
			if(x[15] & y[15]) begin
				additional_angle = 9'b1_1011_0100;
				x1 = {1'd1, x[14:0]};
				y1 = {1'd1, y[14:01};
			end
			else if(x[15] & ~y[15]) begin
				additional_angle = 9'b0_0101_1010;
				x1 = y;
				y1 = {1'd1, x[14:01};
			end
			else if(~x[15] & y[15]) begin
				additional_angle = 9'b1_0101_1010;
				x1 = {1'd1, y[14:01};
				y1 = x;
			end 
			else begin
				additional_angle = 9'b0_0000_0000;
				x1 = x;
				y1 = y;
			end 
	end

	// These values are predefined based on the algorithm
	always_comb begin
		case(iterations)
			4'd0: cordic_angle = 9'd45;
			4'd1: cordic_angle = 9'd26;
			4'd2: cordic_angle = 9'd14;
			4'd3: cordic_angle = 9'd7;
			4'd4: cordic_angle = 9'd4;
			4'd5: cordic_angle = 9'd2;
			4'd6: cordic_angle = 9'd1;
			4'd7: cordic_angle = 9'd1;
			default: cordic_angle = 9'd0;
		endcase
	end

	always_ff @(posedge clock, posedge reset) begin 
		if(reset) begin 
			a <= 9'd0; 
			calculating <= 1'd0; 
			iterations <= 1'd0;
			x2 <= 16'd0; 
			y2 <= 16'd0;
		end else if(calculating) begin
			//Here we actually perform the rotations
			if (((16'd1 << iterations) * y2) >= x2) begin
				x2 <= x2 + (y2 >> iterations);
				y2 <= y2 - (x2 >> iterations);
				a <= a + cordic_angle;
			end
		end
		else if(start) begin 
			calculating <= 1'd1;
			x2 <= x1;
			y2 <= y1;
			end 
		end 
	end
endmodule: atan2

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
