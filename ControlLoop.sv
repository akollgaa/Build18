`default_nettype none
/*
* The control loop must interface with an MPU and then take that information
* and decide what to do with it based on what is currently happening with
* the robot. The control loop must then react to the situation and
* send that information to the motor controller and then wait for feedback.
*
* This implementation is going to exhibit a basic PID loop
*
* Below is a explanation of the PID Loop
*
* P - Proportional term: This is simply the difference between the
* expected location and the actual location
* I - Integral term: This is a sum of the total error since the start
* D - Derivative term: This is the difference between the time, t, and
* time, t-1.
*
* Set point  --> Error --> P-Error + I-Error + D-Error --> output
*	      ^	   |	     |         ^         ^
*	      |	   |	     |*kP      |         |<--D-Error(t-1)
*	      |    ----------+---------+---------+
*	Process point
*
* Created by: Adam Kollgaard
* Date: 12/18/25
* */


/**
* Creates a PID loop that tries to self balance the robot
*
* */
/**
  * TODO:
  * Learn what a PID loop is.
  * Change the inputs to roll pitch and yaw for setpoint and MPU
  * Change the desired output to whatever is neccesary; angle/speed
  *
  *
  * */
module ControlLoop(
    input  logic       clock, reset, start,
    input  logic [8:0] mpu_x, mpu_y, mpu_z, // These are all in degrees
    input  logic [8:0] set_x, set_y, set_z,
  	output logic [9:0] out_x, out_y, out_z
);

  logic [8:0] kP, kI, kD; // Scaling constants
  logic [9:0] P, I, D; // summed values


  logic starting;

	PID #(.KP(1), .KI(1), .KD(1), .WIDTH(9)) xPID(.set_point(set_x),
																				.process_point(mpu_x),
																				.out(out_x), .*);
	PID #(.KP(1), .KI(1), .KD(1), .WIDTH(1)) yPID(.set_point(set_y),
																				.process_point(mpu_y),
																				.out(out_y), .*);
	PID #(.KP(1), .KI(1), .KD(1), .WIDTH(1)) zPID(.set_point(set_z),
																				.process_point(mpu_z),
																				.out(out_z), .*);

	always_ff @(posedge clock) begin
		if(reset) begin
			starting <= 1'd0;
		end
		else if(start) begin
			starting <= 1'd1;
		end
	end

endmodule: ControlLoop

// KP, KI, KD are all constants that are specific each PID
// The WIDTH controls how large each 'variable' needs to be
module PID
#(parameter KP = 1, KI = 1, KD = 1, WIDTH = 9)
(
	input  logic clock, reset,
	input  logic [8:0] set_point, process_point,
	output logic [9:0] out
);

logic [WIDTH:0] error, prev_error;
logic [WIDTH:0] P, I, D;

assign error = set_point - process_point;

assign out = P + I + D;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		prev_error <= '0;
	end
	else begin
		prev_error <= error;
	end
end

// Proportional error
always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		P <= '0;
	end
	else begin
		P <= KP * error;
	end
end

// Integral error
always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		I <= '0;
	end
	else begin
		I <= KI * (I + error);
	end
end

// Derivative error
always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		D <= '0;
	end
	else begin
		D <= KD * (error - prev_error);
	end
end

endmodule: PID
