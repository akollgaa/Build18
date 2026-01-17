//`default_nettype none
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
	input  logic signed [8:0] pitch_kP, pitch_kI, pitch_kD, //from bluetooth module for testing 
    input  logic signed [8:0] yaw_kP, yaw_kI, yaw_kD, //from bluetooth module for testing 
	input  logic signed [8:0] mpu_pitch, mpu_yaw,  //from mpu controller
	input  logic signed [8:0] set_pitch, set_yaw, //from bluetooth
  	output logic signed [9:0] target_speed_left, target_speed_right
);

  //logic [8:0] kP, kI, kD; // Scaling constants
  logic [9:0] P, I, D; // summed values

  logic starting; //Don't know if we need this 

	logic signed [9:0] pitch_correction;
    logic signed [9:0] yaw_correction;

	PID pitch_pid(.clock(clock), 
				  .reset(reset), 
				  .set_point(set_pitch), 
				  .process_point(mpu_pitch), 
				  .out(pitch_correction), 
				  .KP(pitch_kP), 
				  .KI(pitch_kI),
				  .KD(pitch_kD));

	PID yaw_pid(.clock(clock), 
				.reset(reset), 
				.set_point(set_yaw), 
				.process_point(mpu_yaw), 
				.out(yaw_correction), 
				.KP(yaw_kP),  
				.KI(yaw_kI),
				.KD(yaw_kD));

	logic signed [11:0] full_left, full_right;
	// Mixing Logic
    always_comb begin
        // The mixing logic turns the two PID corrections into individual motor speeds.
        full_left  = pitch_correction + yaw_correction;
        full_right = pitch_correction - yaw_correction;

		//in case bigger than allowed 
		if (full_left > 10'sd511)  target_speed_left = 10'sd511;
    	else if (full_left < -10'sd511) target_speed_left = -10'sd511;
    	else target_speed_left = full_left[9:0];

		if (full_right > 10'sd511)  target_speed_right = 10'sd511;
    	else if (full_right < -10'sd511) target_speed_right = -10'sd511;
    	else target_speed_right = full_right[9:0];
    end

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
#(parameter WIDTH = 9)
(
	input  logic signed [8:0] KP, KI, KD,
	input  logic clock, reset,
	input  logic signed [8:0] set_point, process_point,
	output logic signed [9:0] out
);

logic [WIDTH:0] error, prev_error;
logic signed [31:0] P, I, D;
logic signed [31:0] integral;

assign error = set_point - process_point;

always_ff @(posedge clock, posedge reset) begin
        if (reset) begin
            prev_error <= 0;
            integral   <= 0;
        end else begin
            prev_error <= error;
            // saturation to prevent windup
            if (integral < 32'sd100000 && integral > -32'sd100000) 
                integral <= integral + error;
        end
end

assign P = error * KP;
assign I = integral * KI;
assign D = (error - prev_error) * KD;

assign out = (P + I + D) >>> 8;

endmodule: PID
