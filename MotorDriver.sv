`default_nettype none
/**
 * This module implements a motor driver interface for the A4988 Stepper motor driver
 * It uses an fsm to generate step pulses based on a continuous speed input 
 * MS1, MS2, MS3 will be hardcoded high to be in 1/16 microstepping mode (allows for higher precision)
 * STEP is triggered on the rising edge
 * DIR is high for clockwise and low for counter clockwise (given by top module based on degrees)
 * EN_N is active low for when the motor is engaged or not
 * Timing: uses a 25 MHz clock
 * 
 * Created by: Ananya Devpura
 * Date: 12/26/25
 */

module MotorDriver(
    input logic clock, 
    input logic reset_n, 
    input logic dir_in,  //1 for clockwise, 0 for counter clockwise 
    input logic [9:0] speed, // In steps/second? is what I'm assuming for now
    input logic run_en, 

    output logic dir,
    output logic step, 
    output logic en_n, 
    output logic ms1, ms2, ms3
);

// Minimum duration for both step high and step low pulses are 1 microsecond
// Assuming clock is 25 MHz so one clock cycle is 40 ns
// 1 microsecond/40 ns = 25 cycles --> use 50 cycles so driver definitely sees pulse
localparam PULSE_WIDTH = 32'd50;

// speed_period is used to calculate how long we need to be in the pulse low state
logic [31:0] speed_period;
always_comb begin
    if (speed <= 10'd0) begin 
        speed_period = 32'd0;
    end
    else begin
        // speed_period = clock / (steps/sec * 16)
        speed_period = 32'd1_562_500 / speed ;

        if (speed_period < 32'd100)
            speed_period = 32'd100; // Minimum total period at 25 MHz
    end
end

logic [31:0] count;
logic reset_count;
logic count_en;

always_ff @ (posedge clock) begin
   if (~reset_n || reset_count)
      count <= 32'd0;
   else if (count_en) 
      count <= count + 32'd1;    
end

enum logic [1:0] {IDLE, HIGH, LOW} state, nextState;

always_ff @ (posedge clock) begin
  if (~reset_n) 
    state <= IDLE;
  else 
    state <= nextState;
end

always_comb begin
    count_en = 1'b0; 
    reset_count = 1'b0;
    case(state)
        IDLE: begin
            if (run_en && speed_period > 32'd0)
                nextState = HIGH;
            else
                nextState = IDLE;
        end
        HIGH: begin
            count_en = 1'b1;
            if (count >= PULSE_WIDTH)  
                nextState = LOW;
            else 
                nextState = HIGH;
        end
        LOW: begin
            if (~run_en)begin
                reset_count = 1'b1;
                nextState = IDLE;
            end
            else if (count >= speed_period) begin
                reset_count = 1'b1;
                nextState = HIGH;
            end
            else begin
                count_en = 1'b1;
                nextState = LOW;
            end
        end
    endcase 
end

assign step = (state == HIGH);
assign dir = dir_in;
assign en_n = ~run_en;
assign ms1 = 1'b1;
assign ms2 = 1'b1;
assign ms3 = 1'b1;

endmodule: MotorDriver