//`default_nettype none

/**
 * Top Module/Chip Interface for BB8
 *
 * Still need to add mpu controller, control loop, bluetooth module
 * Also need to verify motor direction logic
 */

module Top
(input  logic         CLOCK_100,
 input  logic [3:0]   BTN,
 input  logic [15:0]  SW,
 output logic [15:0]  LD,
 output logic [5:0]   GPIO0, GPIO1,
 output logic [7:0]   D1_SEG, D2_SEG,
 output logic [3:0]   D1_AN, D2_AN,
 input  logic         BLE_UART_TX,
 output logic         BLE_UART_RX);

logic clock;
logic reset_n; //Given from bluetooth module?
assign reset_n = ~BTN[0]; //just for now since no bluetooth module

logic run_en; //Given from bluetooth module?
assign run_en = 1'b1; //just for now since no bluetooth module

logic signed [8:0] ble_pitch_kP, ble_pitch_kI, ble_pitch_kD;
logic signed [8:0] ble_yaw_kP, ble_yaw_kI, ble_yaw_kD;
logic signed [8:0] ble_set_pitch, ble_set_yaw;

logic signed [9:0] target_speed_left, target_speed_right;

logic [9:0] motor_speed_x, motor_speed_y;
logic motor_dir_x, motor_dir_y;
logic signed [9:0] mpu_roll, mpu_pitch, mpu_yaw;

logic [6:0] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0;
logic [3:0] BCD7, BCD6, BCD5, BCD4, BCD3, BCD2, BCD1, BCD0;
logic [7:0] blank;

logic [7:0] initialize_mpu, initialize_mpu_motor;

logic pressed_btn;
logic btn_was_pressed;
tri scl, sda;
logic scl_l, sda_l;
logic [7:0] count, count2;

assign scl_l = scl;
assign sda_l = sda;
assign GPIO0[0] = scl_l;
assign GPIO0[1] = sda_l;
assign blank = 8'd0;

SevenSegmentDisplay dut(.*);
SSegDisplayDriver ssd(.dpoints(8'd0), .reset(1'd0), .clk(CLOCK_100),
                      .*);

//logic [8:0] dir_degrees; //Input from bluetooth 0-360
//logic [9:0] target_speed_x, target_speed_y; //Also from bluetooth/control loop?

// Honestly not sure about how motor orientation will work
// This is just placeholder logic
always_comb begin
        motor_dir_x   = (target_speed_left < 0) ? 1'b0 : 1'b1;
        motor_speed_x = (target_speed_left < 0) ? -target_speed_left : target_speed_left;

        motor_dir_y   = (target_speed_right < 0) ? 1'b0 : 1'b1;
        motor_speed_y = (target_speed_right < 0) ? -target_speed_right : target_speed_right;
    end

assign BCD0 = count[3:0];
assign BCD1 = count[7:4];
assign BCD2 = {2'd0, mpu_roll[9:8]};
assign BCD3 = count2[3:0];
assign BCD4 = count2[7:4];
assign BCD5 = {2'd0, mpu_pitch[9:8]};
assign BCD6 = target_speed_left[3:0];
assign BCD7 = target_speed_left[7:4];

// MPU_Controller mpu (.clock(CLOCK_100),
//                     .reset(~reset_n), 
//                     .initialize(initialize_mpu), 
//                     .scl(GPIO0[0]), 
//                     .sda(GPIO0[1]),
//                     .roll(mpu_roll),
//                     .pitch(mpu_pitch), 
//                     .yaw(mpu_yaw));

assign initialize_mpu_motor[0] = SW[0];



always_ff @(posedge clock, negedge reset_n) begin
  if(~reset_n) begin
    count <= 8'd0;
  end
  else if(scl_l) begin
    LD[5] <= 1'b1;
    count <= 8'd1;
  end
end

assign count2 = 8'd0;
/*
always_ff @(posedge clock, negedge reset_n) begin
  if(~reset_n) begin
    count2 <= 8'd0;
  end
  else if(sda) begin
    count2 <= count2 + 8'd1;
  end
end */

always_ff @(posedge clock, negedge reset_n) begin
  if(~reset_n) begin
    pressed_btn <= 1'd0;
  end
  else if(BTN[1]) begin
    pressed_btn <= 1'd1;
  end
end

always_ff @(posedge clock, negedge reset_n) begin
  if(~reset_n) begin
    btn_was_pressed <= 1'd0;
  end
  else if(pressed_btn & ~btn_was_pressed) begin
    btn_was_pressed <= 1'd1;
    initialize_mpu <= 1'd1;
  end
  else if(initialize_mpu) begin
    initialize_mpu <= 1'd0;
  end
end



MPU_Controller mpu (.clock(CLOCK_100),
                    .reset(~reset_n),
                    .initialize(initialize_mpu[0]),
                    .scl(scl),
                    .sda(sda),
                    .roll(mpu_roll),
                    .pitch(mpu_pitch),
                    .yaw(mpu_yaw));
/*
bluetooth_wrapper ble (.clock(clock),
                       .reset(~reset_n),
                       .BLE_UART_TX(BLE_UART_TX),
                       .BLE_UART_RX(BLE_UART_RX),
                       .initialize_mpu_motor(initialize_mpu_motor),
                       .initialize_mpu(initialize_mpu),
                       .ble_pitch_kP(ble_pitch_kP),
                       .ble_pitch_kI(ble_pitch_kI),
                       .ble_pitch_kD(ble_pitch_kD),
                       .ble_yaw_kP(ble_yaw_kP),
                       .ble_yaw_kI(ble_yaw_kI),
                       .ble_yaw_kD(ble_yaw_kD),
                       .ble_set_pitch(ble_set_pitch),
                       .ble_set_yaw(ble_set_yaw),
                       .vector_valid(vector_valid)
                       ); */

assign vector_valid = SW[1];
/*
always_ff @(posedge clock) begin
    if (vector_valid) begin
        LD[7:0] <= initialize_mpu_motor;
        LD[13:8] <= initialize_mpu[5:0];
        LD[15] <= 1'd1;
    end
end */

// ControlLoop controlloop (.clock(CLOCK_100), .reset(~reset_n),
//                          .pitch_kP(ble_pitch_kP), .pitch_kI(ble_pitch_kI), .pitch_kD(ble_pitch_kD),
//                          .yaw_kP(ble_yaw_kP), .yaw_kI(ble_yaw_kI), .yaw_kD(ble_yaw_kD),
//                          .mpu_pitch(mpu_pitch), .mpu_yaw(mpu_yaw),
//                          .set_pitch(ble_set_pitch), .set_yaw(ble_set_yaw),
//                          .target_speed_left(target_speed_left),
//                          .target_speed_right(target_speed_right));

bluetooth_to_motor ble_mtr (.ble_set_pitch(ble_set_pitch)
                            .ble_set_yaw(ble_set_yaw),
                            .motor_speed_x(motor_speed_x),
                            .motor_speed_y(motor_speed_y),
                            .motor_dir_x(motor_dir_x),
                            .motor_dir_y(motor_dir_y));                 

MotorDriver motor_x (.clock(CLOCK_100),
                     .reset_n(reset_n),
                     .dir_in(1'b0),
                     .speed(10'sd200),
                     .run_en(initialize_mpu_motor[0]),
                     .step(GPIO1[0]),
                     .dir(GPIO1[1]),
                     .en_n(GPIO1[4]), // Will be shared with the other motor en
                     .ms1(GPIO1[5]),  // Will be shared with all the other ms1,ms2,ms3
                     .ms2(),
                     .ms3());

MotorDriver motor_y (.clock(CLOCK_100),
                     .reset_n(reset_n),
                     .dir_in(1'b1),
                     .speed(10'sd200),
                     .run_en(initialize_mpu_motor),
                     .step(GPIO1[2]),
                     .dir(GPIO1[3]),
                     .en_n(),
                     .ms1(),
                     .ms2(),
                     .ms3());

// Here we create a direction controller for motor_y
                     //

endmodule: Top

module bluetooth_to_motor
    (input logic signed ble_set_pitch,
     input logic signed ble_set_yaw,
     output logic motor_speed_x, motor_speed_y,
     output logic motor_dir_x, motor_dir_y
    );

    always_comb begin
        if (ble_set_yaw == 0 && ble_set_pitch > 0) begin
            motor_dir_x = 1;
            motor_dir_y = 1;
        end
        else if (ble_set_yaw == 0 && ble_set_pitch < 0) begin
            motor_dir_x = -1;
            motor_dir_y = -1;
        end
        else if (ble_set_yaw < 0) begin
            motor_dir_x = -1;
            motor_dir_y = 1;
        end
        else begin
            motor_dir_x = 1;
            motor_dir_y = -1;
        end
    end

    assign motor_speed_x = ble_set_pitch;
    assign motor_speed_y = ble_set_pitch;
    
endmodule : bluetooth_to_motor
