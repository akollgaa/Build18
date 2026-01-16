`default_nettype none

/**
 * Top Module/Chip Interface for BB8
 * 
 * Still need to add mpu controller, control loop, bluetooth module
 * Also need to verify motor direction logic 
 */

module Top
(input  logic         CLOCK_100, 
 input  logic [3:0]   BTN,
 output logic [5:0]   GPIO0, GPIO1,
 input  logic         BLE_UART_TX,  
 output logic         BLE_UART_RX);

logic clock;
logic reset_n; //Given from bluetooth module?
assign reset_n = ~BTN[0]; //just for now since no bluetooth module

logic run_en; //Given from bluetooth module?
assign run_en = 1'b1; //just for now since no bluetooth module

logic signed [8:0] mpu_pitch, mpu_yaw;
logic signed [8:0] ble_pitch_kP, ble_pitch_kI, ble_pitch_kD;
logic signed [8:0] ble_yaw_kP, ble_yaw_kI, ble_yaw_kD;
logic signed [8:0] ble_set_pitch, ble_set_yaw;

logic signed [9:0] target_speed_left, target_speed_right;

logic [9:0] motor_speed_x, motor_speed_y;
logic motor_dir_x, motor_dir_y;
logic [8:0] mpu_roll, mpu_pitch, mpu_yaw;

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

MPU_Controller mpu (.clock(CLOCK_100),
                    .reset(~reset_n), 
                    .initialize(), 
                    .scl(GPIO0[0]), 
                    .sda(GPIO0[1]),
                    .roll(mpu_roll),
                    .pitch(mpu_pitch), 
                    .yaw(mpu_yaw));


ControlLoop controlloop (.clock(CLOCK_100), .reset(~reset_n),
                         .pitch_kP(ble_pitch_kP), .pitch_kI(ble_pitch_kI), .pitch_kD(ble_pitch_kD),
                         .yaw_kP(ble_yaw_kP), .yaw_kI(ble_yaw_kI), .yaw_kD(ble_yaw_kD),
                         .mpu_pitch(mpu_pitch), .mpu_yaw(mpu_yaw),
                         .set_pitch(ble_set_pitch), .set_yaw(ble_set_yaw),
                         .target_speed_left(target_speed_left),
                         .target_speed_right(target_speed_right));
                        

MotorDriver motor_x (.clock(CLOCK_100),
                     .reset_n(reset_n),
                     .dir_in(motor_dir_x),
                     .speed(motor_speed_x),
                     .run_en(run_en),
                     .step(GPIO1[0]), 
                     .dir(GPIO1[1]), 
                     .en_n(GPIO1[4]), // Will be shared with the other motor en
                     .ms1(GPIO1[5]),  // Will be shared with all the other ms1,ms2,ms3
                     .ms2(), 
                     .ms3());

MotorDriver motor_y (.clock(CLOCK_100),
                     .reset_n(reset_n),
                     .dir_in(motor_dir_y),
                     .speed(motor_speed_y ),
                     .run_en(run_en),
                     .step(GPIO1[2]), 
                     .dir(GPIO1[3]), 
                     .en_n(),
                     .ms1(),
                     .ms2(), 
                     .ms3());


endmodule: Top