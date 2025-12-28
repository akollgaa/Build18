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
logic run_en; //Given from bluetooth module?

logic [8:0] dir_degrees; //Input from bluetooth 0-360
logic [9:0] target_speed_x, target_speed_y; //Also from bluetooth/control loop?

logic [9:0] speed_x, speed_y; 
logic dir_x, dir_y; 

// Honestly not sure about how motor orientation will work 
// This is just placeholder logic 
always_comb begin 
    dir_x = 1'b1;
    dir_y = 1'b1;
    speed_x = target_speed_x;
    speed_y = target_speed_y;
    if (dir_degrees >= 90 && dir_degrees < 270) begin
        dir_x = 1'b0; // Motor X goes backward in this range
    end
    if (dir_degrees >= 180 && dir_degrees < 360) begin
        dir_y = 1'b0; // Motor Y goes backward in this range
    end
    if (dir_degrees == 9'd90) begin
        speed_y = 10'd0; // Force Motor Y to stop for a fully right pivot?
    end
    else if (dir_degrees == 9'd270) begin
        speed_x = 10'd0; // Force Motor X to stop for a fully left pivot?
    end
end


MotorDriver motor_x (.clock(clock),
                     .reset_n(reset_n),
                     .dir_in(dir_x),
                     .speed(speed_x),
                     .run_en(run_en),
                     .step(GPIO1[0]), 
                     .dir(GPIO1[1]), 
                     .en_n(GPIO1[4]), // Will be shared with the other motor en
                     .ms1(GPIO1[5]),  // Will be shared with all the other ms1,ms2,ms3
                     .ms2(), 
                     .ms3());

MotorDriver motor_y (.clock(clock),
                     .reset_n(reset_n),
                     .dir_in(dir_y),
                     .speed(speed_y),
                     .run_en(run_en),
                     .step(GPIO1[2]), 
                     .dir(GPIO1[3]), 
                     .en_n(),
                     .ms1(),
                     .ms2(), 
                     .ms3());


endmodule: Top