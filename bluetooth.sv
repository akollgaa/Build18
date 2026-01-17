// module ChipInterface (
//     input logic CLOCK_100,    // INPUTS
//     input logic [3:0] BTN,
//     input logic [15:0] SW,
//     // OUTPUTS
//     output logic [15:0] LD,
//     output logic [3:0] D1_AN,
//     output logic [3:0] D2_AN,
//     output logic [7:0] D1_SEG,
//     output logic [7:0] D2_SEG,
//     output logic [5:0] GPIO0,
//     output logic [5:0] GPIO1,
//     // BLUETOOTH SIGNALS
//     input logic BLE_UART_TX,
//     output logic BLE_UART_RX,
//     // UART SIGNALS
//     input logic UART_RXD,
//     output logic UART_TXD
// );

//   logic [7:0] rx_byte;
//   logic       rx_valid;
//   logic clk, rst;
//   logic [255:0][7:0] packet;
//   logic [7:0] packet_len;
//   logic packet_ready;

//   always_ff @(posedge clk)
//     if (packet_ready) begin
//       LD[7:0] = packet[0];
//       LD[15:8] = packet[1]; 
//     end



//   assign clk = CLOCK_100;
//   assign rst = BTN[0];
//   assign BLE_UART_RX = 1'b1;

//   uart_rx uart_rx_i (
//       .clk        (clk),
//       .rst        (rst),
//       .rxd        (BLE_UART_TX),
//       .data       (rx_byte),
//       .data_valid (rx_valid)
//   );

//   ble_packet_rx pkt_rx_i (
//       .clk          (clk),
//       .rst          (rst),
//       .rx_byte      (rx_byte),
//       .rx_valid     (rx_valid),
//       .packet       (packet),
//       .packet_len   (packet_len),
//       .packet_ready (packet_ready)
//   );

// endmodule : ChipInterface

module bluetooth_wrapper (
    input logic clock,
    input logic reset,
    input logic BLE_UART_TX,
    output logic BLE_UART_RX,
    output logic [7:0] initialize_mpu_motor, initialize_mpu,
    output logic [7:0] ble_pitch_kP, ble_pitch_kI, ble_pitch_kD,
    output logic [7:0] ble_yaw_kP, ble_yaw_kI, ble_yaw_kD,
    output logic [7:0] ble_set_pitch, ble_set_yaw,
    output logic vector_valid
);
    logic [7:0] rx_byte;
    logic       rx_valid;
    logic clk, rst;
    logic [255:0][7:0] packet; //packed array that holds each 8 bit portion of a packet
    logic [7:0] packet_len;
    logic packet_ready;

    //reads bluetooth message off UART
    uart_rx uart_rx (
        .clk        (clk),
        .rst        (rst),
        .rxd        (BLE_UART_TX),
        .data       (rx_byte),
        .data_valid (rx_valid)
    );

    //assembles into a functional packet
    ble_packet_rx pkt_rx (
        .clk          (clk),
        .rst          (rst),
        .rx_byte      (rx_byte),
        .rx_valid     (rx_valid),
        .packet       (packet),
        .packet_len   (packet_len),
        .packet_ready (packet_ready)
    );

    //identifies direction and magnitude values
    ble_vector_parser parser (
        .clk                (clk),
        .rst                (rst),
        .packet             (packet),
        .packet_len         (packet_len),
        .packet_ready       (packet_ready),
        .initialize_mpu_motor (initialize_mpu_motor),
        .initialize_mpu       (initialize_mpu),
        .ble_pitch_kP         (ble_pitch_kP),
        .ble_pitch_kI         (ble_pitch_kI),
        .ble_pitch_kD         (ble_pitch_kD),
        .ble_yaw_kP           (ble_yaw_kP),
        .ble_yaw_kI           (ble_yaw_kI),
        .ble_yaw_kD           (ble_yaw_kD),
        .ble_set_pitch        (ble_set_pitch),
        .ble_set_yaw          (ble_set_yaw),
        .vector_valid       (vector_valid)
    );

endmodule : bluetooth_wrapper


module uart_rx #(
    parameter int CLK_FREQ = 100_000_000,
    parameter int BAUD     = 115200
)(
    input  logic clk,
    input  logic rst,

    input  logic rxd,          // from BLE module
    output logic [7:0] data,   // received byte
    output logic data_valid    // 1-cycle pulse when byte ready
);

    localparam int CLKS_PER_BIT = CLK_FREQ / BAUD;

    typedef enum logic [2:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_t;

    state_t state;

    logic [$clog2(CLKS_PER_BIT)-1:0] clk_cnt;
    logic [2:0] bit_idx;
    logic [7:0] rx_shift;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state      <= IDLE;
            clk_cnt    <= 0;
            bit_idx    <= 0;
            rx_shift   <= 0;
            data       <= 0;
            data_valid <= 0;
        end else begin
            data_valid <= 0;

            case (state)

                IDLE: begin
                    if (rxd == 1'b0) begin  // start bit detected
                        clk_cnt <= CLKS_PER_BIT / 2;
                        state   <= START;
                    end
                end

                START: begin
                    if (clk_cnt == 0) begin
                        clk_cnt <= CLKS_PER_BIT - 1;
                        bit_idx <= 0;
                        state   <= DATA;
                    end else begin
                        clk_cnt <= clk_cnt - 1;
                    end
                end

                DATA: begin
                    if (clk_cnt == 0) begin
                        rx_shift[bit_idx] <= rxd;
                        clk_cnt <= CLKS_PER_BIT - 1;

                        if (bit_idx == 7)
                            state <= STOP;
                        else
                            bit_idx <= bit_idx + 1;
                    end else begin
                        clk_cnt <= clk_cnt - 1;
                    end
                end

                STOP: begin
                    if (clk_cnt == 0) begin
                        data       <= rx_shift;
                        data_valid <= 1'b1;
                        state      <= IDLE;
                    end else begin
                        clk_cnt <= clk_cnt - 1;
                    end
                end

            endcase
        end
    end

endmodule

module ble_packet_rx #(
    parameter int MAX_LEN = 256
)(
    input  logic clk,
    input  logic rst,

    input  logic [7:0] rx_byte,
    input  logic       rx_valid,

    output logic [MAX_LEN-1:0][7:0] packet,
    output logic [7:0] packet_len,
    output logic       packet_ready
);

    logic [7:0] index;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            index         <= 0;
            packet_len   <= 0;
            packet_ready <= 0;
        end else begin
            packet_ready <= 0;

            if (rx_valid) begin
                if (rx_byte == 8'h0A) begin  // newline
                    packet_len   <= index;
                    packet_ready <= 1'b1;
                    index        <= 0;
                end else begin
                    packet[index] <= rx_byte;
                    index <= index + 1;
                end
            end
        end
    end

endmodule

module ble_vector_parser (
    input  logic        clk,
    input  logic        rst,

    input  logic [255:0][7:0]  packet,
    input  logic [7:0]  packet_len,
    input  logic        packet_ready,

    output logic [7:0] initialize_mpu_motor, initialize_mpu,
    output logic [7:0] ble_pitch_kP, ble_pitch_kI, ble_pitch_kD,
    output logic [7:0] ble_yaw_kP, ble_yaw_kI, ble_yaw_kD,
    output logic [7:0] ble_set_pitch, ble_set_yaw,
    output logic              vector_valid
);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            initialize_mpu_motor <= 0; 
            initialize_mpu       <= 0;
            ble_set_pitch        <= 0;
            ble_set_yaw          <= 0;
            ble_pitch_kP         <= 0;
            ble_pitch_kI         <= 0; 
            ble_pitch_kD         <= 0;
            ble_yaw_kP           <= 0;
            ble_yaw_kI           <= 0;
            ble_yaw_kD           <= 0;
        end else begin
            vector_valid <= 0;

            if (packet_ready) begin
                // Expect exactly 2 data bytes
                if (packet_len == 10) begin
                    initialize_mpu_motor <= packet[0];
                    initialize_mpu       <= packet[1];  
                    ble_set_pitch        <= packet[2];
                    ble_set_yaw          <= packet[3];
                    ble_pitch_kP         <= packet[4];
                    ble_pitch_kI         <= packet[5];  
                    ble_pitch_kD         <= packet[6];
                    ble_yaw_kP           <= packet[7];
                    ble_yaw_kI           <= packet[8];
                    ble_yaw_kD           <= packet[9];
                    vector_valid <= 1'b1;
                end
            end
        end
    end

endmodule