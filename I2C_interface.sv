//`default_nettype none
/**
*
* Implements a I2C interface to work with the boolean board.
* The GPIO pins will be used to transfer data. The boolean board
* will act as a controller device. It will also transfer paylaod information
* accordingly.
*
* Created by: Adam Kollgaard
* Date: 12/19/25
* */

/**
* An interface for the I2C protocol. Provides the ability for a single
* Read/Write operation on a single byte. Designed specifically for
* the MPU 6050
* */

module I2C_Interface(
	input logic clock, reset,
	input logic re, we,
	input logic start,
	input logic [7:0] address,
	input logic [7:0] we_data,
	inout tri scl, sda,
  output  logic en_sda, // This is for debugging
	output logic [7:0] re_data,
	output logic we_success,
	output logic done
);

logic en_sda1, en_sda2;

logic [7:0] addr;

logic [7:0] we_payload;
logic we_start;
logic we_done;

logic [7:0] re_payload;
logic re_start;
logic re_done;

logic data_clk;
tri clk;

Clock_Gen clockGen(.clock, .reset, .clk, .data_clk);

I2C_Write writer(.clock(clock), .reset(reset),
					       .address(addr),
					       .data(we_payload),
					       .start(we_start),
                 .data_clk(data_clk),
                 .clk(clk),
                 .sda(sda), .en_sda(en_sda1),
					       .success(we_success),
					       .done(we_done));

I2C_Read reader(.clock(clock), .reset(reset),
				        .address(addr),
				        .addr_in(re_start),
                .data_clk(data_clk),
                .clk(clk),
                .sda(sda), .en_sda(en_sda2),
				        .data(re_payload),
				        .done(re_done));

assign scl = clk;

always_comb begin
  en_sda = 1'd0;
  if(we) begin
    if(en_sda1) begin
      en_sda = 1'd1;
    end
  end
  else if(re) begin
    if(en_sda2) begin
      en_sda = 1'd1;
    end
  end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | (re_start | we_start)) begin
		re_start <= 1'd0;
		we_start <= 1'd0;
	end
	else if(start) begin
		if(re) begin
			re_start <= 1'd1;
		end
		else if(we) begin
			we_start <= 1'd1;
		end
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | done) begin
		done <= 1'd0;
	end
	else if(re_done | we_done) begin
		done <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		re_data <= 8'd0;
	end
	else if(re_done) begin
		re_data <= re_payload;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		we_payload <= 8'd0;
	end
	else if(start) begin
		we_payload <= we_data;
	end
end

always_ff @(posedge clock, posedge reset) begin
  if(reset) begin
    addr <= 8'd0;
  end
  else if(start) begin
    addr <= address;
  end
end

endmodule: I2C_Interface


/**
* The I2C Write transaction is as follows
*
* Start Condition: Pulls SDA low while SCL is high
* Address byte: Sends the 7bit worker address and a 0 bit for write
* ACK: The worker pulls the SDA line low
* Data: The controller sends the address of the register to write to
* ACK: The worker pulls the SDA line low
* Data: The controller sends the data to be written
* ACK: The worker pulls the SDA line low
* STOP: Pulls SDA high while SCL is high
*
*	The above transaction is for a WRITE operation on the given
*	address for specifically an MPU 6050 board. Only writes
*	for a single byte
*
* */
module I2C_Write
#(parameter WORKER = 7'b110_1000)
(
	input logic clock, reset,
	input logic [7:0] address,
	input logic [7:0] data,
	input logic start,
  input logic data_clk, clk,
	inout tri sda,
  output logic en_sda,
	output logic success,
	output logic done
);

localparam HALF_CYCLE = 500;
localparam FULL_CYCLE = 1000;

logic en;
logic data_bus, data_sda;

logic [9:0] start_count;
logic start_done, start_sending, start_wait;
logic send_start, start_data;

logic addr_start, addr_done, addr_sending;
logic addr_out;

logic [7:0] reg_data;
logic reg_start, reg_done, reg_sending;
logic reg_out;

logic [7:0] payload;
logic payload_start, payload_done, payload_sending;
logic payload_out;

logic got_ack;

logic [9:0] stop_count;
logic stop_done, stop_sending, pullStop;
logic stop_start, stop_low;

logic done_sending, close_bus;


Send_Byte workerAddr(.clock(clock), .reset,
					 .start(addr_start),
					 .clk(data_clk),
					 .data({WORKER, 1'd0}),
					 .out_data(addr_out),
					 .done(addr_done));

Send_Byte workerReg(.clock(clock), .reset,
					.start(reg_start),
					.clk(data_clk),
					.data(reg_data),
					.out_data(reg_out),
					.done(reg_done));

Send_Byte workerData(.clock(clock), .reset,
					 .start(payload_start),
					 .clk(data_clk),
					 .data(payload),
					 .out_data(payload_out),
					 .done(payload_done));

assign sda = (en) ? data_bus : 1'bz;
assign data_sda = sda;

assign en_sda = ~en;

assign start_data = start_count < HALF_CYCLE;
assign start_done = start_count == FULL_CYCLE;
assign stop_done = stop_count > HALF_CYCLE;
assign done_sending = addr_done | reg_done | payload_done;

enum logic [3:0] {INIT = 4'd0, START = 4'd1, ADDR = 4'd2,
				  ACK1 = 4'd3, REG = 4'd4, ACK2 = 4'd5,
				  DATA = 4'd6, ACK3 = 4'd7, STOP = 4'd8} state, nextState;

always_comb begin
	en = 1'd0;
  if(close_bus) begin
    en = 1'd0;
  end
	else if(start_sending) begin
		data_bus = start_data;
		en = 1'd1;
	end
	else if(addr_sending | addr_start) begin
		data_bus = addr_out;
		en = 1'd1;
	end
	else if(reg_sending) begin
		data_bus = reg_out;
		en = 1'd1;
	end
	else if(payload_sending) begin
		data_bus = payload_out;
		en = 1'd1;
	end
	else if((stop_sending | stop_done) & stop_low) begin
		data_bus = pullStop;
		en = 1'd1;
	end
end

// Assumes the ACK signal is high before the clock edge rises
always_comb begin
	if(~en & clk) begin
	  got_ack = ~data_sda;
  end
	else begin
		got_ack = 1'd0;
	end
end

always_ff @(posedge clock, posedge reset) begin
  if(reset) begin
    close_bus <= 1'd0;
  end
  else if(done_sending & got_ack) begin
    close_bus <= 1'd1;
  end
  else if(data_clk) begin
    close_bus <= 1'd0;
  end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		reg_data <= 8'd0;
	end
	else if(start) begin
		reg_data <= address;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		payload <= 8'd0;
	end
	else if(start) begin
		payload <= data;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | start_done) begin
		start_count <= 10'd0;
		start_sending <= 1'd0;
		start_wait <= 1'd0;
	end
	else if(send_start) begin
		start_sending <= 1'd1;
	end
	else if(start_sending & ~clk) begin
		start_wait <= 1'd1;
  end
	else if(start_sending & start_wait & clk) begin
		start_count <= start_count + 10'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | (stop_count > FULL_CYCLE)) begin
		stop_count <= 10'd0;
    stop_low <= 1'd0;
		pullStop <= 1'd0;
	end
	else if(stop_done) begin
		pullStop <= 1'd1;
		stop_count <= stop_count + 10'd1;
	end
	else if(stop_start | stop_sending) begin
    if(stop_low & clk) begin
		  stop_count <= stop_count + 10'd1;
    end
    else if(~clk) begin
      stop_low <= 1'd1;
    end
	end
end

// Next state logic
always_comb begin
	case(state)
		INIT: begin
			nextState = (start) ? START : INIT;
		end
		START: begin
			nextState = (start_done) ? ADDR : START;
		end
		ADDR: begin
			nextState = (addr_done) ? ACK1 : ADDR;
		end
		ACK1: begin
			if(~en & clk) begin
				nextState = (got_ack) ? REG : INIT;
			end
			else begin
				nextState = ACK1;
			end
		end
		REG: begin
			nextState = (reg_done) ? ACK2 : REG;
		end
		ACK2: begin
			if(~en & clk) begin
				nextState = (got_ack) ? DATA : INIT;
			end
			else begin
				nextState = ACK2;
			end
		end
		DATA: begin
			nextState = (payload_done) ? ACK3 : DATA;
		end
		ACK3: begin
			if(~en & clk) begin
				nextState = (got_ack) ? STOP : INIT;
			end
			else begin
				nextState = ACK3;
			end
		end
		STOP: begin
			nextState = (stop_done) ? INIT : STOP;
		end
	endcase
end

// Output logic
always_comb begin
	send_start = 1'd0;
	addr_start = 1'd0;
	reg_start = 1'd0;
	payload_start = 1'd0;
	addr_sending = 1'd0;
	reg_sending = 1'd0;
	payload_sending = 1'd0;
	stop_sending = 1'd0;
    stop_start = 1'd0;
	success = 1'd0;
    done = 1'd0;
	case(state)
		INIT: begin
			if(start) begin
				send_start = 1'd1;
			end
		end
		START: begin
			if(start_done) begin
				addr_start = 1'd1;
			end
		end
		ADDR: begin
			addr_sending = 1'd1;
		end
		ACK1: begin
			if(~en & clk) begin
				if(got_ack) begin
					reg_start = 1'd1;
					success = 1'd1;
				end
			end
		end
		REG: begin
			reg_sending = 1'd1;
		end
		ACK2: begin
			if(~en & clk) begin
				if(got_ack) begin
					payload_start = 1'd1;
					success = 1'd1;
				end
			end
		end
		DATA: begin
			payload_sending = 1'd1;
		end
		ACK3: begin
			if(~en & clk) begin
				if(got_ack) begin
					stop_start = 1'd1;
					success = 1'd1;
				end
			end
		end
		STOP: begin
			stop_sending = 1'd1;
      if(stop_done) begin
        done = 1'd1;
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

endmodule: I2C_Write
/**
* The I2C Read transaction is as follows
*
* Start Condition: Pulls SDA low while SCL is high
* Address byte: Sends the 7bit worker address and a 0 bit for write
* ACK: The worker pulls the SDA line low
* Data: The controller sends the address of the register to read from
* ACK: The worker pulls the SDA line low
* Start Condition: Pulls SDA low while SCL is high
* Address byte: Sends the 7bit worker address and a 1 bit for read
* ACK: The worker pulls the SDA line low
* Receive: The controller recieves the 8 bits
* NACK: The controller pulls the SDA line high
* STOP: Pulls SDA high while SCL is high
*
* The above transaction is for a READ operation on the given address
* for specifically an MPU 6050 board. Only reads a single byte
* */
module I2C_Read
#(parameter WORKER = 7'b110_1000) // Assumes pin AD0 is low
(
	input logic clock, reset,
	input logic [7:0] address,
	input logic addr_in, // Doubles as the 'start' signal; Assumes asserts for 1 cycle
  input logic data_clk, clk,
	inout tri sda,
  output logic en_sda,
	output logic [7:0] data,
	output logic done
);

// These are based on the generated clock
localparam HALF_CYCLE = 500;
localparam PART_CYCLE = 750;
localparam FULL_CYCLE = 1000;
localparam TWO_CYCLE = 2000;

logic data_sda;
logic data_bus;
logic en;

logic [7:0] reg_addr;
logic workerAddr_start, workerAddr_out, workerAddr_done;
logic workerAddr_sending;

logic workerData_start, workerData_out, workerData_done;
logic workerData_sending;

logic recData_start, recData_out, recData_done, recData_sending;
logic [7:0] recData;
logic recData_in;

logic [9:0] start_count;
logic start_data;
logic start_sending;
logic start_wait;
logic send_start;
logic start_done;

logic reading;

logic [10:0] timeout_counter;
logic start_timeout;
logic timeout;

logic send_data;

logic got_ack;

logic [10:0] nack_count;
logic start_nack;
logic nack_sending;
logic nack_done;

logic [10:0] stop_count;
logic stop_sending;
logic stop_start;
logic stop_done;
logic pullStop;

logic close_bus, done_sending;

Send_Byte workerAddr(.clock(clock), .reset,
					 .start(workerAddr_start),
                     .clk(data_clk),
					 .data({WORKER, reading}),
					 .out_data(workerAddr_out),
					 .done(workerAddr_done));
Send_Byte workerData(.clock(clock), .reset,
					 .start(workerData_start),
                     .clk(data_clk),
					 .data(reg_addr),
					 .out_data(workerData_out),
					 .done(workerData_done));
Receive_Byte recievedData(.clock(clock), .reset,
						  .start(recData_start),
                          .data_clk(data_clk),
                          .clk(clk),
						  .data_in(recData_in),
						  .data(recData),
                          .done(recData_done),
						  .data_out(recData_out));

assign sda = (en) ? data_bus : 1'bz;
assign data_sda = sda;

assign en_sda = ~en;

assign start_data = start_count < HALF_CYCLE;
assign start_done = start_count == FULL_CYCLE;
assign timeout = timeout_counter >= TWO_CYCLE;
assign nack_done = nack_count > TWO_CYCLE;
assign stop_done = stop_count > FULL_CYCLE;
assign done_sending = workerData_done | workerAddr_done;

enum logic [2:0] {INIT = 3'd0, START = 3'd1, ADDR = 3'd2,
				  ACK = 3'd3, DATA = 3'd4, RECEIVE = 3'd5,
				  NACK = 3'd6, STOP = 3'd7} state, nextState;

always_comb begin
	en = 1'd0;
	recData_in = 1'd0;
	data_bus = 1'd0;
	if(close_bus) begin
		en = 1'd0;
	end
	else if(start_sending) begin
		data_bus = start_data;
		en = 1'd1;
	end
	else if(workerAddr_sending | workerAddr_start) begin
		data_bus = workerAddr_out;
		en = 1'd1;
	end
	else if(workerData_sending) begin
		data_bus = workerData_out;
		en = 1'd1;
	end
	else if(recData_sending) begin
		recData_in = data_sda;
	end
	else if(nack_sending | start_nack) begin
		data_bus = 1'd1;
		en = 1'd1;
	end
	else if(stop_sending | stop_start | stop_done) begin
		data_bus = pullStop;
		en = 1'd1;
	end
end

always_comb begin
	if(~en & clk) begin
		got_ack = ~data_sda;
	end
	else begin
		got_ack = 1'd0;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		close_bus <= 1'd0;
	end
	else if(done_sending & got_ack) begin
		close_bus <= 1'd1;
	end
	else if(data_clk) begin
		close_bus <= 1'd0;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		data <= 8'd0;
	end
	else if(recData_out) begin
		data <= recData;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | done) begin
		send_data <= 1'd0;
	end
	else if(workerData_done) begin
		send_data <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | done) begin
		reading <= 1'd0;
	end
	else if(send_data & send_start) begin
		reading <= 1'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | ~start_timeout) begin
		timeout_counter <= 1'd0;
	end
	else if(start_timeout) begin
		timeout_counter <= timeout_counter + 8'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		reg_addr <= 8'd0;
	end
	else if(addr_in) begin
		reg_addr <= address;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | start_done) begin
		start_count <= 10'd0;
		start_sending <= 1'd0;
		start_wait <= 1'd0;
	end
	else if(send_start) begin
		start_sending <= 1'd1;
	end
	else if(start_sending & ~clk) begin
		start_wait <= 1'd1;
  end
	else if(start_sending & start_wait & clk) begin
		start_count <= start_count + 10'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | (stop_count > TWO_CYCLE)) begin
		stop_count <= 4'd0;
		pullStop <= 1'd0;
	end
	else if(stop_done) begin
		pullStop <= 1'd1;
		stop_count <= stop_count + 4'd1;
	end
	else if(stop_start | stop_sending) begin
		stop_count <= stop_count + 4'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | nack_done) begin
		nack_count <= 4'd0;
	end
	else if(nack_sending | start_nack) begin
		nack_count <= nack_count + 4'd1;
	end
end

// FSM
always_comb begin
	case(state)
		INIT: begin
			nextState = (addr_in) ? START : INIT;
		end
		START: begin
			nextState = (start_done) ? ADDR : START;
		end
		ADDR: begin
			nextState = (workerAddr_done) ? ACK : ADDR;
		end
		ACK: begin
			if(timeout) begin
				nextState = INIT;
			end
			if(got_ack) begin
				if(reading) begin
					nextState = RECEIVE;
				end
				else if(send_data) begin
					nextState = START;
				end
				else begin
					nextState = DATA;
				end
			end
			else begin
				nextState = ACK;
			end
		end
		DATA: begin
			nextState = (workerData_done) ? ACK : DATA;
		end
		RECEIVE: begin
			nextState = (recData_done) ? NACK : RECEIVE;
		end
		NACK: begin
			nextState = (nack_done) ? STOP : NACK;
		end
		STOP: begin
			nextState = (stop_done) ? INIT : STOP;
		end
	endcase
end

// OUTPUT logic

always_comb begin
	send_start = 1'd0;
	start_timeout = 1'd0;
	workerAddr_start = 1'd0;
	workerAddr_sending = 1'd0;
	workerData_start = 1'd0;
	workerData_sending = 1'd0;
	recData_start = 1'd0;
	recData_sending = 1'd0;
  	start_nack = 1'd0;
	nack_sending = 1'd0;
	stop_start = 1'd0;
	stop_sending = 1'd0;
	done = 1'd0;
	case(state)
		INIT: begin
			if(addr_in) begin
				send_start = 1'd1;
			end
		end
		START: begin // Does everything on its own
			if(start_done) begin
				workerAddr_start = 1'd1;
			end
		end
		ADDR: begin
			workerAddr_sending = 1'd1;
		end
		ACK: begin
			start_timeout = 1'd1;
			if(got_ack) begin
				if(reading) begin
					recData_start = 1'd1;
				end
				else if(send_data) begin
					send_start = 1'd1;
				end
				else begin
					workerData_start = 1'd1;
				end
			end
		end
		DATA: begin
			workerData_sending = 1'd1;
		end
		RECEIVE: begin
			recData_sending = 1'd1;
			if(recData_done) begin
				start_nack = 1'd1;
			end
		end
		NACK: begin
			nack_sending = 1'd1;
			if(nack_done) begin
				stop_start = 1'd1;
			end
		end
		STOP: begin
			stop_sending = 1'd1;
			if(stop_done) begin
				done <= 1'd1;
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

endmodule: I2C_Read


module Receive_Byte(
	input  logic 			 clock, reset, start,
  	input  logic       data_clk, clk,
	input  logic 			 data_in,
	output logic [7:0] data,
	output logic 			 done, data_out
);

logic counting;
logic [2:0] count;
logic starting;
logic info;

always_ff @(posedge clock, posedge reset) begin
  if(reset) begin
    info <= 1'd0;
  end
  else if(clk) begin
    info <= data_in;
  end
end

always_ff @(posedge clock, posedge reset) begin
  if(reset | done) begin
    starting <= 1'd0;
  end
  else if(start) begin
    starting <= 1'd1;
  end
end

always_ff @(posedge data_clk, posedge reset) begin
  if(reset | (~counting)) begin
    done <= 1'd0;
  end
  else if(count == 3'd0) begin
    done <= 1'd1;
  end
end

always_ff @(posedge clk, posedge reset) begin
	if(reset | data_out) begin
		data <= 8'd0;
    data_out <= 1'd0;
		count <= 3'd7;
    counting <= 1'd0;
	end
  else if(done) begin
    data <= {data[6:0], info};
    data_out <= 1'd1;
  end
  else if(counting) begin
    count <= count - 3'd1;
    data <= {data[6:0], info};
  end
  else if(starting) begin
    data <= {data[6:0], info};
    counting <= 1'd1;
  end
end

endmodule: Receive_Byte

// Assumes data will always be on the line between the
// assertion of start and the assertion of done.
// Assumes start and done are only asserted once
// Sends data MSB first
module Send_Byte(
	input  logic clock, reset,
  	input  logic start, clk,
	input  logic [7:0] data,
	output logic out_data,
	output logic done
);

logic [3:0] count;
logic counting;
logic starting;

assign out_data = data[count];

always_ff @(posedge clock, posedge reset) begin
  if(reset) begin
    starting <= 1'd0;
  end
  else if(start) begin
    starting <= 1'd1;
  end
  else if(clk) begin
    starting <= 1'd0;
  end
end

always_ff @(posedge clk, posedge reset) begin
	if(reset | done) begin
		counting <= 1'd0;
		count <= 4'd7;
		done <= 1'd0;
	end
	else if(count <= 4'd0) begin
		done <= 1'd1;
	end
	else if(counting) begin
		count <= count - 4'd1;
	end
	else if(starting) begin
		counting <= 1'd1;
	end
end

endmodule: Send_Byte

// Every 1000 clock cycles it flips clk. Effectively divides the clock by 1000
// The purpose of data_clk is to create a clock edge that strikes right before
// clk is asserted. This enables an effective data transfer for I2C data
module Clock_Gen(
	input logic clock, reset,
	inout tri clk,
	output logic data_clk
);

logic clock_temp;
logic [9:0] clock_count;

logic [8:0] data_clk_count;

logic en;

assign clk = (en) ? clock_temp : 1'bz;
assign data_clk = data_clk_count == 9'd500;

assign en = ~reset;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		clock_temp <= 1'd0;
		clock_count <= 10'd0;
	end
	else if(clock_count > 10'd999) begin
		clock_temp <= ~clock_temp;
		clock_count <= 10'd0;
	end
	else begin
		clock_count <= clock_count + 10'd1;
	end
end

always_ff @(posedge clock, posedge reset) begin
	if(reset | clock_temp) begin
		data_clk_count <= 2'd0;
	end
	else if(~clock_temp & ~data_clk) begin
		data_clk_count <= data_clk_count + 2'd1;
	end
end

endmodule: Clock_Gen

/**
* Test bench for the I2C interface.
* Simulates the workings of the actual MPU connected on the BUS.
* Initially tests a simple read and write transactions as well
* as some more real world tests.
*
* */
/*
module I2C_TB();

	logic clock, reset;
	logic re, we, start;
	logic [7:0] address, we_data, re_data;
	logic we_success, done;
	tri scl, sda;
	logic sda_data, en;
	logic data_bus;

	I2C_Interface DUT(.*);

	assign sda = (en) ? sda_data : 1'bz;
	assign scl = 1'bz; // We don't want to manipulate the clock
	assign data_bus = sda;

	task reset_values();
		reset <= 1'd1;
    	sda_data <= 1'd0;
		en <= 1'd0;
		re <= 1'd0;
		we <= 1'd0;
		start <= 1'd0;
		address <= 8'd0;
		we_data <= 8'd0;
		@(posedge clock);
		reset <= 1'd0;
		@(posedge clock);
	endtask

	task wait_ack();
		for(logic [3:0] i = 4'd0; i < 4'd8; i=i+1) begin
			@(posedge scl);
		end
		@(negedge scl);
		en <= 1'd1;
		sda_data <= 1'd0;
		@(negedge scl);
		en <= 1'd0;
	endtask

  task send_info(
    input logic [7:0] data
  );
    en <= 1'd1;
    for(logic [3:0] i = 4'd0; i < 4'd8; i=i+1) begin
      sda_data <= data[4'd7 - i];
      @(negedge scl);
    end
    en <= 1'd0;
  endtask

	task write_transaction(
		input logic [7:0] addr,
		input logic [7:0] data);
		we <= 1'd1;
		address <= addr;
		we_data <= data;
		@(posedge clock);
		start <= 1'd1;
		@(posedge clock);
		start <= 1'd0;
		@(negedge data_bus); // start condition
		wait_ack();
		wait_ack();
		wait_ack();
		@(posedge scl);
		@(negedge scl);
	endtask

  	task read_transaction(
		input  logic [7:0] addr,
		input  logic [7:0] data_in);
		re <= 1'd1;
		address <= addr;
		@(posedge clock);
		start <= 1'd1;
		@(posedge clock);
		start <= 1'd0;
		@(negedge data_bus); // Start condition
		wait_ack();
		wait_ack();
		@(negedge data_bus); // Restart condition
		wait_ack();
		send_info(data_in);
		@(posedge scl);
		@(posedge scl);
		@(negedge scl);
  	endtask

	initial begin
		clock <= 1'd0;
		forever #1 clock <= ~clock;
	end

	initial begin
		reset_values();
		write_transaction(.addr(8'h68), .data(8'h12));
		write_transaction(.addr(8'h23), .data(8'hA9));
		write_transaction(.addr(8'h23), .data(8'h00));
		write_transaction(.addr(8'h23), .data(8'hFF));
		reset_values();
		read_transaction(.addr(8'h76), .data_in(8'h84));
		read_transaction(.addr(8'h3E), .data_in(8'hF0));
		read_transaction(.addr(8'h3E), .data_in(8'h00));
		read_transaction(.addr(8'h3E), .data_in(8'hFF));
		@(posedge scl);
		@(posedge scl);
		@(posedge scl);
		$finish;
	end

endmodule: I2C_TB
*/
