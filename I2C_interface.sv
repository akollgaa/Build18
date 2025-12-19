`default_nettype none
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
* The I2C transaction is as follows
*
* Start Condition: Pulls SDA low while SCL is high
* Address byte: Sends the 7bit worker address and a 0 bit for write
* ACK: The worker pulls the SDA line low
* Data: The controller ends the address of the register to read from
* ACK: The worker pulls the SDA line low
* Start Condition: Pulls SDA low while SCL is high
* Address byte: Sends the 7bit worker address and a 1 bit for read
* ACK: The worker pulls the SDA line low
* Receive: The controller recieves the 8 bits
* NACK: The controller pulls the SDA line high
* STOP: Pulls SDA high while SCL is high
*
* */


// TODO: Create a STD/datapath for the I2C interface based on the above transaction
module I2C_interface
#(parameter WORKER = 8'd0000_0000)
(
	input logic clock, reset,
	input logic [7:0] address,
	input logic addr_in,
	inout tri scl, sda,
	output logic [7:0] data
);

logic [7:0] addr;

tri clk;

Clock_Gen clkGen(.clock, .reset, .clk);

assign scl = clk;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		addr <= 8'd0;
	end
	else if(addr_in) begin
		addr <= address;
	end
end


module: I2C_interface

// Every 4 clock cycles it flips clk. Effectively divides the clock by 4
module Clock_Gen(
	input logic clock, reset,
	inout tri clk
);

logic clock_temp;
logic [2:0] clock_count;

logic en;

assign clk = (en) ? clock_temp : 1'bz;

assign en = ~reset;

always_ff @(posedge clock, posedge reset) begin
	if(reset) begin
		clock_temp <= 1'd0;
		clock_count <= 3'd0;
	end
	else if(clock_count > 3'd3) begin
		clock_temp <= ~clock_temp;
		clock_count <= 3'd0;
	end
	else begin
		clock_count <= clock_count + 3'd1;
	end
end

endmodule: Clock_Gen
