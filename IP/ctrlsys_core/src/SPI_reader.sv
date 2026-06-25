`timescale 1ns/1ps

/* 
Name: Gordon Zhao
File: SPI_reader.sv
Description: an SPI master for performing burst reads on ICM-20948
*/

module SPI_reader #(
    parameter logic [6:0] REG_ADDR = 7'd45, // ICM accel_xout_h starts at address 45
    parameter integer DATA_BYTES = 20, // the number of data bytes to read per sensor
    parameter integer NUM_SENSORS = 1,
    parameter integer SCLK_HALF_PERIOD_CYCLES = 63 // Approximately 1 MHz from a 125 MHz clk
)(
    input logic                         clk,
    input logic                         rst,
    input logic                         start,
    input logic [63:0]                  timestamp,


    output logic [8*DATA_BYTES-1:0] data_out [NUM_SENSORS-1:0],

    output logic [63:0] startRead_timestamp,
    output logic [63:0] doneRead_timestamp,
    output logic busy,
    output logic done,


    // SPI lines
    output logic sclk,
    output logic mosi,
    input  logic [NUM_SENSORS-1:0] miso,
    output logic cs_n 
);

localparam logic [1:0] IDLE      = 2'b00;
localparam logic [1:0] SEND_ADDR = 2'b01;
localparam logic [1:0] WAIT_DATA = 2'b10;
localparam logic [1:0] READ_DATA = 2'b11;

// sequential copies of spi lines
logic cs_drive;
logic mosi_drive;


// SPI timing
logic spi_tick;
logic tick_en;
localparam integer TICK_COUNTER_WIDTH =
    (SCLK_HALF_PERIOD_CYCLES > 1) ? $clog2(SCLK_HALF_PERIOD_CYCLES) : 1;
logic [TICK_COUNTER_WIDTH-1:0] tickCounter;
logic spi_clk_rising;
logic spi_clk_falling;

// State machine
logic [1:0] state;

// data
logic [7:0] data_temp;
logic [2:0] counter;
logic [7:0] num_data_bytes;

initial begin
    if (NUM_SENSORS < 1)
        $error("SPI_reader requires NUM_SENSORS >= 1");
    if (DATA_BYTES < 1 || DATA_BYTES > 256)
        $error("SPI_reader requires 1 <= DATA_BYTES <= 256");
    if (SCLK_HALF_PERIOD_CYCLES < 1)
        $error("SPI_reader requires SCLK_HALF_PERIOD_CYCLES >= 1");
end

assign cs_n = cs_drive;
assign mosi = mosi_drive;
assign sclk = spi_tick;

assign busy = (state != IDLE);



// spi timing (sclk)

always_ff @(posedge clk) begin
    if (rst) begin
        tickCounter <= '0;
        spi_tick <= 1'b0;
        spi_clk_rising <= 1'b0;
        spi_clk_falling <= 1'b0;
    end else begin
        spi_clk_rising <= 1'b0;
        spi_clk_falling <= 1'b0;
        if (tick_en) begin
            if (tickCounter == SCLK_HALF_PERIOD_CYCLES - 1) begin
                tickCounter <= '0;
                spi_tick <= ~spi_tick;
                if (spi_tick == 1) begin
                    spi_clk_falling <= 1'b1;
                end
                else begin
                    spi_clk_rising <= 1'b1;
                end
            end
            else
                tickCounter <= tickCounter + 1'b1;
        end
        else begin
            tickCounter <= '0;
            spi_tick <= 1'b0;
        end
    end
end 


integer sensor_idx;
always_ff @(posedge clk) begin

    // reset behavior
    if (rst) begin
        state <= IDLE;
        cs_drive <= 1'b1; // cs is active low
        tick_en <= 1'b0;
        mosi_drive <= 1'b0;
        done <= 1'b0;
        data_temp <= 8'b0;
        counter <= 3'b0;
        num_data_bytes <= 8'b0;
        startRead_timestamp <= 64'b0;
        doneRead_timestamp <= 64'b0;
        for (sensor_idx = 0; sensor_idx < NUM_SENSORS; sensor_idx = sensor_idx + 1)
            data_out[sensor_idx] <= '0;
    end else begin
    done <= 1'b0;
    case (state)
        IDLE: begin
            if (start) begin
                state <= SEND_ADDR;
                tick_en <= 1'b1;
                data_temp <= {1'b1,REG_ADDR};
                mosi_drive <= 1'b1;
                counter <= 3'd6;
                cs_drive <= 1'b0; // pull cs low to get sensor ready for data
                startRead_timestamp <= timestamp;
            end
        end

        SEND_ADDR: begin
            // data transition on falling edge (i.e. drive data on falling edge)
            if (spi_clk_falling) begin
                mosi_drive <= data_temp[counter];
                if (counter == 0) begin
                    state <= WAIT_DATA;
                    num_data_bytes <= DATA_BYTES - 1;
                end
                counter <= counter - 3'b1; // should reset counter to 7 after 0
            end
        end

        WAIT_DATA: begin
            // The sensor samples the final address bit on the rising edge and
            // drives the first response bit on the following falling edge.
            if (spi_clk_falling)
                state <= READ_DATA;
        end

        READ_DATA: begin
            if (spi_clk_rising) begin
                // read and store data
                for (int sensor_idx = 0; sensor_idx < NUM_SENSORS; sensor_idx++) begin
                    data_out[sensor_idx][num_data_bytes*8+counter] <= miso[sensor_idx];
                end

                if (counter == 0) begin
                    counter <= 7; // reset counter
                    if (num_data_bytes == 0) begin 
                        state <= IDLE;
                        done <= 1'b1;
                        tick_en <= 1'b0;
                        cs_drive <= 1'b1;
                        doneRead_timestamp <= timestamp;
                    end
                    else
                        num_data_bytes <= num_data_bytes - 1;
                end else 
                    counter <= counter - 1;
            end
        end

        default: begin
            state <= IDLE;
            cs_drive <= 1'b1;
            tick_en <= 1'b0;
            done <= 1'b0;
            mosi_drive <= 1'b0;
        end
    endcase
    end
end

endmodule
