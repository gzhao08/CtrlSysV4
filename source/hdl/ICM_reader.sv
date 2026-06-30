/* 
Name: Gordon Zhao
File: ICM_reader.sv
Description: an SPI master for performing burst reads on ICM-20948
*/

import config_pkg::*;

module ICM_reader #(
    parameter logic [6:0] REG_ADDR = 7'd45, // ICM accel_xout_h starts at address 45
    parameter integer SCLK_HALF_PERIOD_CYCLES = 63, // Approximately 1 MHz from a 125 MHz clk
    parameter int NUM_ICM = config_pkg::NUM_ICM,
    parameter int ICM_DATA_BYTES = config_pkg::ICM_DATA_BYTES
)(
    input logic         clk,
    input logic         rst,
    input logic         start,
    input logic [63:0]  timestamp,

    output ICM_frame_t  ICM_frame,

    output logic        busy,
    output logic        done,


    // SPI lines
    output logic sclk,
    output logic mosi,
    input  logic [NUM_ICM-1:0] miso,
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
localparam integer TICK_COUNTER_WIDTH = (SCLK_HALF_PERIOD_CYCLES > 1) ? $clog2(SCLK_HALF_PERIOD_CYCLES) : 1;
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
    if (NUM_ICM != config_pkg::NUM_ICM)
        $error("ICM_reader NUM_ICM must match config_pkg::ICM_frame_t");
    if (ICM_DATA_BYTES != config_pkg::ICM_DATA_BYTES)
        $error("ICM_reader ICM_DATA_BYTES must match config_pkg::ICM_measurement_t");
    if (SCLK_HALF_PERIOD_CYCLES < 1)
        $error("ICM_reader requires SCLK_HALF_PERIOD_CYCLES >= 1");
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
        ICM_frame.init_read_ts <= 64'b0;
        ICM_frame.done_read_ts <= 64'b0;
        for (sensor_idx = 0; sensor_idx < NUM_ICM; sensor_idx = sensor_idx + 1) begin
            ICM_frame.ICM_data[sensor_idx].sensor_id <= sensor_idx[7:0];
            ICM_frame.ICM_data[sensor_idx].data <= '0;
        end
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
                ICM_frame.init_read_ts <= timestamp;
                ICM_frame.done_read_ts <= 64'b0;
                for (sensor_idx = 0; sensor_idx < NUM_ICM; sensor_idx = sensor_idx + 1) begin
                    ICM_frame.ICM_data[sensor_idx].sensor_id <= sensor_idx[7:0];
                    ICM_frame.ICM_data[sensor_idx].data <= '0;
                end
            end
        end

        SEND_ADDR: begin
            // data transition on falling edge (i.e. drive data on falling edge)
            if (spi_clk_falling) begin
                mosi_drive <= data_temp[counter];
                if (counter == 0) begin
                    state <= WAIT_DATA;
                    num_data_bytes <= ICM_DATA_BYTES - 1;
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
                for (sensor_idx = 0; sensor_idx < NUM_ICM; sensor_idx = sensor_idx + 1) begin
                    ICM_frame.ICM_data[sensor_idx].data[num_data_bytes*8+counter] <= miso[sensor_idx];
                end

                if (counter == 0) begin
                    counter <= 7; // reset counter
                    if (num_data_bytes == 0) begin 
                        state <= IDLE;
                        done <= 1'b1;
                        tick_en <= 1'b0;
                        cs_drive <= 1'b1;
                        ICM_frame.done_read_ts <= timestamp;
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
