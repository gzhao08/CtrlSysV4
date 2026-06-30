/*
Name: Gordon Zhao
File: Intan_reader.sv
Description: Synthetic Intan frame source for datapath testing.
*/

import config_pkg::*;

module Intan_reader #(
    parameter int DONE_DELAY_CYCLES = 1,
    parameter int NUM_INTAN = config_pkg::NUM_INTAN,
    parameter int INTAN_DATA_BYTES = config_pkg::INTAN_DATA_BYTES
)(
    input  logic         clk,
    input  logic         rst,
    input  logic         start,
    input  logic [63:0]  timestamp,

    output Intan_frame_t Intan_frame,
    output logic         busy,
    output logic         done
);

localparam int DELAY_COUNTER_WIDTH = (DONE_DELAY_CYCLES > 1) ? $clog2(DONE_DELAY_CYCLES) : 1;

logic [DELAY_COUNTER_WIDTH-1:0] delay_counter;
logic [31:0] sample_counter;

integer sensor_idx;
integer word_idx;

initial begin
    if (DONE_DELAY_CYCLES < 1)
        $error("Intan_reader requires DONE_DELAY_CYCLES >= 1");
    if (NUM_INTAN != config_pkg::NUM_INTAN)
        $error("Intan_reader NUM_INTAN must match config_pkg::Intan_frame_t");
    if (INTAN_DATA_BYTES != config_pkg::INTAN_DATA_BYTES)
        $error("Intan_reader INTAN_DATA_BYTES must match config_pkg::Intan_measurement_t");
end

always_ff @(posedge clk) begin
    if (rst) begin
        busy <= 1'b0;
        done <= 1'b0;
        delay_counter <= '0;
        sample_counter <= 32'b0;
        Intan_frame.init_read_ts <= 64'b0;
        Intan_frame.done_read_ts <= 64'b0;

        for (sensor_idx = 0; sensor_idx < NUM_INTAN; sensor_idx = sensor_idx + 1) begin
            Intan_frame.Intan_data[sensor_idx].sensor_id <= sensor_idx[7:0];
            Intan_frame.Intan_data[sensor_idx].data <= '0;
        end
    end else begin
        done <= 1'b0;

        if (!busy) begin
            if (start) begin
                busy <= 1'b1;
                delay_counter <= DONE_DELAY_CYCLES - 1;
                Intan_frame.init_read_ts <= timestamp;
                Intan_frame.done_read_ts <= 64'b0;

                for (sensor_idx = 0; sensor_idx < NUM_INTAN; sensor_idx = sensor_idx + 1) begin
                    Intan_frame.Intan_data[sensor_idx].sensor_id <= sensor_idx[7:0];

                    for (word_idx = 0; word_idx < INTAN_DATA_BYTES / 4; word_idx = word_idx + 1) begin
                        Intan_frame.Intan_data[sensor_idx].data[word_idx*32 +: 32] <=
                            sample_counter + sensor_idx[31:0] * 32 + word_idx[31:0];
                    end
                end
            end
        end else begin
            if (delay_counter == 0) begin
                busy <= 1'b0;
                done <= 1'b1;
                sample_counter <= sample_counter + 1'b1;
                Intan_frame.done_read_ts <= timestamp;
            end else begin
                delay_counter <= delay_counter - 1'b1;
            end
        end
    end
end

endmodule
