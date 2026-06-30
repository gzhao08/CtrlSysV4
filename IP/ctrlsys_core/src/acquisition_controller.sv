/* 
Name: Gordon Zhao
File: acquisition_controller.sv
Description: FPGA-side acquisition controller
*/

`timescale 1ns/1ps

module acquisition_controller ( 	
	input  logic        clk,
	input  logic        rst,
	input  logic        enable,         // Trigger reads while enable is high
	input  logic [63:0] timestamp,
	input  logic [63:0] sample_period_ICM,
	input  logic [63:0] sample_period_Intan,		  
	output logic        startRead_ICM,       	// ICM read start pulse
	output logic 		startRead_Intan			// Intan RH2164 read start pulse 	
);

	logic [63:0] prev_sample_time_ICM;
	logic [63:0] prev_sample_time_Intan;
	logic prev_enable;

	always_ff @(posedge clk) begin
		if (rst) begin
			prev_sample_time_ICM 	<= 0;
			prev_sample_time_Intan	<= 0;
			startRead_ICM 		<= 0;
			startRead_Intan		<= 0;
			prev_enable		 	<= 0;
		end else begin
			startRead_ICM 		<= 0;
			startRead_Intan		<= 0;

			if (prev_enable == 0 && enable == 1) begin
				prev_sample_time_ICM <= timestamp;
				prev_sample_time_Intan <= timestamp;
				startRead_ICM <= 1;
				startRead_Intan <= 1;
			end

			else if (enable) begin
				// ICM
				if ((timestamp - prev_sample_time_ICM) >= sample_period_ICM) begin
					startRead_ICM <= 1;
					prev_sample_time_ICM <= prev_sample_time_ICM + sample_period_ICM;
				end

				// Intan
				if ((timestamp - prev_sample_time_Intan) >= sample_period_Intan) begin
					startRead_Intan <= 1;
					prev_sample_time_Intan <= prev_sample_time_Intan + sample_period_Intan;
				end
			end
			prev_enable <= enable;
		end
	end

endmodule
