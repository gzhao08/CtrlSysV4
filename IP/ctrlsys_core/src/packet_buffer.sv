`timescale 1ns/1ps

/*
Name: Gordon Zhao
File: packet_buffer.sv
Description: Synchronous BRAM FIFO for packet stream words.
*/

import config_pkg::*;

module packet_buffer #(
    parameter int DATA_WIDTH = 1024,
    parameter int DEPTH_WORDS = 1905,
    parameter int PACKET_WORDS = 127
)(
    input  logic                    clk,
    input  logic                    rst,

    input  logic                    wr_en,
    input  logic [DATA_WIDTH-1:0]   wr_data,

    input  logic                    rd_en,
    output logic [DATA_WIDTH-1:0]   rd_data,

    output logic                    empty,
    output logic                    full,
    output logic                    packet_space,
    output logic                    overflow,
    output logic                    underflow
);

localparam int PTR_WIDTH = (DEPTH_WORDS > 1) ? $clog2(DEPTH_WORDS) : 1;
localparam int COUNT_WIDTH = $clog2(DEPTH_WORDS + 1);

initial begin
    if (DATA_WIDTH < 1)
        $error("packet_buffer requires DATA_WIDTH >= 1");
    if (DEPTH_WORDS < 1)
        $error("packet_buffer requires DEPTH_WORDS >= 1");
    if (PACKET_WORDS < 1)
        $error("packet_buffer requires PACKET_WORDS >= 1");
    if (DEPTH_WORDS < PACKET_WORDS)
        $error("packet_buffer depth must hold at least one full packet");
end

(* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH_WORDS-1];

logic [PTR_WIDTH-1:0] wptr;
logic [PTR_WIDTH-1:0] rptr;
logic [COUNT_WIDTH-1:0] count;
logic [COUNT_WIDTH:0] free_words;

wire do_write;
wire do_read;

assign empty = count == 0;
assign full = count == DEPTH_WORDS;
assign free_words = DEPTH_WORDS - count;
assign packet_space = free_words >= PACKET_WORDS;
assign do_write = wr_en && !full;
assign do_read = rd_en && !empty;

always_ff @(posedge clk) begin
    if (rst) begin
        wptr <= '0;
        rptr <= '0;
        count <= '0;
        rd_data <= '0;
        overflow <= 1'b0;
        underflow <= 1'b0;
    end else begin
        overflow <= wr_en && !do_write;
        underflow <= rd_en && !do_read;

        if (do_write) begin
            mem[wptr] <= wr_data;

            if (wptr == DEPTH_WORDS - 1)
                wptr <= '0;
            else
                wptr <= wptr + 1'b1;
        end

        if (do_read) begin
            rd_data <= mem[rptr];

            if (rptr == DEPTH_WORDS - 1)
                rptr <= '0;
            else
                rptr <= rptr + 1'b1;
        end

        case ({do_write, do_read})
            2'b10: count <= count + 1'b1;
            2'b01: count <= count - 1'b1;
            default: count <= count;
        endcase
    end
end

endmodule
