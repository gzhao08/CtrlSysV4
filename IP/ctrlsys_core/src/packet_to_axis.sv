`timescale 1ns/1ps

/*
Name: Gordon Zhao
File: packet_to_axis.sv
Description: Streams buffered packet words to AXI Stream.
*/

import config_pkg::*;

module packet_to_axis #(
    parameter int DATA_WIDTH = 1024,
    parameter int PACKET_WORDS = 127,
    parameter int PACKET_LAST_BYTES = 120
)(
    input  logic                    clk,
    input  logic                    rst,

    output logic                    fifo_rd_en,
    input  logic [DATA_WIDTH-1:0]   fifo_rd_data,
    input  logic                    fifo_empty,

    output logic                    m_axis_tvalid,
    input  logic                    m_axis_tready,
    output logic [DATA_WIDTH-1:0]   m_axis_tdata,
    output logic [DATA_WIDTH/8-1:0] m_axis_tkeep,
    output logic                    m_axis_tlast
);

localparam int KEEP_WIDTH = DATA_WIDTH / 8;
localparam int WORD_INDEX_WIDTH = (PACKET_WORDS > 1) ? $clog2(PACKET_WORDS) : 1;

typedef enum logic [1:0] {
    IDLE,
    WAIT_FOR_WORD,
    SEND_WORD
} state_t;

state_t state;
logic [WORD_INDEX_WIDTH-1:0] word_index;

initial begin
    if (DATA_WIDTH < 8 || (DATA_WIDTH % 8) != 0)
        $error("packet_to_axis requires DATA_WIDTH to be a positive byte multiple");
    if (PACKET_WORDS < 1)
        $error("packet_to_axis requires PACKET_WORDS >= 1");
    if (PACKET_LAST_BYTES < 1 || PACKET_LAST_BYTES > KEEP_WIDTH)
        $error("packet_to_axis requires 1 <= PACKET_LAST_BYTES <= DATA_WIDTH/8");
end

function automatic logic [KEEP_WIDTH-1:0] keep_for_word(
    input logic [WORD_INDEX_WIDTH-1:0] index
);
    logic [KEEP_WIDTH-1:0] keep;
begin
    keep = '1;

    if (index == PACKET_WORDS - 1 && PACKET_LAST_BYTES != KEEP_WIDTH) begin
        keep = '0;
        keep[PACKET_LAST_BYTES-1:0] = '1;
    end

    keep_for_word = keep;
end
endfunction

always_ff @(posedge clk) begin
    if (rst) begin
        state <= IDLE;
        word_index <= '0;
        fifo_rd_en <= 1'b0;
        m_axis_tvalid <= 1'b0;
        m_axis_tdata <= '0;
        m_axis_tkeep <= '0;
        m_axis_tlast <= 1'b0;
    end else begin
        fifo_rd_en <= 1'b0;

        case (state)
            IDLE: begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast <= 1'b0;
                m_axis_tkeep <= '0;

                if (!fifo_empty) begin
                    fifo_rd_en <= 1'b1;
                    state <= WAIT_FOR_WORD;
                end
            end

            WAIT_FOR_WORD: begin
                m_axis_tdata <= fifo_rd_data;
                m_axis_tkeep <= keep_for_word(word_index);
                m_axis_tlast <= word_index == PACKET_WORDS - 1;
                m_axis_tvalid <= 1'b1;
                state <= SEND_WORD;
            end

            SEND_WORD: begin
                if (m_axis_tready) begin
                    m_axis_tvalid <= 1'b0;

                    if (word_index == PACKET_WORDS - 1)
                        word_index <= '0;
                    else
                        word_index <= word_index + 1'b1;

                    if (!fifo_empty) begin
                        fifo_rd_en <= 1'b1;
                        state <= WAIT_FOR_WORD;
                    end else begin
                        state <= IDLE;
                    end
                end
            end

            default: begin
                state <= IDLE;
                word_index <= '0;
                fifo_rd_en <= 1'b0;
                m_axis_tvalid <= 1'b0;
                m_axis_tlast <= 1'b0;
                m_axis_tkeep <= '0;
            end
        endcase
    end
end

endmodule
