/*
Name: Gordon Zhao
File: packet_writer.sv
Description: Packs Intan-first DMA packet words without storing a full packet_t.
*/

import config_pkg::*;

module packet_writer #(
    parameter int DATA_WIDTH = 1024,
    parameter int INTAN_SAMPLING_RATIO = 30,
    parameter int PACKET_WORDS = 127,
    parameter int INTAN_FRAME_BITS = config_pkg::INTAN_FRAME_BITS,
    parameter int ICM_FRAME_BITS = config_pkg::ICM_FRAME_BITS,
    parameter int PACKET_HEADER_BITS = config_pkg::PACKET_HEADER_BITS
)(
    input  logic                  clk,
    input  logic                  rst,

    input  logic                  ICM_frame_done,
    input  logic                  Intan_frame_done,
    input  ICM_frame_t            ICM_frame_in,
    input  Intan_frame_t          Intan_frame_in,

    input  logic                  packet_ready,
    output logic                  ready,

    output logic                  word_valid,
    input  logic                  word_ready,
    output logic [DATA_WIDTH-1:0] word_data,
    output logic                  packet_done
);

localparam int AXIS_BYTES = DATA_WIDTH / 8;
localparam int INTAN_FRAME_BYTES = INTAN_FRAME_BITS / 8;
localparam int ICM_FRAME_BYTES = ICM_FRAME_BITS / 8;
localparam int PACKET_HEADER_BYTES = PACKET_HEADER_BITS / 8;
localparam int BYTE_INDEX_WIDTH = (AXIS_BYTES > 1) ? $clog2(AXIS_BYTES) : 1;
localparam int INTAN_BYTE_INDEX_WIDTH = (INTAN_FRAME_BYTES > 1) ? $clog2(INTAN_FRAME_BYTES) : 1;
localparam int ICM_BYTE_INDEX_WIDTH = (ICM_FRAME_BYTES > 1) ? $clog2(ICM_FRAME_BYTES) : 1;
localparam int HEADER_BYTE_INDEX_WIDTH = (PACKET_HEADER_BYTES > 1) ? $clog2(PACKET_HEADER_BYTES) : 1;
localparam int INTAN_COUNTER_WIDTH = $clog2(INTAN_SAMPLING_RATIO + 1);

typedef enum logic [2:0] {
    IDLE,
    STREAM_INTAN,
    STREAM_ICM,
    STREAM_HEADER
} state_t;

state_t state;

(* ram_style = "distributed" *) logic [INTAN_FRAME_BITS-1:0] intan_buffer [1:0];
logic intan_wptr;
logic intan_rptr;
logic [1:0] intan_count;
logic intan_overflow;

ICM_frame_t icm_frame_reg;
logic icm_valid;
packet_header_t header_reg;
logic [31:0] packet_counter;

logic [INTAN_COUNTER_WIDTH-1:0] intan_frames_written;
logic [INTAN_COUNTER_WIDTH-1:0] intan_frames_streamed;
logic [INTAN_BYTE_INDEX_WIDTH-1:0] intan_byte_index;
logic [ICM_BYTE_INDEX_WIDTH-1:0] icm_byte_index;
logic [HEADER_BYTE_INDEX_WIDTH-1:0] header_byte_index;
logic [BYTE_INDEX_WIDTH-1:0] pack_byte_count;
logic [DATA_WIDTH-1:0] pack_word;

logic source_valid;
logic source_last;
logic packet_last;
logic [7:0] source_byte;
logic [DATA_WIDTH-1:0] pack_word_with_byte;
logic can_pack_byte;
logic emit_word;
logic consume_intan_frame;

initial begin
    if (DATA_WIDTH < 8 || (DATA_WIDTH % 8) != 0)
        $error("packet_writer requires DATA_WIDTH to be a positive byte multiple");
    if ((INTAN_FRAME_BITS % 8) != 0)
        $error("packet_writer requires INTAN_FRAME_BITS to be byte aligned");
    if ((ICM_FRAME_BITS % 8) != 0)
        $error("packet_writer requires ICM_FRAME_BITS to be byte aligned");
    if ((PACKET_HEADER_BITS % 8) != 0)
        $error("packet_writer requires PACKET_HEADER_BITS to be byte aligned");
    if (INTAN_SAMPLING_RATIO < 1)
        $error("packet_writer requires INTAN_SAMPLING_RATIO >= 1");
    if (PACKET_WORDS < 1)
        $error("packet_writer requires PACKET_WORDS >= 1");
end

function automatic logic [7:0] intan_frame_byte(
    input logic [INTAN_FRAME_BITS-1:0] frame,
    input logic [INTAN_BYTE_INDEX_WIDTH-1:0] byte_index
);
    int unsigned msb;
begin
    msb = INTAN_FRAME_BITS - 1 - 8 * byte_index;
    intan_frame_byte = frame[msb -: 8];
end
endfunction

function automatic logic [7:0] icm_frame_byte(
    input ICM_frame_t frame,
    input logic [ICM_BYTE_INDEX_WIDTH-1:0] byte_index
);
    int unsigned msb;
begin
    msb = ICM_FRAME_BITS - 1 - 8 * byte_index;
    icm_frame_byte = frame[msb -: 8];
end
endfunction

function automatic logic [7:0] header_byte(
    input packet_header_t header,
    input logic [HEADER_BYTE_INDEX_WIDTH-1:0] byte_index
);
    int unsigned msb;
begin
    msb = PACKET_HEADER_BITS - 1 - 8 * byte_index;
    header_byte = header[msb -: 8];
end
endfunction

function automatic logic [DATA_WIDTH-1:0] insert_byte(
    input logic [DATA_WIDTH-1:0] word,
    input logic [BYTE_INDEX_WIDTH-1:0] byte_index,
    input logic [7:0] byte_value
);
    logic [DATA_WIDTH-1:0] result;
begin
    result = word;
    result[8 * byte_index +: 8] = byte_value;
    insert_byte = result;
end
endfunction

assign ready = packet_ready && !icm_valid;
assign can_pack_byte = !word_valid || word_ready;
assign pack_word_with_byte = insert_byte(pack_word, pack_byte_count, source_byte);
assign emit_word = source_valid && (source_last || pack_byte_count == AXIS_BYTES - 1);
assign consume_intan_frame = source_valid &&
                             state == STREAM_INTAN &&
                             intan_byte_index == INTAN_FRAME_BYTES - 1;

always_comb begin
    source_valid = 1'b0;
    source_last = 1'b0;
    packet_last = 1'b0;
    source_byte = 8'b0;

    case (state)
        STREAM_INTAN: begin
            source_valid = intan_count != 0 && intan_frames_streamed < INTAN_SAMPLING_RATIO;
            source_byte = intan_frame_byte(intan_buffer[intan_rptr], intan_byte_index);
        end

        STREAM_ICM: begin
            source_valid = icm_valid;
            source_byte = icm_frame_byte(icm_frame_reg, icm_byte_index);
        end

        STREAM_HEADER: begin
            source_valid = 1'b1;
            source_byte = header_byte(header_reg, header_byte_index);
            source_last = header_byte_index == PACKET_HEADER_BYTES - 1;
            packet_last = source_last;
        end

        default: begin
            source_valid = 1'b0;
        end
    endcase
end

always_ff @(posedge clk) begin
    if (rst) begin
        state <= IDLE;
        intan_wptr <= 1'b0;
        intan_rptr <= 1'b0;
        intan_count <= '0;
        intan_overflow <= 1'b0;
        icm_frame_reg <= '0;
        icm_valid <= 1'b0;
        header_reg <= '0;
        packet_counter <= 32'b0;
        intan_frames_written <= '0;
        intan_frames_streamed <= '0;
        intan_byte_index <= '0;
        icm_byte_index <= '0;
        header_byte_index <= '0;
        pack_byte_count <= '0;
        pack_word <= '0;
        word_valid <= 1'b0;
        word_data <= '0;
        packet_done <= 1'b0;
    end else begin
        packet_done <= 1'b0;

        if (Intan_frame_done) begin
            if (intan_count < 2) begin
                intan_buffer[intan_wptr] <= Intan_frame_in;
                intan_wptr <= ~intan_wptr;
            end else begin
                intan_overflow <= 1'b1;
            end
        end

        if (ICM_frame_done) begin
            icm_frame_reg <= ICM_frame_in;
            icm_valid <= 1'b1;
        end

        case ({Intan_frame_done && intan_count < 2, consume_intan_frame && can_pack_byte})
            2'b10: intan_count <= intan_count + 1'b1;
            2'b01: intan_count <= intan_count - 1'b1;
            default: intan_count <= intan_count;
        endcase

        if (state == IDLE && packet_ready && !word_valid) begin
            state <= STREAM_INTAN;
            intan_overflow <= 1'b0;
            intan_frames_written <= '0;
            intan_frames_streamed <= '0;
            intan_byte_index <= '0;
            icm_byte_index <= '0;
            header_byte_index <= '0;
            pack_byte_count <= '0;
            pack_word <= '0;
        end else if (source_valid && can_pack_byte) begin
            if (emit_word) begin
                word_data <= pack_word_with_byte;
                word_valid <= 1'b1;
                pack_word <= '0;
                pack_byte_count <= '0;
            end else begin
                word_valid <= 1'b0;
                pack_word <= pack_word_with_byte;
                pack_byte_count <= pack_byte_count + 1'b1;
            end

            case (state)
                STREAM_INTAN: begin
                    if (intan_byte_index == INTAN_FRAME_BYTES - 1) begin
                        intan_byte_index <= '0;
                        intan_rptr <= ~intan_rptr;
                        intan_frames_written <= intan_frames_written + 1'b1;
                        intan_frames_streamed <= intan_frames_streamed + 1'b1;

                        if (intan_frames_streamed == INTAN_SAMPLING_RATIO - 1)
                            state <= STREAM_ICM;
                    end else begin
                        intan_byte_index <= intan_byte_index + 1'b1;
                    end
                end

                STREAM_ICM: begin
                    if (icm_byte_index == ICM_FRAME_BYTES - 1) begin
                        icm_byte_index <= '0;
                        icm_valid <= 1'b0;
                        header_reg.packet_num <= packet_counter;
                        header_reg.intan_frame_count <= {
                            {(32-INTAN_COUNTER_WIDTH){1'b0}},
                            intan_frames_written
                        };
                        header_reg.flags <= '0;
                        header_reg.flags[0] <= intan_overflow;
                        state <= STREAM_HEADER;
                    end else begin
                        icm_byte_index <= icm_byte_index + 1'b1;
                    end
                end

                STREAM_HEADER: begin
                    if (header_byte_index == PACKET_HEADER_BYTES - 1) begin
                        header_byte_index <= '0;
                        packet_counter <= packet_counter + 1'b1;
                        packet_done <= 1'b1;
                        state <= IDLE;
                    end else begin
                        header_byte_index <= header_byte_index + 1'b1;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end else if (word_valid && word_ready) begin
            word_valid <= 1'b0;
        end
    end
end

endmodule
