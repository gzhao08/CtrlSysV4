/*
Name: Gordon Zhao
File: packet_writer.sv
Description: Packs fixed-size DMA packets with a metadata trailer.
*/

import config_pkg::*;

module packet_writer #(
    parameter int DATA_WIDTH = 1024,
    parameter int PACKET_BYTES = 24576,
    parameter int PACKET_WORDS = 192,
    parameter int INTAN_FRAME_BITS = config_pkg::INTAN_FRAME_BITS,
    parameter int ICM_FRAME_BITS = config_pkg::ICM_FRAME_BITS,
    parameter int PACKET_TRAILER_BITS = 2048
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
localparam int PACKET_TRAILER_BYTES = PACKET_TRAILER_BITS / 8;
localparam int PACKET_TRAILER_OFFSET_BYTES = PACKET_BYTES - PACKET_TRAILER_BYTES;
localparam int MAX_INTAN_FRAMES = 45;
localparam int TRAILER_INTAN_OFFSET_COUNT = 48;
localparam int INTAN_FIFO_BYTES = MAX_INTAN_FRAMES * INTAN_FRAME_BYTES;

localparam int BYTE_INDEX_WIDTH = (AXIS_BYTES > 1) ? $clog2(AXIS_BYTES) : 1;
localparam int INTAN_BYTE_INDEX_WIDTH = (INTAN_FRAME_BYTES > 1) ? $clog2(INTAN_FRAME_BYTES) : 1;
localparam int ICM_BYTE_INDEX_WIDTH = (ICM_FRAME_BYTES > 1) ? $clog2(ICM_FRAME_BYTES) : 1;
localparam int TRAILER_BYTE_INDEX_WIDTH = (PACKET_TRAILER_BYTES > 1) ? $clog2(PACKET_TRAILER_BYTES) : 1;
localparam int PACKET_BYTE_INDEX_WIDTH = (PACKET_BYTES > 1) ? $clog2(PACKET_BYTES) : 1;
localparam int INTAN_FIFO_PTR_WIDTH = (INTAN_FIFO_BYTES > 1) ? $clog2(INTAN_FIFO_BYTES) : 1;
localparam int INTAN_FIFO_COUNT_WIDTH = $clog2(INTAN_FIFO_BYTES + 1);
localparam int INTAN_FRAME_COUNT_WIDTH = (MAX_INTAN_FRAMES > 1) ? $clog2(MAX_INTAN_FRAMES + 1) : 1;
localparam int INTAN_PACKET_BYTE_COUNT_WIDTH = $clog2(INTAN_FIFO_BYTES + 1);

typedef enum logic [2:0] {
    IDLE,
    STREAM_INTAN,
    STREAM_ICM,
    STREAM_PADDING,
    STREAM_TRAILER
} state_t;

state_t state;

(* ram_style = "block" *) logic [7:0] intan_fifo [0:INTAN_FIFO_BYTES-1];
logic [INTAN_FIFO_PTR_WIDTH-1:0] intan_wptr;
logic [INTAN_FIFO_PTR_WIDTH-1:0] intan_rptr;
logic [INTAN_FIFO_COUNT_WIDTH-1:0] intan_byte_count;
logic [INTAN_FRAME_COUNT_WIDTH-1:0] complete_intan_frames;

Intan_frame_t serialize_frame;
Intan_frame_t pending_intan_frame;
logic serialize_valid;
logic pending_intan_valid;
logic [INTAN_BYTE_INDEX_WIDTH-1:0] serialize_byte_index;

ICM_frame_t icm_frame_reg;
logic icm_pending;

packet_trailer_t trailer_reg;
logic [31:0] packet_counter;
logic [31:0] dropped_intan_frames;
logic [31:0] dropped_icm_frames;

logic [INTAN_FRAME_COUNT_WIDTH-1:0] packet_intan_frames;
logic [INTAN_PACKET_BYTE_COUNT_WIDTH-1:0] packet_intan_bytes;
logic [INTAN_PACKET_BYTE_COUNT_WIDTH-1:0] intan_bytes_streamed;
logic [INTAN_BYTE_INDEX_WIDTH-1:0] intan_byte_index;
logic [ICM_BYTE_INDEX_WIDTH-1:0] icm_byte_index;
logic [TRAILER_BYTE_INDEX_WIDTH-1:0] trailer_byte_index;
logic [PACKET_BYTE_INDEX_WIDTH-1:0] packet_byte_index;
logic [BYTE_INDEX_WIDTH-1:0] pack_byte_count;
logic [DATA_WIDTH-1:0] pack_word;

logic source_valid;
logic source_last;
logic [7:0] source_byte;
logic [DATA_WIDTH-1:0] pack_word_with_byte;
logic can_pack_byte;
logic emit_word;
logic intan_fifo_write;
logic intan_fifo_read;
logic intan_frame_write_done;
logic intan_frame_read_done;
logic [31:0] snapshot_intan_bytes;
logic [31:0] snapshot_valid_data_bytes;
logic [31:0] snapshot_icm_offset;

integer offset_idx;

initial begin
    if (DATA_WIDTH < 8 || (DATA_WIDTH % 8) != 0)
        $error("packet_writer requires DATA_WIDTH to be a positive byte multiple");
    if ((INTAN_FRAME_BITS % 8) != 0)
        $error("packet_writer requires INTAN_FRAME_BITS to be byte aligned");
    if ((ICM_FRAME_BITS % 8) != 0)
        $error("packet_writer requires ICM_FRAME_BITS to be byte aligned");
    if ((PACKET_TRAILER_BITS % 8) != 0)
        $error("packet_writer requires PACKET_TRAILER_BITS to be byte aligned");
    if ((PACKET_BYTES % AXIS_BYTES) != 0)
        $error("packet_writer requires PACKET_BYTES to be an integer number of AXI words");
    if (PACKET_WORDS != PACKET_BYTES / AXIS_BYTES)
        $error("packet_writer PACKET_WORDS must match PACKET_BYTES / AXIS_BYTES");
    if (PACKET_TRAILER_BYTES != 256)
        $error("packet_writer PACKET_TRAILER_BYTES mismatch");
    if (INTAN_FIFO_BYTES < INTAN_FRAME_BYTES)
        $error("packet_writer packet is too small for one Intan frame");
    if ($bits(packet_trailer_t) != PACKET_TRAILER_BITS)
        $error("packet_writer PACKET_TRAILER_BITS mismatch");
end

function automatic logic [7:0] intan_frame_byte(
    input Intan_frame_t frame,
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

function automatic logic [7:0] trailer_byte(
    input packet_trailer_t trailer,
    input logic [TRAILER_BYTE_INDEX_WIDTH-1:0] byte_index
);
    int unsigned msb;
begin
    msb = PACKET_TRAILER_BITS - 1 - 8 * byte_index;
    trailer_byte = trailer[msb -: 8];
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

assign ready = packet_ready && state == IDLE && !icm_pending;
assign can_pack_byte = !word_valid || word_ready;
assign pack_word_with_byte = insert_byte(pack_word, pack_byte_count, source_byte);
assign emit_word = source_valid && (source_last || pack_byte_count == AXIS_BYTES - 1);
assign source_last = packet_byte_index == PACKET_BYTES - 1;

assign intan_fifo_write = serialize_valid && intan_byte_count < INTAN_FIFO_BYTES;
assign intan_fifo_read = source_valid && can_pack_byte && state == STREAM_INTAN;
assign intan_frame_write_done = intan_fifo_write && serialize_byte_index == INTAN_FRAME_BYTES - 1;
assign intan_frame_read_done = intan_fifo_read && intan_byte_index == INTAN_FRAME_BYTES - 1;

assign snapshot_intan_bytes = complete_intan_frames * INTAN_FRAME_BYTES;
assign snapshot_icm_offset = snapshot_intan_bytes;
assign snapshot_valid_data_bytes = snapshot_intan_bytes + ICM_FRAME_BYTES;

always_comb begin
    source_valid = 1'b0;
    source_byte = 8'b0;

    case (state)
        STREAM_INTAN: begin
            source_valid = intan_byte_count != 0 && intan_bytes_streamed < packet_intan_bytes;
            source_byte = intan_fifo[intan_rptr];
        end

        STREAM_ICM: begin
            source_valid = 1'b1;
            source_byte = icm_frame_byte(icm_frame_reg, icm_byte_index);
        end

        STREAM_PADDING: begin
            source_valid = 1'b1;
            source_byte = 8'b0;
        end

        STREAM_TRAILER: begin
            source_valid = 1'b1;
            source_byte = trailer_byte(trailer_reg, trailer_byte_index);
        end

        default: begin
            source_valid = 1'b0;
            source_byte = 8'b0;
        end
    endcase
end

always_ff @(posedge clk) begin
    if (rst) begin
        state <= IDLE;
        intan_wptr <= '0;
        intan_rptr <= '0;
        intan_byte_count <= '0;
        complete_intan_frames <= '0;
        serialize_frame <= '0;
        pending_intan_frame <= '0;
        serialize_valid <= 1'b0;
        pending_intan_valid <= 1'b0;
        serialize_byte_index <= '0;
        icm_frame_reg <= '0;
        icm_pending <= 1'b0;
        trailer_reg <= '0;
        packet_counter <= 32'b0;
        dropped_intan_frames <= 32'b0;
        dropped_icm_frames <= 32'b0;
        packet_intan_frames <= '0;
        packet_intan_bytes <= '0;
        intan_bytes_streamed <= '0;
        intan_byte_index <= '0;
        icm_byte_index <= '0;
        trailer_byte_index <= '0;
        packet_byte_index <= '0;
        pack_byte_count <= '0;
        pack_word <= '0;
        word_valid <= 1'b0;
        word_data <= '0;
        packet_done <= 1'b0;
    end else begin
        packet_done <= 1'b0;

        if (Intan_frame_done) begin
            if (!serialize_valid && !pending_intan_valid &&
                (INTAN_FIFO_BYTES - intan_byte_count) >= INTAN_FRAME_BYTES) begin
                serialize_frame <= Intan_frame_in;
                serialize_valid <= 1'b1;
                serialize_byte_index <= '0;
            end else if (!pending_intan_valid) begin
                pending_intan_frame <= Intan_frame_in;
                pending_intan_valid <= 1'b1;
            end else begin
                dropped_intan_frames <= dropped_intan_frames + 1'b1;
            end
        end else if (!serialize_valid && pending_intan_valid &&
                     (INTAN_FIFO_BYTES - intan_byte_count) >= INTAN_FRAME_BYTES) begin
            serialize_frame <= pending_intan_frame;
            pending_intan_valid <= 1'b0;
            serialize_valid <= 1'b1;
            serialize_byte_index <= '0;
        end

        if (intan_fifo_write) begin
            intan_fifo[intan_wptr] <= intan_frame_byte(serialize_frame, serialize_byte_index);

            if (intan_wptr == INTAN_FIFO_BYTES - 1)
                intan_wptr <= '0;
            else
                intan_wptr <= intan_wptr + 1'b1;

            if (serialize_byte_index == INTAN_FRAME_BYTES - 1) begin
                serialize_byte_index <= '0;
                serialize_valid <= 1'b0;
            end else begin
                serialize_byte_index <= serialize_byte_index + 1'b1;
            end
        end else if (serialize_valid && intan_byte_count == INTAN_FIFO_BYTES) begin
            serialize_valid <= 1'b0;
            serialize_byte_index <= '0;
            dropped_intan_frames <= dropped_intan_frames + 1'b1;
        end

        if (intan_fifo_read) begin
            if (intan_rptr == INTAN_FIFO_BYTES - 1)
                intan_rptr <= '0;
            else
                intan_rptr <= intan_rptr + 1'b1;
        end

        case ({intan_fifo_write, intan_fifo_read})
            2'b10: intan_byte_count <= intan_byte_count + 1'b1;
            2'b01: intan_byte_count <= intan_byte_count - 1'b1;
            default: intan_byte_count <= intan_byte_count;
        endcase

        case ({intan_frame_write_done, intan_frame_read_done})
            2'b10: complete_intan_frames <= complete_intan_frames + 1'b1;
            2'b01: complete_intan_frames <= complete_intan_frames - 1'b1;
            default: complete_intan_frames <= complete_intan_frames;
        endcase

        if (ICM_frame_done) begin
            if (state == IDLE && !icm_pending) begin
                icm_frame_reg <= ICM_frame_in;
                icm_pending <= 1'b1;
            end else begin
                dropped_icm_frames <= dropped_icm_frames + 1'b1;
            end
        end

        if (state == IDLE && packet_ready && icm_pending && !word_valid) begin
            icm_pending <= 1'b0;
            packet_intan_frames <= complete_intan_frames;
            packet_intan_bytes <= snapshot_intan_bytes[INTAN_PACKET_BYTE_COUNT_WIDTH-1:0];
            intan_bytes_streamed <= '0;
            intan_byte_index <= '0;
            icm_byte_index <= '0;
            trailer_byte_index <= '0;
            packet_byte_index <= '0;
            pack_byte_count <= '0;
            pack_word <= '0;

            if (complete_intan_frames == 0)
                state <= STREAM_ICM;
            else
                state <= STREAM_INTAN;

            trailer_reg.magic_ones <= 64'hFFFF_FFFF_FFFF_FFFF;
            trailer_reg.packet_num <= packet_counter;
            trailer_reg.trailer_bytes <= PACKET_TRAILER_BYTES;
            trailer_reg.packet_bytes <= PACKET_BYTES;
            trailer_reg.valid_data_bytes <= snapshot_valid_data_bytes;
            trailer_reg.intan_frame_count <= complete_intan_frames;
            trailer_reg.max_intan_frame_count <= MAX_INTAN_FRAMES;
            trailer_reg.icm_frame_count <= 32'd1;
            trailer_reg.icm_frame_start_index <= snapshot_icm_offset;
            trailer_reg.trailer_start_index <= PACKET_TRAILER_OFFSET_BYTES;
            trailer_reg.flags <= {
                29'b0,
                complete_intan_frames == MAX_INTAN_FRAMES,
                dropped_icm_frames != 0,
                dropped_intan_frames != 0
            };
            trailer_reg.dropped_intan_frames <= dropped_intan_frames;
            trailer_reg.dropped_icm_frames <= dropped_icm_frames;
            trailer_reg.reserved <= '0;

            for (offset_idx = 0; offset_idx < TRAILER_INTAN_OFFSET_COUNT; offset_idx = offset_idx + 1) begin
                if (offset_idx < complete_intan_frames && offset_idx < MAX_INTAN_FRAMES)
                    trailer_reg.intan_frame_start_indices[offset_idx] <= offset_idx * INTAN_FRAME_BYTES;
                else
                    trailer_reg.intan_frame_start_indices[offset_idx] <= 32'b0;
            end

            dropped_intan_frames <= 32'b0;
            dropped_icm_frames <= 32'b0;
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

            if (source_last) begin
                packet_byte_index <= '0;
                packet_counter <= packet_counter + 1'b1;
                packet_done <= 1'b1;
                state <= IDLE;
            end else begin
                packet_byte_index <= packet_byte_index + 1'b1;

                case (state)
                    STREAM_INTAN: begin
                        intan_bytes_streamed <= intan_bytes_streamed + 1'b1;

                        if (intan_byte_index == INTAN_FRAME_BYTES - 1)
                            intan_byte_index <= '0;
                        else
                            intan_byte_index <= intan_byte_index + 1'b1;

                        if (intan_bytes_streamed == packet_intan_bytes - 1)
                            state <= STREAM_ICM;
                    end

                    STREAM_ICM: begin
                        if (icm_byte_index == ICM_FRAME_BYTES - 1) begin
                            icm_byte_index <= '0;
                            if (packet_byte_index == PACKET_TRAILER_OFFSET_BYTES - 1)
                                state <= STREAM_TRAILER;
                            else
                                state <= STREAM_PADDING;
                        end else begin
                            icm_byte_index <= icm_byte_index + 1'b1;
                        end
                    end

                    STREAM_PADDING: begin
                        if (packet_byte_index == PACKET_TRAILER_OFFSET_BYTES - 1)
                            state <= STREAM_TRAILER;
                    end

                    STREAM_TRAILER: begin
                        if (trailer_byte_index == PACKET_TRAILER_BYTES - 1)
                            trailer_byte_index <= '0;
                        else
                            trailer_byte_index <= trailer_byte_index + 1'b1;
                    end

                    default: begin
                        state <= IDLE;
                    end
                endcase
            end
        end else if (word_valid && word_ready) begin
            word_valid <= 1'b0;
        end
    end
end

endmodule
