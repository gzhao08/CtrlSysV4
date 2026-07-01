
package config_pkg;

	localparam int NUM_ICM = 4;
	localparam int NUM_INTAN = 8;
	localparam int ICM_DATA_BYTES = 20;
	localparam int INTAN_DATA_BYTES = 64;
	localparam int INTAN_SAMPLING_RATIO = 30;
	localparam int BUFFER_SIZE = 10;
	localparam int AXIS_DATA_WIDTH = 1024;
	localparam int PACKET_BYTES = 24576;

	// measurement = one sensor's data
	// frame       = all sensors of one type for one read
	// packet      = available Intan frames + one ICM frame + zero padding + metadata trailer

	typedef struct packed{
		logic [7:0] sensor_id;
		logic [8*ICM_DATA_BYTES-1:0] data;
	} ICM_measurement_t;

	typedef struct packed{
		logic [7:0] sensor_id;
		logic [8*INTAN_DATA_BYTES-1:0] data;
	} Intan_measurement_t;

	typedef struct packed{
	    logic [63:0]    init_read_ts; // timestamp that read was initiated
	    logic [63:0]    done_read_ts; // timestamp that read finished
	    ICM_measurement_t [NUM_ICM-1:0] ICM_data;
	} ICM_frame_t;

	typedef struct packed{
	    logic [63:0]    init_read_ts; // timestamp that read was initiated
	    logic [63:0]    done_read_ts; // timestamp that read finished
	    Intan_measurement_t [NUM_INTAN-1:0] Intan_data;
	} Intan_frame_t;

	localparam int AXIS_BYTES = AXIS_DATA_WIDTH / 8;
	localparam int ICM_FRAME_BITS = $bits(ICM_frame_t);
	localparam int INTAN_FRAME_BITS = $bits(Intan_frame_t);
	localparam int ICM_FRAME_BYTES = ICM_FRAME_BITS / 8;
	localparam int INTAN_FRAME_BYTES = INTAN_FRAME_BITS / 8;
	localparam int PACKET_TRAILER_BYTES = 256;
	localparam int PACKET_TRAILER_INTAN_OFFSET_COUNT = 48;
	localparam int PACKET_TRAILER_FIXED_BYTES = 56 + 4 * PACKET_TRAILER_INTAN_OFFSET_COUNT;
	localparam int PACKET_TRAILER_RESERVED_BYTES = PACKET_TRAILER_BYTES - PACKET_TRAILER_FIXED_BYTES;
	localparam int PACKET_TRAILER_OFFSET_BYTES = PACKET_BYTES - PACKET_TRAILER_BYTES;
	localparam int MAX_INTAN_FRAMES_BY_DATA = (PACKET_TRAILER_OFFSET_BYTES - ICM_FRAME_BYTES) / INTAN_FRAME_BYTES;
	localparam int MAX_INTAN_FRAMES_PER_PACKET =
		(MAX_INTAN_FRAMES_BY_DATA < PACKET_TRAILER_INTAN_OFFSET_COUNT) ?
		MAX_INTAN_FRAMES_BY_DATA : PACKET_TRAILER_INTAN_OFFSET_COUNT;
	localparam int MAX_PACKET_DATA_BYTES = MAX_INTAN_FRAMES_PER_PACKET * INTAN_FRAME_BYTES + ICM_FRAME_BYTES;

	// 256 byte packet trailer
	typedef struct packed{
		logic [63:0] 	magic_ones;
		logic [31:0] 	packet_num;
		logic [31:0]	trailer_bytes;
		logic [31:0]	packet_bytes;
		logic [31:0]	valid_data_bytes;
		logic [31:0]	intan_frame_count;
		logic [31:0]	max_intan_frame_count;
		logic [31:0]	icm_frame_count;
		logic [31:0]	icm_frame_start_index;
		logic [31:0]	trailer_start_index;
		logic [31:0]	flags;
		logic [31:0]	dropped_intan_frames;
		logic [31:0]	dropped_icm_frames;
		logic [0:PACKET_TRAILER_INTAN_OFFSET_COUNT-1][31:0] intan_frame_start_indices;
		logic [8*PACKET_TRAILER_RESERVED_BYTES-1:0]	reserved;
	} packet_trailer_t;

	typedef struct packed {
		Intan_frame_t [INTAN_SAMPLING_RATIO-1:0] Intan_frames;
		ICM_frame_t 		ICM_frame;
		packet_trailer_t 	trailer;
	} packet_t;

	localparam int PACKET_TRAILER_BITS = $bits(packet_trailer_t);
	localparam int PACKET_TRAILER_BITS_EXPECTED = 8 * PACKET_TRAILER_BYTES;
	localparam int MAX_PACKET_VALID_BYTES = MAX_PACKET_DATA_BYTES + PACKET_TRAILER_BYTES;
	localparam int PACKET_PAYLOAD_BITS = 8 * MAX_PACKET_VALID_BYTES;
	localparam int PACKET_PAYLOAD_BYTES = MAX_PACKET_VALID_BYTES;
	localparam int PACKET_AXIS_WORDS = PACKET_BYTES / AXIS_BYTES;
	localparam int PACKET_BITS = PACKET_AXIS_WORDS * AXIS_DATA_WIDTH;
	localparam int PACKET_LAST_BYTES = AXIS_BYTES;
	localparam int PACKET_BUFFER_PACKETS = BUFFER_SIZE;
	localparam int PACKET_BUFFER_WORDS = PACKET_AXIS_WORDS * PACKET_BUFFER_PACKETS;

endpackage
