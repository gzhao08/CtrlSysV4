
package config_pkg;

	localparam int NUM_ICM = 4;
	localparam int NUM_INTAN = 8;
	localparam int ICM_DATA_BYTES = 20;
	localparam int INTAN_DATA_BYTES = 64;
	localparam int INTAN_SAMPLING_RATIO = 30;
	localparam int BUFFER_SIZE = 15;
	localparam int AXIS_DATA_WIDTH = 1024;

	// measurement = one sensor's data
	// frame       = all sensors of one type for one read
	// packet      = Intan frames + one ICM frame + 68-byte metadata trailer

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

	// 68 byte header/trailer
	typedef struct packed{
		logic [31:0] 	packet_num;
		logic [31:0]	intan_frame_count;
		logic [479:0]	flags;
	} packet_header_t;

	typedef struct packed {
		Intan_frame_t [INTAN_SAMPLING_RATIO-1:0] Intan_frames;
		ICM_frame_t 		ICM_frame;
		packet_header_t 	header;
	} packet_t;

	localparam int AXIS_BYTES = AXIS_DATA_WIDTH / 8;
	localparam int PACKET_HEADER_BITS = $bits(packet_header_t);
	localparam int ICM_FRAME_BITS = $bits(ICM_frame_t);
	localparam int INTAN_FRAME_BITS = $bits(Intan_frame_t);
	localparam int PACKET_PAYLOAD_BITS = INTAN_SAMPLING_RATIO * INTAN_FRAME_BITS + ICM_FRAME_BITS + PACKET_HEADER_BITS;
	localparam int PACKET_PAYLOAD_BYTES = (PACKET_PAYLOAD_BITS + 7) / 8;
	localparam int PACKET_AXIS_WORDS = (PACKET_PAYLOAD_BITS + AXIS_DATA_WIDTH - 1) / AXIS_DATA_WIDTH;
	localparam int PACKET_BITS = PACKET_AXIS_WORDS * AXIS_DATA_WIDTH;
	localparam int PACKET_BYTES = PACKET_AXIS_WORDS * AXIS_BYTES;
	localparam int PACKET_LAST_BYTES = (PACKET_PAYLOAD_BYTES % AXIS_BYTES) == 0 ? AXIS_BYTES : PACKET_PAYLOAD_BYTES % AXIS_BYTES;
	localparam int PACKET_BUFFER_PACKETS = BUFFER_SIZE;
	localparam int PACKET_BUFFER_WORDS = PACKET_AXIS_WORDS * PACKET_BUFFER_PACKETS;

endpackage
