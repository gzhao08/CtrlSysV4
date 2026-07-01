`timescale 1ns/1ps

import config_pkg::*;

module ctrlsys_core (
    input  logic                         clk,
    input  logic                         rst_n,

    output logic                         spi_sclk,
    output logic                         spi_mosi,
    output logic                         spi_cs_n,
    input  logic [4-1:0]           spi_miso,

    output logic                         axi_spi_io0_i,
    input  logic                         axi_spi_io0_o,
    input  logic                         axi_spi_io0_t,
    output logic                         axi_spi_io1_i,
    input  logic                         axi_spi_io1_o,
    input  logic                         axi_spi_io1_t,
    output logic                         axi_spi_sck_i,
    input  logic                         axi_spi_sck_o,
    input  logic                         axi_spi_sck_t,
    output logic                         axi_spi_ss_i,
    input  logic                         axi_spi_ss_o,
    input  logic                         axi_spi_ss_t,

    output logic                         m_axis_tvalid,
    input  logic                         m_axis_tready,
    output logic [1024-1:0]   m_axis_tdata,
    output logic [1024/8-1:0] m_axis_tkeep,
    output logic                         m_axis_tlast,

    input  logic                         s00_axi_aclk,
    input  logic                         s00_axi_aresetn,
    input  logic [5:0]                   s00_axi_awaddr,
    input  logic [2:0]                   s00_axi_awprot,
    input  logic                         s00_axi_awvalid,
    output logic                         s00_axi_awready,
    input  logic [31:0]                  s00_axi_wdata,
    input  logic [3:0]                   s00_axi_wstrb,
    input  logic                         s00_axi_wvalid,
    output logic                         s00_axi_wready,
    output logic [1:0]                   s00_axi_bresp,
    output logic                         s00_axi_bvalid,
    input  logic                         s00_axi_bready,
    input  logic [5:0]                   s00_axi_araddr,
    input  logic [2:0]                   s00_axi_arprot,
    input  logic                         s00_axi_arvalid,
    output logic                         s00_axi_arready,
    output logic [31:0]                  s00_axi_rdata,
    output logic [1:0]                   s00_axi_rresp,
    output logic                         s00_axi_rvalid,
    input  logic                         s00_axi_rready
);

localparam logic [6:0] SPI_REG_ADDR = 7'd45;

initial begin
    if (10 < 1)
        $error("ctrlsys_core requires 10 >= 1");
    if (1024 < 8 || (1024 % 8) != 0)
        $error("ctrlsys_core 1024 must be a positive byte multiple");
end

logic [63:0] timestamp;
logic start_read_icm;
logic start_read_intan;
logic spi_start;
logic core_rst;

ICM_frame_t icm_frame;
Intan_frame_t intan_frame;

logic spi_reader_sclk;
logic spi_reader_mosi;
logic spi_reader_cs_n;
logic [4-1:0] spi_reader_miso;
logic spi_busy;
logic spi_done;
logic intan_busy;
logic intan_done;
logic axi_spi_miso;

logic packet_fifo_full;
logic packet_fifo_empty;
logic packet_fifo_overflow;
logic packet_fifo_underflow;
logic packet_fifo_rd_en;
logic packet_fifo_wr_en;
logic packet_fifo_packet_space;
logic [1024-1:0] packet_fifo_wr_data;
logic [1024-1:0] packet_fifo_rd_data;
logic packet_writer_ready;
logic packet_writer_word_valid;
logic packet_writer_packet_done;

logic axil_enable;
logic axil_soft_reset;
logic [31:0] axil_sample_period;
logic [63:0] icm_sample_period;
logic [63:0] intan_sample_period;
logic axil_use_axi;
logic axil_clear_error;
logic axil_reset_sample_counter;
logic axil_cpu_clear_irq;
logic packet_done_irq;
logic error_latched;
logic [31:0] sample_count;
logic [31:0] error_code;
logic [31:0] data_word0;
logic [31:0] data_word1;
logic [31:0] data_word2;
logic [31:0] data_word3;
logic [31:0] data_word4;
logic [31:0] data_word5;
logic [31:0] data_word6;
logic [31:0] data_word7;

integer frame_sensor_index;

logic rst_meta;
logic rst_sync;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rst_meta <= 1'b1;
        rst_sync <= 1'b1;
    end else begin
        rst_meta <= 1'b0;
        rst_sync <= rst_meta;
    end
end

assign core_rst = rst_sync || axil_soft_reset;
assign spi_start = start_read_icm && !axil_use_axi && !spi_busy;
assign axi_spi_io0_i = axi_spi_io0_o;
assign axi_spi_io1_i = axi_spi_miso;
assign axi_spi_sck_i = axi_spi_sck_o;
assign axi_spi_ss_i = axi_spi_ss_o;
assign icm_sample_period = {32'b0, axil_sample_period};
assign intan_sample_period = {32'b0, axil_sample_period} / 30;
assign packet_fifo_wr_en = packet_writer_word_valid && !packet_fifo_full;

axil_regs u_axil_regs (
    .enable(axil_enable),
    .soft_reset(axil_soft_reset),
    .sample_period(axil_sample_period),
    .useAXI(axil_use_axi),
    .clear_error(axil_clear_error),
    .reset_sample_counter(axil_reset_sample_counter),
    .cpu_clear_irq(axil_cpu_clear_irq),
    .busy(spi_busy || intan_busy),
    .error(error_latched),
    .read_in_progress(spi_busy || intan_busy),
    .packet_done(packet_done_irq),
    .sample_count(sample_count),
    .error_code(error_code),
    .state({2'b0, spi_busy, axil_use_axi}),
    .data_word0(data_word0),
    .data_word1(data_word1),
    .data_word2(data_word2),
    .data_word3(data_word3),
    .data_word4(data_word4),
    .data_word5(data_word5),
    .data_word6(data_word6),
    .data_word7(data_word7),
    .s00_axi_aclk(s00_axi_aclk),
    .s00_axi_aresetn(s00_axi_aresetn),
    .s00_axi_awaddr(s00_axi_awaddr),
    .s00_axi_awprot(s00_axi_awprot),
    .s00_axi_awvalid(s00_axi_awvalid),
    .s00_axi_awready(s00_axi_awready),
    .s00_axi_wdata(s00_axi_wdata),
    .s00_axi_wstrb(s00_axi_wstrb),
    .s00_axi_wvalid(s00_axi_wvalid),
    .s00_axi_wready(s00_axi_wready),
    .s00_axi_bresp(s00_axi_bresp),
    .s00_axi_bvalid(s00_axi_bvalid),
    .s00_axi_bready(s00_axi_bready),
    .s00_axi_araddr(s00_axi_araddr),
    .s00_axi_arprot(s00_axi_arprot),
    .s00_axi_arvalid(s00_axi_arvalid),
    .s00_axi_arready(s00_axi_arready),
    .s00_axi_rdata(s00_axi_rdata),
    .s00_axi_rresp(s00_axi_rresp),
    .s00_axi_rvalid(s00_axi_rvalid),
    .s00_axi_rready(s00_axi_rready)
);

stopwatch_64 u_stopwatch_64 (
    .clk(clk),
    .rst(core_rst),
    .timestamp_counter(timestamp)
);

acquisition_controller u_acquisition_controller (
    .clk(clk),
    .rst(core_rst),
    .enable(axil_enable),
    .timestamp(timestamp),
    .sample_period_ICM(icm_sample_period),
    .sample_period_Intan(intan_sample_period),
    .startRead_ICM(start_read_icm),
    .startRead_Intan(start_read_intan)
);

ICM_reader #(
    .REG_ADDR(SPI_REG_ADDR)
) u_icm_reader (
    .clk(clk),
    .rst(core_rst),
    .start(spi_start),
    .timestamp(timestamp),
    .ICM_frame(icm_frame),
    .busy(spi_busy),
    .done(spi_done),
    .sclk(spi_reader_sclk),
    .mosi(spi_reader_mosi),
    .miso(spi_reader_miso),
    .cs_n(spi_reader_cs_n)
);

Intan_reader u_intan_reader (
    .clk(clk),
    .rst(core_rst),
    .start(start_read_intan),
    .timestamp(timestamp),
    .Intan_frame(intan_frame),
    .busy(intan_busy),
    .done(intan_done)
);

packet_writer u_packet_writer (
    .clk(clk),
    .rst(core_rst || !axil_enable),
    .ICM_frame_done(spi_done),
    .Intan_frame_done(intan_done),
    .ICM_frame_in(icm_frame),
    .Intan_frame_in(intan_frame),
    .packet_ready(packet_fifo_packet_space),
    .ready(packet_writer_ready),
    .word_valid(packet_writer_word_valid),
    .word_ready(!packet_fifo_full),
    .word_data(packet_fifo_wr_data),
    .packet_done(packet_writer_packet_done)
);

SPI_mux #(
    .NUM_SENSORS(4)
) u_spi_mux (
    .axi_enable(axil_use_axi && !spi_busy),
    .reader_sclk(spi_reader_sclk),
    .reader_mosi(spi_reader_mosi),
    .reader_cs_n(spi_reader_cs_n),
    .reader_miso(spi_reader_miso),
    .axi_sclk(axi_spi_sck_t ? 1'b0 : axi_spi_sck_o),
    .axi_mosi(axi_spi_io0_t ? 1'b0 : axi_spi_io0_o),
    .axi_cs_n(axi_spi_ss_t ? 1'b1 : axi_spi_ss_o),
    .axi_miso(axi_spi_miso),
    .spi_sclk(spi_sclk),
    .spi_mosi(spi_mosi),
    .spi_cs_n(spi_cs_n),
    .spi_miso(spi_miso)
);

always_ff @(posedge clk) begin
    if (core_rst) begin
        sample_count    <= 32'b0;
        error_latched   <= 1'b0;
        error_code      <= 32'b0;
        packet_done_irq <= 1'b0;
        data_word0      <= 32'b0;
        data_word1      <= 32'b0;
        data_word2      <= 32'b0;
        data_word3      <= 32'b0;
        data_word4      <= 32'b0;
        data_word5      <= 32'b0;
        data_word6      <= 32'b0;
        data_word7      <= 32'b0;
    end else begin
        if (axil_clear_error) begin
            error_latched <= 1'b0;
            error_code    <= 32'b0;
        end

        if (axil_cpu_clear_irq)
            packet_done_irq <= 1'b0;
        else if (packet_writer_packet_done)
            packet_done_irq <= 1'b1;

        if (axil_reset_sample_counter)
            sample_count <= 32'b0;
        else if (packet_writer_packet_done)
            sample_count <= sample_count + 1'b1;

        if (packet_fifo_overflow || packet_fifo_underflow) begin
            error_latched <= 1'b1;
            error_code <= 32'h0000_0001;
        end

        if (packet_writer_packet_done) begin
            data_word0 <= sample_count;
            data_word1 <= 192;
            data_word2 <= 1920;
            data_word3 <= icm_frame.init_read_ts[31:0];
            data_word4 <= icm_frame.init_read_ts[63:32];
            data_word5 <= icm_frame.done_read_ts[31:0];
            data_word6 <= icm_frame.done_read_ts[63:32];
            data_word7 <= 24576;
        end
    end
end

packet_buffer #(
    .DATA_WIDTH(1024),
    .DEPTH_WORDS(1920),
    .PACKET_WORDS(192)
) u_packet_buffer (
    .clk(clk),
    .rst(core_rst),
    .wr_en(packet_fifo_wr_en),
    .wr_data(packet_fifo_wr_data),
    .rd_en(packet_fifo_rd_en),
    .rd_data(packet_fifo_rd_data),
    .empty(packet_fifo_empty),
    .full(packet_fifo_full),
    .packet_space(packet_fifo_packet_space),
    .overflow(packet_fifo_overflow),
    .underflow(packet_fifo_underflow)
);

packet_to_axis u_packet_to_axis (
    .clk(clk),
    .rst(core_rst),
    .fifo_rd_en(packet_fifo_rd_en),
    .fifo_rd_data(packet_fifo_rd_data),
    .fifo_empty(packet_fifo_empty),
    .m_axis_tvalid(m_axis_tvalid),
    .m_axis_tready(m_axis_tready),
    .m_axis_tdata(m_axis_tdata),
    .m_axis_tkeep(m_axis_tkeep),
    .m_axis_tlast(m_axis_tlast)
);

endmodule
