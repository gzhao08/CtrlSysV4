#ifndef SENSOR_TEST_HW_H
#define SENSOR_TEST_HW_H

#include <stddef.h>
#include <stdint.h>

#define SENSOR_TEST_DEFAULT_UDMABUF "/dev/udmabuf0"
#define SENSOR_TEST_DEFAULT_UIO "/dev/uio/api"
#define SENSOR_TEST_SAMPLE_CLOCK_HZ 125000000u
#define SENSOR_TEST_1MS_TICKS (SENSOR_TEST_SAMPLE_CLOCK_HZ / 1000u)
#define SENSOR_TEST_FRAME_WORDS 9u
#define SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES 24576u
#define SENSOR_TEST_INTAN8_ICM4_PACKET_WORDS 6144u
#define SENSOR_TEST_ICM_INTAN_PACKET_BYTES SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES
#define SENSOR_TEST_ICM_INTAN_PACKET_WORDS SENSOR_TEST_INTAN8_ICM4_PACKET_WORDS
#define SENSOR_TEST_TCP_MAGIC 0x4353444du /* "CSDM" */
#define SENSOR_TEST_TCP_VERSION 1u

typedef struct sensor_test sensor_test_t;

int sensor_test_open(sensor_test_t **test);
int sensor_test_initialize_icm20948(sensor_test_t *test);
int sensor_test_prepare_dma_udmabuf(sensor_test_t *test, const char *device);
int sensor_test_prepare_dma_reserved(sensor_test_t *test,
                                     uint64_t physical_address);
int sensor_test_prepare_dma_udmabuf_sized(sensor_test_t *test,
                                          const char *device,
                                          size_t required_bytes);
int sensor_test_prepare_dma_reserved_sized(sensor_test_t *test,
                                           uint64_t physical_address,
                                           size_t required_bytes);
int sensor_test_run_axil_only(sensor_test_t *test);
int sensor_test_run_dma(sensor_test_t *test);
int sensor_test_run_dma_interrupts(sensor_test_t *test,
                                   const char *uio_device,
                                   unsigned transfer_count,
                                   uint32_t sample_period_ticks,
                                   int stream_fd,
                                   int print_frames);
int sensor_test_run_dma_interrupts_sized(sensor_test_t *test,
                                         const char *uio_device,
                                         unsigned transfer_count,
                                         uint32_t sample_period_ticks,
                                         int stream_fd,
                                         int print_frames,
                                         size_t transfer_bytes);
void sensor_test_close(sensor_test_t *test);

#endif
