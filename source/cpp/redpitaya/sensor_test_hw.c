#define _DEFAULT_SOURCE

#include "sensor_test_hw.h"

#include <errno.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define CORE_BASE       0x40000000u
#define DMA_BASE        0x40400000u
#define AXI_SPI_BASE    0x41e00000u
#define MAP_SIZE        0x10000u

#define CORE_CONTROL    0x00u
#define CORE_PERIOD     0x04u
#define CORE_COMMAND    0x0cu
#define CORE_STATUS     0x10u
#define CORE_COUNT      0x14u
#define CORE_ERROR      0x1cu
#define CORE_DATA0      0x20u

#define CONTROL_ENABLE  (1u << 0)
#define CONTROL_RESET   (1u << 1)
#define CONTROL_USE_AXI (1u << 2)
#define COMMAND_CLEAR   ((1u << 0) | (1u << 1) | (1u << 2))
#define STATUS_ERROR    (1u << 1)

#define S2MM_DMACR      0x30u
#define S2MM_DMASR      0x34u
#define S2MM_DA         0x48u
#define S2MM_DA_MSB     0x4cu
#define S2MM_LENGTH     0x58u
#define DMA_RUN         (1u << 0)
#define DMA_RESET       (1u << 2)
#define DMA_IDLE        (1u << 1)
#define DMA_ERROR_MASK  ((1u << 4) | (1u << 5) | (1u << 6))
#define DMA_IOC_IRQ_EN  (1u << 12)
#define DMA_ERR_IRQ_EN  (1u << 14)
#define DMA_IOC_IRQ     (1u << 12)
#define DMA_ERR_IRQ     (1u << 14)
#define DMA_IRQ_MASK    (DMA_IOC_IRQ | DMA_ERR_IRQ)

#define SPI_SRR          0x40u
#define SPI_CR           0x60u
#define SPI_SR           0x64u
#define SPI_DTR          0x68u
#define SPI_DRR          0x6cu
#define SPI_SSR          0x70u
#define SPI_CR_ENABLE    (1u << 1)
#define SPI_CR_MASTER    (1u << 2)
#define SPI_CR_CPOL      (1u << 3)
#define SPI_CR_CPHA      (1u << 4)
#define SPI_CR_TX_RESET  (1u << 5)
#define SPI_CR_RX_RESET  (1u << 6)
#define SPI_CR_MANUAL_SS (1u << 7)
#define SPI_CR_INHIBIT   (1u << 8)
#define SPI_SR_RX_EMPTY  (1u << 0)
#define SPI_SR_ERROR     ((1u << 4) | (1u << 6) | (1u << 7) | (1u << 8) | \
                          (1u << 9) | (1u << 10))

#define ICM_WHO_AM_I         0x00u
#define ICM_USER_CTRL        0x03u
#define ICM_LP_CONFIG        0x05u
#define ICM_PWR_MGMT_1       0x06u
#define ICM_PWR_MGMT_2       0x07u
#define ICM_I2C_MST_STATUS   0x17u
#define ICM_USER_BANK_SEL    0x7fu
#define ICM_WHO_AM_I_VALUE   0xeau

#define ICM_BANK_0           0x00u
#define ICM_BANK_3           0x30u
#define ICM_USER_I2C_IF_DIS  (1u << 4)
#define ICM_USER_I2C_MST_EN  (1u << 5)
#define ICM_LP_I2C_MST_CYCLE (1u << 6)
#define ICM_I2C_MST_DONE     (1u << 6)
#define ICM_I2C_MST_ERR      ((1u << 0) | (1u << 1) | (1u << 2) | \
                              (1u << 3) | (1u << 4) | (1u << 5))

#define ICM_I2C_MST_ODR_CONFIG 0x00u
#define ICM_I2C_MST_CTRL       0x01u
#define ICM_I2C_MST_DELAY_CTRL 0x02u
#define ICM_I2C_SLV0_ADDR      0x03u
#define ICM_I2C_SLV0_REG       0x04u
#define ICM_I2C_SLV0_CTRL      0x05u
#define ICM_I2C_SLV1_ADDR      0x07u
#define ICM_I2C_SLV1_REG       0x08u
#define ICM_I2C_SLV1_CTRL      0x09u
#define ICM_I2C_SLV4_ADDR      0x13u
#define ICM_I2C_SLV4_REG       0x14u
#define ICM_I2C_SLV4_CTRL      0x15u
#define ICM_I2C_SLV4_DO        0x16u
#define ICM_I2C_SLV4_DI        0x17u
#define ICM_I2C_READ           0x80u
#define ICM_I2C_SLV_EN         0x80u
#define ICM_I2C_MST_CLK_345KHZ 0x07u

#define AK09916_I2C_ADDR       0x0cu
#define AK09916_WIA1           0x00u
#define AK09916_WIA2           0x01u
#define AK09916_HXL            0x11u
#define AK09916_ST2            0x18u
#define AK09916_CNTL2          0x31u
#define AK09916_CNTL3          0x32u
#define AK09916_WIA1_VALUE     0x48u
#define AK09916_WIA2_VALUE     0x09u
#define AK09916_RESET          0x01u
#define AK09916_CONT_100HZ     0x08u

#define FRAME_WORDS     SENSOR_TEST_FRAME_WORDS
#define FRAME_BYTES     (FRAME_WORDS * sizeof(uint32_t))
#define SAMPLE_PERIOD   SENSOR_TEST_SAMPLE_CLOCK_HZ /* 1 s at runtime FCLK. */
#define TIMEOUT_MS      2000u

static uint32_t axi_spi_mode_bits;

struct dma_buffer {
    int fd;
    void *mapping;
    size_t mapping_size;
    volatile uint32_t *words;
    uint64_t physical_address;
};

struct sensor_test {
    int mem_fd;
    volatile uint32_t *core;
    volatile uint32_t *dma;
    volatile uint32_t *axi_spi;
    struct dma_buffer buffer;
};

static volatile uint32_t *map_registers(int mem_fd, off_t address)
{
    void *mapping = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                         MAP_SHARED, mem_fd, address);
    if (mapping == MAP_FAILED) {
        fprintf(stderr, "mmap(0x%08jx): %s\n", (uintmax_t)address,
                strerror(errno));
        return NULL;
    }
    return (volatile uint32_t *)mapping;
}

static uint32_t reg_read(volatile uint32_t *base, unsigned offset)
{
    return base[offset / sizeof(uint32_t)];
}

static void reg_write(volatile uint32_t *base, unsigned offset, uint32_t value)
{
    base[offset / sizeof(uint32_t)] = value;
    __sync_synchronize();
}

static uint64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
}

static int axi_spi_transfer(volatile uint32_t *spi, const uint8_t *tx,
                            uint8_t *rx, size_t length)
{
    const uint32_t idle_control = SPI_CR_ENABLE | SPI_CR_MASTER |
                                  SPI_CR_MANUAL_SS | SPI_CR_INHIBIT |
                                  axi_spi_mode_bits;
    uint64_t deadline;
    size_t received = 0;
    size_t i;

    reg_write(spi, SPI_CR, idle_control | SPI_CR_TX_RESET | SPI_CR_RX_RESET);
    reg_write(spi, SPI_CR, idle_control);
    reg_write(spi, SPI_SSR, 0u);
    for (i = 0; i < length; ++i)
        reg_write(spi, SPI_DTR, tx[i]);
    reg_write(spi, SPI_CR, idle_control & ~SPI_CR_INHIBIT);

    deadline = monotonic_ms() + 100u;
    while (received < length && monotonic_ms() < deadline) {
        uint32_t status = reg_read(spi, SPI_SR);
        if (status & SPI_SR_ERROR)
            break;
        if (!(status & SPI_SR_RX_EMPTY))
            rx[received++] = (uint8_t)reg_read(spi, SPI_DRR);
    }

    reg_write(spi, SPI_CR, idle_control);
    reg_write(spi, SPI_SSR, 1u);
    return received == length ? 0 : -1;
}

static int icm_write_register(volatile uint32_t *spi, uint8_t address,
                              uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(address & 0x7fu), value};
    uint8_t rx[2];
    return axi_spi_transfer(spi, tx, rx, sizeof(tx));
}

static int icm_read_register(volatile uint32_t *spi, uint8_t address,
                             uint8_t *value)
{
    uint8_t tx[2] = {(uint8_t)(address | 0x80u), 0u};
    uint8_t rx[2];
    if (axi_spi_transfer(spi, tx, rx, sizeof(tx)) != 0)
        return -1;
    *value = rx[1];
    return 0;
}

static int icm_select_bank(volatile uint32_t *spi, uint8_t bank)
{
    return icm_write_register(spi, ICM_USER_BANK_SEL, bank);
}

static int icm_write_bank_register(volatile uint32_t *spi, uint8_t bank,
                                   uint8_t address, uint8_t value)
{
    if (icm_select_bank(spi, bank) != 0)
        return -1;
    return icm_write_register(spi, address, value);
}

static int icm_read_bank_register(volatile uint32_t *spi, uint8_t bank,
                                  uint8_t address, uint8_t *value)
{
    if (icm_select_bank(spi, bank) != 0)
        return -1;
    return icm_read_register(spi, address, value);
}

static int ak09916_wait_aux_transaction(volatile uint32_t *spi)
{
    uint64_t deadline = monotonic_ms() + 100u;

    while (monotonic_ms() < deadline) {
        uint8_t status;

        if (icm_read_bank_register(spi, ICM_BANK_0, ICM_I2C_MST_STATUS,
                                   &status) != 0)
            return -1;
        if (status & ICM_I2C_MST_ERR) {
            fprintf(stderr, "AK09916 aux-I2C error, I2C_MST_STATUS=0x%02x\n",
                    status);
            return -1;
        }
        if (status & ICM_I2C_MST_DONE)
            return 0;
        usleep(1000);
    }

    fprintf(stderr, "AK09916 aux-I2C transaction timed out.\n");
    return -1;
}

static int ak09916_write_register(volatile uint32_t *spi, uint8_t address,
                                  uint8_t value)
{
    uint8_t status;

    if (icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_ADDR,
                                AK09916_I2C_ADDR) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_REG,
                                address) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_DO,
                                value) != 0)
        return -1;
    if (icm_read_bank_register(spi, ICM_BANK_0, ICM_I2C_MST_STATUS,
                               &status) != 0)
        return -1;
    if (icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_CTRL,
                                ICM_I2C_SLV_EN) != 0)
        return -1;

    return ak09916_wait_aux_transaction(spi);
}

static int ak09916_read_register(volatile uint32_t *spi, uint8_t address,
                                 uint8_t *value)
{
    uint8_t status;

    if (icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_ADDR,
                                ICM_I2C_READ | AK09916_I2C_ADDR) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_REG,
                                address) != 0)
        return -1;
    if (icm_read_bank_register(spi, ICM_BANK_0, ICM_I2C_MST_STATUS,
                               &status) != 0)
        return -1;
    if (icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_CTRL,
                                ICM_I2C_SLV_EN) != 0)
        return -1;
    if (ak09916_wait_aux_transaction(spi) != 0)
        return -1;

    return icm_read_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV4_DI, value);
}

static int initialize_ak09916(volatile uint32_t *spi)
{
    uint8_t wia1 = 0;
    uint8_t wia2 = 0;

    if (icm_write_bank_register(spi, ICM_BANK_0, ICM_USER_CTRL,
                                ICM_USER_I2C_IF_DIS |
                                ICM_USER_I2C_MST_EN) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_0, ICM_LP_CONFIG,
                                ICM_LP_I2C_MST_CYCLE) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_MST_CTRL,
                                ICM_I2C_MST_CLK_345KHZ) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_MST_ODR_CONFIG,
                                0x00u) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_MST_DELAY_CTRL,
                                0x00u) != 0)
        return -1;
    usleep(10000);

    if (ak09916_read_register(spi, AK09916_WIA1, &wia1) != 0 ||
        ak09916_read_register(spi, AK09916_WIA2, &wia2) != 0)
        return -1;
    printf("AK09916 identity: WIA1=0x%02x WIA2=0x%02x\n", wia1, wia2);
    if (wia1 != AK09916_WIA1_VALUE || wia2 != AK09916_WIA2_VALUE) {
        fprintf(stderr, "WARNING: AK09916 identity is 0x%02x/0x%02x, "
                        "expected 0x%02x/0x%02x; continuing anyway.\n",
                wia1, wia2, AK09916_WIA1_VALUE, AK09916_WIA2_VALUE);
    }

    if (ak09916_write_register(spi, AK09916_CNTL3, AK09916_RESET) != 0)
        return -1;
    usleep(100000);
    if (ak09916_write_register(spi, AK09916_CNTL2,
                               AK09916_CONT_100HZ) != 0)
        return -1;

    if (icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV0_ADDR,
                                ICM_I2C_READ | AK09916_I2C_ADDR) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV0_REG,
                                AK09916_HXL) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV0_CTRL,
                                ICM_I2C_SLV_EN | 6u) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV1_ADDR,
                                ICM_I2C_READ | AK09916_I2C_ADDR) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV1_REG,
                                AK09916_ST2) != 0 ||
        icm_write_bank_register(spi, ICM_BANK_3, ICM_I2C_SLV1_CTRL,
                                ICM_I2C_SLV_EN | 1u) != 0 ||
        icm_select_bank(spi, ICM_BANK_0) != 0)
        return -1;

    printf("AK09916 magnetometer configured for continuous 100 Hz reads.\n");
    return 0;
}

static int print_raw_identity_transaction(volatile uint32_t *spi,
                                          const char *mode_name)
{
    const uint8_t tx[4] = {ICM_WHO_AM_I | 0x80u, 0u, 0u, 0u};
    uint8_t rx[4];
    unsigned i;

    if (axi_spi_transfer(spi, tx, rx, sizeof(tx)) != 0)
        return -1;

    printf("WHO_AM_I %s raw RX:", mode_name);
    for (i = 0; i < sizeof(rx); ++i)
        printf(" 0x%02x", rx[i]);
    printf("\n");
    return 0;
}

static int initialize_icm20948(volatile uint32_t *core,
                               volatile uint32_t *spi)
{
    uint8_t mode0_identity[4];
    uint8_t mode3_identity[4];
    uint8_t identity = 0;
    unsigned i;

    reg_write(core, CORE_CONTROL, CONTROL_USE_AXI);
    reg_write(spi, SPI_SRR, 0xau);
    usleep(1000);

    axi_spi_mode_bits = 0;
    if (icm_write_register(spi, ICM_USER_BANK_SEL, 0x00u) != 0)
        goto transaction_failure;
    if (print_raw_identity_transaction(spi, "mode 0") != 0)
        goto transaction_failure;
    printf("WHO_AM_I mode 0:");
    for (i = 0; i < 4; ++i) {
        if (icm_read_register(spi, ICM_WHO_AM_I, &mode0_identity[i]) != 0)
            goto transaction_failure;
        printf(" 0x%02x", mode0_identity[i]);
    }
    printf("\n");

    axi_spi_mode_bits = SPI_CR_CPOL | SPI_CR_CPHA;
    if (icm_write_register(spi, ICM_USER_BANK_SEL, 0x00u) != 0)
        goto transaction_failure;
    if (print_raw_identity_transaction(spi, "mode 3") != 0)
        goto transaction_failure;
    printf("WHO_AM_I mode 3:");
    for (i = 0; i < 4; ++i) {
        if (icm_read_register(spi, ICM_WHO_AM_I, &mode3_identity[i]) != 0)
            goto transaction_failure;
        printf(" 0x%02x", mode3_identity[i]);
    }
    printf("\n");

    axi_spi_mode_bits = 0;
    identity = mode0_identity[3];
    for (i = 0; i < 4; ++i) {
        if (mode0_identity[i] == ICM_WHO_AM_I_VALUE) {
            identity = mode0_identity[i];
            break;
        }
    }
    if (identity != ICM_WHO_AM_I_VALUE) {
        for (i = 0; i < 4; ++i) {
            if (mode3_identity[i] == ICM_WHO_AM_I_VALUE) {
                identity = mode3_identity[i];
                axi_spi_mode_bits = SPI_CR_CPOL | SPI_CR_CPHA;
                break;
            }
        }
    }

    if (identity != ICM_WHO_AM_I_VALUE) {
        fprintf(stderr, "WARNING: WHO_AM_I is 0x%02x, expected 0x%02x; "
                        "continuing anyway.\n",
                identity, ICM_WHO_AM_I_VALUE);
    } else {
        printf("PASS: ICM-20948 WHO_AM_I=0xEA.\n");
    }

    if (icm_write_register(spi, ICM_PWR_MGMT_1, 0x80u) != 0) {
        reg_write(core, CORE_CONTROL, 0);
        return -1;
    }
    usleep(100000);

    if (icm_write_register(spi, ICM_USER_BANK_SEL, ICM_BANK_0) != 0 ||
        icm_write_register(spi, ICM_USER_CTRL,
                           ICM_USER_I2C_IF_DIS | ICM_USER_I2C_MST_EN) != 0 ||
        icm_write_register(spi, ICM_PWR_MGMT_1, 0x01u) != 0 ||
        icm_write_register(spi, ICM_PWR_MGMT_2, 0x00u) != 0) {
        fprintf(stderr, "FAIL: could not initialize the ICM-20948.\n");
        reg_write(core, CORE_CONTROL, 0);
        return -1;
    }
    usleep(10000);
    if (initialize_ak09916(spi) != 0) {
        fprintf(stderr, "WARNING: could not initialize the AK09916 "
                        "magnetometer; external sensor bytes may be stale.\n");
        icm_write_register(spi, ICM_USER_BANK_SEL, ICM_BANK_0);
    }
    reg_write(core, CORE_CONTROL, 0);
    printf("ICM-20948 wake/configuration sequence completed in bank 0.\n");
    return 0;

transaction_failure:
    fprintf(stderr, "FAIL: ICM-20948 did not complete an SPI transaction.\n");
    reg_write(core, CORE_CONTROL, 0);
    return -1;
}

static int read_u64_file(const char *path, uint64_t *value)
{
    char text[64];
    char *end;
    FILE *file = fopen(path, "r");
    if (!file)
        return -1;
    if (!fgets(text, sizeof(text), file)) {
        fclose(file);
        return -1;
    }
    fclose(file);
    errno = 0;
    *value = strtoull(text, &end, 0);
    return errno == 0 && end != text ? 0 : -1;
}

static int open_udmabuf_sized(struct dma_buffer *buffer, const char *device,
                              size_t required_bytes)
{
    const char *name = strrchr(device, '/');
    char path[256];
    uint64_t size;

    name = name ? name + 1 : device;
    snprintf(path, sizeof(path), "/sys/class/u-dma-buf/%s/phys_addr", name);
    if (read_u64_file(path, &buffer->physical_address) != 0)
        return -1;
    snprintf(path, sizeof(path), "/sys/class/u-dma-buf/%s/size", name);
    if (read_u64_file(path, &size) != 0 || size < required_bytes)
        return -1;

    buffer->fd = open(device, O_RDWR | O_SYNC);
    if (buffer->fd < 0)
        return -1;
    buffer->mapping_size = (size_t)size;
    buffer->mapping = mmap(NULL, buffer->mapping_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, buffer->fd, 0);
    if (buffer->mapping == MAP_FAILED) {
        close(buffer->fd);
        buffer->fd = -1;
        return -1;
    }
    buffer->words = (volatile uint32_t *)buffer->mapping;
    return 0;
}

static int open_reserved_memory(struct dma_buffer *buffer, int mem_fd,
                                uint64_t physical_address,
                                size_t required_bytes)
{
    long page_size = sysconf(_SC_PAGESIZE);
    uint64_t page_base = physical_address & ~((uint64_t)page_size - 1u);
    size_t offset = (size_t)(physical_address - page_base);
    size_t required = offset + required_bytes;

    buffer->fd = -1;
    buffer->physical_address = physical_address;
    buffer->mapping_size = (required + (size_t)page_size - 1u) &
                           ~((size_t)page_size - 1u);
    buffer->mapping = mmap(NULL, buffer->mapping_size, PROT_READ | PROT_WRITE,
                           MAP_SHARED, mem_fd, (off_t)page_base);
    if (buffer->mapping == MAP_FAILED)
        return -1;
    buffer->words = (volatile uint32_t *)((uint8_t *)buffer->mapping + offset);
    return 0;
}

static int overlaps_system_ram(uint64_t address, size_t size)
{
    FILE *file = fopen("/proc/iomem", "r");
    char line[256];
    uint64_t last = address + size - 1u;

    if (!file)
        return -1;

    while (fgets(line, sizeof(line), file)) {
        unsigned long long first_ram;
        unsigned long long last_ram;
        char name[64];

        if (sscanf(line, " %llx-%llx : %63[^\n]",
                   &first_ram, &last_ram, name) == 3 &&
            strcmp(name, "System RAM") == 0 &&
            address <= (uint64_t)last_ram && last >= (uint64_t)first_ram) {
            fclose(file);
            return 1;
        }
    }

    fclose(file);
    return 0;
}

static int is_redpitaya_reserved_memory(uint64_t address, size_t size,
                                        uint64_t *region_start,
                                        uint64_t *region_size)
{
    FILE *pipe = popen("monitor -r", "r");
    char line[256];
    uint64_t start = 0;
    uint64_t bytes = 0;
    int have_start = 0;
    int have_size = 0;
    uint64_t last;

    if (!pipe)
        return 0;

    while (fgets(line, sizeof(line), pipe)) {
        unsigned long long value;

        if (sscanf(line, " start: %llx", &value) == 1) {
            start = (uint64_t)value;
            have_start = 1;
        } else if (sscanf(line, " size: %llx", &value) == 1) {
            bytes = (uint64_t)value;
            have_size = 1;
        }
    }
    pclose(pipe);

    if (!have_start || !have_size || bytes == 0 || size == 0 ||
        address > UINT64_MAX - (uint64_t)size + 1u)
        return 0;

    last = address + (uint64_t)size - 1u;
    if (address >= start && last < start + bytes) {
        *region_start = start;
        *region_size = bytes;
        return 1;
    }
    return 0;
}

static void close_dma_buffer(struct dma_buffer *buffer)
{
    if (buffer->mapping && buffer->mapping != MAP_FAILED)
        munmap(buffer->mapping, buffer->mapping_size);
    if (buffer->fd >= 0)
        close(buffer->fd);
    buffer->fd = -1;
    buffer->mapping = NULL;
    buffer->mapping_size = 0;
    buffer->words = NULL;
    buffer->physical_address = 0;
}

static void print_sensor_bytes(const uint32_t sensor_words[5])
{
    uint8_t bytes[20];
    unsigned i;
    printf("Sensor bytes (register 0x2D onward):\n  ");
    for (i = 0; i < 20; ++i) {
        unsigned word = 4u - i / 4u;
        unsigned shift = (3u - i % 4u) * 8u;
        bytes[i] = (uint8_t)((sensor_words[word] >> shift) & 0xffu);
        printf("%02x%c", bytes[i], i == 19 ? '\n' : ' ');
    }

    printf("Magnetometer raw from AK09916 HXL..HZH: X=%" PRId16
           " Y=%" PRId16 " Z=%" PRId16 "\n",
           (int16_t)((uint16_t)bytes[14] | ((uint16_t)bytes[15] << 8)),
           (int16_t)((uint16_t)bytes[16] | ((uint16_t)bytes[17] << 8)),
           (int16_t)((uint16_t)bytes[18] | ((uint16_t)bytes[19] << 8)));
}

static int run_axil_only_test(volatile uint32_t *core)
{
    uint32_t sensor_words[5];
    uint64_t deadline;
    int all_zero = 1;
    int all_ones = 1;
    unsigned i;

    reg_write(core, CORE_CONTROL, CONTROL_RESET);
    usleep(1000);
    reg_write(core, CORE_CONTROL, 0);
    reg_write(core, CORE_COMMAND, COMMAND_CLEAR);
    reg_write(core, CORE_PERIOD, SAMPLE_PERIOD);
    reg_write(core, CORE_CONTROL, CONTROL_ENABLE);

    deadline = monotonic_ms() + TIMEOUT_MS;
    while (reg_read(core, CORE_COUNT) == 0 && monotonic_ms() < deadline) {
        if (reg_read(core, CORE_STATUS) & STATUS_ERROR)
            break;
        usleep(1000);
    }
    reg_write(core, CORE_CONTROL, 0);

    if (reg_read(core, CORE_COUNT) == 0) {
        fprintf(stderr, "FAIL: no sensor sample completed; status=0x%08" PRIx32
                        ", error=0x%08" PRIx32 "\n",
                reg_read(core, CORE_STATUS), reg_read(core, CORE_ERROR));
        return EXIT_FAILURE;
    }
    if (reg_read(core, CORE_STATUS) & STATUS_ERROR) {
        fprintf(stderr, "FAIL: core error code 0x%08" PRIx32 "\n",
                reg_read(core, CORE_ERROR));
        return EXIT_FAILURE;
    }

    /* AXI-Lite data_word3 is the MSW; data_word7 is the LSW. */
    for (i = 0; i < 5; ++i) {
        sensor_words[i] = reg_read(core, CORE_DATA0 + (7u - i) * 4u);
        if (sensor_words[i] != 0)
            all_zero = 0;
        if (sensor_words[i] != UINT32_MAX)
            all_ones = 0;
    }

    printf("PASS: the custom SPI reader completed an AXI-Lite sample.\n");
    printf("Sample count: %" PRIu32 "\n", reg_read(core, CORE_COUNT));
    printf("Start timestamp: 0x%08" PRIx32 "%08" PRIx32 "\n",
           reg_read(core, CORE_DATA0 + 4u), reg_read(core, CORE_DATA0));
    print_sensor_bytes(sensor_words);

    if (all_zero || all_ones) {
        fprintf(stderr, "FAIL: sensor returned all %s; check power, CS, MISO, "
                        "and SPI mode.\n", all_zero ? "00" : "FF");
        return EXIT_FAILURE;
    }
    printf("PASS: response is not an all-zero or all-FF disconnected-bus pattern.\n");
    return EXIT_SUCCESS;
}

static int reset_s2mm_dma(volatile uint32_t *dma)
{
    uint64_t deadline;

    reg_write(dma, S2MM_DMACR, DMA_RESET);
    deadline = monotonic_ms() + TIMEOUT_MS;
    while ((reg_read(dma, S2MM_DMACR) & DMA_RESET) && monotonic_ms() < deadline)
        usleep(1000);
    if (reg_read(dma, S2MM_DMACR) & DMA_RESET) {
        fprintf(stderr, "DMA reset timed out.\n");
        return -1;
    }

    reg_write(dma, S2MM_DMASR, DMA_IRQ_MASK | DMA_ERROR_MASK);
    return 0;
}

static int wait_for_uio_interrupt(int uio_fd, uint32_t *irq_count)
{
    ssize_t bytes;

    do {
        bytes = read(uio_fd, irq_count, sizeof(*irq_count));
    } while (bytes < 0 && errno == EINTR);

    if (bytes != (ssize_t)sizeof(*irq_count)) {
        if (bytes < 0)
            perror("read UIO interrupt");
        else
            fprintf(stderr, "short UIO interrupt read: %zd bytes\n", bytes);
        return -1;
    }
    return 0;
}

static int enable_uio_interrupt(int uio_fd)
{
    uint32_t enable = 1;
    ssize_t bytes;

    do {
        bytes = write(uio_fd, &enable, sizeof(enable));
    } while (bytes < 0 && errno == EINTR);

    if (bytes != (ssize_t)sizeof(enable)) {
        if (bytes < 0)
            perror("enable UIO interrupt");
        else
            fprintf(stderr, "short UIO interrupt write: %zd bytes\n", bytes);
        return -1;
    }
    return 0;
}

static void print_dma_frame(unsigned transfer_index, uint32_t irq_count,
                            uint32_t sample_count,
                            const uint32_t frame[FRAME_WORDS])
{
    uint32_t sensor_words[5];
    unsigned i;

    for (i = 0; i < 5; ++i)
        sensor_words[i] = frame[4 + i];

    printf("\nDMA interrupt sample %u, UIO irq count %" PRIu32
           ", core sample count %" PRIu32 "\n",
           transfer_index, irq_count, sample_count);
    printf("Start timestamp: 0x%08" PRIx32 "%08" PRIx32 "\n",
           frame[1], frame[0]);
    printf("Done timestamp:  0x%08" PRIx32 "%08" PRIx32 "\n",
           frame[3], frame[2]);
    print_sensor_bytes(sensor_words);
}

static int send_all(int fd, const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    size_t sent = 0;

    while (sent < length) {
        ssize_t chunk;
#ifdef MSG_NOSIGNAL
        chunk = send(fd, bytes + sent, length - sent, MSG_NOSIGNAL);
#else
        chunk = send(fd, bytes + sent, length - sent, 0);
#endif
        if (chunk < 0) {
            if (errno == EINTR)
                continue;
            perror("send TCP sample");
            return -1;
        }
        if (chunk == 0) {
            fprintf(stderr, "TCP peer closed while sending sample.\n");
            return -1;
        }
        sent += (size_t)chunk;
    }

    return 0;
}

static int send_dma_frame(int stream_fd, uint32_t sequence, uint32_t irq_count,
                          uint32_t sample_count,
                          const uint32_t frame[FRAME_WORDS])
{
    uint32_t packet[6 + FRAME_WORDS];
    unsigned i;

    packet[0] = htonl(SENSOR_TEST_TCP_MAGIC);
    packet[1] = htonl(SENSOR_TEST_TCP_VERSION);
    packet[2] = htonl(sequence);
    packet[3] = htonl(irq_count);
    packet[4] = htonl(sample_count);
    packet[5] = htonl(FRAME_WORDS);
    for (i = 0; i < FRAME_WORDS; ++i)
        packet[6 + i] = htonl(frame[i]);

    return send_all(stream_fd, packet, sizeof(packet));
}

static int send_dma_words(int stream_fd, uint32_t sequence, uint32_t irq_count,
                          uint32_t sample_count, const uint32_t *words,
                          size_t word_count)
{
    uint32_t *packet;
    size_t i;
    int result;

    if (word_count > UINT32_MAX - 6u ||
        word_count > (SIZE_MAX / sizeof(*packet)) - 6u) {
        fprintf(stderr, "TCP packet is too large to encode.\n");
        return -1;
    }

    packet = malloc((6u + word_count) * sizeof(*packet));
    if (!packet) {
        perror("allocate TCP packet");
        return -1;
    }

    packet[0] = htonl(SENSOR_TEST_TCP_MAGIC);
    packet[1] = htonl(SENSOR_TEST_TCP_VERSION);
    packet[2] = htonl(sequence);
    packet[3] = htonl(irq_count);
    packet[4] = htonl(sample_count);
    packet[5] = htonl((uint32_t)word_count);
    for (i = 0; i < word_count; ++i)
        packet[6 + i] = htonl(words[i]);

    result = send_all(stream_fd, packet, (6u + word_count) * sizeof(*packet));
    free(packet);
    return result;
}

static void print_dma_words(unsigned transfer_index, uint32_t irq_count,
                            uint32_t sample_count, const uint32_t *words,
                            size_t word_count)
{
    size_t i;
    size_t preview = word_count < 16u ? word_count : 16u;

    printf("\nDMA interrupt sample %u, UIO irq count %" PRIu32
           ", core sample count %" PRIu32 "\n",
           transfer_index, irq_count, sample_count);
    printf("Frame words: %zu, frame bytes: %zu\n",
           word_count, word_count * sizeof(uint32_t));
    printf("First %zu words:\n  ", preview);
    for (i = 0; i < preview; ++i)
        printf("0x%08" PRIx32 "%c", words[i],
               i + 1 == preview ? '\n' : ' ');

    if (word_count > preview) {
        size_t tail_start = word_count > 4u ? word_count - 4u : preview;
        printf("Last %zu words:\n  ", word_count - tail_start);
        for (i = tail_start; i < word_count; ++i)
            printf("0x%08" PRIx32 "%c", words[i],
                   i + 1 == word_count ? '\n' : ' ');
    }
}

static size_t dma_buffer_available_bytes(const struct dma_buffer *buffer)
{
    uintptr_t mapping;
    uintptr_t words;
    size_t offset;

    if (!buffer || !buffer->mapping || !buffer->words)
        return 0;

    mapping = (uintptr_t)buffer->mapping;
    words = (uintptr_t)buffer->words;
    if (words < mapping)
        return 0;

    offset = (size_t)(words - mapping);
    if (offset > buffer->mapping_size)
        return 0;

    return buffer->mapping_size - offset;
}

static int ensure_dma_mapped(sensor_test_t *test)
{
    if (!test)
        return -1;
    if (test->dma)
        return 0;
    test->dma = map_registers(test->mem_fd, DMA_BASE);
    return test->dma ? 0 : -1;
}

int sensor_test_open(sensor_test_t **out)
{
    sensor_test_t *test;

    if (!out)
        return -1;
    *out = NULL;

    test = calloc(1, sizeof(*test));
    if (!test)
        return -1;
    test->mem_fd = -1;
    test->buffer.fd = -1;

    test->mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (test->mem_fd < 0) {
        perror("open /dev/mem");
        sensor_test_close(test);
        return -1;
    }

    test->core = map_registers(test->mem_fd, CORE_BASE);
    if (!test->core) {
        sensor_test_close(test);
        return -1;
    }

    test->axi_spi = map_registers(test->mem_fd, AXI_SPI_BASE);
    if (!test->axi_spi) {
        sensor_test_close(test);
        return -1;
    }

    *out = test;
    return 0;
}

int sensor_test_initialize_icm20948(sensor_test_t *test)
{
    if (!test || !test->core || !test->axi_spi)
        return -1;
    return initialize_icm20948(test->core, test->axi_spi);
}

int sensor_test_prepare_dma_udmabuf(sensor_test_t *test, const char *device)
{
    return sensor_test_prepare_dma_udmabuf_sized(test, device, FRAME_BYTES);
}

int sensor_test_prepare_dma_udmabuf_sized(sensor_test_t *test,
                                          const char *device,
                                          size_t required_bytes)
{
    if (!test || !device || ensure_dma_mapped(test) != 0)
        return -1;

    close_dma_buffer(&test->buffer);
    if (open_udmabuf_sized(&test->buffer, device, required_bytes) != 0) {
        fprintf(stderr, "Cannot map %s, read its sysfs address, or find "
                        "at least %zu DMA bytes.\n",
                device, required_bytes);
        fprintf(stderr, "Load u-dma-buf, or use --phys with kernel-reserved RAM.\n");
        return -1;
    }
    return 0;
}

int sensor_test_prepare_dma_reserved(sensor_test_t *test,
                                     uint64_t physical_address)
{
    return sensor_test_prepare_dma_reserved_sized(test, physical_address,
                                                 FRAME_BYTES);
}

int sensor_test_prepare_dma_reserved_sized(sensor_test_t *test,
                                           uint64_t physical_address,
                                           size_t required_bytes)
{
    int overlap;

    if (!test || ensure_dma_mapped(test) != 0)
        return -1;

    overlap = overlaps_system_ram(physical_address, required_bytes);
    if (overlap > 0) {
        uint64_t rp_start = 0;
        uint64_t rp_size = 0;

        if (!is_redpitaya_reserved_memory(physical_address, required_bytes,
                                          &rp_start, &rp_size)) {
            fprintf(stderr, "Refusing DMA address 0x%08" PRIx64
                            ": it is still listed as System RAM and is "
                            "not in the Red Pitaya monitor -r reserved "
                            "range.\n",
                    physical_address);
            return -1;
        }
        fprintf(stderr, "Using Red Pitaya reserved memory: start=0x%08"
                        PRIx64 ", size=0x%08" PRIx64 ".\n",
                rp_start, rp_size);
    }
    if (overlap < 0) {
        fprintf(stderr, "Cannot verify reserved memory using /proc/iomem.\n");
        return -1;
    }

    close_dma_buffer(&test->buffer);
    if (open_reserved_memory(&test->buffer, test->mem_fd,
                             physical_address, required_bytes) != 0) {
        perror("map reserved DMA memory");
        return -1;
    }
    return 0;
}

int sensor_test_run_axil_only(sensor_test_t *test)
{
    if (!test || !test->core)
        return -1;
    return run_axil_only_test(test->core) == EXIT_SUCCESS ? 0 : -1;
}

int sensor_test_run_dma(sensor_test_t *test)
{
    volatile uint32_t *core;
    volatile uint32_t *dma;
    struct dma_buffer *buffer;
    uint32_t frame[FRAME_WORDS];
    uint32_t sensor_words[5];
    uint64_t deadline;
    unsigned i;

    if (!test || !test->core || !test->dma || !test->buffer.words) {
        fprintf(stderr, "DMA is not ready; call a prepare_dma function first.\n");
        return -1;
    }

    core = test->core;
    dma = test->dma;
    buffer = &test->buffer;

    if (buffer->physical_address > UINT32_MAX) {
        fprintf(stderr, "DMA buffer is outside the DMA's 32-bit address range.\n");
        return -1;
    }
    for (i = 0; i < FRAME_WORDS; ++i)
        buffer->words[i] = 0xdead0000u | i;
    __sync_synchronize();

    reg_write(core, CORE_CONTROL, CONTROL_RESET);
    usleep(1000);
    reg_write(core, CORE_CONTROL, 0);
    reg_write(core, CORE_COMMAND, COMMAND_CLEAR);
    reg_write(core, CORE_PERIOD, SAMPLE_PERIOD);

    reg_write(dma, S2MM_DMACR, DMA_RESET);
    deadline = monotonic_ms() + TIMEOUT_MS;
    while ((reg_read(dma, S2MM_DMACR) & DMA_RESET) && monotonic_ms() < deadline)
        usleep(1000);
    if (reg_read(dma, S2MM_DMACR) & DMA_RESET) {
        fprintf(stderr, "DMA reset timed out.\n");
        return -1;
    }

    reg_write(dma, S2MM_DMACR, DMA_RUN);
    reg_write(dma, S2MM_DA, (uint32_t)buffer->physical_address);
    reg_write(dma, S2MM_DA_MSB, 0);
    reg_write(dma, S2MM_LENGTH, FRAME_BYTES);
    reg_write(core, CORE_CONTROL, CONTROL_ENABLE);

    deadline = monotonic_ms() + TIMEOUT_MS;
    while (monotonic_ms() < deadline) {
        uint32_t dma_status = reg_read(dma, S2MM_DMASR);
        if (dma_status & DMA_ERROR_MASK) {
            fprintf(stderr, "DMA error, S2MM_DMASR=0x%08" PRIx32 "\n",
                    dma_status);
            goto stop;
        }
        if ((dma_status & DMA_IDLE) && reg_read(core, CORE_COUNT) != 0)
            break;
        usleep(1000);
    }

stop:
    reg_write(core, CORE_CONTROL, 0);
    __sync_synchronize();
    if (!(reg_read(dma, S2MM_DMASR) & DMA_IDLE) ||
        reg_read(core, CORE_COUNT) == 0) {
        fprintf(stderr, "Timed out: core status=0x%08" PRIx32
                        ", DMA status=0x%08" PRIx32 "\n",
                reg_read(core, CORE_STATUS), reg_read(dma, S2MM_DMASR));
        return -1;
    }
    if (reg_read(core, CORE_STATUS) & STATUS_ERROR) {
        fprintf(stderr, "Core error code: 0x%08" PRIx32 "\n",
                reg_read(core, CORE_ERROR));
        return -1;
    }

    for (i = 0; i < FRAME_WORDS; ++i)
        frame[i] = buffer->words[i];
    for (i = 0; i < 5; ++i)
        sensor_words[i] = frame[4 + i];

    printf("PASS: one 36-byte frame arrived through AXI DMA.\n");
    printf("Sample count: %" PRIu32 "\n", reg_read(core, CORE_COUNT));
    printf("Start timestamp: 0x%08" PRIx32 "%08" PRIx32 "\n",
           frame[1], frame[0]);
    printf("Done timestamp:  0x%08" PRIx32 "%08" PRIx32 "\n",
           frame[3], frame[2]);
    print_sensor_bytes(sensor_words);

    for (i = 0; i < 5; ++i) {
        uint32_t snapshot = reg_read(core, CORE_DATA0 + (3u + i) * 4u);
        if (snapshot != frame[8u - i]) {
            fprintf(stderr, "FAIL: AXI-Lite and DMA sensor data differ.\n");
            return -1;
        }
    }
    printf("PASS: DMA data matches the AXI-Lite sensor snapshot.\n");
    return 0;
}

int sensor_test_run_dma_interrupts(sensor_test_t *test,
                                   const char *uio_device,
                                   unsigned transfer_count,
                                   uint32_t sample_period_ticks,
                                   int stream_fd,
                                   int print_frames)
{
    volatile uint32_t *core;
    volatile uint32_t *dma;
    struct dma_buffer *buffer;
    uint32_t frame[FRAME_WORDS];
    unsigned completed = 0;
    int core_enabled = 0;
    int uio_fd;

    if (!test || !test->core || !test->dma || !test->buffer.words) {
        fprintf(stderr, "DMA is not ready; call a prepare_dma function first.\n");
        return -1;
    }
    if (!uio_device || !*uio_device || sample_period_ticks == 0) {
        fprintf(stderr, "Invalid UIO device or sample period.\n");
        return -1;
    }

    core = test->core;
    dma = test->dma;
    buffer = &test->buffer;

    if (buffer->physical_address > UINT32_MAX) {
        fprintf(stderr, "DMA buffer is outside the DMA's 32-bit address range.\n");
        return -1;
    }

    uio_fd = open(uio_device, O_RDWR);
    if (uio_fd < 0) {
        fprintf(stderr, "open %s: %s\n", uio_device, strerror(errno));
        return -1;
    }

    reg_write(core, CORE_CONTROL, CONTROL_RESET);
    usleep(1000);
    reg_write(core, CORE_CONTROL, 0);
    reg_write(core, CORE_COMMAND, COMMAND_CLEAR);
    reg_write(core, CORE_PERIOD, sample_period_ticks);

    if (reset_s2mm_dma(dma) != 0)
        goto failure;
    reg_write(dma, S2MM_DMACR, DMA_RUN | DMA_IOC_IRQ_EN | DMA_ERR_IRQ_EN);

    printf("Interrupt DMA test: sample period=%" PRIu32
           " ticks, frame=%zu bytes, UIO=%s\n",
           sample_period_ticks, (size_t)FRAME_BYTES, uio_device);

    while (transfer_count == 0 || completed < transfer_count) {
        uint32_t irq_count = 0;
        uint32_t dma_status;
        unsigned i;

        for (i = 0; i < FRAME_WORDS; ++i)
            buffer->words[i] = 0xfeed0000u | i;
        __sync_synchronize();

        reg_write(dma, S2MM_DMASR, DMA_IRQ_MASK);
        if (enable_uio_interrupt(uio_fd) != 0)
            goto failure;

        reg_write(dma, S2MM_DA, (uint32_t)buffer->physical_address);
        reg_write(dma, S2MM_DA_MSB, 0);
        reg_write(dma, S2MM_LENGTH, FRAME_BYTES);

        if (!core_enabled) {
            reg_write(core, CORE_CONTROL, CONTROL_ENABLE);
            core_enabled = 1;
        }

        if (wait_for_uio_interrupt(uio_fd, &irq_count) != 0)
            goto failure;

        dma_status = reg_read(dma, S2MM_DMASR);
        reg_write(dma, S2MM_DMASR, dma_status & DMA_IRQ_MASK);

        if (dma_status & DMA_ERROR_MASK) {
            fprintf(stderr, "DMA error after interrupt, S2MM_DMASR=0x%08"
                            PRIx32 "\n", dma_status);
            goto failure;
        }
        if (!(dma_status & DMA_IOC_IRQ)) {
            fprintf(stderr, "UIO interrupt without DMA completion, "
                            "S2MM_DMASR=0x%08" PRIx32 "\n",
                    dma_status);
            goto failure;
        }

        __sync_synchronize();
        for (i = 0; i < FRAME_WORDS; ++i)
            frame[i] = buffer->words[i];

        ++completed;
        if (stream_fd >= 0 &&
            send_dma_frame(stream_fd, completed, irq_count,
                           reg_read(core, CORE_COUNT), frame) != 0)
            goto failure;

        if (print_frames) {
            print_dma_frame(completed, irq_count, reg_read(core, CORE_COUNT),
                            frame);
            fflush(stdout);
        }

        if (reg_read(core, CORE_STATUS) & STATUS_ERROR) {
            fprintf(stderr, "Core error code: 0x%08" PRIx32 "\n",
                    reg_read(core, CORE_ERROR));
            goto failure;
        }
    }

    reg_write(core, CORE_CONTROL, 0);
    reg_write(dma, S2MM_DMACR, 0);
    close(uio_fd);
    return 0;

failure:
    reg_write(core, CORE_CONTROL, 0);
    reg_write(dma, S2MM_DMACR, 0);
    close(uio_fd);
    return -1;
}

int sensor_test_run_dma_interrupts_sized(sensor_test_t *test,
                                         const char *uio_device,
                                         unsigned transfer_count,
                                         uint32_t sample_period_ticks,
                                         int stream_fd,
                                         int print_frames,
                                         size_t transfer_bytes)
{
    volatile uint32_t *core;
    volatile uint32_t *dma;
    struct dma_buffer *buffer;
    uint32_t *frame = NULL;
    size_t frame_words;
    unsigned completed = 0;
    int core_enabled = 0;
    int uio_fd;

    if (!test || !test->core || !test->dma || !test->buffer.words) {
        fprintf(stderr, "DMA is not ready; call a prepare_dma function first.\n");
        return -1;
    }
    if (!uio_device || !*uio_device || sample_period_ticks == 0 ||
        transfer_bytes == 0 || (transfer_bytes % sizeof(uint32_t)) != 0 ||
        transfer_bytes > UINT32_MAX) {
        fprintf(stderr, "Invalid UIO device, sample period, or transfer size.\n");
        return -1;
    }

    core = test->core;
    dma = test->dma;
    buffer = &test->buffer;
    frame_words = transfer_bytes / sizeof(uint32_t);

    if (dma_buffer_available_bytes(buffer) < transfer_bytes) {
        fprintf(stderr, "DMA buffer mapping is too small: need %zu bytes, "
                        "have %zu bytes.\n",
                transfer_bytes, dma_buffer_available_bytes(buffer));
        return -1;
    }
    if (buffer->physical_address > UINT32_MAX) {
        fprintf(stderr, "DMA buffer is outside the DMA's 32-bit address range.\n");
        return -1;
    }

    frame = malloc(transfer_bytes);
    if (!frame) {
        perror("allocate DMA frame copy");
        return -1;
    }

    uio_fd = open(uio_device, O_RDWR);
    if (uio_fd < 0) {
        fprintf(stderr, "open %s: %s\n", uio_device, strerror(errno));
        free(frame);
        return -1;
    }

    reg_write(core, CORE_CONTROL, CONTROL_RESET);
    usleep(1000);
    reg_write(core, CORE_CONTROL, 0);
    reg_write(core, CORE_COMMAND, COMMAND_CLEAR);
    reg_write(core, CORE_PERIOD, sample_period_ticks);

    if (reset_s2mm_dma(dma) != 0)
        goto failure;
    reg_write(dma, S2MM_DMACR, DMA_RUN | DMA_IOC_IRQ_EN | DMA_ERR_IRQ_EN);

    printf("Interrupt DMA test: sample period=%" PRIu32
           " ticks, frame=%zu bytes, UIO=%s\n",
           sample_period_ticks, transfer_bytes, uio_device);

    while (transfer_count == 0 || completed < transfer_count) {
        uint32_t irq_count = 0;
        uint32_t dma_status;
        size_t i;

        for (i = 0; i < frame_words; ++i)
            buffer->words[i] = 0xfeed0000u | (uint32_t)i;
        __sync_synchronize();

        reg_write(dma, S2MM_DMASR, DMA_IRQ_MASK);
        if (enable_uio_interrupt(uio_fd) != 0)
            goto failure;

        reg_write(dma, S2MM_DA, (uint32_t)buffer->physical_address);
        reg_write(dma, S2MM_DA_MSB, 0);
        reg_write(dma, S2MM_LENGTH, (uint32_t)transfer_bytes);

        if (!core_enabled) {
            reg_write(core, CORE_CONTROL, CONTROL_ENABLE);
            core_enabled = 1;
        }

        if (wait_for_uio_interrupt(uio_fd, &irq_count) != 0)
            goto failure;

        dma_status = reg_read(dma, S2MM_DMASR);
        reg_write(dma, S2MM_DMASR, dma_status & DMA_IRQ_MASK);

        if (dma_status & DMA_ERROR_MASK) {
            fprintf(stderr, "DMA error after interrupt, S2MM_DMASR=0x%08"
                            PRIx32 "\n", dma_status);
            goto failure;
        }
        if (!(dma_status & DMA_IOC_IRQ)) {
            fprintf(stderr, "UIO interrupt without DMA completion, "
                            "S2MM_DMASR=0x%08" PRIx32 "\n",
                    dma_status);
            goto failure;
        }

        __sync_synchronize();
        for (i = 0; i < frame_words; ++i)
            frame[i] = buffer->words[i];

        ++completed;
        if (stream_fd >= 0 &&
            send_dma_words(stream_fd, completed, irq_count,
                           reg_read(core, CORE_COUNT), frame,
                           frame_words) != 0)
            goto failure;

        if (print_frames) {
            print_dma_words(completed, irq_count, reg_read(core, CORE_COUNT),
                            frame, frame_words);
            fflush(stdout);
        }

        if (reg_read(core, CORE_STATUS) & STATUS_ERROR) {
            fprintf(stderr, "Core error code: 0x%08" PRIx32 "\n",
                    reg_read(core, CORE_ERROR));
            goto failure;
        }
    }

    reg_write(core, CORE_CONTROL, 0);
    reg_write(dma, S2MM_DMACR, 0);
    close(uio_fd);
    free(frame);
    return 0;

failure:
    reg_write(core, CORE_CONTROL, 0);
    reg_write(dma, S2MM_DMACR, 0);
    close(uio_fd);
    free(frame);
    return -1;
}

void sensor_test_close(sensor_test_t *test)
{
    if (!test)
        return;
    if (test->core)
        reg_write(test->core, CORE_CONTROL, 0);
    close_dma_buffer(&test->buffer);
    if (test->dma)
        munmap((void *)test->dma, MAP_SIZE);
    if (test->axi_spi)
        munmap((void *)test->axi_spi, MAP_SIZE);
    if (test->core)
        munmap((void *)test->core, MAP_SIZE);
    if (test->mem_fd >= 0)
        close(test->mem_fd);
    free(test);
}
