#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
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
#define ICM_PWR_MGMT_1       0x06u
#define ICM_PWR_MGMT_2       0x07u
#define ICM_USER_BANK_SEL    0x7fu
#define ICM_WHO_AM_I_VALUE   0xeau

#define FRAME_WORDS     9u
#define FRAME_BYTES     (FRAME_WORDS * sizeof(uint32_t))
#define SAMPLE_PERIOD   50000000u /* 1 s at the design's 50 MHz clock. */
#define TIMEOUT_MS      2000u

static uint32_t axi_spi_mode_bits;

struct dma_buffer {
    int fd;
    void *mapping;
    size_t mapping_size;
    volatile uint32_t *words;
    uint64_t physical_address;
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

    if (icm_write_register(spi, ICM_USER_BANK_SEL, 0x00u) != 0 ||
        icm_write_register(spi, ICM_USER_CTRL, 0x10u) != 0 ||
        icm_write_register(spi, ICM_PWR_MGMT_1, 0x01u) != 0 ||
        icm_write_register(spi, ICM_PWR_MGMT_2, 0x00u) != 0) {
        fprintf(stderr, "FAIL: could not initialize the ICM-20948.\n");
        reg_write(core, CORE_CONTROL, 0);
        return -1;
    }
    usleep(10000);
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

static int open_udmabuf(struct dma_buffer *buffer, const char *device)
{
    const char *name = strrchr(device, '/');
    char path[256];
    uint64_t size;

    name = name ? name + 1 : device;
    snprintf(path, sizeof(path), "/sys/class/u-dma-buf/%s/phys_addr", name);
    if (read_u64_file(path, &buffer->physical_address) != 0)
        return -1;
    snprintf(path, sizeof(path), "/sys/class/u-dma-buf/%s/size", name);
    if (read_u64_file(path, &size) != 0 || size < FRAME_BYTES)
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
                                uint64_t physical_address)
{
    long page_size = sysconf(_SC_PAGESIZE);
    uint64_t page_base = physical_address & ~((uint64_t)page_size - 1u);
    size_t offset = (size_t)(physical_address - page_base);
    size_t required = offset + FRAME_BYTES;

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

static void close_dma_buffer(struct dma_buffer *buffer)
{
    if (buffer->mapping && buffer->mapping != MAP_FAILED)
        munmap(buffer->mapping, buffer->mapping_size);
    if (buffer->fd >= 0)
        close(buffer->fd);
}

static void print_sensor_bytes(const uint32_t sensor_words[5])
{
    unsigned i;
    printf("Sensor bytes (register 0x2D onward):\n  ");
    for (i = 0; i < 20; ++i) {
        unsigned word = 4u - i / 4u;
        unsigned shift = (3u - i % 4u) * 8u;
        printf("%02" PRIx32 "%c", (sensor_words[word] >> shift) & 0xffu,
               i == 19 ? '\n' : ' ');
    }
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

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [--no-dma | --udmabuf /dev/udmabuf0 | "
            "--phys RESERVED_ADDRESS]\n",
            program);
}

int main(int argc, char **argv)
{
    const char *udmabuf = "/dev/udmabuf0";
    uint64_t reserved_address = 0;
    int use_reserved = 0;
    int no_dma = 0;
    int mem_fd = -1;
    int result = EXIT_FAILURE;
    volatile uint32_t *core = NULL;
    volatile uint32_t *dma = NULL;
    volatile uint32_t *axi_spi = NULL;
    struct dma_buffer buffer = {.fd = -1};
    uint32_t frame[FRAME_WORDS];
    uint32_t sensor_words[5];
    uint64_t deadline;
    unsigned i;

    if (argc == 2 && strcmp(argv[1], "--no-dma") == 0) {
        no_dma = 1;
    } else if (argc == 3 && strcmp(argv[1], "--udmabuf") == 0) {
        udmabuf = argv[2];
    } else if (argc == 3 && strcmp(argv[1], "--phys") == 0) {
        char *end;
        reserved_address = strtoull(argv[2], &end, 0);
        if (*argv[2] == '\0' || *end != '\0' || (reserved_address & 3u)) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        use_reserved = 1;
    } else if (argc != 1) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        goto cleanup;
    }
    core = map_registers(mem_fd, CORE_BASE);
    if (!core)
        goto cleanup;
    axi_spi = map_registers(mem_fd, AXI_SPI_BASE);
    if (!axi_spi || initialize_icm20948(core, axi_spi) != 0)
        goto cleanup;

    if (no_dma) {
        result = run_axil_only_test(core);
        goto cleanup;
    }

    dma = map_registers(mem_fd, DMA_BASE);
    if (!dma)
        goto cleanup;

    if (use_reserved) {
        int overlap = overlaps_system_ram(reserved_address, FRAME_BYTES);
        if (overlap > 0) {
            fprintf(stderr, "Refusing DMA address 0x%08" PRIx64
                            ": it is still listed as System RAM.\n",
                    reserved_address);
            goto cleanup;
        }
        if (overlap < 0) {
            fprintf(stderr, "Cannot verify reserved memory using /proc/iomem.\n");
            goto cleanup;
        }
        if (open_reserved_memory(&buffer, mem_fd, reserved_address) != 0) {
            perror("map reserved DMA memory");
            goto cleanup;
        }
    } else if (open_udmabuf(&buffer, udmabuf) != 0) {
        fprintf(stderr, "Cannot map %s or read its sysfs address.\n", udmabuf);
        fprintf(stderr, "Load u-dma-buf, or use --phys with kernel-reserved RAM.\n");
        goto cleanup;
    }

    if (buffer.physical_address > UINT32_MAX) {
        fprintf(stderr, "DMA buffer is outside the DMA's 32-bit address range.\n");
        goto cleanup;
    }
    for (i = 0; i < FRAME_WORDS; ++i)
        buffer.words[i] = 0xdead0000u | i;
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
        goto cleanup;
    }

    reg_write(dma, S2MM_DMACR, DMA_RUN);
    reg_write(dma, S2MM_DA, (uint32_t)buffer.physical_address);
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
        goto cleanup;
    }
    if (reg_read(core, CORE_STATUS) & STATUS_ERROR) {
        fprintf(stderr, "Core error code: 0x%08" PRIx32 "\n",
                reg_read(core, CORE_ERROR));
        goto cleanup;
    }

    for (i = 0; i < FRAME_WORDS; ++i)
        frame[i] = buffer.words[i];
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
            goto cleanup;
        }
    }
    printf("PASS: DMA data matches the AXI-Lite sensor snapshot.\n");
    result = EXIT_SUCCESS;

cleanup:
    if (core)
        reg_write(core, CORE_CONTROL, 0);
    close_dma_buffer(&buffer);
    if (dma)
        munmap((void *)dma, MAP_SIZE);
    if (axi_spi)
        munmap((void *)axi_spi, MAP_SIZE);
    if (core)
        munmap((void *)core, MAP_SIZE);
    if (mem_fd >= 0)
        close(mem_fd);
    return result;
}
