#include "sensor_test_hw.h"

#include <stdint.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define DEFAULT_TCP_PORT 5000u

static void usage(const char *program)
{
    fprintf(stderr,
            "Usage: %s [--count N] [--uio /dev/uio/api] "
            "[--tcp-port PORT | --no-tcp] [--quiet] "
            "[--udmabuf /dev/udmabuf0 | --phys RESERVED_ADDRESS]\n"
            "       --count 0 runs until interrupted.\n",
            program);
}

static int parse_u64(const char *text, uint64_t *value)
{
    char *end;

    if (!text || !*text)
        return -1;
    *value = strtoull(text, &end, 0);
    return *end == '\0' ? 0 : -1;
}

static int parse_unsigned(const char *text, unsigned *value)
{
    uint64_t parsed;

    if (parse_u64(text, &parsed) != 0 || parsed > UINT32_MAX)
        return -1;
    *value = (unsigned)parsed;
    return 0;
}

static void configure_low_latency_socket(int fd)
{
    int value = 1;
    int tos = IPTOS_LOWDELAY;

    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &value, sizeof(value)) != 0)
        perror("setsockopt TCP_NODELAY");

    if (setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) != 0)
        perror("setsockopt IP_TOS");
}

static int open_tcp_server(unsigned port)
{
    int server_fd;
    int client_fd;
    int reuse = 1;
    struct sockaddr_in address;

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return -1;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                   sizeof(reuse)) != 0) {
        perror("setsockopt SO_REUSEADDR");
        close(server_fd);
        return -1;
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons((uint16_t)port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        perror("bind TCP server");
        close(server_fd);
        return -1;
    }
    if (listen(server_fd, 1) != 0) {
        perror("listen TCP server");
        close(server_fd);
        return -1;
    }

    printf("Waiting for TCP receiver on port %u...\n", port);
    do {
        client_fd = accept(server_fd, NULL, NULL);
    } while (client_fd < 0 && errno == EINTR);

    close(server_fd);
    if (client_fd < 0) {
        perror("accept TCP receiver");
        return -1;
    }

    configure_low_latency_socket(client_fd);
    printf("TCP receiver connected.\n");
    return client_fd;
}

int main(int argc, char **argv)
{
    const char *udmabuf = SENSOR_TEST_DEFAULT_UDMABUF;
    const char *uio = SENSOR_TEST_DEFAULT_UIO;
    uint64_t reserved_address = 0;
    unsigned count = 10;
    unsigned tcp_port = DEFAULT_TCP_PORT;
    int use_reserved = 0;
    int tcp_enabled = 1;
    int print_frames = 1;
    int stream_fd = -1;
    int result = EXIT_FAILURE;
    int i;
    sensor_test_t *test = NULL;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
            if (parse_unsigned(argv[++i], &count) != 0) {
                usage(argv[0]);
                return EXIT_FAILURE;
            }
        } else if (strcmp(argv[i], "--uio") == 0 && i + 1 < argc) {
            uio = argv[++i];
        } else if (strcmp(argv[i], "--tcp-port") == 0 && i + 1 < argc) {
            if (parse_unsigned(argv[++i], &tcp_port) != 0 ||
                tcp_port == 0 || tcp_port > 65535u) {
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            tcp_enabled = 1;
        } else if (strcmp(argv[i], "--no-tcp") == 0) {
            tcp_enabled = 0;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            print_frames = 0;
        } else if (strcmp(argv[i], "--udmabuf") == 0 && i + 1 < argc) {
            udmabuf = argv[++i];
            use_reserved = 0;
        } else if (strcmp(argv[i], "--phys") == 0 && i + 1 < argc) {
            if (parse_u64(argv[++i], &reserved_address) != 0 ||
                (reserved_address & 3u)) {
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            use_reserved = 1;
        } else {
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    printf("Initializing ICM path, then sampling every 1 ms (%u clock ticks).\n",
           SENSOR_TEST_1MS_TICKS);
    printf("8 Intan + 4 ICM DMA packet: %u bytes (%u 32-bit words).\n",
           SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES,
           SENSOR_TEST_INTAN8_ICM4_PACKET_WORDS);

    if (tcp_enabled) {
        stream_fd = open_tcp_server(tcp_port);
        if (stream_fd < 0)
            goto cleanup;
    }

    if (sensor_test_open(&test) != 0 ||
        sensor_test_initialize_icm20948(test) != 0)
        goto cleanup;

    if (use_reserved) {
        if (sensor_test_prepare_dma_reserved_sized(
                test, reserved_address,
                SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES) != 0)
            goto cleanup;
    } else if (sensor_test_prepare_dma_udmabuf_sized(
                   test, udmabuf,
                   SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES) != 0) {
        goto cleanup;
    }

    result = sensor_test_run_dma_interrupts_sized(
                 test, uio, count, SENSOR_TEST_1MS_TICKS,
                 stream_fd, print_frames,
                 SENSOR_TEST_INTAN8_ICM4_PACKET_BYTES) == 0
           ? EXIT_SUCCESS : EXIT_FAILURE;

cleanup:
    if (stream_fd >= 0)
        close(stream_fd);
    sensor_test_close(test);
    return result;
}
