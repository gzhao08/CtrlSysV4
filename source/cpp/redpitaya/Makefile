CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -Wpedantic -std=c11

.PHONY: all clean

all: single_sensor_test dma_interrupt_test ICM_Intan_test intan8_icm4_dma_interrupt_test

single_sensor_test: single_sensor_test.c sensor_test_hw.c sensor_test_hw.h
	$(CC) $(CFLAGS) -o $@ single_sensor_test.c sensor_test_hw.c

dma_interrupt_test: dma_interrupt_test.c sensor_test_hw.c sensor_test_hw.h
	$(CC) $(CFLAGS) -o $@ dma_interrupt_test.c sensor_test_hw.c

ICM_Intan_test: ICM_Intan_test.c sensor_test_hw.c sensor_test_hw.h
	$(CC) $(CFLAGS) -o $@ ICM_Intan_test.c sensor_test_hw.c

intan8_icm4_dma_interrupt_test: intan8_icm4_dma_interrupt_test.c sensor_test_hw.c sensor_test_hw.h
	$(CC) $(CFLAGS) -o $@ intan8_icm4_dma_interrupt_test.c sensor_test_hw.c

clean:
	rm -f single_sensor_test dma_interrupt_test ICM_Intan_test intan8_icm4_dma_interrupt_test
