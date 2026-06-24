#!/usr/bin/env python3
"""Receive CtrlSys DMA samples from dma_interrupt_test over TCP."""

from __future__ import annotations

import argparse
import datetime as dt
import socket
import struct
import time
from dataclasses import dataclass


MAGIC = 0x4353444D  # "CSDM"
VERSION = 1
FRAME_WORDS = 9
PACKET_WORDS = 6 + FRAME_WORDS
PACKET_STRUCT = struct.Struct("!" + "I" * PACKET_WORDS)
SAMPLE_CLOCK_HZ = 125_000_000


@dataclass
class SampleRecord:
    sequence: int
    irq_count: int
    core_count: int
    arrival_epoch_ns: int
    arrival_perf_ns: int
    fpga_start_ticks: int
    fpga_done_ticks: int
    read_us: float


def recv_exact(sock: socket.socket, length: int) -> bytes:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("TCP connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def set_tcp_quickack(sock: socket.socket) -> None:
    quickack = getattr(socket, "TCP_QUICKACK", None)
    if quickack is None:
        return
    try:
        sock.setsockopt(socket.IPPROTO_TCP, quickack, 1)
    except OSError:
        pass


def configure_low_latency_socket(sock: socket.socket) -> None:
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x10)
    except OSError:
        pass
    set_tcp_quickack(sock)


def timestamp_text(epoch_ns: int) -> str:
    timestamp = dt.datetime.fromtimestamp(epoch_ns / 1_000_000_000).astimezone()
    return timestamp.isoformat(timespec="microseconds")


def sensor_bytes_from_frame(frame: tuple[int, ...]) -> bytes:
    data_words = frame[4:9]
    values = []
    for index in range(20):
        word = data_words[4 - index // 4]
        shift = (3 - index % 4) * 8
        values.append((word >> shift) & 0xFF)
    return bytes(values)


def plot_records(records: list[SampleRecord]) -> None:
    if not records:
        print("no records to plot")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; install it with: python -m pip install matplotlib")
        return

    x = [record.sequence for record in records]
    delta_ms = [
        0.0 if index == 0 else
        (records[index].arrival_perf_ns - records[index - 1].arrival_perf_ns)
        / 1_000_000
        for index in range(len(records))
    ]
    fpga_delta_ms = [
        0.0 if index == 0 else
        (records[index].fpga_start_ticks - records[index - 1].fpga_start_ticks)
        * 1_000 / SAMPLE_CLOCK_HZ
        for index in range(len(records))
    ]

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, delta_ms, "o-", color="tab:orange", label="PC inter-arrival")
    ax.set_xlabel("sample sequence")
    ax.set_ylabel("PC inter-arrival (ms)")
    ax.tick_params(axis="y", labelcolor="tab:orange")
    ax.grid(True, alpha=0.3)

    fpga_ax = ax.twinx()
    fpga_ax.plot(x, fpga_delta_ms, "s--", color="tab:blue",
                 label="FPGA start inter-arrival")
    fpga_ax.set_ylabel("FPGA start inter-arrival (ms)")
    fpga_ax.tick_params(axis="y", labelcolor="tab:blue")

    lines = ax.get_lines() + fpga_ax.get_lines()
    labels = [line.get_label() for line in lines]
    ax.legend(lines, labels, loc="best")

    fig.suptitle("Red Pitaya DMA packet inter-arrival timing")
    fig.tight_layout()
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive timestamped CtrlSys DMA samples over TCP."
    )
    parser.add_argument("host", help="Red Pitaya hostname or IP address")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--count", type=int, default=0,
                        help="number of packets to receive; 0 means forever")
    parser.add_argument("--raw-hex", action="store_true",
                        help="also print the 20 sensor bytes as hex")
    parser.add_argument("--plot", action="store_true",
                        help="plot packet arrival times after capture ends")
    parser.add_argument("--quiet", action="store_true",
                        help="record samples without printing each packet")
    args = parser.parse_args()

    packet_size = PACKET_STRUCT.size
    first_perf_ns: int | None = None
    previous_perf_ns: int | None = None
    records: list[SampleRecord] = []
    received = 0

    try:
        with socket.create_connection((args.host, args.port)) as sock:
            configure_low_latency_socket(sock)
            print(f"connected to {args.host}:{args.port}, packet_size={packet_size}")

            while args.count == 0 or received < args.count:
                payload = recv_exact(sock, packet_size)
                set_tcp_quickack(sock)
                arrival_epoch_ns = time.time_ns()
                arrival_perf_ns = time.perf_counter_ns()

                if first_perf_ns is None:
                    first_perf_ns = arrival_perf_ns
                delta_ms = 0.0 if previous_perf_ns is None else (
                    arrival_perf_ns - previous_perf_ns
                ) / 1_000_000
                elapsed_ms = (arrival_perf_ns - first_perf_ns) / 1_000_000
                previous_perf_ns = arrival_perf_ns

                words = PACKET_STRUCT.unpack(payload)
                magic, version, sequence, irq_count, core_count, frame_words = words[:6]
                frame = words[6:]

                if magic != MAGIC:
                    raise ValueError(f"bad magic 0x{magic:08x}")
                if version != VERSION:
                    raise ValueError(f"unsupported version {version}")
                if frame_words != FRAME_WORDS:
                    raise ValueError(f"unexpected frame_words {frame_words}")

                start_ticks = (frame[1] << 32) | frame[0]
                done_ticks = (frame[3] << 32) | frame[2]
                read_us = (done_ticks - start_ticks) * 1_000_000 / SAMPLE_CLOCK_HZ
                records.append(SampleRecord(
                    sequence=sequence,
                    irq_count=irq_count,
                    core_count=core_count,
                    arrival_epoch_ns=arrival_epoch_ns,
                    arrival_perf_ns=arrival_perf_ns,
                    fpga_start_ticks=start_ticks,
                    fpga_done_ticks=done_ticks,
                    read_us=read_us,
                ))

                line = (
                    f"{timestamp_text(arrival_epoch_ns)} "
                    f"elapsed_ms={elapsed_ms:.3f} delta_ms={delta_ms:.3f} "
                    f"seq={sequence} irq={irq_count} core_count={core_count} "
                    f"fpga_start={start_ticks} fpga_done={done_ticks} "
                    f"read_us={read_us:.3f}"
                )
                if args.raw_hex:
                    line += " sensor_hex=" + sensor_bytes_from_frame(frame).hex(" ")
                if not args.quiet:
                    print(line, flush=True)

                received += 1
    except KeyboardInterrupt:
        print("\nstopped by user")
    finally:
        if args.plot:
            plot_records(records)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
