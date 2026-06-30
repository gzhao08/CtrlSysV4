#!/usr/bin/env python3
"""Receive CtrlSys DMA samples from dma_interrupt_test over TCP."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import socket
import struct
import time
from dataclasses import dataclass


MAGIC = 0x4353444D  # "CSDM"
VERSION = 1
FRAME_WORDS = 9
HEADER_WORDS = 6
HEADER_STRUCT = struct.Struct("!" + "I" * HEADER_WORDS)
MAX_FRAME_WORDS = 1_000_000
SAMPLE_CLOCK_HZ = 125_000_000
NUM_INTAN = 8
NUM_ICM = 4
INTAN_SAMPLING_RATIO = 30
INTAN_DATA_BYTES = 64
ICM_DATA_BYTES = 20
PACKET_HEADER_BYTES = 68
INTAN_MEASUREMENT_BYTES = 1 + INTAN_DATA_BYTES
ICM_MEASUREMENT_BYTES = 1 + ICM_DATA_BYTES
INTAN_FRAME_BYTES = 16 + NUM_INTAN * INTAN_MEASUREMENT_BYTES
ICM_FRAME_BYTES = 16 + NUM_ICM * ICM_MEASUREMENT_BYTES
PACKET_PAYLOAD_BYTES = (
    INTAN_SAMPLING_RATIO * INTAN_FRAME_BYTES
    + ICM_FRAME_BYTES
    + PACKET_HEADER_BYTES
)


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


@dataclass
class CapturedPacket:
    sequence: int
    irq_count: int
    core_count: int
    frame_words: int
    frame_bytes: bytes


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
    if len(frame) < FRAME_WORDS:
        return b""
    data_words = frame[4:9]
    values = []
    for index in range(20):
        word = data_words[4 - index // 4]
        shift = (3 - index % 4) * 8
        values.append((word >> shift) & 0xFF)
    return bytes(values)


def word_bytes(word: int, byte_order: str) -> bytes:
    little = word.to_bytes(4, "little")

    if byte_order == "little":
        return little
    if byte_order == "big":
        return word.to_bytes(4, "big")
    if byte_order == "swap16":
        return little[2:4] + little[0:2]
    if byte_order == "reverse16":
        return bytes((little[1], little[0], little[3], little[2]))

    raise ValueError(f"unsupported DMA word byte order {byte_order!r}")


def frame_words_to_dma_bytes(frame: tuple[int, ...],
                             byte_order: str = "little") -> bytes:
    return b"".join(word_bytes(word, byte_order) for word in frame)


def packet_layout_score(data: bytes) -> int:
    score = 0
    expected_trailer_offset = (
        INTAN_SAMPLING_RATIO * INTAN_FRAME_BYTES + ICM_FRAME_BYTES
    )

    if len(data) >= INTAN_FRAME_BYTES:
        expected_ids = list(range(NUM_INTAN - 1, -1, -1))
        intan_ids = [
            data[16 + index * INTAN_MEASUREMENT_BYTES]
            for index in range(NUM_INTAN)
        ]
        score += sum(
            1 for observed, expected in zip(intan_ids, expected_ids)
            if observed == expected
        )

    if len(data) >= expected_trailer_offset + PACKET_HEADER_BYTES:
        packet_num = int.from_bytes(
            data[expected_trailer_offset:expected_trailer_offset + 4],
            "big",
        )
        intan_frame_count = int.from_bytes(
            data[expected_trailer_offset + 4:expected_trailer_offset + 8],
            "big",
        )
        flags = data[
            expected_trailer_offset + 8:
            expected_trailer_offset + PACKET_HEADER_BYTES
        ]
        if intan_frame_count == INTAN_SAMPLING_RATIO:
            score += 16
        if flags.count(0) >= 56:
            score += 4
        if packet_num < 1_000_000:
            score += 1

    return score


def choose_dma_byte_order(frame: tuple[int, ...]) -> str:
    candidates = ("little", "swap16", "big", "reverse16")
    scored = [
        (packet_layout_score(frame_words_to_dma_bytes(frame, candidate)),
         candidate)
        for candidate in candidates
    ]
    scored.sort(reverse=True)
    if scored[0][0] < 10:
        return "little"
    return scored[0][1]


def hex_preview(data: bytes, max_bytes: int) -> str:
    if len(data) <= max_bytes:
        return data.hex(" ")
    return data[:max_bytes].hex(" ") + f" ... ({len(data)} bytes total)"


def read_u64_be(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 8], "big")


def print_intan_frame(data: bytes, frame_index: int, max_sensors: int,
                      max_data_bytes: int) -> None:
    offset = frame_index * INTAN_FRAME_BYTES
    init_ts = read_u64_be(data, offset)
    done_ts = read_u64_be(data, offset + 8)

    print(f"  Intan frame {frame_index}: init_ts={init_ts} done_ts={done_ts}")
    offset += 16
    for sensor_index in range(min(NUM_INTAN, max_sensors)):
        sensor_offset = offset + sensor_index * INTAN_MEASUREMENT_BYTES
        sensor_id = data[sensor_offset]
        sensor_data = data[
            sensor_offset + 1:sensor_offset + 1 + INTAN_DATA_BYTES
        ]
        print(
            f"    Intan measurement {sensor_index}: "
            f"sensor_id={sensor_id} data={hex_preview(sensor_data, max_data_bytes)}"
        )


def print_icm_frame(data: bytes, max_sensors: int, max_data_bytes: int) -> None:
    offset = INTAN_SAMPLING_RATIO * INTAN_FRAME_BYTES
    init_ts = read_u64_be(data, offset)
    done_ts = read_u64_be(data, offset + 8)

    print(f"  ICM frame: init_ts={init_ts} done_ts={done_ts}")
    offset += 16
    for sensor_index in range(min(NUM_ICM, max_sensors)):
        sensor_offset = offset + sensor_index * ICM_MEASUREMENT_BYTES
        sensor_id = data[sensor_offset]
        sensor_data = data[sensor_offset + 1:sensor_offset + 1 + ICM_DATA_BYTES]
        print(
            f"    ICM measurement {sensor_index}: "
            f"sensor_id={sensor_id} data={hex_preview(sensor_data, max_data_bytes)}"
        )


def print_packet_trailer(data: bytes, frame_words: int) -> None:
    if len(data) < PACKET_PAYLOAD_BYTES:
        print("  Packet is too short to contain the expected trailer/header")
        return

    offset = INTAN_SAMPLING_RATIO * INTAN_FRAME_BYTES + ICM_FRAME_BYTES
    packet_num = int.from_bytes(data[offset:offset + 4], "big")
    intan_frame_count = int.from_bytes(data[offset + 4:offset + 8], "big")
    flags = data[offset + 8:offset + PACKET_HEADER_BYTES]
    padding = len(data) - PACKET_PAYLOAD_BYTES

    print(
        f"  Trailer/header: packet_num={packet_num} "
        f"intan_frame_count={intan_frame_count} "
        f"flags={hex_preview(flags, 16)} padding_bytes={padding} "
        f"frame_words={frame_words}"
    )


def find_plausible_trailer_offsets(data: bytes) -> list[int]:
    offsets: list[int] = []
    search_start = max(0, PACKET_PAYLOAD_BYTES - 2048)
    search_end = min(len(data) - PACKET_HEADER_BYTES + 1,
                     PACKET_PAYLOAD_BYTES + 2048)

    for offset in range(search_start, search_end):
        intan_frame_count = int.from_bytes(data[offset + 4:offset + 8], "big")
        flags = data[offset + 8:offset + PACKET_HEADER_BYTES]
        if intan_frame_count == INTAN_SAMPLING_RATIO and flags.count(0) >= 56:
            offsets.append(offset)

    return offsets


def print_packet_sanity(data: bytes) -> None:
    expected_trailer_offset = (
        INTAN_SAMPLING_RATIO * INTAN_FRAME_BYTES + ICM_FRAME_BYTES
    )
    expected_ids = list(range(NUM_INTAN - 1, -1, -1))
    intan_ids = [
        data[16 + index * INTAN_MEASUREMENT_BYTES]
        for index in range(NUM_INTAN)
    ] if len(data) >= INTAN_FRAME_BYTES else []

    print(
        f"  Sanity: expected Intan IDs at frame 0 physical offsets "
        f"{expected_ids}, observed {intan_ids}"
    )

    if len(data) >= expected_trailer_offset + 8:
        intan_frame_count = int.from_bytes(
            data[expected_trailer_offset + 4:expected_trailer_offset + 8],
            "big",
        )
        if intan_frame_count != INTAN_SAMPLING_RATIO:
            candidates = find_plausible_trailer_offsets(data)
            if candidates:
                shifts = [
                    candidate - expected_trailer_offset
                    for candidate in candidates[:8]
                ]
                print(
                    f"  Sanity: expected trailer offset "
                    f"{expected_trailer_offset}, but intan_frame_count="
                    f"{intan_frame_count}; plausible trailer shifts={shifts}"
                )
            else:
                print(
                    f"  Sanity: expected trailer offset "
                    f"{expected_trailer_offset}, but intan_frame_count="
                    f"{intan_frame_count}; no nearby plausible trailer found"
                )


def print_captured_sensor_data(packets: list[CapturedPacket],
                               max_intan_frames: int,
                               max_sensors: int,
                               max_data_bytes: int) -> None:
    if not packets:
        print("no captured sensor packets to print")
        return

    print("\nDecoded sensor data:")
    for packet in packets:
        print(
            f"Packet seq={packet.sequence} irq={packet.irq_count} "
            f"core_count={packet.core_count}"
        )
        if len(packet.frame_bytes) < PACKET_PAYLOAD_BYTES:
            print(
                f"  Expected at least {PACKET_PAYLOAD_BYTES} payload bytes, "
                f"received {len(packet.frame_bytes)} bytes"
            )
            continue

        for frame_index in range(min(INTAN_SAMPLING_RATIO, max_intan_frames)):
            print_intan_frame(
                packet.frame_bytes,
                frame_index,
                max_sensors,
                max_data_bytes,
            )
        print_icm_frame(packet.frame_bytes, max_sensors, max_data_bytes)
        print_packet_trailer(packet.frame_bytes, packet.frame_words)
        print_packet_sanity(packet.frame_bytes)


def print_captured_raw_bytes(packets: list[CapturedPacket],
                             bytes_per_line: int) -> None:
    if not packets:
        print("no captured sensor packets to print")
        return

    bytes_per_line = max(1, bytes_per_line)

    print("\nRaw packet bytes:")
    for packet in packets:
        print(
            f"Packet seq={packet.sequence} irq={packet.irq_count} "
            f"core_count={packet.core_count} bytes={len(packet.frame_bytes)}"
        )

        for offset in range(0, len(packet.frame_bytes), bytes_per_line):
            chunk = packet.frame_bytes[offset:offset + bytes_per_line]
            print(f"  {offset:05x}: {chunk.hex(' ')}")


def open_csv_writer(path: str | None) -> tuple[object | None, csv.writer | None]:
    if path is None:
        return None, None

    csv_file = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow([
        "sequence",
        "irq_count",
        "core_count",
        "arrival_iso",
        "arrival_epoch_ns",
        "pc_elapsed_ms",
        "pc_inter_arrival_ms",
        "fpga_start_ticks",
        "fpga_done_ticks",
        "fpga_inter_arrival_ticks",
        "fpga_inter_arrival_ms",
        "read_us",
        "sensor_hex",
    ])
    return csv_file, writer


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
    parser.add_argument("--csv", metavar="PATH",
                        help="write received samples to a CSV file")
    parser.add_argument("--flush-every", type=int, default=1000,
                        help="flush the CSV file every N rows; 0 flushes only at the end")
    parser.add_argument("--print-sensor-data", action="store_true",
                        help="print decoded Intan/ICM packet contents after capture")
    parser.add_argument("--print-raw-bytes", action="store_true",
                        help="print raw reconstructed DMA packet bytes after capture")
    parser.add_argument("--print-packets", type=int, default=1,
                        help="number of received packets to print after capture")
    parser.add_argument("--print-intan-frames", type=int, default=2,
                        help="Intan frames to decode per printed packet")
    parser.add_argument("--print-sensors", type=int, default=8,
                        help="sensor measurements to decode per printed frame")
    parser.add_argument("--print-data-bytes", type=int, default=64,
                        help="data bytes to print per decoded measurement")
    parser.add_argument("--raw-bytes-per-line", type=int, default=32,
                        help="bytes per line when using --print-raw-bytes")
    parser.add_argument("--dma-byte-order",
                        choices=("auto", "little", "swap16", "big", "reverse16"),
                        default="auto",
                        help="byte order used to reconstruct packet bytes from DMA 32-bit words")
    args = parser.parse_args()

    header_size = HEADER_STRUCT.size
    first_perf_ns: int | None = None
    previous_perf_ns: int | None = None
    previous_start_ticks: int | None = None
    records: list[SampleRecord] = []
    captured_packets: list[CapturedPacket] = []
    selected_dma_byte_order: str | None = None
    received = 0
    csv_file, csv_writer = open_csv_writer(args.csv)

    try:
        with socket.create_connection((args.host, args.port)) as sock:
            configure_low_latency_socket(sock)
            print(f"connected to {args.host}:{args.port}, header_size={header_size}")

            while args.count == 0 or received < args.count:
                header_payload = recv_exact(sock, header_size)
                magic, version, sequence, irq_count, core_count, frame_words = (
                    HEADER_STRUCT.unpack(header_payload)
                )

                if magic != MAGIC:
                    raise ValueError(f"bad magic 0x{magic:08x}")
                if version != VERSION:
                    raise ValueError(f"unsupported version {version}")
                if frame_words == 0 or frame_words > MAX_FRAME_WORDS:
                    raise ValueError(f"unexpected frame_words {frame_words}")

                frame_payload = recv_exact(sock, frame_words * 4)
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

                frame = struct.unpack("!" + "I" * frame_words, frame_payload)
                if selected_dma_byte_order is None:
                    selected_dma_byte_order = (
                        choose_dma_byte_order(frame)
                        if args.dma_byte_order == "auto"
                        else args.dma_byte_order
                    )
                    if args.print_sensor_data or args.print_raw_bytes:
                        print(
                            f"using DMA word byte order: "
                            f"{selected_dma_byte_order}"
                        )
                frame_bytes = frame_words_to_dma_bytes(
                    frame,
                    selected_dma_byte_order or "little",
                )

                if frame_words == FRAME_WORDS:
                    start_ticks = (frame[1] << 32) | frame[0]
                    done_ticks = (frame[3] << 32) | frame[2]
                else:
                    start_ticks = 0
                    done_ticks = 0
                fpga_delta_ticks = (
                    0 if previous_start_ticks is None or start_ticks == 0
                    else start_ticks - previous_start_ticks
                )
                fpga_delta_ms = fpga_delta_ticks * 1_000 / SAMPLE_CLOCK_HZ
                if start_ticks != 0:
                    previous_start_ticks = start_ticks
                read_us = (
                    0.0 if start_ticks == 0
                    else (done_ticks - start_ticks) * 1_000_000 / SAMPLE_CLOCK_HZ
                )
                if args.plot:
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
                if (
                    (args.print_sensor_data or args.print_raw_bytes)
                    and len(captured_packets) < max(0, args.print_packets)
                ):
                    captured_packets.append(CapturedPacket(
                        sequence=sequence,
                        irq_count=irq_count,
                        core_count=core_count,
                        frame_words=frame_words,
                        frame_bytes=frame_bytes,
                    ))

                if csv_writer is not None:
                    csv_writer.writerow([
                        sequence,
                        irq_count,
                        core_count,
                        timestamp_text(arrival_epoch_ns),
                        arrival_epoch_ns,
                        f"{elapsed_ms:.6f}",
                        f"{delta_ms:.6f}",
                        start_ticks,
                        done_ticks,
                        fpga_delta_ticks,
                        f"{fpga_delta_ms:.6f}",
                        f"{read_us:.6f}",
                        sensor_bytes_from_frame(frame).hex(" "),
                    ])
                    if args.flush_every > 0 and (received + 1) % args.flush_every == 0:
                        csv_file.flush()

                line = (
                    f"{timestamp_text(arrival_epoch_ns)} "
                    f"elapsed_ms={elapsed_ms:.3f} delta_ms={delta_ms:.3f} "
                    f"seq={sequence} irq={irq_count} core_count={core_count} "
                    f"frame_words={frame_words}"
                )
                if frame_words == FRAME_WORDS:
                    line += (
                        f" fpga_start={start_ticks} fpga_done={done_ticks} "
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
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        if args.plot:
            plot_records(records)
        if args.print_sensor_data:
            print_captured_sensor_data(
                captured_packets,
                max(0, args.print_intan_frames),
                max(0, args.print_sensors),
                max(0, args.print_data_bytes),
            )
        if args.print_raw_bytes:
            print_captured_raw_bytes(
                captured_packets,
                args.raw_bytes_per_line,
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
