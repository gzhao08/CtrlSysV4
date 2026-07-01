#!/usr/bin/env python3
"""
Generate a standalone HTML/SVG visualization of packet byte layout versus AXI
Stream word boundaries.

By default this reads source/hdl/config_pkg.sv, computes the current frame sizes,
and draws the fixed packet as Intan frames, ICM frame, zero padding, then trailer.
"""

from __future__ import annotations

import argparse
import html
import math
import operator
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "source" / "hdl" / "config_pkg.sv"
DEFAULT_OUTPUT = REPO_ROOT / "build" / "packet_layout.html"


ALLOWED_BINOPS = {
    "Add": operator.add,
    "Sub": operator.sub,
    "Mult": operator.mul,
    "FloorDiv": operator.floordiv,
}


@dataclass(frozen=True)
class LayoutItem:
    name: str
    start: int
    size: int
    kind: str
    detail: str = ""

    @property
    def end_exclusive(self) -> int:
        return self.start + self.size


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate an HTML/SVG packet layout visualization."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Path to config_pkg.sv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output HTML file",
    )
    parser.add_argument(
        "--layout",
        choices=("tight", "intan-first", "current-rtl"),
        default="tight",
        help="Layout to visualize. tight packs Intan+ICM+trailer; intan-first shows the older Intan+ICM+header order; current-rtl matches the older padded packet_builder behavior.",
    )
    parser.add_argument(
        "--trailer-bytes",
        type=int,
        default=None,
        help="Override packet trailer size in bytes.",
    )
    parser.add_argument(
        "--icm-frame-overhead-bytes",
        type=int,
        default=None,
        help="Override non-measurement overhead in each ICM frame.",
    )
    parser.add_argument(
        "--intan-frame-overhead-bytes",
        type=int,
        default=None,
        help="Override non-measurement overhead in each Intan frame.",
    )
    return parser.parse_args()


def safe_eval(expr: str, constants: Dict[str, int]) -> int:
    """Evaluate a simple SystemVerilog integer expression."""
    import ast

    expr = expr.strip()
    expr = re.sub(r"\b\d+'[sdhboSDHBO][0-9a-fA-F_xXzZ]+\b", sv_literal_to_int, expr)

    def visit(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return visit(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        if isinstance(node, ast.Name):
            if node.id not in constants:
                raise ValueError(f"unknown constant in expression {expr!r}: {node.id}")
            return constants[node.id]
        if isinstance(node, ast.BinOp):
            op_name = type(node.op).__name__
            if op_name not in ALLOWED_BINOPS:
                raise ValueError(f"unsupported operator in expression {expr!r}: {op_name}")
            return ALLOWED_BINOPS[op_name](visit(node.left), visit(node.right))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -visit(node.operand)
        raise ValueError(f"unsupported expression {expr!r}: {ast.dump(node)}")

    tree = ast.parse(expr, mode="eval")
    return visit(tree)


def sv_literal_to_int(match: re.Match[str]) -> str:
    literal = match.group(0)
    _, rest = literal.split("'", 1)
    base_char = rest[0].lower()
    digits = rest[1:].replace("_", "")
    digits = re.sub(r"[xXzZ]", "0", digits)
    base = {"d": 10, "h": 16, "b": 2, "o": 8, "s": 10}.get(base_char, 10)
    if base_char == "s" and len(rest) > 1:
        base_char = rest[1].lower()
        digits = rest[2:].replace("_", "")
        digits = re.sub(r"[xXzZ]", "0", digits)
        base = {"d": 10, "h": 16, "b": 2, "o": 8}.get(base_char, 10)
    return str(int(digits, base))


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def parse_localparams(text: str) -> Dict[str, int]:
    clean = strip_comments(text)
    constants: Dict[str, int] = {}
    pattern = re.compile(r"\blocalparam\s+(?:int\s+)?(\w+)\s*=\s*([^;]+);")
    for name, expr in pattern.findall(clean):
        try:
            constants[name] = safe_eval(expr, constants)
        except (SyntaxError, ValueError):
            # Some derived localparams depend on $bits(), which Python will not
            # evaluate. The visualization computes those directly below.
            continue
    return constants


def get_struct_body(text: str, type_name: str) -> str:
    clean = strip_comments(text)
    pattern = re.compile(
        r"typedef\s+struct\s+packed\s*\{(?P<body>[^{}]*)\}\s*(?P<name>\w+)\s*;",
        flags=re.S,
    )
    for match in pattern.finditer(clean):
        if match.group("name") == type_name:
            return match.group("body")
    raise ValueError(f"could not find struct {type_name}")


def logic_width_bits(range_text: Optional[str], constants: Dict[str, int]) -> int:
    if not range_text:
        return 1
    msb, lsb = range_text.strip()[1:-1].split(":", 1)
    return abs(safe_eval(msb, constants) - safe_eval(lsb, constants)) + 1


def scalar_logic_bits(struct_body: str, constants: Dict[str, int]) -> int:
    total = 0
    pattern = re.compile(r"\blogic\s*(\[[^\]]+\])?\s+\w+\s*;")
    for range_text in pattern.findall(struct_body):
        total += logic_width_bits(range_text or None, constants)
    return total


def require_byte_aligned(name: str, bits: int) -> int:
    if bits % 8 != 0:
        raise ValueError(f"{name} is {bits} bits, which is not byte-aligned")
    return bits // 8


def ceil_to_multiple(value: int, multiple: int) -> int:
    return ((value + multiple - 1) // multiple) * multiple


def byte_range_text(start: int, size: int, axis_bytes: int) -> str:
    end = start + size - 1
    return (
        f"bytes {start}-{end}, "
        f"words {start // axis_bytes}-{end // axis_bytes}, "
        f"lanes {start % axis_bytes}-{end % axis_bytes}"
    )


def build_layout(
    *,
    layout: str,
    header_bytes: int,
    icm_frame_bytes: int,
    intan_frame_bytes: int,
    intan_count: int,
    axis_bytes: int,
) -> Tuple[List[LayoutItem], int]:
    items: List[LayoutItem] = []
    offset = 0

    def add(name: str, size: int, kind: str, detail: str = "") -> None:
        nonlocal offset
        items.append(LayoutItem(name=name, start=offset, size=size, kind=kind, detail=detail))
        offset += size

    def add_padding(name: str, boundary: int) -> None:
        nonlocal offset
        padded = ceil_to_multiple(offset, boundary)
        if padded > offset:
            add(name, padded - offset, "padding")

    if layout == "tight":
        for index in range(intan_count):
            add(f"Intan frame {index}", intan_frame_bytes, "intan")
        add("ICM frame", icm_frame_bytes, "icm")
        add("Packet trailer", header_bytes, "header")
    elif layout == "intan-first":
        for index in range(intan_count):
            add(f"Intan frame {index}", intan_frame_bytes, "intan")
        add("ICM frame", icm_frame_bytes, "icm")
        add("Packet header", header_bytes, "header")
        add_padding("Final packet padding", axis_bytes)
    else:
        add("Packet header", header_bytes, "header")
        add("ICM frame", icm_frame_bytes, "icm")
        add_padding("Header/ICM word padding", axis_bytes)
        for index in range(intan_count):
            frame_start = offset
            add(f"Intan frame {index}", intan_frame_bytes, "intan")
            frame_end = frame_start + intan_frame_bytes
            padded_end = ceil_to_multiple(frame_end, axis_bytes)
            if padded_end > frame_end:
                add(f"Intan frame {index} padding", padded_end - frame_end, "padding")

    return items, offset


def color_for(item: LayoutItem) -> str:
    if item.kind == "header":
        return "#5b6472"
    if item.kind == "icm":
        return "#2576b8"
    if item.kind == "padding":
        return "#d7dbe2"
    match = re.search(r"(\d+)$", item.name)
    index = int(match.group(1)) if match else 0
    palette = [
        "#2a9d8f",
        "#5abf90",
        "#8abf4f",
        "#c7a53b",
        "#e07a5f",
        "#c65a8a",
        "#7b61b3",
        "#4d83c4",
    ]
    return palette[index % len(palette)]


def split_item_by_word(item: LayoutItem, axis_bytes: int) -> Iterable[Tuple[int, int, int]]:
    cursor = item.start
    remaining = item.size
    while remaining > 0:
        word_index = cursor // axis_bytes
        lane = cursor % axis_bytes
        span = min(remaining, axis_bytes - lane)
        yield word_index, lane, span
        cursor += span
        remaining -= span


def render_svg(items: List[LayoutItem], total_bytes: int, axis_bytes: int) -> str:
    word_count = total_bytes // axis_bytes
    lane_px = 8
    row_h = 26
    label_w = 112
    top_h = 28
    width = label_w + axis_bytes * lane_px + 24
    height = top_h + word_count * row_h + 28

    parts = [
        f'<svg class="packet-map" viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="Packet layout with {word_count} AXI words">'
    ]
    parts.append(f'<text class="axis-label" x="{label_w}" y="18">128 byte AXI words</text>')

    for word in range(word_count):
        y = top_h + word * row_h
        parts.append(f'<text class="word-label" x="8" y="{y + 17}">word {word}</text>')
        parts.append(
            f'<rect class="word-bg" x="{label_w}" y="{y}" '
            f'width="{axis_bytes * lane_px}" height="{row_h - 4}" />'
        )
        for lane in range(0, axis_bytes + 1, 16):
            x = label_w + lane * lane_px
            parts.append(
                f'<line class="lane-line" x1="{x}" y1="{y}" x2="{x}" y2="{y + row_h - 4}" />'
            )

    for item in items:
        for word, lane, span in split_item_by_word(item, axis_bytes):
            y = top_h + word * row_h + 3
            x = label_w + lane * lane_px
            title = html.escape(
                f"{item.name}: {byte_range_text(item.start, item.size, axis_bytes)}"
            )
            parts.append(
                f'<rect class="item {item.kind}" x="{x}" y="{y}" '
                f'width="{span * lane_px}" height="{row_h - 10}" '
                f'fill="{color_for(item)}">'
                f"<title>{title}</title></rect>"
            )
            if span * lane_px > 48 and item.kind != "padding":
                label = html.escape(item.name.replace("Intan frame ", "I"))
                parts.append(
                    f'<text class="item-label" x="{x + 4}" y="{y + 13}">{label}</text>'
                )

    parts.append("</svg>")
    return "\n".join(parts)


def render_table(items: List[LayoutItem], axis_bytes: int) -> str:
    rows = []
    for item in items:
        end = item.end_exclusive - 1
        rows.append(
            "<tr>"
            f"<td>{html.escape(item.name)}</td>"
            f"<td>{item.size}</td>"
            f"<td>{item.start}</td>"
            f"<td>{end}</td>"
            f"<td>{item.start // axis_bytes}</td>"
            f"<td>{item.start % axis_bytes}</td>"
            f"<td>{end // axis_bytes}</td>"
            f"<td>{end % axis_bytes}</td>"
            "</tr>"
        )
    return "\n".join(rows)


def render_html(
    *,
    constants: Dict[str, int],
    config_path: Path,
    layout_name: str,
    header_bytes: int,
    icm_measurement_bytes: int,
    intan_measurement_bytes: int,
    icm_frame_overhead_bytes: int,
    intan_frame_overhead_bytes: int,
    icm_frame_bytes: int,
    intan_frame_bytes: int,
    items: List[LayoutItem],
    total_bytes: int,
    axis_bytes: int,
) -> str:
    total_words = total_bytes // axis_bytes
    logical_bytes = header_bytes + icm_frame_bytes + (
        constants["INTAN_SAMPLING_RATIO"] * intan_frame_bytes
    )
    padding_bytes = total_bytes - logical_bytes

    summary_rows = [
        ("Config", str(config_path)),
        ("Layout", layout_name),
        ("NUM_ICM", str(constants["NUM_ICM"])),
        ("NUM_INTAN", str(constants["NUM_INTAN"])),
        ("INTAN_SAMPLING_RATIO", str(constants["INTAN_SAMPLING_RATIO"])),
        ("AXIS_DATA_WIDTH", f'{constants["AXIS_DATA_WIDTH"]} bits'),
        ("AXI word", f"{axis_bytes} bytes"),
        ("Header", f"{header_bytes} bytes"),
        ("ICM measurement", f"{icm_measurement_bytes} bytes"),
        ("Intan measurement", f"{intan_measurement_bytes} bytes"),
        ("ICM frame overhead", f"{icm_frame_overhead_bytes} bytes"),
        ("Intan frame overhead", f"{intan_frame_overhead_bytes} bytes"),
        ("ICM frame", f"{icm_frame_bytes} bytes"),
        ("Intan frame", f"{intan_frame_bytes} bytes"),
        ("Logical data", f"{logical_bytes} bytes"),
        ("Padding in visualized layout", f"{padding_bytes} bytes"),
        ("DMA packet", f"{total_bytes} bytes / {total_words} AXI words"),
    ]
    summary_html = "\n".join(
        f"<tr><th>{html.escape(name)}</th><td>{html.escape(value)}</td></tr>"
        for name, value in summary_rows
    )

    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Packet Layout</title>
<style>
:root {{
    color-scheme: light;
    --bg: #f6f7f9;
    --panel: #ffffff;
    --ink: #17202a;
    --muted: #5c6672;
    --line: #d9dee6;
}}
body {{
    margin: 0;
    background: var(--bg);
    color: var(--ink);
    font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}}
main {{
    max-width: 1240px;
    margin: 0 auto;
    padding: 24px;
}}
h1 {{
    margin: 0 0 6px;
    font-size: 24px;
    font-weight: 700;
    letter-spacing: 0;
}}
.subtle {{
    color: var(--muted);
    margin: 0 0 20px;
}}
.grid {{
    display: grid;
    grid-template-columns: minmax(280px, 380px) 1fr;
    gap: 18px;
    align-items: start;
}}
section {{
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 8px;
    padding: 16px;
}}
.map-wrap {{
    overflow-x: auto;
}}
table {{
    width: 100%;
    border-collapse: collapse;
}}
th, td {{
    border-bottom: 1px solid var(--line);
    padding: 6px 8px;
    text-align: left;
    white-space: nowrap;
}}
th {{
    font-weight: 650;
}}
.summary th {{
    width: 48%;
    color: var(--muted);
}}
.packet-map {{
    min-width: 1160px;
    width: 100%;
    height: auto;
}}
.axis-label {{
    fill: var(--muted);
    font-size: 12px;
}}
.word-label {{
    fill: var(--muted);
    font-size: 12px;
}}
.word-bg {{
    fill: #eef1f5;
    stroke: #cfd6df;
}}
.lane-line {{
    stroke: #cfd6df;
    stroke-width: 0.75;
}}
.item {{
    stroke: rgba(23, 32, 42, 0.35);
    stroke-width: 0.8;
}}
.padding {{
    stroke-dasharray: 3 2;
}}
.item-label {{
    fill: #fff;
    font-size: 11px;
    pointer-events: none;
}}
.padding + .item-label {{
    fill: #17202a;
}}
.table-wrap {{
    max-height: 520px;
    overflow: auto;
}}
.legend {{
    display: flex;
    flex-wrap: wrap;
    gap: 10px 16px;
    margin: 0 0 12px;
}}
.legend span {{
    display: inline-flex;
    align-items: center;
    gap: 6px;
    color: var(--muted);
}}
.swatch {{
    width: 14px;
    height: 14px;
    border-radius: 3px;
    border: 1px solid rgba(23, 32, 42, 0.25);
}}
@media (max-width: 900px) {{
    main {{
        padding: 16px;
    }}
    .grid {{
        grid-template-columns: 1fr;
    }}
}}
</style>
</head>
<body>
<main>
<h1>Packet Layout</h1>
<p class="subtle">Hover over colored regions in the SVG to inspect byte offsets, AXI word numbers, and byte lanes.</p>
<div class="grid">
<section>
<h2>Summary</h2>
<table class="summary">
{summary_html}
</table>
</section>
<section>
<h2>AXI Word Map</h2>
<div class="legend">
<span><i class="swatch" style="background:#5b6472"></i>Header</span>
<span><i class="swatch" style="background:#2576b8"></i>ICM</span>
<span><i class="swatch" style="background:#2a9d8f"></i>Intan</span>
<span><i class="swatch" style="background:#d7dbe2"></i>Padding</span>
</div>
<div class="map-wrap">
{render_svg(items, total_bytes, axis_bytes)}
</div>
</section>
</div>
<section style="margin-top:18px">
<h2>Ranges</h2>
<div class="table-wrap">
<table>
<thead>
<tr>
<th>Name</th><th>Size bytes</th><th>Start byte</th><th>End byte</th>
<th>Start word</th><th>Start lane</th><th>End word</th><th>End lane</th>
</tr>
</thead>
<tbody>
{render_table(items, axis_bytes)}
</tbody>
</table>
</div>
</section>
</main>
</body>
</html>
"""


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    text = config_path.read_text(encoding="utf-8")
    constants = parse_localparams(text)

    required = [
        "NUM_ICM",
        "NUM_INTAN",
        "ICM_DATA_BYTES",
        "INTAN_DATA_BYTES",
        "INTAN_SAMPLING_RATIO",
        "AXIS_DATA_WIDTH",
    ]
    missing = [name for name in required if name not in constants]
    if missing:
        raise SystemExit(f"missing constants in {config_path}: {', '.join(missing)}")
    if constants["AXIS_DATA_WIDTH"] % 8 != 0:
        raise SystemExit("AXIS_DATA_WIDTH must be byte-aligned")

    trailer_bytes = args.trailer_bytes
    if trailer_bytes is None:
        trailer_bytes = constants.get("PACKET_TRAILER_BYTES", 256)
    icm_measurement_bits = scalar_logic_bits(get_struct_body(text, "ICM_measurement_t"), constants)
    intan_measurement_bits = scalar_logic_bits(get_struct_body(text, "Intan_measurement_t"), constants)
    icm_frame_overhead_bits = scalar_logic_bits(get_struct_body(text, "ICM_frame_t"), constants)
    intan_frame_overhead_bits = scalar_logic_bits(get_struct_body(text, "Intan_frame_t"), constants)

    icm_measurement_bytes = require_byte_aligned("ICM_measurement_t", icm_measurement_bits)
    intan_measurement_bytes = require_byte_aligned(
        "Intan_measurement_t", intan_measurement_bits
    )

    icm_frame_overhead_bytes = args.icm_frame_overhead_bytes
    if icm_frame_overhead_bytes is None:
        icm_frame_overhead_bytes = require_byte_aligned(
            "ICM_frame_t scalar overhead", icm_frame_overhead_bits
        )

    intan_frame_overhead_bytes = args.intan_frame_overhead_bytes
    if intan_frame_overhead_bytes is None:
        intan_frame_overhead_bytes = require_byte_aligned(
            "Intan_frame_t scalar overhead", intan_frame_overhead_bits
        )

    icm_frame_bytes = (
        icm_frame_overhead_bytes + constants["NUM_ICM"] * icm_measurement_bytes
    )
    intan_frame_bytes = (
        intan_frame_overhead_bytes + constants["NUM_INTAN"] * intan_measurement_bytes
    )
    axis_bytes = constants["AXIS_DATA_WIDTH"] // 8

    items, total_bytes = build_layout(
        layout=args.layout,
        header_bytes=trailer_bytes,
        icm_frame_bytes=icm_frame_bytes,
        intan_frame_bytes=intan_frame_bytes,
        intan_count=constants["INTAN_SAMPLING_RATIO"],
        axis_bytes=axis_bytes,
    )
    fixed_packet_bytes = constants.get("PACKET_BYTES", total_bytes)
    if fixed_packet_bytes < total_bytes:
        raise SystemExit(
            f"PACKET_BYTES={fixed_packet_bytes} is smaller than layout size "
            f"{total_bytes}"
        )
    if args.layout == "tight" and items and items[-1].kind == "header":
        trailer_item = items.pop()
        total_bytes -= trailer_item.size
        trailer_offset = fixed_packet_bytes - trailer_item.size
        if trailer_offset > total_bytes:
            items.append(LayoutItem(
                name="Fixed packet padding",
                start=total_bytes,
                size=trailer_offset - total_bytes,
                kind="padding",
                detail="Zero padding before trailer",
            ))
        items.append(LayoutItem(
            name=trailer_item.name,
            start=trailer_offset,
            size=trailer_item.size,
            kind=trailer_item.kind,
            detail=trailer_item.detail,
        ))
        total_bytes = fixed_packet_bytes
    elif fixed_packet_bytes > total_bytes:
        items.append(LayoutItem(
            name="Fixed packet padding",
            start=total_bytes,
            size=fixed_packet_bytes - total_bytes,
            kind="padding",
            detail="Zero padding up to PACKET_BYTES",
        ))
        total_bytes = fixed_packet_bytes

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        render_html(
            constants=constants,
            config_path=config_path,
            layout_name=args.layout,
            header_bytes=trailer_bytes,
            icm_measurement_bytes=icm_measurement_bytes,
            intan_measurement_bytes=intan_measurement_bytes,
            icm_frame_overhead_bytes=icm_frame_overhead_bytes,
            intan_frame_overhead_bytes=intan_frame_overhead_bytes,
            icm_frame_bytes=icm_frame_bytes,
            intan_frame_bytes=intan_frame_bytes,
            items=items,
            total_bytes=total_bytes,
            axis_bytes=axis_bytes,
        ),
        encoding="utf-8",
    )

    print(f"wrote {output_path}")
    print(f"layout={args.layout}")
    print(f"packet_bytes={total_bytes}")
    print(f"axis_words={total_bytes // axis_bytes}")
    print(f"axis_bytes={axis_bytes}")


if __name__ == "__main__":
    main()
