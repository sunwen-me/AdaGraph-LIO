#!/usr/bin/env python3

"""Inspect PCD files and write machine-readable point-count/bounds evidence."""

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path
from typing import Any, Dict, List, Tuple


def read_header(stream: Any) -> Tuple[Dict[str, Any], int]:
    header: Dict[str, Any] = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PCD header ended before DATA")
        text = line.decode("ascii").strip()
        if not text or text.startswith("#"):
            continue
        fields = text.split()
        tag = fields[0].upper()
        if tag == "DATA":
            header["data"] = fields[1].lower()
            return header, stream.tell()
        header[tag] = fields[1:]


def scalar_format(size: int, type_code: str) -> str:
    if type_code == "F" and size == 4:
        return "f"
    if type_code == "F" and size == 8:
        return "d"
    if type_code == "I" and size == 1:
        return "b"
    if type_code == "I" and size == 2:
        return "h"
    if type_code == "I" and size == 4:
        return "i"
    if type_code == "I" and size == 8:
        return "q"
    if type_code == "U" and size == 1:
        return "B"
    if type_code == "U" and size == 2:
        return "H"
    if type_code == "U" and size == 4:
        return "I"
    if type_code == "U" and size == 8:
        return "Q"
    raise ValueError(f"unsupported PCD field type/size {type_code}/{size}")


def inspect(path: Path) -> Dict[str, Any]:
    with path.open("rb") as stream:
        header, data_offset = read_header(stream)
        names = list(header.get("FIELDS", []))
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = list(header.get("TYPE", []))
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(names))]
        if not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError(f"{path}: inconsistent PCD field metadata")
        if not all(count == 1 for count in counts):
            raise ValueError(f"{path}: COUNT values other than one are unsupported")
        offsets: Dict[str, int] = {}
        formats: Dict[str, str] = {}
        record_size = 0
        for name, size, type_code in zip(names, sizes, types):
            offsets[name] = record_size
            formats[name] = scalar_format(size, type_code)
            record_size += size
        if not all(name in offsets for name in ("x", "y", "z")):
            raise ValueError(f"{path}: x/y/z fields are required")

        width = int(header.get("WIDTH", ["0"])[0])
        height = int(header.get("HEIGHT", ["1"])[0])
        expected = int(header.get("POINTS", [str(width * height)])[0])
        data = stream.read()
        points: List[Tuple[float, float, float]] = []
        if header.get("data") == "binary":
            if len(data) != expected * record_size:
                raise ValueError(
                    f"{path}: binary payload size {len(data)} != "
                    f"{expected * record_size}"
                )
            for index in range(expected):
                record = data[index * record_size:(index + 1) * record_size]
                values = []
                for name in ("x", "y", "z"):
                    size = struct.calcsize(formats[name])
                    values.append(struct.unpack_from("<" + formats[name], record,
                                                     offsets[name])[0])
                points.append((float(values[0]), float(values[1]), float(values[2])))
        elif header.get("data") == "ascii":
            for line in data.decode("ascii").splitlines():
                if line.strip():
                    values = line.split()
                    points.append(tuple(float(values[names.index(name)])
                                        for name in ("x", "y", "z")))
            if len(points) != expected:
                raise ValueError(f"{path}: ASCII point count mismatch")
        else:
            raise ValueError(f"{path}: unsupported DATA encoding")

    finite = all(math.isfinite(value) for point in points for value in point)
    if points and finite:
        minimum = [min(point[index] for point in points) for index in range(3)]
        maximum = [max(point[index] for point in points) for index in range(3)]
    else:
        minimum = []
        maximum = []
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "data": header.get("data"),
        "declared_point_count": expected,
        "point_count": len(points),
        "record_size_bytes": record_size,
        "finite": finite,
        "minimum": minimum,
        "maximum": maximum,
        "file_size_bytes": path.stat().st_size,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect one or more PCD files.")
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = {
        "schema_version": 1,
        "files": [inspect(path) for path in args.paths],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
    temporary.replace(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
