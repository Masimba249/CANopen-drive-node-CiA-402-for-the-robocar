#!/usr/bin/env python3
"""
Cross-check the hand-written EDS against the firmware's object dictionary.

An EDS that has drifted from the firmware is worse than no EDS: a
configuration tool will happily write to objects that do not exist and
silently mis-decode the ones that do. This builds the real table (host_tests
has no ESP-IDF dependency), asks it what it contains, and diffs.

    ./check_eds.py              # builds the dumper if needed
    ./check_eds.py --eds ../eds/robocar_drive.eds

Exit code 0 = they agree.
"""
from __future__ import annotations

import argparse
import configparser
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HOST_TESTS = ROOT / "host_tests"
DUMPER = HOST_TESTS / "build" / "od_dump"
DEFAULT_EDS = ROOT / "master" / "eds" / "robocar_drive.eds"

# CiA 301 data type codes, for readable messages.
TYPE_NAMES = {
    0x0001: "BOOLEAN", 0x0002: "INTEGER8", 0x0003: "INTEGER16",
    0x0004: "INTEGER32", 0x0005: "UNSIGNED8", 0x0006: "UNSIGNED16",
    0x0007: "UNSIGNED32", 0x0008: "REAL32", 0x0009: "VISIBLE_STRING",
    0x000A: "OCTET_STRING", 0x000F: "DOMAIN",
}


def build_dumper() -> Path:
    subprocess.run(["make", "-s", "od_dump"], cwd=HOST_TESTS, check=True)
    return DUMPER


def load_firmware_od() -> dict[tuple[int, int], dict]:
    if not DUMPER.exists():
        build_dumper()
    out = subprocess.run([str(DUMPER)], check=True, capture_output=True, text=True).stdout
    entries = {}
    for line in out.splitlines()[1:]:
        idx, sub, dtype, r, w, const, tpdo, rpdo, size = line.split(",")
        entries[(int(idx, 16), int(sub))] = {
            "datatype": int(dtype, 16),
            "readable": r == "1",
            "writable": w == "1",
            "const": const == "1",
            "pdo": tpdo == "1" or rpdo == "1",
            "size": int(size),
        }
    return entries


def load_eds(path: Path) -> tuple[dict[tuple[int, int], dict], configparser.ConfigParser]:
    cp = configparser.ConfigParser(strict=False)
    cp.optionxform = str           # EDS keys are case sensitive in practice
    with open(path, "r", encoding="utf-8") as f:
        cp.read_file(f)

    entries = {}
    for section in cp.sections():
        name = section.strip()
        sub = 0
        if "sub" in name.lower():
            head, _, tail = name.lower().partition("sub")
            try:
                index = int(head, 16)
                sub = int(tail)
            except ValueError:
                continue
        else:
            try:
                index = int(name, 16)
            except ValueError:
                continue        # [FileInfo], [DeviceInfo], ...

        s = cp[section]
        if "ObjectType" not in s:
            continue
        objtype = int(s["ObjectType"], 0)
        if objtype in (0x8, 0x9) and "DataType" not in s:
            continue            # the ARRAY/RECORD header itself carries no data

        access = s.get("AccessType", "ro").strip().lower()
        entries[(index, sub)] = {
            "datatype": int(s["DataType"], 0),
            "readable": access in ("ro", "rw", "rwr", "rww", "const"),
            "writable": access in ("wo", "rw", "rwr", "rww"),
            "const": access == "const",
            "pdo": s.get("PDOMapping", "0").strip() == "1",
            "name": s.get("ParameterName", "?"),
            "default": s.get("DefaultValue"),
        }
    return entries, cp


def check_object_lists(cp: configparser.ConfigParser, eds: dict) -> list[str]:
    """Every index listed in the object lists must have a section."""
    problems = []
    listed = set()
    for section in ("MandatoryObjects", "OptionalObjects", "ManufacturerObjects"):
        if section not in cp:
            problems.append(f"[{section}] is missing")
            continue
        s = cp[section]
        declared = int(s.get("SupportedObjects", "0"))
        got = [k for k in s if k != "SupportedObjects"]
        if declared != len(got):
            problems.append(
                f"[{section}] SupportedObjects={declared} but {len(got)} entries listed")
        for key in got:
            index = int(s[key], 0)
            listed.add(index)
            if not any(i == index for (i, _) in eds):
                problems.append(f"[{section}] lists 0x{index:04X} but it has no section")

    present = {i for (i, _) in eds}
    for index in sorted(present - listed):
        problems.append(f"object 0x{index:04X} has a section but is in no object list")
    return problems


def check_subnumber(cp: configparser.ConfigParser, eds: dict) -> list[str]:
    """SubNumber must equal the number of sub-sections actually present."""
    problems = []
    for section in cp.sections():
        if "SubNumber" not in cp[section]:
            continue
        try:
            index = int(section, 16)
        except ValueError:
            continue
        declared = int(cp[section]["SubNumber"], 0)
        actual = sum(1 for (i, _) in eds if i == index)
        if declared != actual:
            problems.append(
                f"[{section}] SubNumber={declared} but {actual} sub-sections exist")
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eds", default=str(DEFAULT_EDS))
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    fw = load_firmware_od()
    eds, cp = load_eds(Path(args.eds))

    print(f"firmware object dictionary: {len(fw)} entries")
    print(f"EDS {Path(args.eds).name}: {len(eds)} entries")

    problems: list[str] = []

    only_fw = sorted(set(fw) - set(eds))
    only_eds = sorted(set(eds) - set(fw))
    for index, sub in only_fw:
        problems.append(f"0x{index:04X}:{sub:02X} is in the firmware but not the EDS")
    for index, sub in only_eds:
        problems.append(f"0x{index:04X}:{sub:02X} is in the EDS but not the firmware")

    for key in sorted(set(fw) & set(eds)):
        a, b = fw[key], eds[key]
        index, sub = key
        where = f"0x{index:04X}:{sub:02X} ({b['name']})"
        if a["datatype"] != b["datatype"]:
            problems.append(
                f"{where} data type: firmware {TYPE_NAMES.get(a['datatype'], a['datatype'])}, "
                f"EDS {TYPE_NAMES.get(b['datatype'], b['datatype'])}")
        if a["writable"] != b["writable"]:
            problems.append(
                f"{where} writability: firmware {'rw' if a['writable'] else 'ro'}, "
                f"EDS {'writable' if b['writable'] else 'read-only'}")
        if a["readable"] != b["readable"]:
            problems.append(f"{where} readability differs")
        if a["pdo"] != b["pdo"]:
            problems.append(
                f"{where} PDO mappable: firmware {a['pdo']}, EDS {b['pdo']}")
        if args.verbose:
            print(f"  ok {where}")

    problems += check_object_lists(cp, eds)
    problems += check_subnumber(cp, eds)

    if problems:
        print(f"\n{len(problems)} problem(s):")
        for p in problems:
            print(f"  {p}")
        return 1

    print("\nEDS and firmware agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
