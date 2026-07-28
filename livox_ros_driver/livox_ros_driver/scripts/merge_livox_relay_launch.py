#!/usr/bin/env python3
"""Inject the stable relay include into an otherwise site-owned launch file.

This is a deliberately narrow fallback for a textual three-way merge conflict.
It preserves the local file byte-for-byte except for two fixed insertions and
refuses files which already contain any relay-manager integration.
"""

from __future__ import annotations

import argparse
import os
import re
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


MARKER = "LIVOX_RELAY_LAUNCH_INTEGRATION"
RELAY_CHILD = "$(find livox_ros_driver)/launch/livox_power_cycle.launch"
MAX_LAUNCH_BYTES = 1024 * 1024


class MergeError(ValueError):
    pass


def _parse(text: str, scope: str) -> ET.Element:
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise MergeError("%s is not valid XML: %s" % (scope, exc)) from exc
    if root.tag != "launch":
        raise MergeError("%s root element must be launch" % scope)
    return root


def _assert_unintegrated(root: ET.Element, text: str) -> None:
    if MARKER in text:
        raise MergeError("local launch already contains the relay marker")
    if any(
        node.get("name") == "relay_power_cycle_enable"
        for node in root.findall("arg")
    ):
        raise MergeError("local launch already defines relay_power_cycle_enable")
    if any("power_cycle" in node.get("file", "").lower() for node in root.iter("include")):
        raise MergeError("local launch already contains a power-cycle include")
    if any(
        node.get("type") == "livox_power_cycle_manager.py"
        or node.get("name") == "livox_power_cycle_manager"
        for node in root.iter("node")
    ):
        raise MergeError("local launch already contains a relay manager node")


def _validate_integrated(text: str) -> None:
    root = _parse(text, "merged launch")
    if text.count(MARKER) != 1:
        raise MergeError("merged launch must contain exactly one relay marker")
    relay_args = [
        node
        for node in root.findall("arg")
        if node.get("name") == "relay_power_cycle_enable"
    ]
    if len(relay_args) != 1 or relay_args[0].get("default") != "false":
        raise MergeError("merged relay enable arg is not the fixed safe default")
    includes = [
        node
        for node in root.findall("include")
        if node.get("file") == RELAY_CHILD
    ]
    if len(includes) != 1:
        raise MergeError("merged launch must contain exactly one relay include")
    child_args = includes[0].findall("arg")
    if len(child_args) != 1 or child_args[0].attrib != {
        "name": "enable",
        "value": "$(arg relay_power_cycle_enable)",
    }:
        raise MergeError("merged relay include has unexpected child arguments")
    drivers = [
        node for node in root.findall("node") if node.get("name") == "livox_driver"
    ]
    if len(drivers) != 1:
        raise MergeError("local launch must contain exactly one direct livox_driver node")
    children = list(root)
    if children.index(includes[0]) > children.index(drivers[0]):
        raise MergeError("relay include must appear before livox_driver")


def merge(local_path: Path, output_path: Path) -> None:
    if local_path.is_symlink() or not local_path.is_file():
        raise MergeError("local launch must be a regular non-symlink file")
    raw = local_path.read_bytes()
    if len(raw) > MAX_LAUNCH_BYTES:
        raise MergeError("local launch exceeds %d bytes" % MAX_LAUNCH_BYTES)
    has_bom = raw.startswith(b"\xef\xbb\xbf")
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise MergeError("local launch is not UTF-8: %s" % exc) from exc
    root = _parse(text, "local launch")
    _assert_unintegrated(root, text)

    root_args = root.findall("arg")
    if not root_args:
        raise MergeError("local launch has no direct root arguments")
    newline = "\r\n" if "\r\n" in text else "\n"
    arg_lines = list(
        re.finditer(
            r"(?m)^(?P<indent>[ \t]*)<arg\b[^>\r\n]*/>[^\S\r\n]*"
            r"(?:<!--[^\r\n]*-->)?[^\S\r\n]*(?:\r?\n|$)",
            text,
        )
    )
    if len(arg_lines) != len(root_args):
        raise MergeError(
            "root arg layout is ambiguous; expected %d single-line args, found %d"
            % (len(root_args), len(arg_lines))
        )
    arg_indent = arg_lines[-1].group("indent")
    arg_insert = (
        arg_indent
        + '<arg name="relay_power_cycle_enable" default="false"/>'
        + "   <!-- true: enable automatic shared-relay OFF/ON -->"
        + newline
    )
    text = text[: arg_lines[-1].end()] + arg_insert + text[arg_lines[-1].end() :]

    driver_start = re.search(
        r"(?ms)^(?P<indent>[ \t]*)<node\b(?=[^>]*\bname\s*=\s*"
        r"[\"']livox_driver[\"'])[^>]*>",
        text,
    )
    if driver_start is None:
        raise MergeError("cannot locate the direct livox_driver node in source text")
    indent = driver_start.group("indent")
    child_indent = indent + "    "
    include_block = (
        indent
        + "<!-- "
        + MARKER
        + " -->"
        + newline
        + indent
        + '<include file="'
        + RELAY_CHILD
        + '">'
        + newline
        + child_indent
        + '<arg name="enable" value="$(arg relay_power_cycle_enable)"/>'
        + newline
        + indent
        + "</include>"
        + newline
        + indent
        + "<!-- End stable relay power-cycle include. -->"
        + newline
        + newline
    )
    text = text[: driver_start.start()] + include_block + text[driver_start.start() :]
    _validate_integrated(text)

    output_parent = output_path.parent
    if output_parent.is_symlink() or not output_parent.is_dir():
        raise MergeError("output parent must be a regular directory")
    mode = stat.S_IMODE(local_path.stat().st_mode)
    encoded = (b"\xef\xbb\xbf" if has_bom else b"") + text.encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(
        prefix=output_path.name + ".new.", dir=str(output_parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, output_path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        merge(Path(args.local), Path(args.output))
    except (MergeError, OSError) as exc:
        print("Relay launch structural merge failed: %s" % exc, file=sys.stderr)
        return 2
    print("Relay launch structural merge completed: %s" % args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
