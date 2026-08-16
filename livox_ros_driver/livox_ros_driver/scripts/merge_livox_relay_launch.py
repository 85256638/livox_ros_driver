#!/usr/bin/env python3
"""Inject stable relay and compact-monitor hooks into a site-owned launch.

This is a deliberately narrow fallback for a textual three-way merge conflict.
It preserves site parameters/remaps byte-for-byte except for bounded fixed
insertions and refuses ambiguous relay or monitor integrations.
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
MONITOR_MARKER = "LIVOX_MONITOR_LAYOUT_V1"
MONITOR_LAYOUT_ARG = "monitor_layout"
MONITOR_LAYOUT_VALUE = "--layout $(arg monitor_layout)"
MONITOR_LAYOUTS = {"compact", "full", "history"}
NETWORK_HEALTH_MARKER = "LIVOX_NETWORK_HEALTH_V1"
NETWORK_HEALTH_ENABLE_ARG = "network_health_enable"
NETWORK_HEALTH_CONFIG_ARG = "network_health_config"
NETWORK_HEALTH_CONFIG_VALUE = "$(env HOME)/.config/livox/network_health.json"
NETWORK_HEALTH_NODE_NAME = "livox_network_health_monitor"
NETWORK_HEALTH_NODE_TYPE = "livox_network_health_monitor.py"
NETWORK_HEALTH_ARGS = "--config $(arg network_health_config)"
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


def _has_relay_artifacts(root: ET.Element, text: str) -> bool:
    return (
        MARKER in text
        or any(
            node.get("name") == "relay_power_cycle_enable"
            for node in root.findall("arg")
        )
        or any(
            "power_cycle" in node.get("file", "").lower()
            for node in root.iter("include")
        )
        or any(
            node.get("type") == "livox_power_cycle_manager.py"
            or node.get("name") == "livox_power_cycle_manager"
            for node in root.iter("node")
        )
    )


def _network_health_nodes(root: ET.Element):
    return [
        node
        for node in root.findall("node")
        if node.get("name") == NETWORK_HEALTH_NODE_NAME
        or node.get("type") == NETWORK_HEALTH_NODE_TYPE
    ]


def _has_network_health_artifacts(root: ET.Element, text: str) -> bool:
    return (
        NETWORK_HEALTH_MARKER in text
        or any(
            node.get("name") == NETWORK_HEALTH_ENABLE_ARG
            or node.get("name") == NETWORK_HEALTH_CONFIG_ARG
            for node in root.findall("arg")
        )
        or bool(_network_health_nodes(root))
    )


def _validate_relay_integration(
    text: str, scope: str, require_safe_default: bool = False
) -> ET.Element:
    root = _parse(text, scope)
    if text.count(MARKER) != 1:
        raise MergeError("%s must contain exactly one relay marker" % scope)
    relay_args = [
        node
        for node in root.findall("arg")
        if node.get("name") == "relay_power_cycle_enable"
    ]
    if len(relay_args) != 1:
        raise MergeError(
            "%s must contain exactly one relay_power_cycle_enable arg" % scope
        )
    relay_default = relay_args[0].get("default", "").strip().lower()
    if relay_default not in {"true", "false"}:
        raise MergeError(
            "%s relay enable default must be literal true or false" % scope
        )
    if require_safe_default and relay_default != "false":
        raise MergeError("new relay integration must use the fixed safe false default")
    includes = [
        node
        for node in root.findall("include")
        if node.get("file") == RELAY_CHILD
    ]
    if len(includes) != 1:
        raise MergeError("%s must contain exactly one direct relay include" % scope)
    power_cycle_includes = [
        node
        for node in root.iter("include")
        if "power_cycle" in node.get("file", "").lower()
    ]
    if (
        len(power_cycle_includes) != 1
        or power_cycle_includes[0] is not includes[0]
    ):
        raise MergeError(
            "%s contains a second or nonstandard power-cycle include" % scope
        )
    if any(
        node.get("type") == "livox_power_cycle_manager.py"
        or node.get("name") == "livox_power_cycle_manager"
        for node in root.iter("node")
    ):
        raise MergeError("%s contains a legacy inline relay manager" % scope)
    child_args = includes[0].findall("arg")
    if len(child_args) != 1 or child_args[0].attrib != {
        "name": "enable",
        "value": "$(arg relay_power_cycle_enable)",
    }:
        raise MergeError("%s relay include has unexpected child arguments" % scope)
    drivers = [
        node for node in root.findall("node") if node.get("name") == "livox_driver"
    ]
    if len(drivers) != 1:
        raise MergeError("%s must contain exactly one direct livox_driver node" % scope)
    children = list(root)
    if children.index(includes[0]) > children.index(drivers[0]):
        raise MergeError("relay include must appear before livox_driver")
    return root


def _validate_network_health_integration(
    root: ET.Element, text: str, scope: str
) -> None:
    if text.count(NETWORK_HEALTH_MARKER) != 1:
        raise MergeError(
            "%s must contain exactly one network health marker" % scope
        )
    enable_args = [
        node
        for node in root.findall("arg")
        if node.get("name") == NETWORK_HEALTH_ENABLE_ARG
    ]
    config_args = [
        node
        for node in root.findall("arg")
        if node.get("name") == NETWORK_HEALTH_CONFIG_ARG
    ]
    if len(enable_args) != 1:
        raise MergeError(
            "%s must contain exactly one network_health_enable arg" % scope
        )
    if len(config_args) != 1:
        raise MergeError(
            "%s must contain exactly one network_health_config arg" % scope
        )
    if enable_args[0].get("default", "").strip().lower() not in {
        "true",
        "false",
    }:
        raise MergeError(
            "%s network_health_enable default must be literal true or false"
            % scope
        )
    if config_args[0].get("default") != NETWORK_HEALTH_CONFIG_VALUE:
        raise MergeError(
            "%s network_health_config must use the fixed external config path"
            % scope
        )

    nodes = _network_health_nodes(root)
    if len(nodes) != 1:
        raise MergeError(
            "%s must contain exactly one direct network health node (found %d)"
            % (scope, len(nodes))
        )
    node = nodes[0]
    expected = {
        "if": "$(arg network_health_enable)",
        "name": NETWORK_HEALTH_NODE_NAME,
        "pkg": "livox_ros_driver",
        "type": NETWORK_HEALTH_NODE_TYPE,
        "output": "screen",
        "respawn": "true",
        "respawn_delay": "5",
        "args": NETWORK_HEALTH_ARGS,
    }
    if node.attrib != expected:
        raise MergeError(
            "%s network health node has unexpected attributes" % scope
        )

    drivers = [
        child
        for child in root.findall("node")
        if child.get("name") == "livox_driver"
    ]
    if len(drivers) != 1:
        raise MergeError(
            "%s must contain exactly one direct livox_driver node" % scope
        )
    children = list(root)
    if children.index(node) > children.index(drivers[0]):
        raise MergeError(
            "network health node must appear before livox_driver"
        )


def _validate_monitor_integration(root: ET.Element, text: str) -> None:
    monitor_nodes = [
        node
        for node in root.findall("node")
        if node.get("name") == "livox_stats_monitor"
        or node.get("type") == "livox_stats_monitor.py"
    ]
    if len(monitor_nodes) > 1:
        raise MergeError("launch contains multiple direct stats monitor nodes")
    if monitor_nodes:
        monitor_args = [
            node
            for node in root.findall("arg")
            if node.get("name") == MONITOR_LAYOUT_ARG
        ]
        if text.count(MONITOR_MARKER) != 1:
            raise MergeError("monitor layout marker must occur exactly once")
        if (
            len(monitor_args) != 1
            or monitor_args[0].get("default") not in MONITOR_LAYOUTS
        ):
            raise MergeError("monitor_layout arg is missing or invalid")
        node_args = monitor_nodes[0].get("args", "")
        if (
            node_args.split().count("--layout") != 1
            or MONITOR_LAYOUT_VALUE not in node_args
        ):
            raise MergeError("stats monitor must consume monitor_layout exactly once")


def _validate_integrated(
    text: str, require_safe_relay_default: bool = False
) -> None:
    root = _validate_relay_integration(
        text,
        "merged launch",
        require_safe_default=require_safe_relay_default,
    )
    _validate_network_health_integration(root, text, "merged launch")
    _validate_monitor_integration(root, text)


def _root_arg_lines(text: str, root: ET.Element):
    root_args = root.findall("arg")
    if not root_args:
        raise MergeError("local launch has no direct root arguments")
    arg_lines = list(
        re.finditer(
            r"(?m)^(?P<indent>[ \t]*)<arg\b[^>\r\n]*/>[^\S\r\n]*"
            r"(?:<!--[^\r\n]*-->)?[^\S\r\n]*(?:\r?\n|$)",
            text,
        )
    )
    by_indent = {}
    for match in arg_lines:
        by_indent.setdefault(match.group("indent"), []).append(match)
    candidates = [
        matches for matches in by_indent.values() if len(matches) == len(root_args)
    ]
    if len(candidates) != 1:
        raise MergeError(
            "root arg layout is ambiguous; expected %d direct single-line args, "
            "found %d single-line arg elements across all depths"
            % (len(root_args), len(arg_lines))
        )
    return root_args, candidates[0]


def _inject_monitor_layout(text: str, root: ET.Element, newline: str) -> str:
    monitor_nodes = [
        node
        for node in root.findall("node")
        if node.get("name") == "livox_stats_monitor"
        or node.get("type") == "livox_stats_monitor.py"
    ]
    if not monitor_nodes:
        return text
    if len(monitor_nodes) != 1:
        raise MergeError("local launch must contain at most one stats monitor node")

    root_args, arg_lines = _root_arg_lines(text, root)
    layout_args = [
        node for node in root_args if node.get("name") == MONITOR_LAYOUT_ARG
    ]
    marker_count = text.count(MONITOR_MARKER)
    if len(layout_args) > 1:
        raise MergeError("local launch contains duplicate monitor_layout args")
    if not layout_args:
        if marker_count:
            raise MergeError("monitor layout marker exists without its arg")
        monitor_indexes = [
            index
            for index, node in enumerate(root_args)
            if node.get("name") == "monitor"
        ]
        insert_index = monitor_indexes[0] if monitor_indexes else len(root_args) - 1
        match = arg_lines[insert_index]
        indent = match.group("indent")
        block = (
            indent
            + "<!-- "
            + MONITOR_MARKER
            + ": fixed-height compact default with explicit diagnostic fallbacks. -->"
            + newline
            + indent
            + '<arg name="monitor_layout" default="compact"/>'
            + "   <!-- compact/full/history -->"
            + newline
        )
        text = text[: match.end()] + block + text[match.end() :]
    else:
        if layout_args[0].get("default") not in MONITOR_LAYOUTS:
            raise MergeError("existing monitor_layout default is invalid")
        if marker_count > 1:
            raise MergeError("monitor layout marker is duplicated")
        if marker_count == 0:
            layout_index = root_args.index(layout_args[0])
            match = arg_lines[layout_index]
            marker = (
                match.group("indent")
                + "<!-- "
                + MONITOR_MARKER
                + " -->"
                + newline
            )
            text = text[: match.start()] + marker + text[match.start() :]

    updated_root = _parse(text, "monitor-integrated launch")
    updated_nodes = [
        node
        for node in updated_root.findall("node")
        if node.get("name") == "livox_stats_monitor"
        or node.get("type") == "livox_stats_monitor.py"
    ]
    if len(updated_nodes) != 1:
        raise MergeError("cannot uniquely locate stats monitor after arg insertion")
    existing_args = updated_nodes[0].get("args")
    if existing_args is not None and "--layout" in existing_args:
        if (
            existing_args.split().count("--layout") != 1
            or MONITOR_LAYOUT_VALUE not in existing_args
        ):
            raise MergeError("existing stats monitor layout args are ambiguous")
        return text

    monitor_start = re.search(
        r"(?ms)^(?P<indent>[ \t]*)<node\b(?=[^>]*\bname\s*=\s*"
        r"[\"']livox_stats_monitor[\"'])[^>]*>",
        text,
    )
    if monitor_start is None:
        raise MergeError("cannot locate stats monitor node in source text")
    opening = monitor_start.group(0)
    args_match = re.search(r"\bargs\s*=\s*([\"'])(.*?)\1", opening, re.S)
    if args_match is not None:
        existing = args_match.group(2).strip()
        replacement = (existing + " " + MONITOR_LAYOUT_VALUE).strip()
        opening = (
            opening[: args_match.start(2)]
            + replacement
            + opening[args_match.end(2) :]
        )
    else:
        close_index = opening.rfind("/>")
        if close_index < 0:
            close_index = opening.rfind(">")
        if close_index < 0:
            raise MergeError("stats monitor node has no closing bracket")
        opening = (
            opening[:close_index]
            + ' args="'
            + MONITOR_LAYOUT_VALUE
            + '"'
            + opening[close_index:]
        )
    return text[: monitor_start.start()] + opening + text[monitor_start.end() :]


def _inject_network_health(text: str, root: ET.Element, newline: str) -> str:
    if _has_network_health_artifacts(root, text):
        _validate_network_health_integration(root, text, "local launch")
        return text

    root_args, arg_lines = _root_arg_lines(text, root)
    arg_indent = arg_lines[-1].group("indent")
    block = (
        arg_indent
        + '<arg name="network_health_enable" default="true"/>'
        + "   <!-- independent 1Hz ARP/ICMP health probe -->"
        + newline
        + arg_indent
        + '<arg name="network_health_config" default="'
        + NETWORK_HEALTH_CONFIG_VALUE
        + '"/>'
        + newline
    )
    text = text[: arg_lines[-1].end()] + block + text[arg_lines[-1].end() :]

    root = _parse(text, "network-health-arg-integrated launch")
    driver_start = re.search(
        r"(?ms)^(?P<indent>[ \t]*)<node\b(?=[^>]*\bname\s*=\s*"
        r"[\"']livox_driver[\"'])[^>]*>",
        text,
    )
    if driver_start is None:
        raise MergeError("cannot locate the direct livox_driver node in source text")
    indent = driver_start.group("indent")
    node_block = (
        indent
        + "<!-- "
        + NETWORK_HEALTH_MARKER
        + ": probe is isolated from the point-cloud data plane. -->"
        + newline
        + indent
        + '<node if="$(arg network_health_enable)" '
        + 'name="livox_network_health_monitor"'
        + newline
        + indent
        + '      pkg="livox_ros_driver" type="livox_network_health_monitor.py"'
        + newline
        + indent
        + '      output="screen" respawn="true" respawn_delay="5"'
        + newline
        + indent
        + '      args="--config $(arg network_health_config)"/>'
        + newline
    )
    return text[: driver_start.start()] + node_block + text[driver_start.start() :]


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
    if any(
        node.get("type") == "livox_power_cycle_manager.py"
        or node.get("name") == "livox_power_cycle_manager"
        for node in root.iter("node")
    ):
        raise MergeError("local launch already contains a relay manager node")
    relay_already_integrated = _has_relay_artifacts(root, text)
    if relay_already_integrated:
        _validate_relay_integration(text, "local launch")
    network_health_already_integrated = _has_network_health_artifacts(root, text)
    if network_health_already_integrated:
        _validate_network_health_integration(root, text, "local launch")

    newline = "\r\n" if "\r\n" in text else "\n"
    text = _inject_monitor_layout(text, root, newline)
    if not network_health_already_integrated:
        root = _parse(text, "monitor-integrated local launch")
        text = _inject_network_health(text, root, newline)
    if not relay_already_integrated:
        root = _parse(text, "monitor-integrated local launch")
        _root_args, arg_lines = _root_arg_lines(text, root)
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
    _validate_integrated(
        text, require_safe_relay_default=not relay_already_integrated
    )

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
