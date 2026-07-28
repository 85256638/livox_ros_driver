#!/usr/bin/env python3
"""Validate the three site identities used by shared-relay recovery."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Set


BCODE = re.compile(r"^[A-Za-z0-9]{15}$")


class ValidationError(ValueError):
    pass


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValidationError("cannot read %s: %s" % (path, exc)) from exc


def _codes(rows: Iterable[object], scope: str) -> Set[str]:
    result: Set[str] = set()
    for row in rows:
        if not isinstance(row, str) or BCODE.fullmatch(row) is None:
            raise ValidationError("%s contains invalid broadcast code %r" % (scope, row))
        if row in result:
            raise ValidationError("%s contains duplicate broadcast code %s" % (scope, row))
        result.add(row)
    return result


def validate(relay_path: Path, driver_path: Path, launch_path: Path) -> int:
    driver = _load_json(driver_path)
    if not isinstance(driver, dict) or not isinstance(driver.get("lidar_config"), list):
        raise ValidationError("Driver config must contain lidar_config array")
    driver_rows = driver["lidar_config"]
    if any(not isinstance(row, dict) for row in driver_rows):
        raise ValidationError("Driver lidar_config entries must be objects")
    for row in driver_rows:
        if "enable_connect" in row and not isinstance(
            row["enable_connect"], bool
        ):
            raise ValidationError("Driver enable_connect values must be booleans")
    enabled_driver_codes = _codes(
        [
            row.get("broadcast_code")
            for row in driver_rows
            if row.get("enable_connect") is True
        ],
        "Driver enabled whitelist",
    )
    if not enabled_driver_codes:
        raise ValidationError("Driver enabled whitelist is empty")

    try:
        root = ET.parse(str(launch_path)).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValidationError("cannot read launch %s: %s" % (launch_path, exc)) from exc
    enable_args = [
        node for node in root.findall("arg")
        if node.get("name") == "relay_power_cycle_enable"
    ]
    if len(enable_args) != 1:
        raise ValidationError("launch must define relay_power_cycle_enable exactly once")
    launch_value = enable_args[0].get("default", "").strip().lower()
    if launch_value not in {"true", "false"}:
        raise ValidationError("relay_power_cycle_enable default must be true or false")
    launch_armed = launch_value == "true"

    relay = _load_json(relay_path)
    if not isinstance(relay, dict) or relay.get("schema_version") != 2:
        raise ValidationError("relay config schema_version must be 2")
    groups = relay.get("power_groups")
    if not isinstance(groups, dict):
        raise ValidationError("relay config power_groups must be an object")
    enabled_members = []
    for group_id, row in groups.items():
        if not isinstance(row, dict):
            raise ValidationError("relay group %s must be an object" % group_id)
        if row.get("enabled") is True:
            members = row.get("members")
            if not isinstance(members, list) or len(members) != 4:
                raise ValidationError(
                    "enabled relay group %s must contain exactly four members" % group_id
                )
            enabled_members.extend(members)
    enabled_relay_codes = _codes(enabled_members, "enabled relay groups")

    if launch_armed and not enabled_relay_codes:
        raise ValidationError(
            "launch is armed but relay config has no enabled power group"
        )
    if launch_armed and enabled_relay_codes != enabled_driver_codes:
        missing = sorted(enabled_driver_codes - enabled_relay_codes)
        extra = sorted(enabled_relay_codes - enabled_driver_codes)
        raise ValidationError(
            "armed relay members must exactly match Driver whitelist; "
            "missing=%s extra=%s" % (missing, extra)
        )

    remap_codes = set()
    prefix = "/livox/lidar_"
    for node in root.iter("remap"):
        source = node.get("from", "")
        if source.startswith(prefix) and BCODE.fullmatch(source[len(prefix) :]):
            remap_codes.add(source[len(prefix) :])
    if remap_codes != enabled_driver_codes:
        print(
            "WARNING: launch lidar remaps differ from Driver whitelist; "
            "missing=%s extra=%s"
            % (
                sorted(enabled_driver_codes - remap_codes),
                sorted(remap_codes - enabled_driver_codes),
            ),
            file=sys.stderr,
        )

    if not launch_armed and enabled_relay_codes != enabled_driver_codes:
        print(
            "WARNING: relay is disabled by launch; before arming, make enabled "
            "relay members exactly match the Driver whitelist",
            file=sys.stderr,
        )
    print(
        "Site identity valid: launch_armed=%s driver=%d relay_enabled=%d remaps=%d"
        % (
            str(launch_armed).lower(),
            len(enabled_driver_codes),
            len(enabled_relay_codes),
            len(remap_codes),
        )
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--relay-config", required=True)
    parser.add_argument("--driver-config", required=True)
    parser.add_argument("--launch", required=True)
    args = parser.parse_args()
    try:
        return validate(
            Path(args.relay_config), Path(args.driver_config), Path(args.launch)
        )
    except ValidationError as exc:
        print("Site identity validation failed: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
