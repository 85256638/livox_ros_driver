#!/usr/bin/env python3
"""Low-rate per-LiDAR network health probe.

The probe is deliberately a separate ROS process. It never owns the Driver's
data plane and it never toggles relay hardware. It publishes one JSON frame per
second; the Driver decides whether a verified network episode warrants a
bounded soft reboot and, only after that, a relay request.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import re
import shutil
import socket
import subprocess
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


CONFIG_SCHEMA_VERSION = 1
DEFAULT_WINDOW_SECONDS = 10.0
DEFAULT_PROBE_INTERVAL_SECONDS = 1.0
DEFAULT_UNSTABLE_FAILURES = 2
DEFAULT_UNREACHABLE_CONSECUTIVE_FAILURES = 3
DEFAULT_HEALTHY_CONSECUTIVE_SUCCESSES = 5
DEFAULT_SOFT_REBOOT_MAX_ATTEMPTS = 3
DEFAULT_SOFT_REBOOT_INTERVAL_SECONDS = 5.0
DEFAULT_SOFT_REBOOT_ACK_TIMEOUT_SECONDS = 2.0
STATE_UNKNOWN = "UNKNOWN"
STATE_OK = "NET_OK"
STATE_DEGRADED = "NET_DEGRADED"
STATE_UNSTABLE = "NET_UNSTABLE"
STATE_UNREACHABLE = "NET_UNREACHABLE"
SAFE_CODE = re.compile(r"^[A-Za-z0-9]{15}$")
SAFE_IP = re.compile(r"^[0-9a-fA-F:.]+$")


class ConfigurationError(ValueError):
    pass


def _number(data: Mapping[str, Any], key: str, default: float, minimum: float, maximum: float) -> float:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError("%s must be a number" % key)
    value = float(value)
    if value < minimum or value > maximum:
        raise ConfigurationError("%s must be between %s and %s" % (key, minimum, maximum))
    return value


def _integer(data: Mapping[str, Any], key: str, default: int, minimum: int, maximum: int) -> int:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError("%s must be an integer" % key)
    if value < minimum or value > maximum:
        raise ConfigurationError("%s must be between %s and %s" % (key, minimum, maximum))
    return value


@dataclass(frozen=True)
class Target:
    broadcast_code: str
    ip: str
    handle: int
    interface: str = ""


@dataclass(frozen=True)
class MonitorConfig:
    probe_interval_seconds: float
    window_seconds: float
    unstable_failures: int
    unreachable_consecutive_failures: int
    healthy_consecutive_successes: int
    soft_reboot_max_attempts: int
    soft_reboot_interval_seconds: float
    soft_reboot_ack_timeout_seconds: float
    targets: Tuple[Target, ...]


@dataclass
class TargetWindow:
    window_seconds: float
    unstable_failures: int
    unreachable_consecutive_failures: int
    healthy_consecutive_successes: int
    samples: deque = field(default_factory=deque)
    consecutive_failures: int = 0
    consecutive_successes: int = 0
    state: str = STATE_UNKNOWN

    def observe(self, success: bool, now: float) -> Dict[str, Any]:
        self.samples.append((now, bool(success)))
        if success:
            self.consecutive_successes += 1
            self.consecutive_failures = 0
        else:
            self.consecutive_failures += 1
            self.consecutive_successes = 0
        cutoff = now - self.window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        failures = sum(1 for _, ok in self.samples if not ok)
        total = len(self.samples)
        if self.consecutive_failures >= self.unreachable_consecutive_failures:
            self.state = STATE_UNREACHABLE
        elif failures >= self.unstable_failures:
            self.state = STATE_UNSTABLE
        elif failures:
            self.state = STATE_DEGRADED
        elif self.consecutive_successes >= self.healthy_consecutive_successes:
            self.state = STATE_OK
        else:
            # A clean sample before the healthy-success threshold is still
            # observation, not a loss. Do not turn normal startup probing
            # into a false NET_DEGRADED alarm.
            self.state = STATE_UNKNOWN
        return {
            "state": self.state,
            "window_samples": total,
            "window_failures": failures,
            "consecutive_failures": self.consecutive_failures,
            "consecutive_successes": self.consecutive_successes,
            "loss_percent": (100.0 * failures / total) if total else 0.0,
        }


def load_config(path: str) -> MonitorConfig:
    config_path = Path(os.path.expandvars(os.path.expanduser(path)))
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ConfigurationError("cannot read network health config: %s" % exc)
    if not isinstance(raw, Mapping) or raw.get("schema_version") != CONFIG_SCHEMA_VERSION:
        raise ConfigurationError("network health config schema_version must be 1")
    interval = _number(raw, "probe_interval_seconds", DEFAULT_PROBE_INTERVAL_SECONDS, 0.5, 10.0)
    window = _number(raw, "window_seconds", DEFAULT_WINDOW_SECONDS, 3.0, 60.0)
    unstable = _integer(raw, "unstable_failures", DEFAULT_UNSTABLE_FAILURES, 2, 20)
    unreachable = _integer(raw, "unreachable_consecutive_failures", DEFAULT_UNREACHABLE_CONSECUTIVE_FAILURES, 3, 30)
    healthy = _integer(raw, "healthy_consecutive_successes", DEFAULT_HEALTHY_CONSECUTIVE_SUCCESSES, 3, 30)
    soft_max = _integer(raw, "soft_reboot_max_attempts", DEFAULT_SOFT_REBOOT_MAX_ATTEMPTS, 3, 10)
    soft_interval = _number(raw, "soft_reboot_interval_seconds", DEFAULT_SOFT_REBOOT_INTERVAL_SECONDS, 2.0, 30.0)
    soft_ack = _number(raw, "soft_reboot_ack_timeout_seconds", DEFAULT_SOFT_REBOOT_ACK_TIMEOUT_SECONDS, 0.5, 5.0)
    if soft_ack >= soft_interval:
        raise ConfigurationError("soft_reboot_ack_timeout_seconds must be < soft_reboot_interval_seconds")
    targets_raw = raw.get("targets")
    if not isinstance(targets_raw, list) or not targets_raw:
        raise ConfigurationError("targets must be a non-empty array")
    targets: List[Target] = []
    seen_codes = set()
    seen_ips = set()
    for row in targets_raw:
        if not isinstance(row, Mapping):
            raise ConfigurationError("each target must be an object")
        code = row.get("broadcast_code")
        ip = row.get("ip")
        handle = row.get("handle")
        interface = row.get("interface", "")
        if not isinstance(code, str) or not SAFE_CODE.fullmatch(code):
            raise ConfigurationError("target broadcast_code must be 15 alphanumeric characters")
        if not isinstance(ip, str) or not SAFE_IP.fullmatch(ip) or ip in seen_ips:
            raise ConfigurationError("target ip must be unique and valid")
        if isinstance(handle, bool) or not isinstance(handle, int) or handle < 0 or handle > 31:
            raise ConfigurationError("target handle must be between 0 and 31")
        if not isinstance(interface, str):
            raise ConfigurationError("target interface must be a string")
        if code in seen_codes:
            raise ConfigurationError("target broadcast_code must be unique")
        seen_codes.add(code)
        seen_ips.add(ip)
        targets.append(Target(code, ip, handle, interface))
    if window < interval:
        raise ConfigurationError("window_seconds must be >= probe_interval_seconds")
    return MonitorConfig(
        interval,
        window,
        unstable,
        unreachable,
        healthy,
        soft_max,
        soft_interval,
        soft_ack,
        tuple(targets),
    )


def _default_interface(ip: str) -> str:
    try:
        result = subprocess.run(
            ["ip", "route", "get", ip],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    tokens = result.stdout.split()
    if "dev" in tokens:
        index = tokens.index("dev")
        if index + 1 < len(tokens):
            return tokens[index + 1]
    return ""


def _run_probe(target: Target, arping_available: bool) -> Tuple[bool, str, Optional[float], str]:
    interface = target.interface or _default_interface(target.ip)

    def run_command(command: List[str], method: str) -> Tuple[bool, str, Optional[float], str]:
        started = time.monotonic()
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.5,
                check=False,
            )
            elapsed = (time.monotonic() - started) * 1000.0
            if result.returncode == 0:
                return True, method, elapsed, ""
            return False, method, elapsed, "returncode=%d" % result.returncode
        except (OSError, subprocess.SubprocessError) as exc:
            return False, method, None, str(exc)

    # Prefer ARP when available, but do not turn an unprivileged arping binary
    # into four permanent false alarms.  Fall back to ICMP on any ARP command
    # failure; the second probe still has to succeed before the sample is OK.
    if arping_available:
        command = ["arping", "-c", "1", "-w", "1"]
        if interface:
            command.extend(["-I", interface])
        command.append(target.ip)
        arp_success, arp_method, arp_rtt, arp_error = run_command(command, "ARP")
        if arp_success:
            return arp_success, arp_method, arp_rtt, arp_error

    command = ["ping", "-c", "1", "-W", "1"]
    if interface:
        command.extend(["-I", interface])
    command.append(target.ip)
    icmp_success, icmp_method, icmp_rtt, icmp_error = run_command(command, "ICMP")
    if icmp_success:
        return icmp_success, icmp_method, icmp_rtt, icmp_error
    if arping_available:
        return False, "ARP+ICMP", icmp_rtt, "arp=%s; icmp=%s" % (arp_error, icmp_error)
    return icmp_success, icmp_method, icmp_rtt, icmp_error


def _build_frame(config: MonitorConfig, states: Mapping[str, Dict[str, Any]], shared: bool, now: float) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "type": "LIVOX_NETWORK_HEALTH",
        "timestamp": now,
        "window_seconds": config.window_seconds,
        "probe_interval_seconds": config.probe_interval_seconds,
        "soft_reboot_max_attempts": config.soft_reboot_max_attempts,
        "soft_reboot_interval_seconds": config.soft_reboot_interval_seconds,
        "soft_reboot_ack_timeout_seconds": config.soft_reboot_ack_timeout_seconds,
        "shared_network_suspected": shared,
        "devices": list(states.values()),
    }


def run_ros(config: MonitorConfig) -> int:
    import rospy  # type: ignore
    from std_msgs.msg import String  # type: ignore

    rospy.init_node("livox_network_health_monitor")
    publisher = rospy.Publisher("livox/network_health", String, queue_size=1)
    windows = {
        target.broadcast_code: TargetWindow(
            config.window_seconds,
            config.unstable_failures,
            config.unreachable_consecutive_failures,
            config.healthy_consecutive_successes,
        )
        for target in config.targets
    }
    arping_available = shutil.which("arping") is not None
    if not arping_available and shutil.which("ping") is None:
        rospy.logerr("neither arping nor ping is available; network monitor disabled")
        return 2
    rospy.loginfo("Livox network health monitor: targets=%d window=%.1fs interval=%.1fs probe=%s soft-reboots=%d/%0.1fs ack=%0.1fs", len(config.targets), config.window_seconds, config.probe_interval_seconds, "ARP/ICMP" if arping_available else "ICMP", config.soft_reboot_max_attempts, config.soft_reboot_interval_seconds, config.soft_reboot_ack_timeout_seconds)
    rate = rospy.Rate(1.0 / config.probe_interval_seconds)
    while not rospy.is_shutdown():
        now = time.time()
        results: Dict[str, Dict[str, Any]] = {}
        with ThreadPoolExecutor(max_workers=len(config.targets)) as executor:
            probes = list(executor.map(lambda target: _run_probe(target, arping_available), config.targets))
        for target, (success, method, rtt_ms, error) in zip(config.targets, probes):
            policy = windows[target.broadcast_code].observe(success, now)
            results[target.broadcast_code] = {
                "broadcast_code": target.broadcast_code,
                "ip": target.ip,
                "handle": target.handle,
                "state": policy["state"],
                "success": success,
                "probe": method,
                "rtt_ms": rtt_ms,
                "error": error,
                **policy,
            }
        bad = sum(1 for row in results.values() if row["state"] in (STATE_UNSTABLE, STATE_UNREACHABLE))
        frame = _build_frame(config, results, bad >= 2, now)
        publisher.publish(String(data=json.dumps(frame, separators=(",", ":"), sort_keys=True)))
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--validate-config", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
    except ConfigurationError as exc:
        if args.validate_config:
            parser.error(str(exc))
            return 2
        import rospy  # type: ignore

        rospy.init_node("livox_network_health_monitor")
        rospy.logwarn(
            "Livox network health monitor is idle: %s; create the site "
            "config before enabling automatic network recovery",
            exc,
        )
        rospy.spin()
        return 0
    if args.validate_config:
        print("Configuration valid: targets=%d window=%.1fs interval=%.1fs unstable=%d unreachable=%d healthy=%d soft_reboots=%d interval=%.1fs ack=%.1fs" % (len(config.targets), config.window_seconds, config.probe_interval_seconds, config.unstable_failures, config.unreachable_consecutive_failures, config.healthy_consecutive_successes, config.soft_reboot_max_attempts, config.soft_reboot_interval_seconds, config.soft_reboot_ack_timeout_seconds))
        return 0
    return run_ros(config)


if __name__ == "__main__":
    raise SystemExit(main())
