#!/usr/bin/env python3
#
# Livox LiDAR stats dashboard.
#
# Run in a SEPARATE terminal from the driver. It subscribes to the
# /livox/lidar_stats topic published by livox_ros_driver and redraws the
# table in place (clears the screen each update), so it stays pinned and
# is completely isolated from the driver's scrolling log output.
#
#   python3 livox_stats_monitor.py
#   # or, after a catkin build with the install rule:
#   rosrun livox_ros_driver livox_stats_monitor.py
#
import json
import sys
import threading
import time
import rospy
from std_msgs.msg import String


_lock = threading.Lock()
_render_lock = threading.Lock()
_stats_text = "Waiting for /livox/lidar_stats ...\n"
_power_status = {}


def _render():
    with _render_lock:
        with _lock:
            stats_text = _stats_text
            rows = list(_power_status.values())
        # ESC[2J = clear screen, ESC[H = cursor to home (top-left)
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(stats_text)
        if rows:
            sys.stdout.write("Power-cycle manager (shared power groups):\n")
            for row in sorted(
                rows,
                key=lambda item: (
                    item.get("power_group", ""),
                    item.get("broadcast_code", ""),
                ),
            ):
                stamp = row.get("timestamp", 0)
                try:
                    stamp_text = time.strftime(
                        "%H:%M:%S", time.localtime(float(stamp))
                    )
                except (TypeError, ValueError, OSError):
                    stamp_text = "--:--:--"
                group = row.get("power_group") or "manager"
                trigger = row.get("broadcast_code") or "-"
                members = row.get("members")
                member_count = len(members) if isinstance(members, list) else 0
                state = row.get("state", "?")
                detail = row.get("detail", "")
                if (
                    state in {"MANAGER_HEARTBEAT", "MANAGER_READY"}
                    and isinstance(stamp, (int, float))
                    and time.time() - float(stamp) > 30
                ):
                    state = "MANAGER_STALE"
                    detail = "no manager heartbeat for %.0fs; %s" % (
                        time.time() - float(stamp),
                        detail,
                    )
                sys.stdout.write(
                    "  %s group=%s members=%d trigger=%-15s %-24s %s\n"
                    % (
                        stamp_text,
                        group,
                        member_count,
                        trigger,
                        state,
                        detail,
                    )
                )
        sys.stdout.write("\n(updated: %.1f)\n" % rospy.get_time())
        sys.stdout.flush()


def cb(msg):
    global _stats_text
    with _lock:
        _stats_text = msg.data
    _render()


def power_cb(msg):
    try:
        payload = json.loads(msg.data)
        if payload.get("type") != "POWER_CYCLE_STATUS":
            return
    except (AttributeError, TypeError, ValueError):
        return
    power_group = payload.get("power_group")
    if power_group:
        key = "group:%s" % power_group
    else:
        key = payload.get("broadcast_code") or "__manager__"
    with _lock:
        _power_status[key] = payload
    _render()


def main():
    rospy.init_node("livox_stats_monitor", anonymous=True)
    rospy.Subscriber("livox/lidar_stats", String, cb, queue_size=1)
    rospy.Subscriber("livox/power_cycle_status", String, power_cb, queue_size=32)
    rospy.Subscriber(
        "livox/power_cycle_heartbeat", String, power_cb, queue_size=8
    )
    sys.stdout.write("Waiting for /livox/lidar_stats ...\n")
    sys.stdout.flush()
    rospy.spin()


if __name__ == "__main__":
    main()
