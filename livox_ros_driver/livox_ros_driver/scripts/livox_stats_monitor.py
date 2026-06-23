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
import sys
import rospy
from std_msgs.msg import String


def cb(msg):
    # ESC[2J = clear screen, ESC[H = cursor to home (top-left)
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(msg.data)
    sys.stdout.write("\n(updated: %.1f)\n" % rospy.get_time())
    sys.stdout.flush()


def main():
    rospy.init_node("livox_stats_monitor", anonymous=True)
    rospy.Subscriber("livox/lidar_stats", String, cb, queue_size=1)
    sys.stdout.write("Waiting for /livox/lidar_stats ...\n")
    sys.stdout.flush()
    rospy.spin()


if __name__ == "__main__":
    main()
