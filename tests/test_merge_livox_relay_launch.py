#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = (
    ROOT
    / "livox_ros_driver/livox_ros_driver/scripts/merge_livox_relay_launch.py"
)
SPEC = importlib.util.spec_from_file_location("merge_livox_relay_launch", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
merger = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(merger)


def _launch(
    newline="\n", monitor_node=True, monitor_layout=None, monitor_args=None
):
    lines = [
        "<launch>",
        '    <arg name="monitor" default="true"/>   <!-- site value -->',
    ]
    if monitor_layout is not None:
        lines.extend(
            [
                "    <!-- %s -->" % merger.MONITOR_MARKER,
                '    <arg name="monitor_layout" default="%s"/>'
                % monitor_layout,
            ]
        )
    lines.extend(
        [
            '    <arg name="health_log" default="true"/>',
            '    <param name="auto_recover" value="$(arg health_log)"/>',
            '    <remap from="/livox/lidar_TESTLIDAR000001" to="/site/lidar_1"/>',
            '    <node pkg="tf" type="static_transform_publisher" name="site_tf" args="0 0 0 0 0 0 /base /lidar 5"/>',
            '    <node name="livox_driver" pkg="livox_ros_driver"',
            '          type="livox_ros_driver_node" required="true" output="screen"/>',
        ]
    )
    if monitor_node:
        args = ""
        if monitor_args is not None:
            args = ' args="%s"' % monitor_args
        lines.append(
            '    <node if="$(arg monitor)" name="livox_stats_monitor" '
            'pkg="livox_ros_driver" type="livox_stats_monitor.py"%s/>' % args
        )
    lines.extend(["</launch>", ""])
    return newline.join(lines)


class StructuralLaunchMergeTests(unittest.TestCase):
    def test_preserves_site_content_and_injects_one_safe_integration(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            source = _launch()
            local.write_text(source, encoding="utf-8", newline="")
            merger.merge(local, output)
            merged = output.read_text(encoding="utf-8")
            self.assertIn('default="true"/>   <!-- site value -->', merged)
            self.assertIn('to="/site/lidar_1"', merged)
            self.assertEqual(merged.count(merger.MARKER), 1)
            self.assertEqual(merged.count(merger.RELAY_CHILD), 1)
            self.assertEqual(merged.count('name="relay_power_cycle_enable"'), 1)
            self.assertEqual(merged.count(merger.MONITOR_MARKER), 1)
            self.assertEqual(merged.count('name="monitor_layout"'), 1)
            root = ET.parse(output).getroot()
            layout = next(
                node
                for node in root.findall("arg")
                if node.get("name") == "monitor_layout"
            )
            self.assertEqual(layout.get("default"), "compact")
            monitor = next(
                node
                for node in root.findall("node")
                if node.get("name") == "livox_stats_monitor"
            )
            self.assertEqual(monitor.get("args"), merger.MONITOR_LAYOUT_VALUE)
            children = list(root)
            include = next(node for node in children if node.tag == "include")
            driver = next(
                node
                for node in children
                if node.tag == "node" and node.get("name") == "livox_driver"
            )
            self.assertLess(children.index(include), children.index(driver))

    def test_preserves_utf8_bom_crlf_and_file_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            local.write_bytes(b"\xef\xbb\xbf" + _launch("\r\n").encode("utf-8"))
            os.chmod(local, 0o640)
            merger.merge(local, output)
            raw = output.read_bytes()
            self.assertTrue(raw.startswith(b"\xef\xbb\xbf"))
            self.assertIn(b"\r\n", raw)
            if os.name != "nt":
                self.assertEqual(output.stat().st_mode & 0o777, 0o640)

    def test_preserves_existing_monitor_args_and_appends_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            local.write_text(
                _launch(monitor_args="--site-label pit-4"), encoding="utf-8"
            )
            merger.merge(local, output)
            root = ET.parse(output).getroot()
            monitor = next(
                node
                for node in root.findall("node")
                if node.get("name") == "livox_stats_monitor"
            )
            self.assertEqual(
                monitor.get("args"),
                "--site-label pit-4 " + merger.MONITOR_LAYOUT_VALUE,
            )

    def test_preserves_existing_valid_monitor_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            local.write_text(
                _launch(
                    monitor_layout="full",
                    monitor_args=merger.MONITOR_LAYOUT_VALUE,
                ),
                encoding="utf-8",
            )
            merger.merge(local, output)
            merged = output.read_text(encoding="utf-8")
            self.assertEqual(merged.count(merger.MONITOR_MARKER), 1)
            root = ET.parse(output).getroot()
            layout = next(
                node
                for node in root.findall("arg")
                if node.get("name") == "monitor_layout"
            )
            self.assertEqual(layout.get("default"), "full")

    def test_headless_launch_does_not_gain_monitor_arguments(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            local.write_text(_launch(monitor_node=False), encoding="utf-8")
            merger.merge(local, output)
            merged = output.read_text(encoding="utf-8")
            self.assertNotIn(merger.MONITOR_MARKER, merged)
            self.assertNotIn('name="monitor_layout"', merged)

    def test_refuses_existing_or_ambiguous_power_cycle_integration(self):
        cases = {
            "marker": _launch().replace(
                "</launch>", "<!-- %s -->\n</launch>" % merger.MARKER
            ),
            "inline-manager": _launch().replace(
                "</launch>",
                '<node name="livox_power_cycle_manager" '
                'type="livox_power_cycle_manager.py"/>\n</launch>',
            ),
            "existing-arg": _launch().replace(
                "</launch>",
                '<arg name="relay_power_cycle_enable" default="true"/>\n</launch>',
            ),
        }
        for name, source in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                local = Path(tmp) / "site.launch"
                output = Path(tmp) / "candidate.launch"
                local.write_text(source, encoding="utf-8")
                with self.assertRaises(merger.MergeError):
                    merger.merge(local, output)

    def test_refuses_multiline_root_arg_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            local = Path(tmp) / "site.launch"
            output = Path(tmp) / "candidate.launch"
            source = _launch().replace(
                '<arg name="health_log" default="true"/>',
                '<arg name="health_log"\n         default="true"/>',
            )
            local.write_text(source, encoding="utf-8")
            with self.assertRaisesRegex(merger.MergeError, "layout is ambiguous"):
                merger.merge(local, output)

    def test_refuses_ambiguous_or_invalid_monitor_layout(self):
        cases = {
            "invalid-default": _launch(
                monitor_layout="giant",
                monitor_args=merger.MONITOR_LAYOUT_VALUE,
            ),
            "duplicate-arg": _launch(
                monitor_layout="compact",
                monitor_args=merger.MONITOR_LAYOUT_VALUE,
            ).replace(
                '<arg name="health_log" default="true"/>',
                '<arg name="monitor_layout" default="full"/>\n'
                '    <arg name="health_log" default="true"/>',
            ),
            "wrong-node-args": _launch(
                monitor_layout="compact", monitor_args="--layout full"
            ),
        }
        for name, source in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                local = Path(tmp) / "site.launch"
                output = Path(tmp) / "candidate.launch"
                local.write_text(source, encoding="utf-8")
                with self.assertRaises(merger.MergeError):
                    merger.merge(local, output)


if __name__ == "__main__":
    unittest.main()
