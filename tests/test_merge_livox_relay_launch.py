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


def _launch(newline="\n"):
    lines = [
        "<launch>",
        '    <arg name="monitor" default="true"/>   <!-- site value -->',
        '    <arg name="health_log" default="true"/>',
        '    <param name="auto_recover" value="$(arg health_log)"/>',
        '    <remap from="/livox/lidar_TESTLIDAR000001" to="/site/lidar_1"/>',
        '    <node pkg="tf" type="static_transform_publisher" name="site_tf" args="0 0 0 0 0 0 /base /lidar 5"/>',
        '    <node name="livox_driver" pkg="livox_ros_driver"',
        '          type="livox_ros_driver_node" required="true" output="screen"/>',
        "</launch>",
        "",
    ]
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
            root = ET.parse(output).getroot()
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


if __name__ == "__main__":
    unittest.main()
