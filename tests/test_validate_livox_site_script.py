#!/usr/bin/env python3
"""Safety and path contract for the operator-facing site validator wrapper."""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "validate_livox_site.sh"


class ValidateLivoxSiteScriptTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCRIPT.read_text(encoding="utf-8")

    def test_uses_the_real_nested_source_paths(self):
        self.assertIn(
            'MANAGER="${PACKAGE_DIR}/livox_ros_driver/scripts/'
            'livox_power_cycle_manager.py"',
            self.source,
        )
        self.assertIn(
            'SITE_VALIDATOR="${PACKAGE_DIR}/livox_ros_driver/scripts/'
            'validate_livox_power_cycle_site.py"',
            self.source,
        )
        self.assertIn(
            'EXAMPLE_CONFIG="${PACKAGE_DIR}/config/'
            'livox_power_cycle.example.json"',
            self.source,
        )
        self.assertNotIn("livox_power_cycle_example.json", self.source)

    def test_requires_external_runtime_config_and_rejects_repo_copy(self):
        self.assertIn(
            'RELAY_CONFIG="${HOME}/.config/livox/power_cycle.json"',
            self.source,
        )
        self.assertIn(
            'MISPLACED_CONFIG="${PACKAGE_DIR}/config/'
            'livox_power_cycle.json"',
            self.source,
        )
        self.assertIn("production relay config is misplaced", self.source)
        self.assertIn("production relay config is missing", self.source)

    def test_network_query_is_explicit_read_only_mode_after_offline_checks(self):
        config_check = self.source.index('--validate-config')
        identity_check = self.source.index('python3 "${SITE_VALIDATOR}"')
        relay_check = self.source.index('--check-relays', identity_check)
        self.assertLess(config_check, identity_check)
        self.assertLess(identity_check, relay_check)
        self.assertIn("read-only; no output state is changed", self.source)

    def test_wrapper_never_changes_service_or_relay_output(self):
        self.assertNotIn("systemctl", self.source)
        self.assertNotIn("sudo", self.source)
        self.assertNotIn("--repair-obligations", self.source)
        self.assertNotIn("--mode armed", self.source)


if __name__ == "__main__":
    unittest.main()
