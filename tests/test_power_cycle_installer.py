#!/usr/bin/env python3
"""Static safety contract for the integrated relay systemd installer.

These tests intentionally do not invoke systemctl or touch /etc.  They pin the
ordering and fail-closed properties of the deployment shell script so the
contract can be checked on developer machines and in unprivileged CI.
"""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INSTALLER_PATH = ROOT / "install_livox_power_cycle_service.sh"
DROPIN_TEMPLATE_PATH = (
    ROOT / "systemd" / "livox-ros-driver-power-cycle.conf.in"
)
LEGACY_TEMPLATE_PATH = (
    ROOT / "systemd" / "livox-power-cycle-manager.service.in"
)


class IntegratedInstallerSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.installer = INSTALLER_PATH.read_text(encoding="utf-8")
        cls.dropin = DROPIN_TEMPLATE_PATH.read_text(encoding="utf-8")

    def test_dropin_has_config_independent_start_and_stop_repairs(self):
        expected_command = (
            "/usr/bin/python3 @MANAGER_RUNTIME@ --state-db @STATE_DB@ "
            "--repair-obligations"
        )
        self.assertEqual(
            [
                line
                for line in self.dropin.splitlines()
                if line.startswith("ExecStartPre=")
            ],
            ["ExecStartPre=" + expected_command],
        )
        self.assertEqual(
            [
                line
                for line in self.dropin.splitlines()
                if line.startswith("ExecStopPost=")
            ],
            ["ExecStopPost=" + expected_command],
        )
        self.assertIn("TimeoutStartSec=600", self.dropin)
        self.assertIn("TimeoutStopSec=300", self.dropin)
        self.assertIn("SendSIGKILL=yes", self.dropin)
        self.assertEqual(
            self.dropin.count("LIVOX_POWER_CYCLE_SAFETY_DROPIN_V2"), 1
        )
        self.assertNotIn("--config", self.dropin)
        self.assertNotIn("rosrun", self.dropin)
        self.assertNotIn("/bin/bash", self.dropin)

        placeholders = set(re.findall(r"@[A-Z_]+@", self.dropin))
        for placeholder in placeholders:
            self.assertIn(
                "s|%s|" % placeholder,
                self.installer,
                "installer must render %s" % placeholder,
            )

    def test_installer_only_installs_driver_dropin_and_never_starts_driver(self):
        self.assertFalse(
            LEGACY_TEMPLATE_PATH.exists(),
            "the independently runnable legacy unit template must stay removed",
        )
        self.assertIn(
            'DROPIN_PATH="${DROPIN_DIR}/${DROPIN_NAME}"', self.installer
        )
        self.assertIn(
            'sudo install -m 644 "${rendered}" "${DROPIN_PATH}"',
            self.installer,
        )
        self.assertNotIn("enable --now", self.installer)
        self.assertNotIn("livox-power-cycle-manager.service.in", self.installer)
        self.assertNotRegex(
            self.installer,
            r"sudo\s+systemctl\s+(?:start|restart|enable)\s+"
            r"[\"']?\$\{DRIVER_UNIT_NAME\}",
        )
        self.assertNotRegex(
            self.installer,
            r"sudo\s+install[^\n]*\$\{LEGACY_UNIT_PATH\}",
        )
        self.assertIn("--property=Restart --value", self.installer)
        self.assertIn('== "always"', self.installer)
        self.assertIn("--property=KillMode --value", self.installer)
        self.assertIn('== "control-group"', self.installer)
        self.assertIn("--property=Type --value", self.installer)
        self.assertIn('== "simple"', self.installer)
        self.assertIn("--property=RemainAfterExit --value", self.installer)
        self.assertIn('== "no"', self.installer)
        self.assertIn("--property=ExecStartPre --value", self.installer)
        self.assertIn("--property=ExecStopPost --value", self.installer)
        self.assertIn("EXPECTED_REPAIR_COMMAND=", self.installer)
        self.assertIn("--property=TimeoutStartUSec --value", self.installer)
        self.assertIn("--property=TimeoutStopUSec --value", self.installer)
        self.assertIn("--property=SendSIGKILL --value", self.installer)
        self.assertIn("--property=Environment --value", self.installer)
        self.assertIn("DRIVER_HOME_ENTRIES", self.installer)
        self.assertIn('CONFIG_DIR="${HOME}/.config/livox"', self.installer)
        self.assertNotIn("XDG_CONFIG_HOME", self.installer)
        self.assertNotIn("LIVOX_POWER_CYCLE_CONFIG", self.installer)

    def test_dropin_precedes_legacy_stop_repair_disable_delete(self):
        install_dropin = self.installer.index(
            'sudo install -m 644 "${rendered}" "${DROPIN_PATH}"'
        )
        dropin_confirm = self.installer.index(
            'DRIVER_DROPIN_PATHS="$(systemctl show --property=DropInPaths'
        )
        stop = self.installer.index(
            'sudo systemctl stop "${LEGACY_UNIT_NAME}"'
        )
        repair = self.installer.index(
            'log "独立执行一次与 JSON/ROS 无关的 must-be-ON 修复。"'
        )
        confirm_inactive = self.installer.index(
            '[[ "${LEGACY_ACTIVE_STATE}" == "inactive" ]]'
        )
        disable = self.installer.index(
            'sudo systemctl disable "${LEGACY_UNIT_NAME}"'
        )
        delete = self.installer.index(
            'sudo rm -f -- "${LEGACY_UNIT_PATH}"'
        )
        self.assertLess(install_dropin, dropin_confirm)
        self.assertLess(dropin_confirm, stop)
        self.assertLess(stop, repair)
        self.assertLess(repair, confirm_inactive)
        self.assertLess(confirm_inactive, disable)
        self.assertLess(disable, delete)
        self.assertIn(
            '"${LEGACY_ENABLE_STATE}" == "not-found"', self.installer
        )

    def test_failed_migration_restores_only_a_previously_active_legacy_unit(self):
        self.assertIn("trap cleanup EXIT", self.installer)
        cleanup = self.installer[
            self.installer.index("cleanup() {") : self.installer.index(
                "unit_load_state() {"
            )
        ]
        self.assertIn("exit_status != 0", cleanup)
        self.assertIn("LEGACY_WAS_ACTIVE", cleanup)
        self.assertIn("LEGACY_STOPPED_BY_INSTALLER", cleanup)
        self.assertIn("!LEGACY_MIGRATION_COMPLETE", cleanup)
        self.assertIn(
            'sudo systemctl start "${LEGACY_UNIT_NAME}"', cleanup
        )

        migration = self.installer[
            self.installer.index("LEGACY_INITIAL_ACTIVE_STATE=") :
        ]
        self.assertRegex(
            migration,
            r"active\|activating\|reloading\|deactivating\) "
            r"LEGACY_WAS_ACTIVE=1",
        )
        self.assertIn("LEGACY_STOPPED_BY_INSTALLER=1", migration)
        self.assertLess(
            migration.index("LEGACY_STOPPED_BY_INSTALLER=1"),
            migration.index("LEGACY_MIGRATION_COMPLETE=1"),
        )

    def test_uninstall_requires_inactive_driver_and_successful_repair(self):
        uninstall = self.installer[
            self.installer.index('if [[ "${1:-}" == "--uninstall" ]]') :
            self.installer.index('\nfi\n[[ $# -eq 0 ]]')
        ]
        state_check = uninstall.index(
            'DRIVER_ACTIVE_STATE="$(unit_active_state "${DRIVER_UNIT_NAME}")"'
        )
        inactive_gate = uninstall.index("inactive|failed)")
        repair = uninstall.index("repair_obligations ||")
        move = uninstall.index(
            'sudo mv -- "${DROPIN_PATH}" "${DROPIN_ROLLBACK_PATH}"'
        )
        reload_systemd = uninstall.index("sudo systemctl daemon-reload")
        remove_backup = uninstall.index(
            'sudo rm -f -- "${DROPIN_ROLLBACK_PATH}"'
        )
        self.assertLess(state_check, inactive_gate)
        self.assertLess(inactive_gate, repair)
        self.assertLess(repair, move)
        self.assertLess(move, reload_systemd)
        self.assertLess(reload_systemd, remove_backup)
        self.assertIn("drop-in 已保留", uninstall)
        self.assertIn("DROPIN_ROLLBACK_PATH", self.installer)
        self.assertIn("卸载未完成，正在原子恢复", self.installer)
        self.assertIn("配置和 SQLite 审计记录均保留", uninstall)
        self.assertNotIn('rm -f -- "${CONFIG_FILE}"', uninstall)
        self.assertNotIn('rm -f -- "${STATE_DB}"', uninstall)


if __name__ == "__main__":
    unittest.main()
