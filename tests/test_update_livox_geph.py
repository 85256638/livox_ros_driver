#!/usr/bin/env python3
"""Behavioral tests for update_livox_geph.sh safety transactions."""

import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UPDATER = ROOT / "update_livox_geph.sh"
SITE_JSON = "livox_ros_driver/config/livox_lidar_config_multi.json"
SITE_LAUNCH = "livox_ros_driver/launch/livox_lidar_multi.launch"
RELAY_CHILD = "livox_ros_driver/launch/livox_power_cycle.launch"
MARKER = "LIVOX_RELAY_LAUNCH_INTEGRATION"
MONITOR_MARKER = "LIVOX_MONITOR_LAYOUT_V1"
MONITOR_LAYOUT_VALUE = "--layout $(arg monitor_layout)"
DROPIN = (
    "/etc/systemd/system/livox-ros-driver.service.d/"
    "20-livox-power-cycle-safety.conf"
)


def _function(source, name):
    match = re.search(
        r"(?ms)^" + re.escape(name) + r"\(\) \{\n.*?^\}\n",
        source,
    )
    if match is None:
        raise AssertionError("missing Bash function %s" % name)
    return match.group(0)


def _bash_path(path):
    resolved = str(Path(path).resolve())
    if os.name == "nt":
        drive, tail = os.path.splitdrive(resolved)
        return "/%s%s" % (drive[0].lower(), tail.replace("\\", "/"))
    return resolved


def _find_bash():
    if os.name == "nt":
        candidates = [
            Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
            / "Git"
            / "bin"
            / "bash.exe",
            Path(os.environ.get("ProgramFiles", r"C:\Program Files"))
            / "Git"
            / "usr"
            / "bin"
            / "bash.exe",
        ]
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
    return shutil.which("bash")


def _run(command, cwd=None, check=True, **kwargs):
    return subprocess.run(
        command,
        cwd=cwd,
        check=check,
        text=True,
        encoding="utf-8",
        errors="replace",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **kwargs,
    )


def _launch(
    monitor="true",
    relay=False,
    marker=True,
    valid_xml=True,
    monitor_node=True,
    monitor_layout=None,
    monitor_layout_default="compact",
    monitor_args=None,
):
    if monitor_layout is None:
        monitor_layout = relay and monitor_node
    lines = [
        "<launch>",
        '  <arg name="monitor" default="%s"/>' % monitor,
    ]
    if monitor_layout:
        lines.extend(
            [
                "  <!-- %s -->" % MONITOR_MARKER,
                '  <arg name="monitor_layout" default="%s"/>'
                % monitor_layout_default,
            ]
        )
    lines.extend(
        [
            '  <param name="unchanged_1" value="1"/>',
            '  <param name="unchanged_2" value="2"/>',
            '  <param name="unchanged_3" value="3"/>',
            '  <param name="unchanged_4" value="4"/>',
        ]
    )
    if relay:
        lines.extend(
            [
                '  <arg name="relay_power_cycle_enable" default="false"/>',
            ]
        )
        if marker:
            lines.append("  <!-- %s -->" % MARKER)
        lines.extend(
            [
                '  <include file="$(find livox_ros_driver)/launch/livox_power_cycle.launch">',
                '    <arg name="enable" value="$(arg relay_power_cycle_enable)"/>',
                '  </include>',
            ]
        )
    lines.append(
        '  <node name="livox_driver" pkg="livox_ros_driver" type="livox_ros_driver_node"/>'
    )
    if monitor_node:
        if monitor_args is None and monitor_layout:
            monitor_args = MONITOR_LAYOUT_VALUE
        args_attribute = (
            ' args="%s"' % monitor_args if monitor_args is not None else ""
        )
        lines.append(
            '  <node if="$(arg monitor)" name="livox_stats_monitor" '
            'pkg="livox_ros_driver" type="livox_stats_monitor.py"%s/>'
            % args_attribute
        )
    if valid_xml:
        lines.append("</launch>")
    return ("\n".join(lines) + "\n").encode("utf-8")


class UpdaterSiteTransactionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = UPDATER.read_text(encoding="utf-8")
        cls.bash = _find_bash()
        cls.restore_functions = "\n".join(
            _function(cls.source, name)
            for name in (
                "log",
                "site_path_is_tracked_regular",
                "drop_site_config_stash_if_top",
                "clear_site_config_pending",
                "atomic_copy_file",
                "launch_has_relay_marker",
                "structural_merge_relay_launch",
                "validate_relay_launch_integration",
                "restore_site_config",
            )
        )

    def setUp(self):
        if self.bash is None:
            self.skipTest("Bash is unavailable")

    def _git(self, repo, *args):
        return _run(["git", "-C", str(repo)] + list(args))

    def _prepare_transaction(
        self,
        root,
        upstream_launch,
        local_launch=None,
        local_json=b'{\r\n  "site": "pit-4"\r\n}',
    ):
        repo = root / "driver"
        backup = root / "backup"
        pending = root / "pending"
        json_path = repo / SITE_JSON
        launch_path = repo / SITE_LAUNCH
        json_path.parent.mkdir(parents=True)
        launch_path.parent.mkdir(parents=True)
        base_json = b'{\n  "site": "base"\n}\n'
        base_launch = _launch()
        json_path.write_bytes(base_json)
        launch_path.write_bytes(base_launch)
        merger = repo / "livox_ros_driver/livox_ros_driver/scripts/merge_livox_relay_launch.py"
        merger.parent.mkdir(parents=True, exist_ok=True)
        merger.write_bytes(
            (
                ROOT
                / "livox_ros_driver/livox_ros_driver/scripts/merge_livox_relay_launch.py"
            ).read_bytes()
        )
        self._git(repo.parent, "init", str(repo))
        self._git(repo, "config", "user.name", "Updater Test")
        self._git(repo, "config", "user.email", "updater@example.invalid")
        self._git(repo, "config", "core.autocrlf", "false")
        self._git(repo, "add", SITE_JSON, SITE_LAUNCH)
        self._git(repo, "commit", "-m", "base")

        if local_launch is None:
            local_launch = _launch(monitor="false")
        for area, payloads in (
            ("base", ((SITE_JSON, base_json), (SITE_LAUNCH, base_launch))),
            ("original", ((SITE_JSON, local_json), (SITE_LAUNCH, local_launch))),
        ):
            for relative, payload in payloads:
                target = backup / area / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(payload)

        json_path.write_bytes(b'{\n  "site": "upstream"\n}\n')
        launch_path.write_bytes(upstream_launch)
        pending.write_text(_bash_path(backup), encoding="utf-8")
        return repo, backup, pending, local_json, local_launch

    @staticmethod
    def _fake_python_function():
        return r'''
python_validator() {
  if [[ "${1:-}" == *merge_livox_relay_launch.py ]]; then
    "${REAL_PYTHON}" "$@"
    return
  fi
  cat >/dev/null
  local path="$2"
  local marker="${3:-}"
  local count
  [[ "$(grep -oF '<launch>' "${path}" | wc -l)" == "1" ]] || return 1
  [[ "$(grep -oF '</launch>' "${path}" | wc -l)" == "1" ]] || return 1
  if [[ -n "${marker}" ]]; then
    count="$(grep -oF "${marker}" "${path}" | wc -l)"
    [[ "${count}" == "1" ]] || return 1
    [[ "$(grep -oF 'name="relay_power_cycle_enable"' "${path}" | wc -l)" == "1" ]] || return 1
    [[ "$(grep -oF 'livox_power_cycle.launch' "${path}" | wc -l)" == "1" ]] || return 1
  fi
}
'''

    def _run_restore(self, root, repo, backup, pending):
        sentinel = root / "continued"
        harness = root / "restore-test.sh"
        q = lambda value: shlex.quote(_bash_path(value))
        harness.write_text(
            "#!/usr/bin/env bash\n"
            "set -Eeuo pipefail\n"
            "export PATH=/usr/bin:/mingw64/bin:$PATH\n"
            + "REAL_PYTHON=%s\n" % q(sys.executable)
            + self._fake_python_function()
            + "PYTHON_EXECUTABLE=python_validator\n"
            + "DRIVER_DIR=%s\n" % q(repo)
            + "SITE_CONFIG_BACKUP_DIR=%s\n" % q(backup)
            + "SITE_CONFIG_PENDING_FILE=%s\n" % q(pending)
            + "SITE_CONFIG_PREPARED=1\n"
            + "SITE_CONFIG_STASH_SHA=\"\"\n"
            + "SITE_JSON_PATH=%s\n" % shlex.quote(SITE_JSON)
            + "SITE_LAUNCH_PATH=%s\n" % shlex.quote(SITE_LAUNCH)
            + "SITE_LAUNCH_MERGER_PATH=%s\n"
            % shlex.quote(
                "livox_ros_driver/livox_ros_driver/scripts/merge_livox_relay_launch.py"
            )
            + "SITE_LAUNCH_MARKER=%s\n" % shlex.quote(MARKER)
            + "SITE_CONFIG_CHANGED_PATHS=(\"$SITE_JSON_PATH\" \"$SITE_LAUNCH_PATH\")\n"
            + self.restore_functions
            + "\nif restore_site_config 1; then\n"
            + "  printf success >%s\n" % q(sentinel)
            + "  exit 0\n"
            + "fi\n"
            + "exit 42\n",
            encoding="utf-8",
            newline="\n",
        )
        result = _run(
            [self.bash, "--noprofile", "--norc", _bash_path(harness)],
            check=False,
        )
        return result, sentinel

    def test_json_is_byte_exact_and_launch_is_cleanly_three_way_merged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo, backup, pending, local_json, _ = self._prepare_transaction(
                root, _launch(relay=True)
            )
            result, sentinel = self._run_restore(root, repo, backup, pending)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(sentinel.is_file())
            self.assertEqual((repo / SITE_JSON).read_bytes(), local_json)
            merged = (repo / SITE_LAUNCH).read_text(encoding="utf-8")
            self.assertIn('name="monitor" default="false"', merged)
            self.assertEqual(merged.count(MARKER), 1)
            self.assertEqual(merged.count('name="relay_power_cycle_enable"'), 1)
            self.assertEqual(merged.count(MONITOR_MARKER), 1)
            self.assertEqual(merged.count('name="monitor_layout"'), 1)
            self.assertIn(MONITOR_LAYOUT_VALUE, merged)
            self.assertEqual(
                merged.count('value="$(arg relay_power_cycle_enable)"'), 1
            )
            self.assertEqual(merged.count("livox_power_cycle.launch"), 1)
            self.assertFalse(pending.exists())

    def test_merge_conflict_uses_strict_structural_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo, backup, pending, local_json, local_launch = (
                self._prepare_transaction(
                    root,
                    _launch(monitor="upstream", relay=True),
                    local_launch=_launch(monitor="local"),
                )
            )
            result, sentinel = self._run_restore(root, repo, backup, pending)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(sentinel.is_file())
            self.assertEqual((repo / SITE_JSON).read_bytes(), local_json)
            merged = (repo / SITE_LAUNCH).read_text(encoding="utf-8")
            self.assertIn('name="monitor" default="local"', merged)
            self.assertEqual(merged.count(MARKER), 1)
            self.assertEqual(merged.count("livox_power_cycle.launch"), 1)
            self.assertEqual(merged.count(MONITOR_MARKER), 1)
            self.assertEqual(merged.count('name="monitor_layout"'), 1)
            self.assertIn(MONITOR_LAYOUT_VALUE, merged)
            self.assertNotIn("<<<<<<<", merged)
            candidate = backup / "candidate" / SITE_LAUNCH
            self.assertTrue(candidate.is_file())
            self.assertIn("严格结构化后备合并", result.stderr)

    def test_structural_fallback_rejects_existing_inline_manager(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            unsafe = _launch(monitor="local").replace(
                b"</launch>",
                b'  <node name="livox_power_cycle_manager" pkg="livox_ros_driver" '
                b'type="livox_power_cycle_manager.py"/>\n</launch>',
            )
            repo, backup, pending, local_json, _ = self._prepare_transaction(
                root,
                _launch(monitor="upstream", relay=True),
                local_launch=unsafe,
            )
            result, sentinel = self._run_restore(root, repo, backup, pending)
            self.assertEqual(result.returncode, 42, result.stderr)
            self.assertFalse(sentinel.exists())
            self.assertEqual((repo / SITE_JSON).read_bytes(), local_json)
            self.assertEqual((repo / SITE_LAUNCH).read_bytes(), unsafe)
            self.assertIn("already contains a relay manager node", result.stderr)

    def test_invalid_upstream_marker_or_xml_restores_original_and_stops(self):
        cases = {
            "missing-marker": _launch(relay=True, marker=False),
            "invalid-xml": _launch(relay=True, valid_xml=False),
        }
        for name, upstream in cases.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                repo, backup, pending, _, local_launch = (
                    self._prepare_transaction(root, upstream)
                )
                result, sentinel = self._run_restore(
                    root, repo, backup, pending
                )
                self.assertEqual(result.returncode, 42, result.stderr)
                self.assertFalse(sentinel.exists())
                self.assertEqual(
                    (repo / SITE_LAUNCH).read_bytes(), local_launch
                )
                self.assertIn("禁止编译和重启服务", result.stderr)


class UpdaterBranchTrackingTests(unittest.TestCase):
    TARGET_BRANCH = "network-relay-added"
    LEGACY_BRANCH = "legacy-sdk"

    @classmethod
    def setUpClass(cls):
        cls.source = UPDATER.read_text(encoding="utf-8")
        cls.bash = _find_bash()
        cls.branch_functions = "\n".join(
            _function(cls.source, name)
            for name in (
                "log",
                "die",
                "short_sha",
                "check_clean_checkout",
                "ensure_branch_upstream",
                "update_checkout",
            )
        )

    def setUp(self):
        if self.bash is None:
            self.skipTest("Bash is unavailable")

    @staticmethod
    def _git(repo, *args, check=True):
        return _run(["git", "-C", str(repo)] + list(args), check=check)

    def _prepare_single_branch_clone(self, root):
        origin = root / "origin.git"
        seed = root / "seed"
        checkout = root / "checkout"
        _run(["git", "init", "--bare", str(origin)])
        _run(["git", "init", str(seed)])
        self._git(seed, "config", "user.name", "Updater Test")
        self._git(seed, "config", "user.email", "updater@example.invalid")
        self._git(seed, "config", "core.autocrlf", "false")
        (seed / "version.txt").write_text("legacy\n", encoding="utf-8")
        self._git(seed, "add", "version.txt")
        self._git(seed, "commit", "-m", "legacy")
        self._git(seed, "branch", "-M", self.LEGACY_BRANCH)
        self._git(seed, "remote", "add", "origin", str(origin))
        self._git(seed, "push", "origin", self.LEGACY_BRANCH)
        self._git(seed, "checkout", "-b", self.TARGET_BRANCH)
        (seed / "version.txt").write_text("target\n", encoding="utf-8")
        self._git(seed, "add", "version.txt")
        self._git(seed, "commit", "-m", "target")
        target_sha = self._git(seed, "rev-parse", "HEAD").stdout.strip()
        self._git(seed, "push", "origin", self.TARGET_BRANCH)

        _run(
            [
                "git",
                "clone",
                "--branch",
                self.LEGACY_BRANCH,
                "--single-branch",
                str(origin),
                str(checkout),
            ]
        )
        self._git(
            checkout,
            "fetch",
            "origin",
            "+refs/heads/%s:refs/remotes/origin/%s"
            % (self.TARGET_BRANCH, self.TARGET_BRANCH),
        )
        fetch_refspecs = self._git(
            checkout, "config", "--get-all", "remote.origin.fetch"
        ).stdout.splitlines()
        self.assertIn(
            "+refs/heads/%s:refs/remotes/origin/%s"
            % (self.LEGACY_BRANCH, self.LEGACY_BRANCH),
            fetch_refspecs,
        )
        self.assertNotIn(
            "+refs/heads/%s:refs/remotes/origin/%s"
            % (self.TARGET_BRANCH, self.TARGET_BRANCH),
            fetch_refspecs,
        )
        return checkout, target_sha

    def _run_update_checkout(self, root, checkout, target_sha):
        harness = root / "branch-update-test.sh"
        q = lambda value: shlex.quote(_bash_path(value))
        harness.write_text(
            "#!/usr/bin/env bash\n"
            "set -Eeuo pipefail\n"
            "export PATH=/usr/bin:/mingw64/bin:$PATH\n"
            + self.branch_functions
            + "\nupdate_checkout 'Livox-SDK' %s %s %s\n"
            % (
                q(checkout),
                shlex.quote(self.TARGET_BRANCH),
                shlex.quote(target_sha),
            ),
            encoding="utf-8",
            newline="\n",
        )
        return _run(
            [self.bash, "--noprofile", "--norc", _bash_path(harness)],
            check=False,
        )

    def _assert_repaired_tracking(self, checkout, target_sha):
        self.assertEqual(
            self._git(checkout, "rev-parse", "HEAD").stdout.strip(),
            target_sha,
        )
        self.assertEqual(
            self._git(checkout, "branch", "--show-current").stdout.strip(),
            self.TARGET_BRANCH,
        )
        self.assertEqual(
            self._git(
                checkout,
                "rev-parse",
                "--abbrev-ref",
                "--symbolic-full-name",
                "@{upstream}",
            ).stdout.strip(),
            "origin/%s" % self.TARGET_BRANCH,
        )
        self.assertEqual(
            self._git(
                checkout,
                "config",
                "--get",
                "branch.%s.remote" % self.TARGET_BRANCH,
            ).stdout.strip(),
            "origin",
        )
        self.assertEqual(
            self._git(
                checkout,
                "config",
                "--get",
                "branch.%s.merge" % self.TARGET_BRANCH,
            ).stdout.strip(),
            "refs/heads/%s" % self.TARGET_BRANCH,
        )
        fetch_refspecs = self._git(
            checkout, "config", "--get-all", "remote.origin.fetch"
        ).stdout.splitlines()
        self.assertEqual(
            fetch_refspecs.count(
                "+refs/heads/%s:refs/remotes/origin/%s"
                % (self.TARGET_BRANCH, self.TARGET_BRANCH)
            ),
            1,
        )
        self.assertIn(
            "+refs/heads/%s:refs/remotes/origin/%s"
            % (self.LEGACY_BRANCH, self.LEGACY_BRANCH),
            fetch_refspecs,
        )

    def test_single_branch_clone_can_create_and_track_target(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            checkout, target_sha = self._prepare_single_branch_clone(root)
            result = self._run_update_checkout(root, checkout, target_sha)
            self.assertEqual(result.returncode, 0, result.stderr)
            self._assert_repaired_tracking(checkout, target_sha)

            second = self._run_update_checkout(root, checkout, target_sha)
            self.assertEqual(second.returncode, 0, second.stderr)
            self._assert_repaired_tracking(checkout, target_sha)

    def test_retry_repairs_branch_left_without_upstream(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            checkout, target_sha = self._prepare_single_branch_clone(root)
            self._git(checkout, "checkout", "-b", self.TARGET_BRANCH)
            failed = self._git(
                checkout,
                "branch",
                "--set-upstream-to=origin/%s" % self.TARGET_BRANCH,
                self.TARGET_BRANCH,
                check=False,
            )
            self.assertNotEqual(failed.returncode, 0)

            result = self._run_update_checkout(root, checkout, target_sha)
            self.assertEqual(result.returncode, 0, result.stderr)
            self._assert_repaired_tracking(checkout, target_sha)


class UpdaterEmbeddedValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = UPDATER.read_text(encoding="utf-8")
        function = _function(cls.source, "validate_relay_launch_integration")
        cls.validator = function.split("<<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]

    def _validate(self, payload):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "test.launch"
            path.write_bytes(payload)
            return subprocess.run(
                [sys.executable, "-", str(path), MARKER],
                input=self.validator,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

    def test_exact_embedded_python_accepts_only_valid_unique_integration(self):
        valid = self._validate(_launch(relay=True))
        self.assertEqual(valid.returncode, 0, valid.stderr)
        headless = self._validate(_launch(relay=True, monitor_node=False))
        self.assertEqual(headless.returncode, 0, headless.stderr)
        shipped = _launch(relay=True)
        for name, payload in {
            "missing-marker": _launch(relay=True, marker=False),
            "duplicate-marker": _launch(relay=True).replace(
                b"</launch>",
                ("<!-- %s -->\n</launch>" % MARKER).encode("utf-8"),
            ),
            "invalid-xml": _launch(relay=True, valid_xml=False),
            "nonliteral-enable-default": shipped.replace(
                b'name="relay_power_cycle_enable" default="false"',
                b'name="relay_power_cycle_enable" default="$(env RELAY_ENABLE)"',
                1,
            ),
            "miswired-child-enable": shipped.replace(
                b'value="$(arg relay_power_cycle_enable)"',
                b'value="true"',
                1,
            ),
            "same-basename-wrong-path": shipped.replace(
                b'$(find livox_ros_driver)/launch/livox_power_cycle.launch',
                b'/tmp/livox_power_cycle.launch',
                1,
            ),
            "inline-legacy-manager": shipped.replace(
                b"</launch>",
                b'  <node name="livox_power_cycle_manager" pkg="livox_ros_driver" type="livox_power_cycle_manager.py"/>\n</launch>',
                1,
            ),
            "missing-monitor-marker": shipped.replace(
                ("  <!-- %s -->\n" % MONITOR_MARKER).encode("utf-8"),
                b"",
                1,
            ),
            "missing-monitor-layout-arg": shipped.replace(
                b'  <arg name="monitor_layout" default="compact"/>\n',
                b"",
                1,
            ),
            "invalid-monitor-layout-default": shipped.replace(
                b'name="monitor_layout" default="compact"',
                b'name="monitor_layout" default="giant"',
                1,
            ),
            "monitor-does-not-consume-layout": shipped.replace(
                b' args="--layout $(arg monitor_layout)"',
                b"",
                1,
            ),
        }.items():
            with self.subTest(name=name):
                result = self._validate(payload)
                self.assertNotEqual(result.returncode, 0)


class UpdaterChildLaunchValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = UPDATER.read_text(encoding="utf-8")
        function = _function(source, "validate_relay_child_launch")
        cls.validator = function.split("<<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
        cls.valid = (ROOT / RELAY_CHILD).read_bytes()

    def _validate(self, payload):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "livox_power_cycle.launch"
            path.write_bytes(payload)
            return subprocess.run(
                [sys.executable, "-", str(path)],
                input=self.validator,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

    def test_child_is_armed_only_and_uses_fixed_paths(self):
        result = self._validate(self.valid)
        self.assertEqual(result.returncode, 0, result.stderr)
        cases = {
            "state-db-split": self.valid.replace(
                b"$(env HOME)/.local/state/livox-power-cycle-manager/state.sqlite3",
                b"/tmp/other.sqlite3",
                1,
            ),
            "required-manager": self.valid.replace(
                b'required="false"', b'required="true"', 1
            ),
            "extra-node-attribute": self.valid.replace(
                b'respawn_delay="5"',
                b'respawn_delay="5" launch-prefix="env"',
                1,
            ),
        }
        for name, payload in cases.items():
            with self.subTest(name=name):
                invalid = self._validate(payload)
                self.assertNotEqual(invalid.returncode, 0)


class UpdaterRestartSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = UPDATER.read_text(encoding="utf-8")
        cls.bash = _find_bash()
        cls.safety_functions = "\n".join(
            _function(cls.source, name)
            for name in (
                "log",
                "die",
                "inspect_legacy_power_manager_unit",
                "sync_runtime_manager_helper",
                "validate_driver_safety_dropin",
                "validate_relay_launch_integration",
                "validate_relay_child_launch",
                "validate_driver_restart_safety",
            )
        )

    def setUp(self):
        if self.bash is None:
            self.skipTest("Bash is unavailable")

    def _run_safety(
        self, root, load, active, enabled, default, dropins="", unit_files=""
    ):
        driver = root / "driver"
        home = root / "home"
        home.mkdir()
        launch = driver / SITE_LAUNCH
        child = driver / RELAY_CHILD
        launch.parent.mkdir(parents=True)
        launch.write_bytes(_launch(relay=True).replace(
            b'default="false"',
            ('default="%s"' % default).encode("ascii"),
            1,
        ))
        child.write_bytes(
            (ROOT / RELAY_CHILD).read_bytes()
        )
        manager_source = (
            driver
            / "livox_ros_driver/livox_ros_driver/scripts/"
            "livox_power_cycle_manager.py"
        )
        manager_source.parent.mkdir(parents=True, exist_ok=True)
        manager_source.write_bytes(
            (
                ROOT
                / "livox_ros_driver/livox_ros_driver/scripts/"
                "livox_power_cycle_manager.py"
            ).read_bytes()
        )
        manager_source.with_name("validate_livox_power_cycle_site.py").write_text(
            "# test fixture\n", encoding="utf-8"
        )
        manager_runtime_dir = home / ".local/libexec/livox-power-cycle-manager"
        manager_runtime = manager_runtime_dir / "livox_power_cycle_manager.py"
        dropin_path = root / "20-livox-power-cycle-safety.conf"
        expected_repair = (
            "/usr/bin/python3 %s --state-db %s/.local/state/"
            "livox-power-cycle-manager/state.sqlite3 --repair-obligations"
            % (_bash_path(manager_runtime), _bash_path(home))
        )
        loaded_dropins = ""
        if dropins:
            dropin_path.write_text(
                "[Service]\n"
                "# LIVOX_POWER_CYCLE_SAFETY_DROPIN_V2\n"
                "ExecStartPre=%s\n" % expected_repair
                + "ExecStopPost=%s\n" % expected_repair
                + "TimeoutStartSec=600\n"
                + "TimeoutStopSec=300\n"
                + "SendSIGKILL=yes\n",
                encoding="utf-8",
                newline="\n",
            )
            loaded_dropins = _bash_path(dropin_path)
        sentinel = root / "continued"
        harness = root / "safety-test.sh"
        q = lambda value: shlex.quote(_bash_path(value))
        fake_python = UpdaterSiteTransactionTests._fake_python_function()
        fake_python = fake_python.replace(
            "python_validator() {\n",
            "python_validator() {\n"
            "  if [[ \"${1:-}\" == *validate_livox_power_cycle_site.py ]]; then return 0; fi\n",
            1,
        )
        harness.write_text(
            "#!/usr/bin/env bash\n"
            "set -Eeuo pipefail\n"
            "export PATH=/usr/bin:/mingw64/bin:$PATH\n"
            + "HOME=%s\n" % q(home)
            + fake_python.replace(
                "  fi\n}\n",
                "  else\n"
                "    if grep -q 'name=\"relay_power_cycle_enable\" default=\"true\"' \"${path}\"; then\n"
                "      printf 'true\\n'\n"
                "    else\n"
                "      printf 'false\\n'\n"
                "    fi\n"
                "  fi\n}\n",
            )
            + "systemctl() {\n"
            + "  case \"$*\" in\n"
            + "    *--property=LoadState*) printf '%s\\n' ;;\n" % load
            + "    *--property=ActiveState*) printf '%s\\n' ;;\n" % active
            + "    *is-enabled*) printf '%s\\n' ;;\n" % enabled
            + "    *list-unit-files*) printf '%s\\n' ;;\n" % unit_files
            + "    *--property=DropInPaths*) printf '%s\\n' ;;\n" % loaded_dropins
            + "    *--property=ExecStartPre*) printf '%s\\n' ;;\n" % expected_repair
            + "    *--property=ExecStopPost*) printf '%s\\n' ;;\n" % expected_repair
            + "    *--property=User*) id -un ;;\n"
            + "    *--property=Restart*) printf 'always\\n' ;;\n"
            + "    *--property=KillMode*) printf 'control-group\\n' ;;\n"
            + "    *--property=Type*) printf 'simple\\n' ;;\n"
            + "    *--property=RemainAfterExit*) printf 'no\\n' ;;\n"
            + "    *--property=SendSIGKILL*) printf 'yes\\n' ;;\n"
            + "    *--property=Environment*) printf 'HOME=%s\\n' ;;\n" % _bash_path(home)
            + "    *--property=TimeoutStartUSec*) printf '10min\\n' ;;\n"
            + "    *--property=TimeoutStopUSec*) printf '5min\\n' ;;\n"
            + "    *) return 1 ;;\n"
            + "  esac\n"
            + "}\n"
            + "install() { if [[ \"$1\" == \"-d\" ]]; then mkdir -p -- \"${@: -1}\"; else cp -- \"${@: -2:1}\" \"${@: -1}\"; fi; }\n"
            + "stat() { if [[ \"$*\" == *\"${MANAGER_RUNTIME}\"* ]]; then printf '%s 700\\n' \"$(id -u)\"; else printf '0 644\\n'; fi; }\n"
            + "PYTHON_EXECUTABLE=python_validator\n"
            + "DRIVER_DIR=%s\n" % q(driver)
            + "MANAGER_RUNTIME_DIR=%s\n" % q(manager_runtime_dir)
            + "MANAGER_RUNTIME=%s\n" % q(manager_runtime)
            + "RUNTIME_HELPER_TEMP=\"\"\n"
            + "SITE_JSON_PATH=%s\n" % shlex.quote(SITE_JSON)
            + "SITE_LAUNCH_PATH=%s\n" % shlex.quote(SITE_LAUNCH)
            + "RELAY_CHILD_PATH=%s\n" % shlex.quote(RELAY_CHILD)
            + "SITE_LAUNCH_MARKER=%s\n" % shlex.quote(MARKER)
            + "DRIVER_SAFETY_DROPIN=%s\n" % q(dropin_path)
            + self.safety_functions
            + "\nvalidate_driver_restart_safety\n"
            + "printf success >%s\n" % q(sentinel),
            encoding="utf-8",
            newline="\n",
        )
        result = _run(
            [self.bash, "--noprofile", "--norc", _bash_path(harness)],
            check=False,
        )
        return result, sentinel

    def test_any_loaded_legacy_unit_is_fail_closed_even_disabled_inactive(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, sentinel = self._run_safety(
                Path(tmp), "loaded", "inactive", "disabled", "false"
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(sentinel.exists())
            self.assertIn("旧的独立 livox-power-cycle-manager.service", result.stderr)
            self.assertNotIn("systemctl stop livox-power-cycle-manager", self.source)
            self.assertNotIn("systemctl disable livox-power-cycle-manager", self.source)
            self.assertNotIn("systemctl start livox-power-cycle-manager", self.source)

    def test_dangling_enabled_legacy_unit_is_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, sentinel = self._run_safety(
                Path(tmp), "not-found", "inactive", "enabled", "false", DROPIN
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse(sentinel.exists())
            self.assertIn("enabled=enabled", result.stderr)

    def test_empty_is_enabled_stdout_uses_empty_unit_file_list_as_not_found(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, sentinel = self._run_safety(
                Path(tmp), "not-found", "inactive", "", "false", DROPIN, ""
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(sentinel.is_file())

    def test_missing_safety_dropin_is_fail_closed_for_either_default(self):
        for default in ("false", "true"):
            with self.subTest(default=default), tempfile.TemporaryDirectory() as tmp:
                result, sentinel = self._run_safety(
                    Path(tmp), "not-found", "inactive", "not-found", default
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(sentinel.exists())
                self.assertIn("systemd 尚未加载", result.stderr)

    def test_true_default_is_allowed_with_exact_loaded_dropin(self):
        with tempfile.TemporaryDirectory() as tmp:
            result, sentinel = self._run_safety(
                Path(tmp),
                "not-found",
                "inactive",
                "not-found",
                "true",
                DROPIN,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(sentinel.is_file())


if __name__ == "__main__":
    unittest.main()
