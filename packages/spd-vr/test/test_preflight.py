import json

from spd_vr import preflight


def test_preflight_reports_structured_artifact_failure(tmp_path):
    calls = []

    def fake_run(*args, **kwargs):
        calls.append(args[0])
        return type("Completed", (), {"returncode": 0, "stdout": "", "stderr": ""})()

    results = preflight.run_checks(
        repo_root=tmp_path,
        run_command=fake_run,
        dependency_loader=lambda name: object(),
        display_env={"DISPLAY": ":99"},
        port_checker=lambda endpoint: (True, "free"),
        session_checker=lambda name: (True, "not running"),
        artifact_checker=lambda manifest, urdf: (False, "authoritative URDF hash mismatch"),
    )

    assert all(isinstance(item, preflight.CheckResult) for item in results)
    artifact = next(item for item in results if item.name == "artifacts")
    assert artifact.ok is False
    assert "hash" in artifact.detail
    assert not any("reverse" in command for command in calls)
    assert not any(command[:2] == ["ss", "-H"] for command in calls)


def test_preflight_forwards_selected_pico2_port():
    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        output = (
            "List of devices attached\nPICO-1\tdevice\n"
            if command[:2] == ["adb", "devices"]
            else ""
        )
        return type("Completed", (), {"returncode": 0, "stdout": output, "stderr": ""})()

    results = preflight._check_pico2_adb(
        fake_run,
        selected_serial="PICO-1",
        local_port=10002,
        device_port=11002,
    )

    assert {item.name: item.ok for item in results} == {
        "pico_device": True,
        "adb_forward": True,
    }
    assert calls == [
        ["adb", "devices"],
        ["adb", "-s", "PICO-1", "forward", "tcp:10002", "tcp:11002"],
    ]


def test_preflight_checks_pico2_without_legacy_services(tmp_path):
    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        output = (
            "List of devices attached\nPICO-1\tdevice\n"
            if command[:2] == ["adb", "devices"]
            else ""
        )
        return type("Completed", (), {"returncode": 0, "stdout": output, "stderr": ""})()

    results = preflight.run_checks(
        repo_root=tmp_path,
        selected_serial="PICO-1",
        run_command=fake_run,
        dependency_loader=lambda name: object(),
        display_env={"DISPLAY": ":99"},
        port_checker=lambda endpoint: (True, "free"),
        session_checker=lambda name: (True, "not running"),
        artifact_checker=lambda manifest, urdf: (True, "verified"),
    )

    assert {item.name: item.ok for item in results}["pico_device"] is True
    assert {item.name: item.ok for item in results}["adb_forward"] is True
    assert not any("reverse" in command for command in calls)
    assert not any(command[:2] == ["ss", "-H"] for command in calls)


def test_preflight_cli_returns_nonzero_and_json_lines(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(
        preflight,
        "run_checks",
        lambda **kwargs: [preflight.CheckResult("artifacts", False, "missing model_manifest.yaml")],
    )

    assert preflight.main(["--repo-root", str(tmp_path)]) == 1
    record = json.loads(capsys.readouterr().out.strip())
    assert record == {"detail": "missing model_manifest.yaml", "name": "artifacts", "ok": False}
