import json

from spd_vr import preflight


def test_preflight_reports_structured_required_failure_without_starting_device(tmp_path, monkeypatch):
    calls = []

    def fake_run(*args, **kwargs):
        calls.append(args[0])
        return type("Completed", (), {"returncode": 0, "stdout": "", "stderr": ""})()

    monkeypatch.setattr(preflight, "_run_command", fake_run)
    results = preflight.run_checks(
        repo_root=tmp_path,
        run_command=fake_run,
        sdk_loader=lambda path: object(),
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
    assert not any(len(command) > 2 and command[1:2] == ["reverse"] and command[2] != "--list" for command in calls)


def test_preflight_cli_returns_nonzero_and_json_lines(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(
        preflight,
        "run_checks",
        lambda **kwargs: [preflight.CheckResult("artifacts", False, "missing model_manifest.yaml")],
    )

    assert preflight.main(["--repo-root", str(tmp_path)]) == 1
    record = json.loads(capsys.readouterr().out.strip())
    assert record == {"detail": "missing model_manifest.yaml", "name": "artifacts", "ok": False}
