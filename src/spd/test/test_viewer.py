from spd_vr.simulation.viewer_window import ViewerWindow


def test_viewer_window_maps_subscriber_keys_and_shutdown_once():
    controls = []
    recordings = []
    shutdowns = []
    window = ViewerWindow(
        headless=True,
        joint_control=controls.append,
        recording_control=recordings.append,
        shutdown=lambda: shutdowns.append("shutdown"),
    )
    for key in (ord("E"), ord("C"), 297, 298, ord("R"), ord("S"), ord("D")):
        window.on_key(key)
    window.on_key(ord("Q"))
    window.on_key("escape")
    assert controls == ["e", "c", "f8", "f9"]
    assert recordings == ["start", "success", "discard"]
    assert shutdowns == ["shutdown"]
