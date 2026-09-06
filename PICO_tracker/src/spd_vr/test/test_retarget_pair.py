import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from types import SimpleNamespace

import numpy as np
import pytest
from wuji_retargeting import Retargeter
from wuji_retargeting.opt.base import LPFilter

from spd_vr.pico_hands import PICO_TO_MEDIAPIPE, PicoHandFrame
import spd_vr.retarget_pair as retarget_pair_module
from spd_vr.retarget_pair import HandHoldReason, WujiRetargetPair


_OPEN_MEDIAPIPE_HAND = np.array(
    [
        [0.000, 0.000, 0.000],  # wrist
        [-0.025, 0.010, 0.000], [-0.040, 0.020, 0.000],
        [-0.055, 0.032, 0.000], [-0.070, 0.045, 0.000],  # thumb
        [-0.030, 0.005, 0.000], [-0.032, 0.035, 0.000],
        [-0.033, 0.060, 0.000], [-0.034, 0.085, 0.000],  # index
        [0.000, 0.006, 0.000], [0.000, 0.040, 0.000],
        [0.000, 0.069, 0.000], [0.000, 0.095, 0.000],  # middle
        [0.028, 0.004, 0.000], [0.030, 0.036, 0.000],
        [0.031, 0.062, 0.000], [0.032, 0.086, 0.000],  # ring
        [0.052, 0.000, 0.000], [0.056, 0.029, 0.000],
        [0.058, 0.052, 0.000], [0.060, 0.073, 0.000],  # pinky
    ],
    dtype=np.float64,
)


class FakeRetargeter:
    def __init__(self, *, fail=False):
        self.fail = fail
        names = [f"joint_{index}" for index in range(20)]
        self.optimizer = SimpleNamespace(robot=SimpleNamespace(dof_joint_names=names, joint_limits=np.zeros((20, 2))))

    def reset_filter(self, *_args):
        return None

    def retarget(self, points):
        assert np.asarray(points).shape == (21, 3)
        if self.fail:
            raise RuntimeError("left solver failure")
        return np.arange(20, dtype=np.float64)


class CapturingRetargeter(FakeRetargeter):
    def __init__(self):
        super().__init__()
        self.inputs = []

    def retarget(self, points):
        self.inputs.append(np.asarray(points).copy())
        return super().retarget(points)


def _frame(*, left_active=True, right_active=True, sequence_id=1):
    left = np.zeros((26, 7), dtype=np.float64)
    right = np.zeros((26, 7), dtype=np.float64)
    left[:, 6] = right[:, 6] = 1.0
    for joint in range(26):
        left[joint, :3] = (0.01 * joint, 0.001 * joint, 0.002 * joint)
        right[joint, :3] = (-0.01 * joint, 0.001 * joint, 0.002 * joint)
    return PicoHandFrame(
        left,
        right,
        left_active=left_active,
        right_active=right_active,
        tracking_epoch=1,
        sequence_id=sequence_id,
        timestamp_ns=100 + sequence_id,
    )


def test_fixed_pico_to_mediapipe_mapping_and_exact_20_dof_outputs():
    assert PICO_TO_MEDIAPIPE.tolist() == [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25]
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        FakeRetargeter(),
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )

    result = pair.retarget(_frame())

    assert result.left_valid is True
    assert result.right_valid is True
    assert result.left_qpos.shape == (20,)
    assert result.right_qpos.shape == (20,)
    assert result.left_hold_reason is HandHoldReason.NONE
    assert result.right_hold_reason is HandHoldReason.NONE


def test_pico_adapter_passes_raw_once_to_retargeter():
    left = CapturingRetargeter()
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        left,
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )
    frame = _frame(right_active=False)

    result = pair.retarget(frame)

    assert result.left_valid
    assert len(left.inputs) == 1
    np.testing.assert_allclose(
        left.inputs[0],
        frame.left_hand[PICO_TO_MEDIAPIPE, :3] * frame.left_scale,
    )
    assert not np.allclose(left.inputs[0][0], 0.0)


def test_spd_pico_configs_keep_tuned_wuji2_frame_rotations():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    left = yaml.safe_load((config_dir / "wuji2_pico_left.yaml").read_text())
    right = yaml.safe_load((config_dir / "wuji2_pico_right.yaml").read_text())

    assert left["retarget"]["mediapipe_rotation"] == {
        "x": 180.0,
        "y": 0.0,
        "z": -90.0,
    }
    assert right["retarget"]["mediapipe_rotation"] == {
        "x": 0.0,
        "y": 180.0,
        "z": -90.0,
    }


def test_spd_pico_hand_filter_reaches_step_within_five_source_frames():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    for side in ("left", "right"):
        config = yaml.safe_load(
            (config_dir / f"wuji2_pico_{side}.yaml").read_text()
        )
        filter_ = LPFilter(config["retarget"]["lp_alpha"])
        filter_.next(np.zeros(20))
        output = np.zeros(20)
        for _ in range(5):
            output = filter_.next(np.ones(20))
        np.testing.assert_allclose(output, 1.0, atol=0.05)


@pytest.mark.parametrize("side", ["left", "right"])
def test_mediapipe_finger_semantics_match_wuji2_urdf(side):
    config_dir = Path(__file__).resolve().parents[1] / "config"
    config = yaml.safe_load((config_dir / f"wuji2_pico_{side}.yaml").read_text())
    naming = config["optimizer"]["link_naming"]
    prefix = side[0] + "_"

    assert naming == {
        "prefix": prefix,
        "palm": "wrist",
        "fingers": ["thumb", "index_finger", "middle_finger", "ring_finger", "pinky"],
        "pip": "{finger}_middle",
        "dip": "{finger}_distal",
        "tip": "{finger}_tip",
        "link1": "{finger}_proximal_abd",
    }

    urdf = Path(__file__).resolve().parents[4] / "assets" / "tianji_wuji2" / "tianji_wuji2.urdf"
    joints_by_child = {
        joint.find("child").attrib["link"]: (
            joint.attrib["name"],
            joint.attrib["type"],
            joint.find("parent").attrib["link"],
        )
        for joint in ET.parse(urdf).getroot().findall("joint")
    }
    for finger in naming["fingers"]:
        base = prefix + finger
        pip_joint = prefix + ("thumb_mcp" if finger == "thumb" else f"{finger}_pip")
        dip_joint = prefix + ("thumb_ip" if finger == "thumb" else f"{finger}_dip")
        assert joints_by_child[f"{base}_middle"][0] == pip_joint
        assert joints_by_child[f"{base}_distal"][0] == dip_joint
        assert joints_by_child[f"{base}_tip"] == (
            f"{base}_tip_fixed",
            "fixed",
            f"{base}_distal",
        )


@pytest.mark.parametrize("side", ["left", "right"])
def test_open_hand_retargeting_matches_targets_in_wuji2_wrist_frame(side):
    config = Path(__file__).resolve().parents[1] / "config" / f"wuji2_pico_{side}.yaml"
    retargeter = Retargeter.from_yaml(str(config), side)

    qpos, verbose = retargeter.retarget_verbose(
        _OPEN_MEDIAPIPE_HAND, apply_filter=False
    )
    optimizer = retargeter.optimizer
    robot = optimizer.robot
    robot.compute_forward_kinematics(qpos)
    wrist_id = robot.get_link_index(f"{side[0]}_wrist")
    robot_tip_vectors = robot.get_link_positions_in_frame(
        [robot.get_link_index(name) for name in optimizer.task_link_names],
        wrist_id,
    )
    target_tip_vectors = (
        verbose["mediapipe_kp"][optimizer.MP_TIP_INDICES]
        - verbose["mediapipe_kp"][optimizer.MP_ORIGIN_IDX]
    )
    cosine = np.sum(robot_tip_vectors * target_tip_vectors, axis=1) / (
        np.linalg.norm(robot_tip_vectors, axis=1)
        * np.linalg.norm(target_tip_vectors, axis=1)
    )

    assert np.min(cosine) > 0.90, cosine


def test_failed_or_inactive_left_side_does_not_block_right_side():
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        FakeRetargeter(fail=True),
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )

    result = pair.retarget(_frame())

    assert result.left_valid is False
    assert result.left_hold_reason is HandHoldReason.SOLVER_FAILURE
    assert result.right_valid is True
    assert result.right_hold_reason is HandHoldReason.NONE

    inactive = pair.retarget(_frame(left_active=False, right_active=True, sequence_id=2))
    assert inactive.left_hold_reason is HandHoldReason.INACTIVE
    assert inactive.right_valid is True


class BoundedFakeRetargeter(FakeRetargeter):
    def __init__(self):
        super().__init__()
        self.reset_count = 0
        self.optimizer.robot.joint_limits = np.column_stack(
            (-np.ones(20), np.ones(20))
        )

    def reset_filter(self, *_args):
        self.reset_count += 1

    def retarget(self, points):
        return np.linspace(-2.0, 2.0, 20)


def test_manifest_order_clamps_finite_outputs_and_resets_each_epoch():
    left = BoundedFakeRetargeter()
    right = BoundedFakeRetargeter()
    source_names = [f"joint_{index}" for index in range(20)]
    actuator_names = list(reversed(source_names))
    pair = WujiRetargetPair(
        left,
        right,
        left_actuator_joint_names=actuator_names,
        right_actuator_joint_names=actuator_names,
    )

    first = pair.retarget(_frame(sequence_id=1))
    assert np.isfinite(first.left_qpos).all()
    assert np.all(np.abs(first.left_qpos) <= 1.0)
    assert first.left_qpos[0] == 1.0
    assert first.left_qpos[-1] == -1.0
    assert left.reset_count == right.reset_count == 1

    second = pair.retarget(_frame(sequence_id=2))
    assert second.left_valid and second.right_valid
    assert left.reset_count == right.reset_count == 1
    epoch_reset = pair.retarget(
        PicoHandFrame(
            _frame(sequence_id=3).left_hand,
            _frame(sequence_id=3).right_hand,
            tracking_epoch=2,
            sequence_id=3,
        )
    )
    assert epoch_reset.left_valid and epoch_reset.right_valid
    assert left.reset_count == right.reset_count == 2


def test_invalid_left_hand_or_scale_holds_left_but_keeps_right():
    names = [f"joint_{index}" for index in range(20)]
    pair = WujiRetargetPair(
        FakeRetargeter(),
        FakeRetargeter(),
        left_actuator_joint_names=names,
        right_actuator_joint_names=names,
    )
    frame = _frame(sequence_id=1)
    left = frame.left_hand.copy()
    left[0, 0] = np.nan
    frame = PicoHandFrame(
        left, frame.right_hand, tracking_epoch=1, sequence_id=1, left_scale=-1.0
    )
    result = pair.retarget(frame)
    assert result.left_valid is False
    assert result.left_hold_reason is HandHoldReason.INACTIVE
    assert result.right_valid is True


def test_from_manifest_reads_validated_joint_entries(monkeypatch, tmp_path):
    names = [f"joint_{index}" for index in range(20)]
    entries = [
        {
            "index": index,
            "side": side,
            "group": "hand",
            "joint": f"{side}_{name}",
            "actuator": f"{side}_{name}_position",
            "qpos_address": index,
            "dof_address": index,
            "range": [-1.0, 1.0],
            "velocity_limit": None,
        }
        for side in ("left", "right")
        for index, name in enumerate(names)
    ]
    document = {
        "source": {"urdf": "hand.urdf", "urdf_sha256": ""},
        "hand_joint_order": {
            "left": [f"left_{name}" for name in names],
            "right": [f"right_{name}" for name in names],
        },
        "actuator_order": [entry["actuator"] for entry in entries],
        "joints": entries,
        "manifest_sha256": "",
    }
    urdf = tmp_path / "hand.urdf"
    urdf.write_bytes(b"authoritative")
    document["source"]["urdf_sha256"] = hashlib.sha256(urdf.read_bytes()).hexdigest()
    document["manifest_sha256"] = hashlib.sha256(
        yaml.safe_dump(document, sort_keys=True, allow_unicode=True).encode("utf-8")
    ).hexdigest()
    def make_fake(config, hand_side):
        fake = FakeRetargeter()
        fake.optimizer.robot.dof_joint_names = [f"{hand_side}_{name}" for name in names]
        return fake

    monkeypatch.setattr(retarget_pair_module, "Retargeter", SimpleNamespace(
        from_config=make_fake
    ))
    monkeypatch.setattr(retarget_pair_module, "load_manifest", lambda _: document, raising=False)
    monkeypatch.setattr("spd_vr.manifest.load_manifest", lambda _: document)
    pair = WujiRetargetPair.from_manifest({}, {}, tmp_path / "model_manifest.yaml", urdf)
    assert pair._left_perm.shape == pair._right_perm.shape == (20,)
    document["manifest_sha256"] = "stale"
    with pytest.raises(ValueError, match="manifest hash"):
        WujiRetargetPair.from_manifest({}, {}, tmp_path / "model_manifest.yaml", urdf)
