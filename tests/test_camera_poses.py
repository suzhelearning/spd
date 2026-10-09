"""Camera mount parsing and in-memory application invariants."""
import math
import unittest

import mujoco
import numpy as np

from cameras.camera import CameraError, apply_camera_config, parse_camera_config


_MODEL_XML = """
<mujoco>
  <worldbody>
    <body name="Link_Stand">
      <freejoint name="base_free"/>
      <geom type="sphere" size="0.01" mass="1"/>
      <camera name="top"/>
    </body>
    <body name="l_wrist" pos="0 0.1 0">
      <camera name="left_wrist"/>
    </body>
    <body name="r_wrist" pos="0 -0.1 0">
      <camera name="right_wrist"/>
    </body>
  </worldbody>
</mujoco>
"""


def _camera_document() -> dict[str, object]:
    return {
        "version": 2,
        "calibration_revision": "provisional-test",
        "width": 1280,
        "height": 720,
        "fovy_deg": 70.0,
        "near_m": 0.01,
        "far_m": 3.0,
        "cameras": {
            "top": {
                "parent": "r_wrist",
                "position": [0.1, 0.2, 0.3],
                "rpy_deg": [90.0, 0.0, 90.0],
            },
            "left_wrist": {
                "parent": "Link_Stand",
                "position": [-0.1, 0.2, 0.3],
                "rpy_deg": [0.0, 0.0, 0.0],
            },
            "right_wrist": {
                "parent": "l_wrist",
                "position": [0.1, -0.2, 0.3],
                "rpy_deg": [0.0, 90.0, 0.0],
            },
        },
    }


class CameraPoseTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_string(_MODEL_XML)
        self.data = mujoco.MjData(self.model)

    def test_local_mounts_follow_their_parent_with_the_configured_optical_axes(self):
        document = _camera_document()
        self.data.qpos[:3] = (1.0, 2.0, 3.0)
        self.data.qpos[3:7] = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
        qpos_before = self.data.qpos.copy()
        body_pos_before = self.model.body_pos.copy()
        body_quat_before = self.model.body_quat.copy()
        apply_camera_config(self.model, document)
        mujoco.mj_forward(self.model, self.data)

        np.testing.assert_array_equal(self.data.qpos, qpos_before)
        np.testing.assert_array_equal(self.model.body_pos, body_pos_before)
        np.testing.assert_array_equal(self.model.body_quat, body_quat_before)
        top = self.model.camera("top").id
        left = self.model.camera("left_wrist").id
        right = self.model.camera("right_wrist").id
        np.testing.assert_allclose(self.data.cam_xpos[top], (0.1, 0.1, 0.3), rtol=0, atol=1e-12)
        np.testing.assert_allclose(self.data.cam_xpos[left], (0.8, 1.9, 3.3), rtol=0, atol=1e-12)
        np.testing.assert_allclose(self.data.cam_xpos[right], (0.1, -0.1, 0.3), rtol=0, atol=1e-12)
        rotation = self.data.cam_xmat[top].reshape(3, 3)
        np.testing.assert_allclose(rotation @ (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), rtol=0, atol=1e-12)
        np.testing.assert_allclose(rotation @ (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), rtol=0, atol=1e-12)
        np.testing.assert_allclose(
            self.data.cam_xmat[left].reshape(3, 3),
            ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            rtol=0, atol=1e-12,
        )

        self.data.qpos[:3] += (0.3, -0.4, 0.5)
        mujoco.mj_forward(self.model, self.data)
        np.testing.assert_allclose(self.data.cam_xpos[left], (1.1, 1.5, 3.8), rtol=0, atol=1e-12)
        np.testing.assert_allclose(self.data.cam_xpos[top], (0.1, 0.1, 0.3), rtol=0, atol=1e-12)
        np.testing.assert_allclose(self.data.cam_xpos[right], (0.1, -0.1, 0.3), rtol=0, atol=1e-12)

    def test_invalid_pose_and_missing_mount_leave_cameras_unchanged(self):
        invalid_pose = _camera_document()
        invalid_pose["cameras"]["top"]["rpy_deg"][1] = math.nan
        with self.assertRaises(CameraError):
            parse_camera_config(invalid_pose)

        invalid_world = _camera_document()
        invalid_world["cameras"]["top"]["parent"] = "world"
        with self.assertRaises(CameraError):
            parse_camera_config(invalid_world)

        invalid_mount = _camera_document()
        invalid_mount["cameras"]["right_wrist"]["parent"] = "missing_mount"
        before = {
            field: getattr(self.model, field).copy()
            for field in ("cam_mode", "cam_targetbodyid", "cam_bodyid", "cam_pos", "cam_quat", "cam_fovy")
        }
        with self.assertRaises(CameraError):
            apply_camera_config(self.model, invalid_mount)
        for field, value in before.items():
            np.testing.assert_array_equal(getattr(self.model, field), value)


if __name__ == "__main__":
    unittest.main()
