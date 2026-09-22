"""Explicit NVIDIA EGL device binding; import native rendering only inside workers."""
from __future__ import annotations

import os
import sys
from typing import Any


def configure_device(device_id: int, threads: int) -> None:
    if type(device_id) is not int or device_id < 0:
        raise ValueError("EGL device index must be a nonnegative integer")
    if type(threads) is not int or threads < 1:
        raise ValueError("worker threads must be a positive integer")
    if "mujoco" in sys.modules or "OpenGL.GL" in sys.modules or "OpenGL.EGL" in sys.modules:
        raise RuntimeError("EGL device must be selected before native imports; use spawned workers, not fork")
    os.environ["MUJOCO_GL"] = "egl"
    os.environ["PYOPENGL_PLATFORM"] = "egl"
    os.environ["MUJOCO_EGL_DEVICE_ID"] = str(device_id)
    for variable in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[variable] = str(threads)


def probe_device(device_id: int, expected_gpu_name: str | None = None) -> dict[str, Any]:
    if (os.environ.get("MUJOCO_GL") != "egl"
            or os.environ.get("PYOPENGL_PLATFORM") != "egl"
            or os.environ.get("MUJOCO_EGL_DEVICE_ID") != str(device_id)):
        raise RuntimeError("worker EGL environment is not bound to the requested device")
    import mujoco
    from mujoco.egl import egl_ext as egl
    from OpenGL import GL

    devices = egl.eglQueryDevicesEXT()
    if not 0 <= device_id < len(devices):
        raise RuntimeError(f"EGL device {device_id} does not exist; EGL enumerated {len(devices)} devices")
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><visual><global offwidth="16" offheight="16"/></visual>'
        '<worldbody><light pos="0 0 2"/><geom type="sphere" size=".1"/></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=16, width=16) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
        if pixels.shape != (16, 16, 3):
            raise RuntimeError("EGL renderer probe produced an invalid RGB frame")
        def text(token: int) -> str:
            value = GL.glGetString(token)
            if value is None:
                raise RuntimeError("EGL context did not expose an OpenGL device identity")
            return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else str(value)
        vendor, name, version = text(GL.GL_VENDOR), text(GL.GL_RENDERER), text(GL.GL_VERSION)
        if "nvidia" not in vendor.lower() or any(word in name.lower() for word in ("llvmpipe", "softpipe", "swrast")):
            raise RuntimeError(f"EGL device {device_id} is not NVIDIA hardware rendering: {vendor}; {name}")
        if expected_gpu_name is not None and expected_gpu_name.casefold() not in name.casefold():
            raise RuntimeError(f"EGL device {device_id}: expected {expected_gpu_name!r}, found {name!r}")
    return {"gpu_id": device_id, "egl_device_count": len(devices), "gl_vendor": vendor,
            "gl_renderer": name, "gl_version": version, "mujoco_version": mujoco.mj_versionString()}
