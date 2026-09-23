"""Post-load RGB augmentation; masks and robot pixels are never transformed.

A sample is any uint8 (..., H, W, 3) array. All leading dimensions (usually
sequence time and camera) share one plan. Texture coordinates are normalized
image coordinates, not a claim of recovered world-space surface correspondence.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
from numbers import Integral, Real
from typing import Iterable

import numpy as np
from PIL import Image

_LUMA = np.asarray((0.2126, 0.7152, 0.0722), dtype=np.float32)
_MAX_ID = np.iinfo(np.int32).max


def integer(value: object, name: str, minimum: int = 0, maximum: int = 2**64 - 1) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    return int(value)


def _fraction(value: object, name: str) -> None:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not np.isfinite(value) or not 0 <= value <= 1:
        raise ValueError(f"{name} must be a finite real number in [0, 1]")


def positive_ids(values: Iterable[int]) -> tuple[int, ...]:
    result = tuple(integer(value, "instance ID", 1, _MAX_ID) for value in values)
    if len(set(result)) != len(result):
        raise ValueError("instance IDs must be unique")
    return tuple(sorted(result))


def validate_images(rgb: np.ndarray, masks: np.ndarray, known_instance_ids: Iterable[int] | None = None) -> tuple[int, ...]:
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim < 3 or rgb.shape[-1] != 3:
        raise ValueError("RGB must be a uint8 numpy array with shape (..., H, W, 3)")
    if any(size == 0 for size in rgb.shape):
        raise ValueError("RGB dimensions must be nonempty")
    if not isinstance(masks, np.ndarray) or masks.dtype.kind not in "iu" or masks.shape != rgb.shape[:-1]:
        raise ValueError("instance masks must be integer numpy arrays with shape (..., H, W) matching RGB")
    labels = np.unique(masks)
    if np.any(labels < -3) or np.any(labels > _MAX_ID):
        raise ValueError("instance masks contain invalid reserved or out-of-int32-range IDs")
    observed = tuple(int(value) for value in labels if value > 0)
    if known_instance_ids is None:
        return observed
    known = positive_ids(known_instance_ids)
    if not set(observed).issubset(known):
        raise ValueError(f"unknown positive instance IDs: {sorted(set(observed) - set(known))}")
    return known


@dataclass(frozen=True)
class AugmentationConfig:
    tint_objects: bool = True
    replace_table: bool = True
    replace_background: bool = True
    tint_strength: float = 0.75
    texture_strength: float = 1.0
    surface_detail: float = 0.25
    # None means every task object, never robot/table/environment.
    object_ids: tuple[int, ...] | None = None

    def __post_init__(self) -> None:
        for name in ("tint_objects", "replace_table", "replace_background"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} must be bool")
        for name in ("tint_strength", "texture_strength", "surface_detail"):
            _fraction(getattr(self, name), name)
            object.__setattr__(self, name, float(getattr(self, name)))
        if self.object_ids is not None:
            object.__setattr__(self, "object_ids", positive_ids(self.object_ids))


@dataclass(frozen=True)
class ObjectTint:
    instance_id: int
    color: tuple[float, float, float]


@dataclass(frozen=True)
class SurfaceTexture:
    semantic: str
    # -1 selects a built-in procedural pattern; otherwise an owned bank entry.
    bank_index: int
    pattern: str
    color_a: tuple[float, float, float]
    color_b: tuple[float, float, float]
    frequency: tuple[float, float]
    phase: tuple[float, float]
    quarter_turns: int


@dataclass(frozen=True)
class AugmentationPlan:
    seed: int
    config: AugmentationConfig
    object_tints: tuple[ObjectTint, ...]
    surfaces: tuple[SurfaceTexture, ...]
    texture_sha256: tuple[str, ...]

    def as_dict(self) -> dict:
        """JSON-compatible parameters, including seed and texture-bank identities."""
        return asdict(self)


@dataclass
class AugmentedImages:
    rgb: np.ndarray
    instance_id: np.ndarray
    plan: AugmentationPlan


class VisualAugmenter:
    """Sample one deterministic transform per semantic region, then apply it.

    Bank entries are copied on construction and identified by content hashes.
    No torch dependency, global RNG, file writes, or per-pixel Python loops.
    """

    def __init__(self, config: AugmentationConfig | None = None, *, textures: Iterable[np.ndarray] = ()) -> None:
        self.config = AugmentationConfig() if config is None else config
        if not isinstance(self.config, AugmentationConfig):
            raise ValueError("config must be AugmentationConfig")
        owned = []
        hashes = []
        for texture in textures:
            if (not isinstance(texture, np.ndarray) or texture.dtype != np.uint8 or texture.ndim != 3
                    or texture.shape[2] != 3 or min(texture.shape) == 0):
                raise ValueError("each texture must be a nonempty uint8 [H, W, 3] numpy array")
            value = np.array(texture, copy=True, order="C")
            value.flags.writeable = False
            owned.append(value)
            digest = hashlib.sha256(np.asarray(value.shape, dtype="<i8").tobytes())
            digest.update(value.tobytes())
            hashes.append(digest.hexdigest())
        self._textures = tuple(owned)
        self._texture_hashes = tuple(hashes)

    def sample_plan(self, seed: int, instance_ids: Iterable[int]) -> AugmentationPlan:
        seed = integer(seed, "seed")
        known = positive_ids(instance_ids)
        selected = known if self.config.object_ids is None else self.config.object_ids
        if not set(selected).issubset(known):
            raise ValueError("selected object IDs must be present in instance metadata")
        tints = []
        if self.config.tint_objects:
            for instance_id in selected:
                # Independent streams make a visible object's tint invariant to
                # other objects entering/leaving a sequence or camera view.
                rng = np.random.default_rng(np.random.SeedSequence([seed, 1, instance_id]))
                color = rng.uniform(0.05, 1.0, 3)
                color /= color.max()
                tints.append(ObjectTint(instance_id, tuple(float(x) for x in color)))
        surfaces = []
        for semantic, enabled, stream in (("table", self.config.replace_table, 2),
                                          ("background", self.config.replace_background, 3)):
            if not enabled:
                continue
            rng = np.random.default_rng(np.random.SeedSequence([seed, stream]))
            surfaces.append(SurfaceTexture(
                semantic, int(rng.integers(len(self._textures))) if self._textures else -1,
                str(rng.choice(("grain", "woven", "tiles"))),
                tuple(float(x) for x in rng.uniform(35, 145, 3)),
                tuple(float(x) for x in rng.uniform(155, 245, 3)),
                tuple(float(x) for x in rng.uniform(3, 14, 2)),
                tuple(float(x) for x in rng.uniform(0, 2 * np.pi, 2)),
                int(rng.integers(4)),
            ))
        return AugmentationPlan(seed, self.config, tuple(tints), tuple(surfaces), self._texture_hashes)

    def _texture(self, spec: SurfaceTexture, height: int, width: int) -> np.ndarray:
        if spec.bank_index >= 0:
            bank = np.rot90(self._textures[spec.bank_index], spec.quarter_turns)
            with Image.fromarray(bank) as image:
                with image.resize((width, height), Image.Resampling.BILINEAR) as resized:
                    return np.asarray(resized, dtype=np.float32)
        y, x = np.mgrid[:height, :width].astype(np.float32)
        x /= max(width - 1, 1)
        y /= max(height - 1, 1)
        for _ in range(spec.quarter_turns):
            x, y = y, 1 - x
        u = 2 * np.pi * x * spec.frequency[0] + spec.phase[0]
        v = 2 * np.pi * y * spec.frequency[1] + spec.phase[1]
        if spec.pattern == "grain":
            mix = 0.5 + 0.5 * np.sin(u + 0.65 * np.sin(v))
        elif spec.pattern == "woven":
            mix = 0.5 + 0.25 * (np.sin(u) + np.sin(v))
        else:
            mix = np.where(np.sin(u) * np.sin(v) >= 0, 0.8, 0.2)
        a, b = np.asarray(spec.color_a, dtype=np.float32), np.asarray(spec.color_b, dtype=np.float32)
        return a + mix[..., None] * (b - a)

    def __call__(self, rgb: np.ndarray, instance_id: np.ndarray, *, seed: int,
                 known_instance_ids: Iterable[int] | None = None) -> AugmentedImages:
        known = validate_images(rgb, instance_id, known_instance_ids)
        plan = self.sample_plan(seed, known)
        result = rgb.copy()
        strength = self.config.tint_strength
        for tint in plan.object_tints:
            selection = instance_id == tint.instance_id
            if not np.any(selection) or strength == 0:
                continue
            pixels = rgb[selection].astype(np.float32)
            luminance = pixels @ _LUMA
            target = np.asarray(tint.color, dtype=np.float32)
            target -= target @ _LUMA
            # A luminance-neutral chroma perturbation retains shading, edges,
            # printed detail, and exact per-pixel luminance before quantization.
            chroma = ((1 - strength) * (pixels - luminance[:, None])
                      + strength * 2 * np.minimum(luminance, 255 - luminance)[:, None] * target)
            # Compress chroma, rather than clip channels (which changes luma).
            bounds = np.full_like(chroma, np.inf)
            np.divide(255 - luminance[:, None], chroma, out=bounds, where=chroma > 0)
            np.divide(-luminance[:, None], chroma, out=bounds, where=chroma < 0)
            scale = np.minimum(1, bounds.min(axis=1))
            result[selection] = np.rint(np.clip(luminance[:, None] + chroma * scale[:, None], 0, 255)).astype(np.uint8)
        height, width = rgb.shape[-3:-1]
        for surface in plan.surfaces:
            selection = (instance_id == -3) if surface.semantic == "table" else ((instance_id == -2) | (instance_id == 0))
            if not np.any(selection) or self.config.texture_strength == 0:
                continue
            texture = self._texture(surface, height, width)
            # Broadcasting shares an identical surface plan at every T/C index.
            replacement = np.broadcast_to(texture, rgb.shape)[selection].copy()
            pixels = rgb[selection].astype(np.float32)
            detail = (pixels @ _LUMA - 127.5) * self.config.surface_detail
            replacement += detail[:, None]
            replacement = (1 - self.config.texture_strength) * pixels + self.config.texture_strength * replacement
            result[selection] = np.rint(np.clip(replacement, 0, 255)).astype(np.uint8)
        return AugmentedImages(result, instance_id.copy(), plan)
