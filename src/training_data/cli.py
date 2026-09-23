"""Preview training-time augmentation without rewriting source/render HDF5."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from training_data.augmentation import AugmentationConfig, VisualAugmenter, integer
from training_data.reader import RenderedSequence


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("render", type=Path, help="Completed render-schema-2 HDF5")
    parser.add_argument("--source", type=Path, help="Relocated original trajectory (otherwise metadata source_path)")
    parser.add_argument("--frames", default="0", help="Strictly increasing source row indices, e.g. 0,1,2")
    parser.add_argument("--seed", type=int, required=True, help="Explicit nonnegative uint64 augmentation seed")
    parser.add_argument("--output", type=Path, required=True, help="New PNG contact sheet; never overwritten")
    parser.add_argument("--texture", type=Path, action="append", default=[], help="Texture-bank image; repeat for multiple entries")
    parser.add_argument("--no-object-tint", action="store_true")
    parser.add_argument("--no-table", action="store_true")
    parser.add_argument("--no-background", action="store_true")
    parser.add_argument("--tint-strength", type=float, default=0.75)
    parser.add_argument("--texture-strength", type=float, default=1.0)
    parser.add_argument("--surface-detail", type=float, default=0.25)
    args = parser.parse_args(argv)
    try:
        seed = integer(args.seed, "seed")
        indices = [int(value) for value in args.frames.split(",")]
        if args.output.suffix.lower() != ".png":
            raise ValueError("contact-sheet output must have a .png extension")
        textures = []
        for path in args.texture:
            with Image.open(path) as image:
                with image.convert("RGB") as converted:
                    textures.append(np.array(converted))
        augmenter = VisualAugmenter(AugmentationConfig(
            tint_objects=not args.no_object_tint, replace_table=not args.no_table,
            replace_background=not args.no_background, tint_strength=args.tint_strength,
            texture_strength=args.texture_strength, surface_detail=args.surface_detail,
        ), textures=textures)
        with RenderedSequence(args.render, source_path=args.source) as sequence:
            original = sequence.read(indices)
            augmented = sequence.read(indices, augmentation=augmenter, seed=seed)
            height, width = original.rgb.shape[-3:-1]
            caption_height = 24
            sheet = Image.new("RGB", (width * len(original.camera_names), (height + caption_height) * len(indices) * 2), "white")
            drawing = ImageDraw.Draw(sheet)
            for frame_offset, frame_index in enumerate(indices):
                for variant, sample in enumerate((original, augmented)):
                    top = (frame_offset * 2 + variant) * (height + caption_height)
                    for camera_offset, camera in enumerate(sample.camera_names):
                        left = camera_offset * width
                        label = f"{frame_index} {camera} {'aug' if variant else 'original'}"
                        drawing.text((left + 4, top + 4), label, fill="black")
                        with Image.fromarray(sample.rgb[frame_offset, camera_offset]) as tile:
                            sheet.paste(tile, (left, top + caption_height))
            # Exclusive creation prevents accidental replacement of any artifact.
            with args.output.open("xb") as stream:
                sheet.save(stream, format="PNG")
            sheet.close()
            report = {
                "render": str(sequence.path), "source": str(sequence.source_path),
                "source_sha256": sequence.metadata["source"]["source_sha256"],
                "frames": augmented.frame_indices.tolist(), "cameras": list(augmented.camera_names),
                "output": str(args.output.resolve()), "augmentation": augmented.augmentation.as_dict(),
            }
        print(json.dumps(report, sort_keys=True, indent=2, allow_nan=False))
    except (ValueError, TypeError, KeyError, OSError) as exc:
        parser.error(str(exc))
    return 0
