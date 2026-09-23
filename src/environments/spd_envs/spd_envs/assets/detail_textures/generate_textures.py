"""Offline authoring tool for SPD's original surface artwork (requires Pillow/numpy).

Run this file to reproduce the checked-in textures. No fonts, downloads, or
third-party image assets are used; the lettering is hand-authored stroke art.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import string

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
REVISION = "spd-original-surface-art-2"
LETTER_FACE_RGB = (248, 238, 213)
# Repeated in alphabet order so letter identity has a stable, reproducible ink.
LETTER_INKS = (
    ("red", (209, 39, 43)),
    ("blue", (30, 88, 191)),
    ("green", (22, 135, 65)),
    ("orange", (222, 104, 15)),
    ("violet", (128, 54, 171)),
    ("teal", (0, 132, 147)),
    ("magenta", (191, 40, 116)),
    ("gold", (173, 126, 0)),
)
# Normalized five-by-seven drawing coordinates, not a bitmap font.
STROKES = {
    "A": [[(0, 7), (2.5, 0), (5, 7)], [(1, 4.5), (4, 4.5)]],
    "B": [[(0, 7), (0, 0), (3.4, 0), (5, 1), (5, 2.4), (3.4, 3.5), (0, 3.5)], [(3.4, 3.5), (5, 4.5), (5, 6), (3.4, 7), (0, 7)]],
    "C": [[(5, 1), (3.6, 0), (1.4, 0), (0, 1.4), (0, 5.6), (1.4, 7), (3.6, 7), (5, 6)]],
    "D": [[(0, 7), (0, 0), (3, 0), (5, 1.8), (5, 5.2), (3, 7), (0, 7)]],
    "E": [[(5, 0), (0, 0), (0, 7), (5, 7)], [(0, 3.5), (4, 3.5)]],
    "F": [[(5, 0), (0, 0), (0, 7)], [(0, 3.5), (4, 3.5)]],
    "G": [[(5, 1), (3.6, 0), (1.4, 0), (0, 1.4), (0, 5.6), (1.4, 7), (3.6, 7), (5, 5.6), (5, 3.8), (2.8, 3.8)]],
    "H": [[(0, 0), (0, 7)], [(5, 0), (5, 7)], [(0, 3.5), (5, 3.5)]],
    "I": [[(0.7, 0), (4.3, 0)], [(2.5, 0), (2.5, 7)], [(0.7, 7), (4.3, 7)]],
    "J": [[(0.6, 0), (5, 0), (5, 5.6), (3.6, 7), (1.4, 7), (0, 5.6)]],
    "K": [[(0, 0), (0, 7)], [(5, 0), (0, 3.8), (5, 7)]],
    "L": [[(0, 0), (0, 7), (5, 7)]],
    "M": [[(0, 7), (0, 0), (2.5, 3.8), (5, 0), (5, 7)]],
    "N": [[(0, 7), (0, 0), (5, 7), (5, 0)]],
    "O": [[(1.4, 0), (3.6, 0), (5, 1.4), (5, 5.6), (3.6, 7), (1.4, 7), (0, 5.6), (0, 1.4), (1.4, 0)]],
    "P": [[(0, 7), (0, 0), (3.6, 0), (5, 1.3), (5, 2.5), (3.6, 3.8), (0, 3.8)]],
    "Q": [[(1.4, 0), (3.6, 0), (5, 1.4), (5, 5.2), (3.6, 6.6), (1.4, 6.6), (0, 5.2), (0, 1.4), (1.4, 0)], [(3, 4.8), (5.4, 7.3)]],
    "R": [[(0, 7), (0, 0), (3.6, 0), (5, 1.3), (5, 2.5), (3.6, 3.8), (0, 3.8)], [(2.5, 3.8), (5, 7)]],
    "S": [[(5, 1), (3.6, 0), (1.4, 0), (0, 1.2), (0, 2.4), (1.4, 3.5), (3.6, 3.5), (5, 4.6), (5, 5.8), (3.6, 7), (1.4, 7), (0, 6)]],
    "T": [[(0, 0), (5, 0)], [(2.5, 0), (2.5, 7)]],
    "U": [[(0, 0), (0, 5.6), (1.4, 7), (3.6, 7), (5, 5.6), (5, 0)]],
    "V": [[(0, 0), (2.5, 7), (5, 0)]],
    "W": [[(0, 0), (1, 7), (2.5, 3.5), (4, 7), (5, 0)]],
    "X": [[(0, 0), (5, 7)], [(5, 0), (0, 7)]],
    "Y": [[(0, 0), (2.5, 3.5), (5, 0)], [(2.5, 3.5), (2.5, 7)]],
    "Z": [[(0, 0), (5, 0), (0, 7), (5, 7)]],
}


def main() -> None:
    rng = np.random.default_rng(60219)
    y, x = np.mgrid[0:512, 0:512] / 512
    files = []
    for index, base in enumerate(((199, 157, 103), (172, 123, 76), (218, 188, 143))):
        phase = y * 42 + .48 * np.sin(x * 10 + index) + .18 * np.sin(x * 23 + y * 4)
        grain = (np.sin(phase * np.pi * 2) * 3.2
                 + np.sin(phase * np.pi * 6 + .3) * 1.8
                 - np.maximum(0, np.sin(phase * np.pi * 2)) ** 18 * 9
                 + rng.normal(0, .9, x.shape))
        image = np.clip(np.asarray(base) + grain[..., None] * (1, .85, .65), 0, 255).astype(np.uint8)
        path = ROOT / f"wood_{index}.png"
        Image.fromarray(image).save(path)
        files.append(path)
    for index, base in enumerate(((237, 232, 217), (227, 234, 229))):
        noise = rng.normal(0, 1.4, x.shape)
        speckles = rng.random(x.shape) > .996
        noise[speckles] -= rng.uniform(12, 28, speckles.sum())
        image = np.clip(np.asarray(base) + noise[..., None], 0, 255).astype(np.uint8)
        path = ROOT / f"glaze_{index}.png"
        Image.fromarray(image).save(path)
        files.append(path)
    # Supersampling produces smooth, readable letter strokes without micro-geoms.
    for index, letter in enumerate(string.ascii_uppercase):
        image = Image.new("RGB", (1024, 1024), LETTER_FACE_RGB)
        draw = ImageDraw.Draw(image)
        _, ink = LETTER_INKS[index % len(LETTER_INKS)]
        draw.rounded_rectangle((45, 45, 979, 979), radius=50, outline=ink, width=24)
        for stroke in STROKES[letter]:
            points = [(220 + px * 116, 135 + py * 105) for px, py in stroke]
            draw.line(points, fill=ink, width=66, joint="curve")
            for px, py in points:
                draw.ellipse((px - 33, py - 33, px + 33, py + 33), fill=ink)
        path = ROOT / f"letter_{letter}.png"
        image.resize((256, 256), Image.Resampling.LANCZOS).save(path)
        files.append(path)
    manifest = {
        "revision": REVISION,
        "provenance": "Original SPD procedural wood/glaze and hand-authored A-Z stroke artwork; no external image or font assets.",
        "generator": {"file": Path(__file__).name, "sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
        "letter_art": {
            "face_rgb": list(LETTER_FACE_RGB),
            "ink_palette": [{"name": name, "rgb": list(rgb)} for name, rgb in LETTER_INKS],
            "letters": {
                letter: {
                    "ink_name": LETTER_INKS[index % len(LETTER_INKS)][0],
                    "ink_rgb": list(LETTER_INKS[index % len(LETTER_INKS)][1]),
                    "file": f"letter_{letter}.png",
                    "source": "hand-authored-strokes",
                }
                for index, letter in enumerate(string.ascii_uppercase)
            },
        },
        "files": [{"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} for path in files],
    }
    (ROOT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
