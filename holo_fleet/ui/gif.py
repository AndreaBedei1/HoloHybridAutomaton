"""Short GIFs from saved frames (dashboard or chase camera)."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional

from PIL import Image


def make_gif(frames: Iterable[Path], out: Path, width: int = 960, fps: float = 4.0, every: int = 1,
             max_frames: Optional[int] = None, colors: int = 128) -> Optional[Path]:
    paths = sorted(Path(f) for f in frames)[::max(1, every)]
    if max_frames:
        paths = paths[:max_frames]
    if not paths:
        return None
    imgs = []
    for p in paths:
        im = Image.open(p).convert("RGB")
        h = int(im.height * width / im.width)
        imgs.append(im.resize((width, h), Image.LANCZOS).quantize(colors=colors, method=Image.Quantize.MEDIANCUT))
    out.parent.mkdir(parents=True, exist_ok=True)
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0, optimize=True)
    return out
