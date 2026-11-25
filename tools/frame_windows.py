"""Convert one AI-G 632x464 UYVY frame to a vertically flipped PNG."""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def convert(raw, png):
    data = np.fromfile(raw, dtype=np.uint8)
    if data.size != 464 * 632 * 2:
        raise ValueError(f"Expected one 632x464 UYVY frame ({464 * 632 * 2} bytes), got {data.size}")
    pairs = data.reshape(464, 316, 4).astype(np.int32)
    y = pairs[:, :, 1::2].reshape(464, 632) - 16
    u = pairs[:, :, 0].repeat(2, axis=1) - 128
    v = pairs[:, :, 2].repeat(2, axis=1) - 128
    # BT.601 limited-range YUV, matching the board's SD preview format.
    rgb = np.stack((298*y + 409*v, 298*y - 100*u - 208*v, 298*y + 516*u), axis=2)
    rgb = np.clip((rgb + 128) >> 8, 0, 255).astype(np.uint8)
    Image.fromarray(rgb[::-1]).save(png)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("png", type=Path)
    args = parser.parse_args()
    convert(args.raw, args.png)
