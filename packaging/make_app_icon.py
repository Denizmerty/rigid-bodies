"""Build assets/branding/rigid-bodies.ico from the vector artwork in packaging/icon.

The icon is drawn once as icon/master.svg on a 256 px canvas, and again, simplified and fitted to
the pixel grid, for the small sizes Windows shows most often (icon/sNN.svg, drawn on an NN px
grid). Every size in the icon is rendered from its own drawing when one exists and from the master
otherwise. The renderer is librsvg through ImageMagick (https://imagemagick.org), which must be on
PATH as `magick` or be passed with --magick.

The sizes follow Microsoft's guidance for desktop application icons: 16, 24, 32, 48 and 256 at
least, plus the sizes Windows asks for at 125, 150 and 200 percent display scaling, so it never
has to scale another size to fill one. The 256 px image is stored as PNG and the others as 32-bit
bitmaps with an alpha channel, which every Windows icon consumer reads. The output depends only on
the artwork, so rebuilding it from unchanged sources leaves the file byte for byte the same.

Usage: python packaging/make_app_icon.py [--output PATH] [--magick PATH]
"""
import argparse
import io
import os
import re
import struct
import subprocess
import sys
import tempfile

from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCES = os.path.join(ROOT, 'packaging', 'icon')
SIZES = [16, 20, 24, 30, 32, 36, 40, 48, 60, 64, 72, 80, 96, 256]


def svg_width(path):
    with open(path, encoding='utf-8') as source:
        match = re.search(r'<svg[^>]*\swidth="([0-9.]+)', source.read())
    if not match:
        sys.exit(f'{path} has no width attribute')
    return float(match.group(1))


def render(magick, svg, size):
    # librsvg draws the SVG at width * density / 96 pixels.
    density = 96.0 * size / svg_width(svg)
    with tempfile.TemporaryDirectory() as folder:
        target = os.path.join(folder, 'render.png')
        subprocess.run([magick, '-background', 'none', '-density', f'{density:.6f}', svg, '-resize', f'{size}x{size}!', f'png32:{target}'], check=True)
        with Image.open(target) as rendered:
            image = rendered.convert('RGBA')
    if image.size != (size, size):
        sys.exit(f'{svg} rendered at {image.size}, not {size}x{size}')
    return image


def source_for(size):
    variant = os.path.join(SOURCES, f's{size}.svg')
    return variant if os.path.exists(variant) else os.path.join(SOURCES, 'master.svg')


def bitmap(image):
    # A 32-bit DIB: its header gives twice the height, for the colour rows and the mask rows,
    # both stored bottom-up. The 1-bit mask marks transparent pixels for consumers without alpha.
    width, height = image.size
    flipped = image.transpose(Image.FLIP_TOP_BOTTOM)
    colour = flipped.tobytes('raw', 'BGRA')
    alpha = flipped.getchannel('A').tobytes()
    stride = ((width + 31) // 32) * 4
    mask = bytearray(stride * height)
    for y in range(height):
        for x in range(width):
            if alpha[y * width + x] == 0:
                mask[y * stride + x // 8] |= 0x80 >> (x % 8)
    header = struct.pack('<IiiHHIIiiII', 40, width, height * 2, 1, 32, 0, len(colour) + len(mask), 0, 0, 0, 0)
    return header + colour + bytes(mask)


def png(image):
    stream = io.BytesIO()
    image.save(stream, format='PNG', optimize=True)
    return stream.getvalue()


def write_icon(images, output):
    entries = [png(image) if size >= 256 else bitmap(image) for size, image in images]
    directory = struct.pack('<HHH', 0, 1, len(entries))
    offset = 6 + 16 * len(entries)
    for (size, _), data in zip(images, entries):
        dimension = 0 if size >= 256 else size
        directory += struct.pack('<BBBBHHII', dimension, dimension, 0, 0, 1, 32, len(data), offset)
        offset += len(data)
    with open(output, 'wb') as target:
        target.write(directory)
        for data in entries:
            target.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', default=os.path.join(ROOT, 'assets', 'branding', 'rigid-bodies.ico'))
    parser.add_argument('--magick', default='magick')
    arguments = parser.parse_args()
    images = []
    for size in SIZES:
        source = source_for(size)
        images.append((size, render(arguments.magick, source, size)))
        print(f'{size:>3} px from {os.path.relpath(source, ROOT)}')
    write_icon(images, arguments.output)
    with Image.open(arguments.output) as written:
        sizes = sorted(written.info.get('sizes', []))
    if sorted((size, size) for size in SIZES) != sizes:
        sys.exit(f'{arguments.output} reads back with sizes {sizes}')
    print(f'wrote {os.path.relpath(arguments.output, ROOT)} ({os.path.getsize(arguments.output)} bytes)')


if __name__ == '__main__':
    main()
