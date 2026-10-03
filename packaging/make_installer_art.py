"""Render the committed NSIS bitmaps from their SVG sources (requires ImageMagick)."""

import argparse
import pathlib
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--magick', default='magick')
    args = parser.parse_args()
    directory = pathlib.Path(__file__).resolve().parent / 'installer'
    for name in ('welcome', 'header'):
        subprocess.run([
            args.magick, '-background', '#fafbfd', str(directory / f'{name}.svg'),
            '-alpha', 'off', '-type', 'TrueColor', f'BMP3:{directory / (name + ".bmp")}',
        ], check=True)


if __name__ == '__main__':
    main()
