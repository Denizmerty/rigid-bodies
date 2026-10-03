"""Add the project's derived pictograms to the Phosphor subset.

Phosphor has no long, thin bar, and at menu size its "rectangle" reads exactly like "square", so
the Add menu's Plank needs a glyph of its own. It is derived from the rectangle (U+E3F0): every
outline point is moved outwards horizontally and inwards vertically by a fixed distance, which
keeps Phosphor's stroke width and corner radius while turning the frame into a plank.

Run after regenerating the subset (see assets/fonts/README.md); the script is idempotent.

Usage: python scripts/add_icon_glyphs.py [assets/fonts/Phosphor-Regular-subset.ttf]
"""
import copy
import pathlib
import sys

from fontTools.ttLib import TTFont

SOURCE_CODEPOINT = 0xE3F0
PLANK_CODEPOINT = 0xF101
PLANK_NAME = 'uniF101'
# Font units on a 1024 em: the rectangle spans x 96..928 and y 96..800 around (512, 448).
WIDEN = 32
FLATTEN = 224
CENTRE_X = 512
CENTRE_Y = 448


def add_plank(font):
    cmap = font.getBestCmap()
    source = cmap[SOURCE_CODEPOINT]
    glyf = font['glyf']
    glyph = copy.deepcopy(glyf[source])
    coordinates = glyph.coordinates
    for index in range(len(coordinates)):
        x, y = coordinates[index]
        x = x - WIDEN if x < CENTRE_X else x + WIDEN
        y = y + FLATTEN if y < CENTRE_Y else y - FLATTEN
        coordinates[index] = (x, y)
    glyph.recalcBounds(glyf)
    glyf[PLANK_NAME] = glyph
    if PLANK_NAME not in font.getGlyphOrder():
        font.setGlyphOrder(font.getGlyphOrder() + [PLANK_NAME])
    font['hmtx'][PLANK_NAME] = font['hmtx'][source]
    for table in font['cmap'].tables:
        if table.isUnicode():
            table.cmap[PLANK_CODEPOINT] = PLANK_NAME


def main():
    default = pathlib.Path(__file__).resolve().parent.parent / 'assets' / 'fonts' / 'Phosphor-Regular-subset.ttf'
    path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else default
    font = TTFont(path)
    add_plank(font)
    font.save(path)
    print('wrote', path)


if __name__ == '__main__':
    main()
