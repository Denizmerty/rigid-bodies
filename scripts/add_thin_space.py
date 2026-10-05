"""Give the bundled Inter faces a thin space.

Core groups the digits of long numbers with U+2009 THIN SPACE ("299 792 458"), but the subset made
by make_inter_fonts.py mapped neither U+2009 nor U+202F NARROW NO-BREAK SPACE, so the interface drew
grouped numbers with no gap and the scene atlas drew "?". This script appends one empty glyph,
uni2009, with an advance of 0.2 em (410 units on Inter's 2048-unit em) and maps both code points to
it. Existing glyph ids, outlines, bounding boxes and layout tables are unchanged, and the face keeps
its timestamp.

The scene renderer mirrors the advance in its Inter Medium table (advance_units), so a face whose
thin space has another width is reported. A face that already maps both code points is left
untouched, so the script is idempotent; run it on the faces make_inter_fonts.py writes, before
moving them into assets/fonts (see assets/fonts/README.md).

Usage: python scripts/add_thin_space.py [font.ttf ...]
With no arguments it updates Inter-Regular, Inter-Medium and Inter-SemiBold already in assets/fonts.
"""
import pathlib
import sys

from fontTools.ttLib import TTFont
from fontTools.ttLib.tables._g_l_y_f import Glyph

THIN_SPACE = 0x2009
NARROW_NO_BREAK_SPACE = 0x202F
GLYPH_NAME = 'uni2009'
# 0.2 em, the conventional thin space, on Inter's 2048-unit em.
ADVANCE_EM = 0.2
FACES = ('Inter-Regular.ttf', 'Inter-Medium.ttf', 'Inter-SemiBold.ttf')


def unicode_tables(font):
    return [table for table in font['cmap'].tables if table.isUnicode()]


def add_thin_space(font):
    """Map U+2009 and U+202F to a blank glyph. Returns False when the face already maps both."""
    tables = unicode_tables(font)
    codes = (THIN_SPACE, NARROW_NO_BREAK_SPACE)
    if all(code in table.cmap for table in tables for code in codes):
        return False
    advance = round(font['head'].unitsPerEm * ADVANCE_EM)
    cmap = font.getBestCmap()
    name = cmap.get(THIN_SPACE) or cmap.get(NARROW_NO_BREAK_SPACE)
    if name is None:
        # A blank glyph appended after the existing ones, so every glyph id the layout tables use
        # keeps its meaning.
        name = GLYPH_NAME
        glyf = font['glyf']
        glyf[name] = Glyph()
        if name not in font.getGlyphOrder():
            font.setGlyphOrder(font.getGlyphOrder() + [name])
        font['hmtx'][name] = (advance, 0)
    for table in tables:
        for code in codes:
            table.cmap.setdefault(code, name)
    return True


def report(path, font):
    name = font.getBestCmap()[THIN_SPACE]
    advance = font['hmtx'][name][0]
    expected = round(font['head'].unitsPerEm * ADVANCE_EM)
    note = '' if advance == expected else ' (advance_units in the scene renderer assumes %d; update it to match)' % expected
    return '%s: thin space advance %d%s' % (path, advance, note)


def main():
    directory = pathlib.Path(__file__).resolve().parent.parent / 'assets' / 'fonts'
    paths = [pathlib.Path(argument) for argument in sys.argv[1:]] or [directory / face for face in FACES]
    for path in paths:
        # Bounding boxes and the timestamp are kept as generated: the only change is the new glyph.
        font = TTFont(path, recalcBBoxes=False, recalcTimestamp=False)
        # Every table reads the original glyph order before the new glyph is appended to it.
        font.ensureDecompiled()
        if add_thin_space(font):
            font.save(path)
            font = TTFont(path)
            print('wrote', report(path, font))
        else:
            print('unchanged', report(path, font))


if __name__ == '__main__':
    main()
