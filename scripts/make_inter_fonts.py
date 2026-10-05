"""Build the bundled Inter interface faces from the official variable font.

Steps: instance the variable font at the text optical size, make tabular figures the default by
remapping the digit code points to their `tnum` alternates, and subset to the scripts the
interface and scene labels use. Output names stay "Inter" with the instance's weight class.

Usage: python scripts/make_inter_fonts.py <path to Inter[opsz,wght].ttf>
Writes Inter-Regular.ttf, Inter-Medium.ttf and Inter-SemiBold.ttf to the working directory.
Then, in the same directory, run
python scripts/add_thin_space.py Inter-Regular.ttf Inter-Medium.ttf Inter-SemiBold.ttf
and move the three faces into assets/fonts (see assets/fonts/README.md).
"""
import sys
from fontTools.ttLib import TTFont
from fontTools.varLib import instancer
from fontTools import subset

SOURCE = sys.argv[1] if len(sys.argv) > 1 else 'Inter-var.ttf'
WEIGHTS = {400: 'Regular', 500: 'Medium', 600: 'SemiBold'}

UNICODES = (
    list(range(0x20, 0x7F)) + list(range(0xA0, 0x180)) + list(range(0x370, 0x400))
    + list(range(0x2010, 0x2028)) + list(range(0x2030, 0x2045)) + list(range(0x2070, 0x20A0))
    + list(range(0x2100, 0x2150)) + list(range(0x2190, 0x2200)) + list(range(0x2200, 0x2300))
    + list(range(0x25A0, 0x2600)) + list(range(0x2600, 0x2700))
    # Thin and narrow no-break spaces: Core groups long numbers with U+2009.
    + [0x2009, 0x202F]
)


def tnum_map(font):
    gsub = font['GSUB'].table
    lookup_indices = set()
    for record in gsub.FeatureList.FeatureRecord:
        if record.FeatureTag == 'tnum':
            lookup_indices.update(record.Feature.LookupListIndex)
    mapping = {}
    for index in lookup_indices:
        lookup = gsub.LookupList.Lookup[index]
        for sub in lookup.SubTable:
            if lookup.LookupType == 7:
                sub = sub.ExtSubTable
            if hasattr(sub, 'mapping'):
                mapping.update(sub.mapping)
    return mapping


def main():
    for weight, style in WEIGHTS.items():
        font = TTFont(SOURCE)
        font = instancer.instantiateVariableFont(font, {'wght': weight, 'opsz': 14}, updateFontNames=False)
        mapping = tnum_map(font)
        changed = 0
        for table in font['cmap'].tables:
            if not table.isUnicode():
                continue
            for code in range(0x30, 0x3A):
                glyph = table.cmap.get(code)
                if glyph in mapping:
                    table.cmap[code] = mapping[glyph]
                    changed += 1
        font['OS/2'].usWeightClass = weight
        name = font['name']
        for record in list(name.names):
            if record.nameID in (1, 16):
                record.string = 'Inter'
            elif record.nameID in (2, 17):
                record.string = 'Regular' if weight == 400 else style
            elif record.nameID == 4:
                record.string = 'Inter ' + style
            elif record.nameID == 6:
                record.string = 'Inter-' + style
        # Medium and SemiBold are distinct weights of one family, never synthesised bold.
        font['OS/2'].fsSelection &= ~(1 << 5)
        font['head'].macStyle &= ~1
        if weight == 400:
            font['OS/2'].fsSelection |= 1 << 6
        else:
            font['OS/2'].fsSelection &= ~(1 << 6)
        options = subset.Options()
        options.layout_features = ['kern', 'liga', 'calt', 'ccmp', 'locl', 'mark', 'mkmk', 'case', 'frac', 'sups', 'subs']
        options.name_IDs = ['*']
        options.name_legacy = True
        options.name_languages = ['*']
        options.notdef_outline = True
        options.glyph_names = False
        options.hinting = True
        subsetter = subset.Subsetter(options)
        subsetter.populate(unicodes=UNICODES)
        subsetter.subset(font)
        out = 'Inter-%s.ttf' % style
        font.save(out)
        check = TTFont(out)
        cmap = check.getBestCmap()
        advances = [check['hmtx'][cmap[ord(c)]][0] for c in '0123456789']
        print(out, 'weight', check['OS/2'].usWeightClass, 'digits remapped', changed, 'digit advances', sorted(set(advances)), 'glyphs', len(check.getGlyphOrder()))


if __name__ == '__main__':
    sys.exit(main())
