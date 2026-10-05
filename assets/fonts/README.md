# Interface fonts

Inter Regular, Medium and SemiBold are bundled for the RmlUi interface and the scene labels. Inter
is by Rasmus Andersson and the Inter Project Authors and is distributed under the SIL Open Font
License 1.1; see OFL-Inter.txt for copyright and redistribution terms. Inter has no Reserved Font
Name.

Source: `https://github.com/google/fonts/blob/main/ofl/inter/Inter[opsz,wght].ttf` (Inter 4.001).
The committed static faces are generated from that variable font with
`python scripts/make_inter_fonts.py "Inter[opsz,wght].ttf"` (FontTools 4.53 or newer). The script
generates the text optical size at weights 400, 500 and 600, uses fixed-width digits to keep
changing measurements from shifting, and limits the character set to Latin, Greek,
punctuation, arrows, mathematical operators and common symbols. It writes Inter-Regular.ttf,
Inter-Medium.ttf and Inter-SemiBold.ttf to the working directory. In that same directory run
`python scripts/add_thin_space.py Inter-Regular.ttf Inter-Medium.ttf Inter-SemiBold.ttf`, then
move the three faces into this folder. The script gives each face a blank thin space (advance
0.2 em) for U+2009 and U+202F when the face has none, so numbers grouped as "299 792 458" keep
their gaps in the interface and on the stage. It leaves a face that already maps both code points
untouched, and reports a thin space whose width differs from the 410 units the scene renderer
assumes. Run with no arguments, it checks and patches the faces already in this folder. Runtime
builds never download or subset fonts.

Phosphor Regular 2.1.2 supplies the interface icons and is distributed under the MIT
license in `../licenses/Phosphor-MIT.txt`. The pinned source is
`https://github.com/phosphor-icons/web/blob/v2.1.2/src/regular/Phosphor.ttf`. The committed file is
named `Phosphor-Regular-subset.ttf`. Regenerate it with FontTools 4.53 or newer from the pinned Regular
font using `pyftsubset Phosphor.ttf --unicodes=U+E014,U+E036,U+E038,U+E08A,U+E08C,U+E0A4,U+E10E,U+E128,U+E12A,U+E136,U+E138,U+E13A,U+E13C,U+E154,U+E182,U+E184,U+E18A,U+E18C,U+E1CA,U+E1D0,U+E1D6,U+E1DA,U+E1DC,U+E1FE,U+E208,U+E220,U+E224,U+E246,U+E248,U+E256,U+E272,U+E28E,U+E296,U+E298,U+E2CA,U+E2CE,U+E2D8,U+E2DC,U+E2DE,U+E2E2,U+E2F0,U+E2FA,U+E306,U+E30C,U+E32A,U+E32E,U+E330,U+E39C,U+E39E,U+E3B4,U+E3D0,U+E3D4,U+E3D6,U+E3E2,U+E3E8,U+E3EE,U+E3F0,U+E42A,U+E434,U+E45E,U+E46A,U+E46C,U+E472,U+E476,U+E47C,U+E492,U+E4A8,U+E4E0,U+E4E2,U+E4F6,U+E4F8,U+E57C,U+E5A4,U+E5A6,U+E5D2,U+E628,U+E62C,U+E656,U+E680,U+E682,U+E6A2,U+E6A6,U+E6B8,U+E6D0,U+E6D2,U+E6EE,U+E746,U+E750,U+E758,U+E7BC,U+EADC,U+EAF0,U+EB00,U+EC24 --layout-features='*' --name-IDs='*' --name-legacy --name-languages='*'`, then run
`python scripts/add_icon_glyphs.py`, which adds the project's Plank pictogram at U+F101 by
reshaping the rectangle glyph into a long, thin bar with the same stroke and corners. The codepoints
are the semantic glyphs listed in `rigidbodies/ui/icons.hpp`. Runtime builds never download or
subset it.
