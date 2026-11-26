# Taught-variable files (.v3)

2026-11-26. Nothing written by `scripts/a465-v3` has been loaded on the controller yet. The first
edited file should be loaded in `ash` under supervision (arm stopped, someone at the E-stop) before
any program uses it.

The code is `a465/v3file.py`; the command line is `scripts/a465-v3`; tests are `tests/test_v3file.py`.

## Where this comes from

- S1 to S5: the five real files in `controller/backup-2026-11-23/app/`: `test/test.v3` (136 bytes),
  `caco/caco.v3` (24), `caco/caco2.v3` (30392), `2/2.v3` (24), `SampleProgram/SampleProgram.v3` (48).
- LIB: the controller's own library, `vendor/crs/c500-1.16.471/lib/v3access.r`, read with
  `python3 -m a465.r3dis`. Routine addresses below are the `func_XXXX` labels of that listing.
- MAN: `docs/text/ApplicationShellASH.txt`, `docs/text/RAPL-3LanguageReferenceGuide.txt`,
  `sessions/controller-files/lib/ash.help`.

All words are little-endian 32-bit.

## Layout

### Header (20 bytes, then the name)

| Bytes | Meaning | Source |
|---|---|---|
| 0..3 | magic `ae 10 1a da` (0xda1a10ae) | LIB func_0016 builds it, func_0157 compares it; every sample |
| 4..7 | unknown; zero in all five samples; never touched by the library routines read | S1 to S5 |
| 8..11 | time in controller-clock seconds; the writer sets it to the current time, or old value + 1 if that is the same second | LIB func_02bd |
| 12..13 | save counter: the writer adds 1 each full save (0 in a new file, 2 in test.v3, 29 in caco2.v3) | LIB func_02bd, S3 |
| 14..15 | number of records; the loader refuses a file where this is neither 0 nor the true count (this module requires equality) | LIB func_0157, func_02bd |
| 16.. | the application name as a RAPL string: a length word, the characters, zero padding to a whole word. Files in directory `SampleProgram` carry `Samp`, so it is not always the directory name | LIB func_040f writes it with `str_sizeof`; S1 to S5 |

### Records

Records follow the name, one per variable, in the order the controller keeps them (sorted by
name in all samples; func_0020 sorts them after loading).

| Word | Meaning | Source |
|---|---|---|
| 0 | bits 0..15 length of the whole record in words; bits 16..23 kind; bit 24 taught flag (must be 0 or 1 on load). The controller sets bit 31 in memory for "changed" and clears it before writing | LIB func_0065 (load checks), func_05e0 (sets 0x81000000 when a value is taught), func_02bd |
| 1 | dim1 in the low 16 bits, dim2 in the high 16 bits (0 means absent) | LIB func_04ec, func_05b8 |
| 2.. | the variable name, as a string (length word, characters, padding) | LIB func_04ec, S1 to S5 |
| then | the values, `count` elements of the type's size | LIB func_04ec |

Kinds (LIB func_04ec accepts 1..6; the names are the base types of `new` in MAN, ash.help line 82):
1 int, 2 float, 3 string, 4 ploc (precision location), 5 cloc (cartesian location), 6 gloc.

Element sizes, from func_04ec: int and float one word; a location nine words (36 bytes); a string
`str_sizeof(limit)` words, where `str_sizeof(n) = 1 + ((n + 3) >> 2)` (MAN, RAPL reference). For a
string, dim1 is the character limit (rounded up to a multiple of 4 by the library) and dim2 the array
length. For everything else the element count is `(dim1 or 1) * (dim2 or 1)`.
The record length field and these sizes agree for every record in all five files; the loader checks it.

### Locations

Nine words. Word 0 is four bytes: class (1 cartesian, 2 precision), machine type (9 on this arm),
flags (1 in all samples), check byte, chosen so the 36 bytes sum to zero modulo 256 (every location in
the samples satisfies it; the loader here refuses a file where one does not). Kind 5 variables hold
class 1 elements, kind 4 class 2 (LIB func_04ec, `loc_class_set`).

- Precision: words 1..8 are signed motor counts; the first six are the arm joints. Degrees are
  counts divided by `COUNTS_PER_DEGREE` in `a465/calfile.py`.
- Cartesian: words 1..6 are floats x, y, z, yaw, pitch, roll (naming from the task brief and
  repo notes, not from this library). Words 7 and 8 are also stored.

### End of table and slack

After the last record the writer writes one zero word (LIB func_02bd). Files with no variables (S2, S4)
have no such word. Bytes after the zero word are not read by the loader; in caco2.v3 there are 10092 of
them (0x4f4c to the end), plainly the tail of an earlier, longer save (they contain partial records
such as `passthrough1`, `refer1`, `tecan2` again). They are kept verbatim as `table.slack`.

## What the five files contain

- test.v3: 2 ploc scalars, `here2` and `mt`, each a taught precision location, machine type 9.
- SampleProgram.v3: 1 int, `I`, untaught, value 0.
- caco.v3, 2.v3: empty (24 bytes: header and name only).
- caco2.v3 (application `caco`, 29 saves): 17 variables: 1 ploc array (`carousel1[8]`, taught
  precision positions), 1 int (`close`, untaught, 0), 15 cloc: the arrays `incubator1[3][96]`,
  `passthrough1[4][32]`, `refer1[3][16]`, `scanner1[4][1]`, `sealer1[3][1]`, `tecan1[3][10]`,
  `tecan2[3][10]`, six small `safe_*` ones, and the scalars `s_inc`, `safe_passthrough1`, `safe_sealer1`.
  It looks like a lab-automation cell (carousel, incubator, sealer, scanner, liquid handler positions).
- Totals over the five files, per variable: ploc 3 (carousel1, here2, mt), cloc 15, int 2
  (close, I), float 0, string 0, gloc 0.

## What is not established

- Header bytes 4..7 (all zero): meaning unknown, kept as `table.word1`.
- Floats, strings and glocs (kinds 2, 3, 6) occur in none of the samples. Sizes follow the library's
  allocation routine, but the byte format inside a float (assumed IEEE single, as for locations) and
  inside a string element (a length word then characters is assumed; the high half of the length word
  may hold the limit) is unverified. Strings are therefore shown read-only; `set` refuses them.
- Cartesian words 7 and 8: word 8 is 0 in nearly all elements; word 7 is a non-zero float in the
  range 40 to 105 in every sample. Its meaning (spare axis? a fourth angle?) is unknown. The
  orientation fields 4..6 are also doubtful: in `s_inc` they read 0, 0.27, 0.78 while word 7 reads 91.16,
  whereas in `tecan1` word 4 is near -90. The names yaw, pitch, roll are the brief's, not proved. Units of
  x, y, z are not known (values such as 10.8 or 16.7 suggest the `units` setting is inches).
  Spare precision axes (words 7, 8) are stored and edited raw.
- Two-index arrays: the library does not fix the order. The data in `tecan1[3][10]` repeat with period
  3 in storage order (three positions of one tube, then the next tube), so the first index appears to
  vary fastest, storage index = i + dim1 * j. This is an inference from one array. `NAME[k]` with one
  index always means storage position k; `show` prints both.
- The bit 24 "taught" flag: set when a value is taught (func_05e0). `set` sets it, as `here` would.
  It is shown as `(untaught)` in `list`.
- Adding, deleting or renaming variables is not implemented. The add routine (func_04ec) is fully
  readable, but strings are not verified and ordering, timestamp and counter updates would be guesses.

## Using the script

```
scripts/a465-v3 list FILE [--json]
scripts/a465-v3 show FILE NAME[index]
scripts/a465-v3 set FILE NAME[index] VALUES... -o OUT      # or --in-place; never overwrites otherwise
scripts/a465-v3 check FILE                                 # parse, re-dump, compare
```

`set` values: one number for an int or float; `x y z yaw pitch roll` for a cartesian location; six joint
angles in degrees for a precision location (rounded to whole counts), or with `--counts` six or eight
motor counts. Words 7 and 8 of a location are left as they were. An element whose machine type is 0
(never taught) needs `--machine 9`. Each edit sets the taught flag and recomputes the location's check
byte; the header (time, save counter) and all other bytes are unchanged, and the record count and
lengths are recomputed. Files are written through a temporary file and renamed.

Copy files off the controller with `scripts/a465-getfile`; this script never talks to it.

## Checked against the controller, 2026-11-26

Two read-only checks in `ash test` on the controller (CROS 1.16.471), and one small write to the test
application's own file:

- Precision locations. `print here2` gave `ploc[9,1,(-13600,13233,795,25895,-8336,28163,0,0)]` and
  `print mt` gave `ploc[9,1,(-13600,13265,776,25895,-8336,2493,0,0)]`: the same eight motor counts
  `scripts/a465-v3 list` decodes from `test.v3`.
- Cartesian locations. With the arm at its zero pose, `new _zc` and `here _zc` created a cartesian
  variable (ash's `_` prefix makes a cloc; the stored name is `zc`). `print _zc` gave
  `cloc[9,17,(0.0544844,3.42336e-06,1041.40,0.00360000,-89.9892,0.00000,0.00000,0.00000)]`, and `here`
  showed WORLD X 0.054, Y 0.000, Z 1041.400 mm, Z-Rot 0.004, Y-Rot -89.989, X-Rot 0.000 degrees. The file
  copied back afterwards (188 bytes) decodes to x 0.05448, y 3.4e-06, z 1041, yaw 0.0036, pitch -89.99,
  roll 0 and re-dumps byte for byte.

So words 1 to 6 of a cartesian location are X, Y, Z, then the rotations about Z, Y and X (yaw, pitch,
roll), in the controller's units at the time of saving (now metric: mm and degrees); words 7 and 8 are
axes 7 and 8 (the track), zero on this cell. The numbers in the old `caco2.v3` (x about 17, z about 12,
word 7 about 40 to 105) fit a cell that was set to inches and had a track; the file does not record
its units. In `print`'s output the first number (9) and the second (1 for a ploc, 17 for this cloc) come
from the location's first word (machine type and flags).

Still not checked on the controller: a file edited on the PC and sent back.
