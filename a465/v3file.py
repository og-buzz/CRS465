"""The controller's taught-variable files, `<application>.v3`.

Worked out on 2026-11-26 from the five copies in controller/backup-2026-11-23/app and from the
RAPL-3 bytecode of the controller's own library, vendor/crs/c500-1.16.471/lib/v3access.r. The
layout, with the routine each part comes from, is written up in docs/V3-FILES.md.

In outline (all words are little-endian 32-bit):

    header   magic 0xda1a10ae | word 1 (unknown, 0 in every sample) | time | (record count << 16 |
             save counter) | the application name as a string (a length word, then the characters
             padded with zeros to a whole word)
    records  one per variable, then a zero word ends the table (empty files have no such word);
             bytes after that word are left over from earlier, longer saves and are kept as is
    record   word 0 = length in words (low 16 bits) | kind << 16 | taught << 24
             word 1 = dim1 | dim2 << 16
             the name as a string, then the values

Kinds: 1 int, 2 float, 3 string, 4 ploc (precision location), 5 cloc (cartesian location), 6 gloc.
A location is nine words; word 0 = class, machine type, flags and a check byte that makes the 36
bytes sum to zero. Anything this module cannot interpret is kept as `bytes` and dumped back
unchanged. Adding and deleting variables is not implemented.
"""
import math
import struct
from dataclasses import dataclass, field

from .calfile import COUNTS_PER_DEGREE

MAGIC = 0xDA1A10AE
KIND_NAMES = {1: "int", 2: "float", 3: "string", 4: "ploc", 5: "cloc", 6: "gloc"}
INT, FLOAT, STRING, PLOC, CLOC, GLOC = 1, 2, 3, 4, 5, 6
LOC_KINDS = (PLOC, CLOC, GLOC)
CLASS_CARTESIAN, CLASS_PRECISION = 1, 2
LOC_BYTES = 36
HEADER_FIXED = 16
CARTESIAN_FIELDS = ("x", "y", "z", "yaw", "pitch", "roll")


class V3Error(ValueError):
    """The bytes are not a .v3 file this module understands, or an edit cannot be made safely."""


def _str_words(n):
    """RAPL str_sizeof: words to store a string of n characters."""
    return 1 + ((n + 3) >> 2)


def _check_byte(first3):
    return (-sum(first3)) & 0xFF


@dataclass
class Location:
    """One nine-word location as stored. `words` are words 1..8 verbatim (unsigned)."""
    cls: int
    machine: int
    flags: int
    check: int
    words: tuple

    @classmethod
    def from_bytes(cls, b):
        if len(b) != LOC_BYTES:
            raise V3Error("a location is %d bytes" % LOC_BYTES)
        return cls(b[0], b[1], b[2], b[3], struct.unpack_from("<8I", b, 4))

    def to_bytes(self):
        return bytes((self.cls, self.machine, self.flags, self.check)) + struct.pack("<8I", *self.words)

    @property
    def check_ok(self):
        return sum(self.to_bytes()) % 256 == 0

    def recheck(self):
        self.check = _check_byte((self.cls, self.machine, self.flags) + tuple(self.to_bytes()[4:]))

    @property
    def is_cartesian(self):
        return self.cls == CLASS_CARTESIAN

    @property
    def is_precision(self):
        return self.cls == CLASS_PRECISION

    def floats(self):
        return struct.unpack("<8f", struct.pack("<8I", *self.words))

    def counts(self):
        return struct.unpack("<8i", struct.pack("<8I", *self.words))

    @property
    def cartesian(self):
        """(x, y, z, yaw, pitch, roll) as floats, or None if this is not a cartesian location."""
        return self.floats()[:6] if self.is_cartesian else None

    @property
    def joints_deg(self):
        """Six joint angles in degrees from the motor counts, or None if not a precision location."""
        if not self.is_precision:
            return None
        return tuple(c / k for c, k in zip(self.counts()[:6], COUNTS_PER_DEGREE))

    @property
    def unknown(self):
        """Words 7 and 8 of a cartesian location, 7 and 8 of a precision one (spare axes): kept raw."""
        return tuple(self.words[6:8])

    def describe(self):
        if self.is_cartesian:
            f = self.floats()
            s = " ".join("%s=%.4g" % (n, v) for n, v in zip(CARTESIAN_FIELDS, f[:6]))
            return "cart %s extra=%.4g,%.4g" % (s, f[6], f[7])
        if self.is_precision:
            c = self.counts()
            return "prec joints[deg]=%s counts=%s" % (
                ",".join("%.2f" % j for j in self.joints_deg), ",".join(str(x) for x in c))
        return "class %d words=%s" % (self.cls, ",".join("%08x" % w for w in self.words))


@dataclass
class Variable:
    name: str
    kind: int
    dim1: int
    dim2: int
    taught: int
    data: bytes
    name_pad: bytes = b""

    @property
    def type_name(self):
        return KIND_NAMES[self.kind]

    @property
    def dims(self):
        """Array dimensions; () for a scalar. For a string, dim1 is the character limit, not a dimension."""
        if self.kind == STRING:
            return (self.dim2,) if self.dim2 else ()
        return tuple(d for d in (self.dim1, self.dim2) if d)

    @property
    def elem_bytes(self):
        if self.kind in (INT, FLOAT):
            return 4
        if self.kind == STRING:
            return 4 * _str_words(self.dim1)
        return LOC_BYTES

    @property
    def count(self):
        if self.kind == STRING:
            return self.dim2 or 1
        return (self.dim1 or 1) * (self.dim2 or 1)

    def raw_element(self, k):
        if not 0 <= k < self.count:
            raise V3Error("%s has %d elements; index %d is out of range" % (self.name, self.count, k))
        e = self.elem_bytes
        return self.data[k * e:(k + 1) * e]

    def element(self, k):
        """Decoded element k in storage order: int, float, str or Location."""
        b = self.raw_element(k)
        if self.kind == INT:
            return struct.unpack("<i", b)[0]
        if self.kind == FLOAT:
            return struct.unpack("<f", b)[0]
        if self.kind == STRING:
            return b[4:].split(b"\0", 1)[0].decode("latin-1")
        return Location.from_bytes(b)

    def flat_index(self, idx):
        """Storage index for a user index tuple. Two indices use first-index-fastest order, which is
        inferred from tecan1 in caco2.v3 (see docs/V3-FILES.md), not read from the library."""
        dims = self.dims
        if not dims:
            if idx:
                raise V3Error("%s is not an array" % self.name)
            return 0
        if not idx:
            raise V3Error("%s is an array of %s; give an index" % (self.name, "x".join(map(str, dims))))
        if len(idx) == 1:
            return idx[0]
        if len(idx) != len(dims) or self.kind == STRING:
            raise V3Error("%s takes %d indices" % (self.name, len(dims)))
        i, j = idx
        if not (0 <= i < self.dim1 and 0 <= j < self.dim2):
            raise V3Error("index [%d][%d] outside %s[%d][%d]" % (i, j, self.name, self.dim1, self.dim2))
        return i + self.dim1 * j

    def _store(self, k, b):
        e = self.elem_bytes
        if len(b) != e:
            raise V3Error("internal: element size")
        self.data = self.data[:k * e] + b + self.data[(k + 1) * e:]
        self.taught = 1

    def set_int(self, k, v):
        if self.kind != INT:
            raise V3Error("%s is a %s, not an int" % (self.name, self.type_name))
        if int(v) != v or not -2**31 <= v < 2**31:
            raise V3Error("%r is not a 32-bit integer" % (v,))
        self.raw_element(k)
        self._store(k, struct.pack("<i", int(v)))

    def set_float(self, k, v):
        if self.kind != FLOAT:
            raise V3Error("%s is a %s, not a float" % (self.name, self.type_name))
        if not math.isfinite(v):
            raise V3Error("float value must be finite")
        self.raw_element(k)
        self._store(k, struct.pack("<f", v))

    def _loc_for_edit(self, k, machine):
        if self.kind not in LOC_KINDS:
            raise V3Error("%s is a %s, not a location" % (self.name, self.type_name))
        loc = self.element(k)
        if machine is not None:
            loc.machine = machine
        if loc.machine == 0:
            raise V3Error("%s[%d] has machine type 0 (never taught); pass machine=9 to set it" % (self.name, k))
        return loc

    def set_cartesian(self, k, values, machine=None):
        """x, y, z, yaw, pitch, roll; words 7 and 8 are left alone."""
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise V3Error("a cartesian location needs six finite numbers: x y z yaw pitch roll")
        loc = self._loc_for_edit(k, machine)
        if not loc.is_cartesian:
            raise V3Error("%s[%d] is a precision location; give joint angles" % (self.name, k))
        w = struct.unpack("<6I", struct.pack("<6f", *values))
        loc.words = w + loc.words[6:]
        loc.recheck()
        self._store(k, loc.to_bytes())

    def set_counts(self, k, counts, machine=None):
        """Motor counts for the six arm joints (eight values also set the two spare axes)."""
        if len(counts) not in (6, 8) or any(int(c) != c or not -2**31 <= c < 2**31 for c in counts):
            raise V3Error("a precision location needs 6 (or 8) integer motor counts")
        loc = self._loc_for_edit(k, machine)
        if not loc.is_precision:
            raise V3Error("%s[%d] is a cartesian location; give x y z yaw pitch roll" % (self.name, k))
        w = struct.unpack("<%dI" % len(counts), struct.pack("<%di" % len(counts), *map(int, counts)))
        loc.words = w + loc.words[len(w):]
        loc.recheck()
        self._store(k, loc.to_bytes())

    def set_joints(self, k, degrees, machine=None):
        """Six joint angles in degrees (rounded to whole motor counts)."""
        if len(degrees) != 6 or not all(math.isfinite(v) for v in degrees):
            raise V3Error("a precision location needs six finite joint angles in degrees")
        self.set_counts(k, [round(d * c) for d, c in zip(degrees, COUNTS_PER_DEGREE)], machine)

    def summary(self):
        if self.count == 1 and not self.dims:
            e = self.element(0)
            return e.describe() if isinstance(e, Location) else repr(e)
        return "%d elements" % self.count


@dataclass
class V3Table:
    tag: str
    tag_pad: bytes
    word1: bytes                 # header bytes 4..7, unknown (zero in all samples)
    timestamp: int               # header bytes 8..11, controller clock seconds
    saves: int                   # low 16 bits of header word 3; the controller adds one per full save
    variables: list = field(default_factory=list)
    end_word: bytes = b""        # the zero word that ends the table; empty when the file has none
    slack: bytes = b""           # bytes after the end word (old data), kept verbatim

    @property
    def unknown(self):
        return {"word1": self.word1, "slack": self.slack}

    def find(self, name):
        for v in self.variables:
            if v.name == name:
                return v
        raise V3Error("no variable named %r" % name)


def load(path):
    """Read a .v3 file. `path` may be a filename or a bytes object."""
    if isinstance(path, (bytes, bytearray)):
        return parse(bytes(path))
    with open(path, "rb") as f:
        return parse(f.read())


def parse(d):
    n = len(d)
    if n < 20:
        raise V3Error("file is %d bytes; a .v3 header is at least 20" % n)
    magic, w1, stamp, w3 = struct.unpack_from("<IIII", d, 0)
    if magic != MAGIC:
        raise V3Error("bad magic %08x (expected %08x)" % (magic, MAGIC))
    count_field, saves = w3 >> 16, w3 & 0xFFFF
    tag, tag_pad, o = _read_string(d, HEADER_FIXED, "header")
    table = V3Table(tag, tag_pad, d[4:8], stamp, saves)
    while True:
        if o == n:
            break
        if o + 4 > n:
            raise V3Error("%d stray bytes at offset %d" % (n - o, o))
        w0 = struct.unpack_from("<I", d, o)[0]
        ln = w0 & 0xFFFF
        if ln == 0:
            table.end_word = d[o:o + 4]
            table.slack = d[o + 4:]
            break
        kind, taught = (w0 >> 16) & 0xFF, w0 >> 24
        if ln <= 4 or kind not in KIND_NAMES or taught not in (0, 1):
            raise V3Error("bad record header %08x at offset %d" % (w0, o))
        if o + 4 * ln > n:
            raise V3Error("record at offset %d runs past the end of the file" % o)
        rec = d[o:o + 4 * ln]
        dim1, dim2 = struct.unpack_from("<HH", rec, 4)
        name, pad, p = _read_string(rec, 8, "record at offset %d" % o)
        v = Variable(name, kind, dim1, dim2, taught, rec[p:], pad)
        if dim1 == 0 and dim2 != 0 or kind == STRING and dim1 == 0:
            raise V3Error("%s: impossible dimensions %d,%d" % (name, dim1, dim2))
        if len(v.data) != v.elem_bytes * v.count:
            raise V3Error("%s: record length %d words does not match its type and dimensions" % (name, ln))
        if kind in LOC_KINDS:
            for k in range(v.count):
                if not v.element(k).check_ok:
                    raise V3Error("%s[%d]: location check byte is wrong" % (name, k))
        table.variables.append(v)
        o += 4 * ln
    if count_field != len(table.variables):
        raise V3Error("header says %d records, found %d" % (count_field, len(table.variables)))
    return table


def _read_string(b, o, what):
    if o + 4 > len(b):
        raise V3Error("%s: string runs past the end" % what)
    ln = struct.unpack_from("<I", b, o)[0]
    end = o + 4 + ln
    padded = o + 4 + ((ln + 3) & ~3)
    if ln > 255 or padded > len(b):
        raise V3Error("%s: bad string length %d" % (what, ln))
    return b[o + 4:end].decode("latin-1"), b[end:padded], padded


def _pack_string(s, pad):
    raw = s.encode("latin-1")
    padlen = ((len(raw) + 3) & ~3) - len(raw)
    if len(pad) != padlen:
        pad = b"\0" * padlen
    return struct.pack("<I", len(raw)) + raw + pad


def dump(t):
    """Serialise a table. Lengths and the record count are recomputed; everything else is as held."""
    out = [struct.pack("<I", MAGIC), t.word1, struct.pack("<II", t.timestamp, (len(t.variables) << 16) | t.saves),
           _pack_string(t.tag, t.tag_pad)]
    for v in t.variables:
        name = _pack_string(v.name, v.name_pad)
        body = struct.pack("<HH", v.dim1, v.dim2) + name + v.data
        ln = (4 + len(body)) // 4
        if ln > 0xFFFF:
            raise V3Error("%s is too large for a record (%d words)" % (v.name, ln))
        out.append(struct.pack("<I", ln | (v.kind << 16) | (v.taught << 24)) + body)
    out.append(t.end_word)
    out.append(t.slack)
    return b"".join(out)


def parse_ref(text):
    """'NAME', 'NAME[3]' or 'NAME[2][5]' -> (name, (indices...))."""
    if "[" not in text:
        return text, ()
    name, rest = text.split("[", 1)
    parts = rest.split("]")
    idx = []
    for i, p in enumerate(parts):
        if i == len(parts) - 1:
            if p:
                raise V3Error("cannot read %r" % text)
            break
        p = p[1:] if i else p
        if not p.lstrip("-").isdigit():
            raise V3Error("cannot read index in %r" % text)
        idx.append(int(p))
    if not idx or len(idx) > 2 or any(i < 0 for i in idx):
        raise V3Error("cannot read %r" % text)
    return name, tuple(idx)


def set_value(table, ref, values, counts=False, machine=None):
    """Set one element named by `ref` (see parse_ref) from `values` (numbers, as parsed from text)."""
    name, idx = parse_ref(ref)
    try:
        values = [float(x) for x in values]
    except ValueError as e:
        raise V3Error("not a number: %s" % e)
    v = table.find(name)
    k = v.flat_index(idx)
    v.raw_element(k)
    if v.kind == INT:
        if len(values) != 1 or values[0] != int(values[0]):
            raise V3Error("an int takes one whole number")
        v.set_int(k, int(values[0]))
    elif v.kind == FLOAT:
        if len(values) != 1:
            raise V3Error("a float takes one number")
        v.set_float(k, values[0])
    elif v.kind == STRING:
        raise V3Error("setting strings is not supported: the layout of a string element's length word "
                      "is not established (no sample contains a string)")
    else:
        loc = v.element(k)
        if counts:
            v.set_counts(k, [int(x) for x in values], machine)
        elif loc.is_cartesian:
            v.set_cartesian(k, list(values), machine)
        elif loc.is_precision:
            v.set_joints(k, list(values), machine)
        else:
            raise V3Error("%s[%d] has location class %d, which is not understood" % (name, k, loc.cls))
    return v, k
