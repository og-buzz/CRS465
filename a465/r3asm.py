"""Minimal assembler for compiled RAPL-3 programs (the CROS controller's bytecode).

    python3 -m a465.r3asm SOURCE.r3s -o PROGRAM          assemble
    python3 -m a465.r3asm --lift PROGRAM > SOURCE.r3s    turn a compiled program back into source

Written 2026-11-24. There is no compiler for this unit; this writes the file format the controller
loads. The proof that it writes that format is tests/test_r3asm.py: every vendor program of the
standard shape (below) is lifted to source with lift(), assembled again, and comes out byte for byte.

Where each part of the format comes from
----------------------------------------
Instruction words, operand fields, the frame initialiser after `enter` and the data stream: from
a465/r3dis.py (its OPS table is the instruction reference; its init_stream() the stream decoder).
The ENCODER of the stream is not in any file; its rule was inferred from the samples and holds for
every data stream and every frame initialiser of all 26 files under sessions/controller-files/ and
controller/backup-2026-11-23/app/test/ (programs and both libraries):

    a relocated pointer is 0xfb + its 4 bytes; a run of 3..255 zero bytes is 0xfd n, a longer run
    0xfe + u16 n (runs stop at a pointer); a run of 1 or 2 zero bytes is written as itself; a byte
    0xfb..0xff is 0xfc + the byte; every other byte stands for itself; the stream ends with 0xff.

A frame initialiser always covers the frame's n-1 words below the frame pointer's word (846 of 846
`enter`s); in a program where they are all zero the source says just `enter n`. Unused operand bytes
are zero in every instruction of every sample, and `call` (0x45) always has 0x81 in byte 1.

The header (a465/crosprog.py has the check value and the library records):

    word 0  magic 0x10adab1f       word 1  0              word 2  build time (.build)
    word 3  0x0081xxxx: the entry  word 4  data words << 16 | code words
    word 6  byte where the code ends (64 + 4 * code words)  word 8  library table
    word 12 names block            word 13 1              word 14 check (words 0..14 sum to 0)
    words 5, 7, 9, 10, 11, 15  0

Word 3 is written as the address of the entry routine in the same form the code uses for a
relocated pointer to the program's own text (`push addr <lib1 text 0x15>` in sbin/fastacid: bit 23
text, bits 16-22 library slot 1, low half the word). That reading explains 0x0081 and the low half
pointing at an `enter` in all 24 programs (0 in the two libraries); that the loader uses it as the
entry is an inference. Code word 0 is always 0x00000044 (`goto 0x0000`); in a library the words
after it are the vector table (`goto routine`, one per routine number: syslib word 0x14 is
`goto 0x012d`, its `delay`), so word 0 reads as vector 0, unused. The assembler writes it itself.

After the code: the data stream, zero bytes up to a 4-byte boundary, the library table (u16 count,
u16 0, then per library u16 name offset, u16 slot, u32 build time, u16 0, u16 that library's own
header word 1 >> 16 (5 for syslib, 2 for robotlib on this unit; it is called `word1` here), u32
signature), then the names block (u32 size padded to 4, the NUL-terminated names in record order,
zero padding). Nothing follows.

Not handled: files whose header words 1, 9, 10 or 13 are not 0, 0, 0, 1 (the libraries, and
bin/pendant, which also exports routines); lift() refuses them.

Source language
---------------
One statement per line; `;` starts a comment; `name:` defines a label (code labels are word
addresses, data labels data-word numbers). Numbers are decimal or 0x hex, signed; 'c' is a
character code; names defined with .equ may be used wherever a number is.

    .build 0x377a89a8                          header word 2
    .entry main                                header word 3
    .lib syslib 2 0x377a8993 0x0a0d4875 5      name, slot, build time, signature, word1
    .equ NAME value
    .data   /   .code                          switch section
    string [CAP] "text"                        data: header word {len, CAP}, CAP bytes (default
                                               CAP = len rounded up to 4); escapes as in Python
    word v, v, ...    zeros N    ptr ADDR      data words; N zero words; a relocated pointer
    .word v, ...                               raw code words
    case TARGET, LO, HI    endcases            a switch table: rows, then the 0 word

ADDR is text:LABEL or data:LABEL (this program, slot 1) or a raw relocatable value. Instructions
are a mnemonic and comma-separated operands in field order (A first, then B and C or the 16-bit
field); frame operands are Ln (fp-n) and An (fp+n); G operands are data labels or Gn; push forms
take an optional trailing `tag N`. See MNEMONICS for each opcode's form and r3dis.OPS for what it
does. `enter N [init HEX]`, `libcall LIB:NUMBER` (number or routine name), `builtin NAME|NUMBER`.
"""
import argparse
import ast
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from a465 import r3dis

HEADER_BYTES = 64
MAGIC = 0x10ADAB1F
WORD0 = 0x00000044
CALL_A = 0x81
TEXT_BIT = 0x800000
OWN_SLOT = 1
PROGRAM_WORD13 = 1

# opcode -> mnemonic. The operand form of each is r3dis.OPS[op][0].
MNEMONICS = {
    0x00: "lor", 0x01: "land", 0x02: "or", 0x03: "xor", 0x04: "and",
    0x05: "eq", 0x06: "eqc", 0x07: "ne", 0x08: "nec", 0x09: "gt", 0x0a: "gtc", 0x0b: "ge", 0x0c: "gec",
    0x0d: "lt", 0x0e: "ltc", 0x0f: "le", 0x10: "lec", 0x11: "shru", 0x12: "shruc", 0x13: "shl",
    0x14: "shlc", 0x15: "add", 0x16: "addc", 0x17: "sub", 0x18: "subc", 0x19: "mul", 0x1a: "mulc",
    0x1b: "div", 0x1c: "divc", 0x1d: "mod", 0x1e: "modc", 0x1f: "neg", 0x20: "not", 0x21: "lnot",
    0x22: "itof", 0x23: "flor", 0x24: "fland", 0x25: "feq", 0x26: "fne", 0x27: "fgt", 0x28: "fge",
    0x29: "flt", 0x2a: "fle", 0x2b: "fadd", 0x2c: "fsub", 0x2d: "fmul", 0x2e: "fdiv", 0x2f: "fneg",
    0x30: "flnot", 0x31: "ftoi", 0x32: "ldlit", 0x33: "ldi", 0x34: "pushlit", 0x35: "pushi",
    0x36: "ldaddr", 0x37: "pushaddr", 0x38: "ld9", 0x39: "push9lit", 0x3a: "ldp", 0x3b: "stp",
    0x3c: "ldp9", 0x3d: "stp9", 0x3e: "if", 0x3f: "ifnot", 0x40: "switch", 0x41: "check",
    0x42: "iff", 0x43: "ifnotf", 0x44: "goto", 0x45: "call", 0x46: "lea", 0x47: "leag",
    0x48: "get", 0x49: "getg", 0x4a: "put", 0x4b: "putg", 0x4c: "get9", 0x4d: "getg9",
    0x4e: "put9", 0x4f: "putg9", 0x50: "push", 0x51: "push9", 0x52: "pop", 0x53: "pop9",
    0x54: "enter", 0x55: "alloc0", 0x56: "libcall", 0x57: "ret", 0x58: "retv", 0x59: "retv9",
    0x5a: "chkpop", 0x5b: "builtin", 0x5c: "memcopy", 0x5d: "memzero", 0x5e: "streq",
    0x5f: "strne", 0x60: "strgt", 0x61: "strge", 0x62: "strlt", 0x63: "strle", 0x64: "strcopy",
    0x65: "ldx", 0x66: "stx", 0x67: "builtinx", 0x68: "pushlea", 0x69: "pushleag", 0x6a: "pushsp",
    0x6b: "checksp", 0x6c: "raise", 0x6d: "try",
    # r3dis calls 0x6e "reraise". The 3.1 interpreter's handler (r3interp 0x5324 -> 0x11c4) pops
    # the six-word record 0x6d pushed and restores the previous handler; homezc runs it on the way
    # out of a guarded call, robotlib's server call runs it in its handler before returning.
    0x6e: "trypop",
    0x6f: "op6f", 0x70: "op70", 0x71: "op71",     # in no sample this module reproduces
}
OPCODES = {name: op for op, name in MNEMONICS.items()}
assert set(MNEMONICS) == set(r3dis.OPS)

BUILTIN_NUMBERS = {name.rstrip("*"): n for n, name in r3dis.BUILTINS.items()}
PUSH_FORMS = ("litP", "lit9P", "iP", "rP", "gP")


class AsmError(ValueError):
    """The source cannot be assembled; the message names the line."""


class LiftError(ValueError):
    """The file is not a program of the shape this module writes."""


@dataclass
class Library:
    name: str
    slot: int
    build_time: int
    signature: int
    word1: int


# Library stamps of THIS controller (CROS 1.16.471): build time = the library's header word 2,
# signature = its word 13, word1 = its word 1 >> 16 (sessions/controller-files/lib/*.r).
SYSLIB = Library("syslib", 2, 0x377A8993, 0x0A0D4875, 5)
ROBOTLIB = Library("robotlib", 3, 0x377A899B, 0xAE952686, 2)


def library_record(lib_file, name, slot):
    """Library(...) for linking to a library file: its build time, signature and word 1 >> 16."""
    w = struct.unpack_from("<16I", lib_file, 0)
    return Library(name, slot, w[2], w[13], w[1] >> 16)


def robotlib_names(path=None):
    """{name: number} from controller/robotlib-functions.txt (the library's own symbol records)."""
    path = Path(path) if path else Path(__file__).resolve().parent.parent / "controller" / "robotlib-functions.txt"
    out = {}
    if path.exists():
        for line in path.read_text().splitlines():
            m = re.match(r"0x([0-9a-f]+)\s+(\w+)", line)
            if m:
                out[m.group(2)] = int(m.group(1), 16)
    return out


LIB_ROUTINES = {"syslib": {name: n for n, name in r3dis.SYSLIB.items()}}


# ----------------------------------------------------------------------------------------------
# The data stream (and frame initialiser) encoding

def encode_stream(data, relocs=()):
    """The run-length stream r3dis.init_stream() decodes back to (data, relocs). See the module doc."""
    relocs = set(relocs)
    out, i = bytearray(), 0
    while i < len(data):
        if i in relocs:
            out.append(0xFB)
            out += data[i:i + 4]
            i += 4
            continue
        b = data[i]
        if b == 0:
            j = i
            while j < len(data) and data[j] == 0 and j not in relocs:
                j += 1
            run = j - i
            while run >= 3:
                n = min(run, 0xFFFF)
                out += bytes((0xFD, n)) if n <= 255 else bytes((0xFE, n & 0xFF, n >> 8))
                run -= n
            out += bytes(run)
            i = j
            continue
        out += bytes((0xFC, b)) if b >= 0xFB else bytes((b,))
        i += 1
    out.append(0xFF)
    return bytes(out)


# ----------------------------------------------------------------------------------------------
# Source parsing

def _scan(text):
    """(index, char, inside_quotes) for each character, honouring backslash escapes in quotes."""
    quote, k = None, 0
    while k < len(text):
        ch = text[k]
        if quote:
            yield k, ch, True
            if ch == "\\" and k + 1 < len(text):
                k += 1
                yield k, text[k], True
            elif ch == quote:
                quote = None
        else:
            if ch in "\"'":
                quote = ch
                yield k, ch, True
            else:
                yield k, ch, False
        k += 1


def _strip_comment(line):
    for k, ch, quoted in _scan(line):
        if ch == ";" and not quoted:
            return line[:k]
    return line


def _split_operands(text):
    out, start = [], 0
    for k, ch, quoted in _scan(text):
        if ch == "," and not quoted:
            out.append(text[start:k].strip())
            start = k + 1
    if text[start:].strip():
        out.append(text[start:].strip())
    return out


def quote_bytes(b):
    """A double-quoted literal for bytes, using only printable ASCII and escapes."""
    out = []
    for c in b:
        ch = chr(c)
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\r":
            out.append("\\r")
        elif ch == "\t":
            out.append("\\t")
        elif 32 <= c < 127:
            out.append(ch)
        else:
            out.append(f"\\x{c:02x}")
    return '"' + "".join(out) + '"'


def _unquote(token, where):
    try:
        value = ast.literal_eval("b" + token)
    except (ValueError, SyntaxError) as e:
        raise AsmError(f"{where}: bad string {token}: {e}") from None
    if not isinstance(value, bytes):
        raise AsmError(f"{where}: bad string {token}")
    return value


@dataclass
class _Stmt:
    kind: str          # "insn", "word", "case", "endcases", "string", "dword", "zeros", "ptr"
    where: str
    name: str = ""
    args: tuple = ()
    tag: object = None
    size: int = 1
    addr: int = 0


class Assembler:
    def __init__(self):
        self.build_time = None
        self.entry = None
        self.libs = []
        self.equ = {}
        self.labels = {}           # name -> (section, address once assembled, statement index)
        self.code, self.data = [], []
        self.lib_routines = dict(LIB_ROUTINES)
        self.lib_routines.setdefault("robotlib", robotlib_names())

    # -- pass 1 -----------------------------------------------------------------------------
    def parse(self, source, name="source"):
        section = "code"
        for n, raw in enumerate(source.splitlines(), 1):
            where = f"{name}:{n}"
            line = _strip_comment(raw).strip()
            while True:
                m = re.match(r"^([A-Za-z_$][\w.$]*):\s*(.*)$", line)
                if not m:
                    break
                label, line = m.group(1), m.group(2)
                if label in self.labels or label in self.equ:
                    raise AsmError(f"{where}: {label} defined twice")
                self.labels[label] = (section, None, len(self.code if section == "code" else self.data))
            if not line:
                continue
            word, rest = (re.split(r"\s+", line, maxsplit=1) + [""])[:2]
            rest = rest.strip()
            if word == ".data":
                section = "data"
            elif word == ".code":
                section = "code"
            elif word == ".build":
                self.build_time = rest
            elif word == ".entry":
                self.entry = rest
            elif word == ".lib":
                p = rest.split()
                if len(p) != 5:
                    raise AsmError(f"{where}: .lib needs name slot build signature word1")
                self.libs.append(Library(p[0], *(int(x, 0) for x in p[1:])))
            elif word == ".equ":
                p = rest.split(None, 1)
                if len(p) != 2:
                    raise AsmError(f"{where}: .equ needs a name and a value")
                self.equ[p[0]] = self._value(p[1], where)
            elif section == "data":
                self.data.append(self._data_stmt(word, rest, where))
            elif word == ".word":
                for v in _split_operands(rest):
                    self.code.append(_Stmt("word", where, args=(v,)))
            elif word == "case":
                ops = _split_operands(rest)
                if len(ops) != 3:
                    raise AsmError(f"{where}: case needs target, lo, hi")
                self.code.append(_Stmt("case", where, args=tuple(ops), size=3))
            elif word == "endcases":
                self.code.append(_Stmt("endcases", where))
            else:
                self.code.append(self._insn_stmt(word, rest, where))
        return self

    def _data_stmt(self, word, rest, where):
        if word == "string":
            m = re.match(r'^(?:([-\w]+)\s+)?(".*")$', rest)
            if not m:
                raise AsmError(f"{where}: string needs [capacity] \"text\"")
            text = _unquote(m.group(2), where)
            cap = self._value(m.group(1), where) if m.group(1) else max(4, (len(text) + 3) // 4 * 4)
            if not 0 < cap < 0x10000 or len(text) > cap:
                raise AsmError(f"{where}: string of {len(text)} bytes does not fit capacity {cap}")
            return _Stmt("string", where, args=(text, cap), size=1 + (cap + 3) // 4)
        if word == "word":
            ops = _split_operands(rest)
            return _Stmt("dword", where, args=tuple(ops), size=len(ops))
        if word == "zeros":
            n = self._value(rest, where)
            return _Stmt("zeros", where, args=(n,), size=n)
        if word == "ptr":
            return _Stmt("ptr", where, args=(rest,))
        raise AsmError(f"{where}: {word!r} is not a data statement")

    def _insn_stmt(self, word, rest, where):
        if word not in OPCODES:
            raise AsmError(f"{where}: unknown mnemonic {word!r}")
        op = OPCODES[word]
        form = r3dis.OPS[op][0]
        tag = None
        m = re.search(r"(?:^|,|\s)tag\s+(\S+)\s*$", rest)
        if m:
            if form not in PUSH_FORMS and op != 0x37:
                raise AsmError(f"{where}: {word} takes no tag")
            tag = m.group(1)
            rest = rest[:m.start()].rstrip().rstrip(",")
        ops = tuple(_split_operands(rest))
        size = {"lit": 2, "litP": 2, "lit9": 10, "lit9P": 10}.get(form, 1)
        st = _Stmt("insn", where, name=word, args=ops, tag=tag, size=size)
        if form == "enter":
            n, init = self._enter_init(ops, where)
            st.args = (n, init)
            st.size = 1 + (len(encode_stream(init)) + 3) // 4
        return st

    def _enter_init(self, ops, where):
        if not ops:
            raise AsmError(f"{where}: enter needs a frame size")
        parts = ops[0].split()
        n = self._value(parts[0], where)
        if not 1 <= n <= 32767:
            raise AsmError(f"{where}: frame size {n} out of range")
        if len(parts) == 1:
            init = bytes(4 * (n - 1))
        elif len(parts) == 3 and parts[1] == "init":
            init = bytes.fromhex(parts[2])
            if len(init) != 4 * (n - 1):
                raise AsmError(f"{where}: an initialiser for a frame of {n} words has {4 * (n - 1)} bytes")
        else:
            raise AsmError(f"{where}: enter N [init HEX]")
        return n, init

    # -- values -----------------------------------------------------------------------------
    def _value(self, text, where):
        text = text.strip()
        if text in self.equ:
            return self.equ[text]
        m = re.match(r"^'(.|\\.)'$", text)
        if m:
            b = _unquote('"' + m.group(1) + '"', where)
            return b[0]
        try:
            return int(text, 0)
        except ValueError:
            pass
        m = re.match(r"^([\w.$]+)\s*([-+])\s*(\S+)$", text)
        if m:
            base = self._value(m.group(1), where)
            off = self._value(m.group(3), where)
            return base + off if m.group(2) == "+" else base - off
        raise AsmError(f"{where}: {text!r} is not a number or a defined name")

    def _label(self, text, section, where):
        text = text.strip()
        if text in self.labels:
            sec, addr, _ = self.labels[text]
            if sec != section:
                raise AsmError(f"{where}: {text} is a {sec} label, not a {section} one")
            return addr
        if section == "data":
            m = re.match(r"^G(\d+)$", text)
            if m:
                return int(m.group(1))
        return self._value(text, where)

    def _frame(self, text, where):
        """fp offset of Ln (-n) or An (+n); the caller checks that it fits its field."""
        m = re.match(r"^([LA])(\d+)$", text.strip())
        if not m:
            raise AsmError(f"{where}: {text!r} is not a frame operand (Ln or An)")
        return -int(m.group(2)) if m.group(1) == "L" else int(m.group(2))

    def _addr(self, text, where):
        """A relocatable pointer value: text:LABEL / data:LABEL (slot 1) or a raw number."""
        text = text.strip()
        m = re.match(r"^(text|data):(.+)$", text)
        if m:
            word = self._label(m.group(2), "code" if m.group(1) == "text" else "data", where)
            if not 0 <= word < 0x10000:
                raise AsmError(f"{where}: address {word} out of range")
            return (TEXT_BIT if m.group(1) == "text" else 0) | OWN_SLOT << 16 | word
        return self._value(text, where) & 0xFFFFFFFF

    # -- pass 2 -----------------------------------------------------------------------------
    def assemble(self):
        addr = 1
        for st in self.code:
            st.addr = addr
            addr += st.size
        code_words = addr
        daddr = 0
        for st in self.data:
            st.addr = daddr
            daddr += st.size
        data_words = daddr
        if code_words > 0xFFFF or data_words > 0xFFFF:
            raise AsmError(f"{code_words} code words / {data_words} data words do not fit 16 bits")
        index = {"code": {k: st.addr for k, st in enumerate(self.code)}, "data": {k: st.addr for k, st in enumerate(self.data)}}
        for label, (sec, _, k) in list(self.labels.items()):
            self.labels[label] = (sec, index[sec].get(k, code_words if sec == "code" else data_words), k)
        if self.build_time is None:
            raise AsmError("no .build")
        build_time = self._value(self.build_time, ".build") & 0xFFFFFFFF
        if not self.entry:
            raise AsmError("no .entry")
        entry = self._label(self.entry, "code", ".entry")

        words = [WORD0]
        for st in self.code:
            words += self._encode(st)
        assert len(words) == code_words, (len(words), code_words)
        data, relocs = bytearray(), []
        for st in self.data:
            data += self._encode_data(st, relocs)
        assert len(data) == 4 * data_words
        return build_file(words, bytes(data), relocs, self.libs, build_time, entry)

    def _encode(self, st):
        w = st.where
        if st.kind == "word":
            return [self._value(st.args[0], w) & 0xFFFFFFFF]
        if st.kind == "case":
            target = self._label(st.args[0], "code", w)
            return [target & 0xFFFF, self._value(st.args[1], w) & 0xFFFFFFFF, self._value(st.args[2], w) & 0xFFFFFFFF]
        if st.kind == "endcases":
            return [0]
        op = OPCODES[st.name]
        form = r3dis.OPS[op][0]
        ops = st.args

        def need(n):
            if len(ops) != n:
                raise AsmError(f"{w}: {st.name} takes {n} operand(s), got {len(ops)}")

        def s8(v, what):
            if not -128 <= v <= 127:
                raise AsmError(f"{w}: {what} {v} does not fit a signed byte")
            return v & 0xFF

        def s16(v, what):
            if not -32768 <= v <= 32767:
                raise AsmError(f"{w}: {what} {v} does not fit a signed 16-bit field")
            return v & 0xFFFF

        def u16(v, what):
            if not 0 <= v <= 0xFFFF:
                raise AsmError(f"{w}: {what} {v} does not fit an unsigned 16-bit field")
            return v

        tag = s8(self._value(st.tag, w), "tag") if st.tag is not None else 0
        A = B = C = 0
        hi = 0
        extra = []
        if form == "ABC":
            need(3)
            A, B, C = (s8(self._frame(x, w), "operand") for x in ops)
        elif form == "ABc":
            need(3)
            A, B = (s8(self._frame(x, w), "operand") for x in ops[:2])
            C = s8(self._value(ops[2], w), "constant")
        elif form == "AB":
            need(2)
            A, B = (s8(self._frame(x, w), "operand") for x in ops)
        elif form == "Ai":
            need(2)
            A = s8(self._frame(ops[0], w), "operand")
            hi = s16(self._value(ops[1], w), "immediate")
        elif form in ("Aj", "switch"):
            need(2)
            A = s8(self._frame(ops[0], w), "operand")
            hi = u16(self._label(ops[1], "code", w), "address")
        elif form == "Ar":
            need(2)
            A = s8(self._frame(ops[0], w), "operand")
            hi = s16(self._frame(ops[1], w), "operand")
        elif form == "Ag":
            need(2)
            A = s8(self._frame(ops[0], w), "operand")
            hi = u16(self._label(ops[1], "data", w), "data word")
        elif form == "lit":
            need(2)
            A = s8(self._frame(ops[0], w), "operand")
            extra = [self._addr(ops[1], w) if op == 0x36 else self._value(ops[1], w) & 0xFFFFFFFF]
        elif form == "litP":
            need(1)
            A = tag
            extra = [self._addr(ops[0], w) if op == 0x37 else self._value(ops[0], w) & 0xFFFFFFFF]
        elif form == "lit9":
            need(10)
            A = s8(self._frame(ops[0], w), "operand")
            extra = [self._value(x, w) & 0xFFFFFFFF for x in ops[1:]]
        elif form == "lit9P":
            need(9)
            A = tag
            extra = [self._value(x, w) & 0xFFFFFFFF for x in ops]
        elif form == "iP":
            need(1)
            A = tag
            hi = s16(self._value(ops[0], w), "immediate")
        elif form == "rP":
            need(1)
            A = tag
            hi = s16(self._frame(ops[0], w), "operand")
        elif form == "gP":
            need(1)
            A = tag
            hi = u16(self._label(ops[0], "data", w), "data word")
        elif form == "r":
            need(1)
            hi = s16(self._frame(ops[0], w), "operand")
        elif form == "r0":
            need(1)
            hi = 0 if ops[0].strip() == "0" else s16(self._frame(ops[0], w), "operand")
        elif form == "n":
            need(1)
            hi = s16(-self._value(ops[0], w), "size")
        elif form == "enter":
            n, init = ops
            hi = s16(-n, "frame size")
            stream = encode_stream(init)
            stream += bytes(-len(stream) % 4)
            extra = list(struct.unpack(f"<{len(stream) // 4}I", stream))
        elif form == "libcall":
            need(1)
            lib, _, number = ops[0].partition(":")
            slots = {x.name: x.slot for x in self.libs}
            if lib not in slots:
                raise AsmError(f"{w}: no .lib {lib!r}")
            A = 0x80 | slots[lib]
            names = self.lib_routines.get(lib, {})
            hi = u16(names[number] if number in names else self._value(number, w), "routine number")
        elif form == "builtin":
            need(1)
            hi = u16(BUILTIN_NUMBERS[ops[0]] if ops[0] in BUILTIN_NUMBERS else self._value(ops[0], w), "builtin number")
        elif form == "none":
            need(0)
        elif form == "call":
            need(1)
            A = CALL_A
            hi = u16(self._label(ops[0], "code", w), "address")
        elif form == "j":
            need(1)
            hi = u16(self._label(ops[0], "code", w), "address")
        else:                                           # pragma: no cover - every form is above
            raise AsmError(f"{w}: form {form} not handled")
        if form not in PUSH_FORMS and op != 0x37 and st.tag is not None:
            raise AsmError(f"{w}: {st.name} takes no tag")
        if hi:
            B, C = hi & 0xFF, hi >> 8
        return [op | (A & 0xFF) << 8 | B << 16 | C << 24] + extra

    def _encode_data(self, st, relocs):
        w = st.where
        if st.kind == "string":
            text, cap = st.args
            body = text + bytes(4 * ((cap + 3) // 4) - len(text))
            return struct.pack("<HH", len(text), cap) + body
        if st.kind == "dword":
            return b"".join(struct.pack("<I", self._value(v, w) & 0xFFFFFFFF) for v in st.args)
        if st.kind == "zeros":
            return bytes(4 * st.args[0])
        if st.kind == "ptr":
            relocs.append(4 * st.addr)
            return struct.pack("<I", self._addr(st.args[0], w))
        raise AsmError(f"{w}: {st.kind}")              # pragma: no cover


def build_file(code_words, data, relocs, libs, build_time, entry):
    """A complete program file from code words (word 0 included), data-segment bytes, the byte
    offsets of its relocated pointers, the libraries it links to, its build time and entry word."""
    code = struct.pack(f"<{len(code_words)}I", *code_words)
    code_end = HEADER_BYTES + len(code)
    stream = encode_stream(data, relocs)
    stream += bytes(-len(stream) % 4)
    table_at = code_end + len(stream)
    names, offsets = bytearray(), []
    for lib in libs:
        offsets.append(len(names))
        names += lib.name.encode("ascii") + b"\0"
    names += bytes(-len(names) % 4)
    table = struct.pack("<HH", len(libs), 0) + b"".join(
        struct.pack("<HHIHHI", off, lib.slot, lib.build_time, 0, lib.word1, lib.signature) for off, lib in zip(offsets, libs))
    names_at = table_at + len(table)
    header = [0] * 16
    header[0] = MAGIC
    header[2] = build_time
    header[3] = TEXT_BIT | OWN_SLOT << 16 | entry
    header[4] = (len(data) // 4) << 16 | len(code_words)
    header[6] = code_end
    header[8] = table_at
    header[12] = names_at
    header[13] = PROGRAM_WORD13
    header[14] = -sum(header[:14]) & 0xFFFFFFFF
    return struct.pack("<16I", *header) + code + stream + table + struct.pack("<I", len(names)) + bytes(names)


def assemble(source, name="source"):
    """Bytes of the program file the source describes."""
    return Assembler().parse(source, name).assemble()


# ----------------------------------------------------------------------------------------------
# Lifting a compiled program back to source

def _s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


def _fr(off):
    return f"L{-off}" if off < 0 else f"A{off}"


def lift(image, name="program"):
    """Source text that assembles back to exactly these bytes, for a program of the standard shape.
    Raises LiftError naming what does not fit."""
    if image[:4] != struct.pack("<I", MAGIC):
        raise LiftError("no 1f ab ad 10 magic")
    w = struct.unpack_from("<16I", image, 0)
    shape = {1: 0, 5: 0, 7: 0, 9: 0, 10: 0, 11: 0, 13: PROGRAM_WORD13, 15: 0}
    odd = [f"word {k} is {w[k]:#x}" for k, v in shape.items() if w[k] != v]
    if w[3] >> 16 != (TEXT_BIT | OWN_SLOT << 16) >> 16:
        odd.append(f"word 3 is {w[3]:#x}")
    if odd:
        raise LiftError(f"{name}: not the standard program shape: " + ", ".join(odd))
    mod = r3dis.Module(image, name)
    if mod.code[0] != WORD0:
        raise LiftError(f"{name}: code word 0 is {mod.code[0]:#x}")
    count = struct.unpack_from("<H", image, w[8])[0]
    libs = []
    for k in range(count):
        off, slot, built, zero, word1, sig = struct.unpack_from("<HHIHHI", image, w[8] + 4 + 16 * k)
        end = image.index(b"\0", w[12] + 4 + off)
        libs.append(Library(image[w[12] + 4 + off:end].decode("ascii"), slot, built, sig, word1))
        if zero:
            raise LiftError(f"{name}: library record {k} has {zero:#x} where 0 is expected")
    insns = r3dis.decode(mod)
    by_addr = {ins.addr: ins for ins in insns}

    def text_target(v):
        return v & TEXT_BIT and (v >> 16) & 0x7F == OWN_SLOT and (v & 0xFFFF) in by_addr

    calls, jumps = {w[3] & 0xFFFF}, set()
    for ins in insns:
        if ins.kind == "data" and ins.addr in by_addr and ins.text.startswith("; switch"):
            for k in range(0, ins.words - 1, 3):
                jumps.add(mod.code[ins.addr + k] & 0xFFFF)
        elif ins.target is not None:
            (calls if ins.op == 0x45 else jumps).add(ins.target)
        if ins.op in (0x36, 0x37) and ins.kind != "data" and text_target(mod.code[ins.addr + 1]):
            calls.add(mod.code[ins.addr + 1] & 0xFFFF)
    data_ptrs = [struct.unpack_from("<I", mod.data, at)[0] for at in mod.data_relocs]
    calls |= {v & 0xFFFF for v in data_ptrs if text_target(v)}

    def label(a):
        if a not in by_addr:
            return f"{a:#x}"
        return f"func_{a:04x}" if a in calls else f"loc_{a:04x}"

    def addr_text(v):
        if text_target(v):
            return "text:" + label(v & 0xFFFF)
        if not v & TEXT_BIT and (v >> 16) & 0x7F == OWN_SLOT:
            return f"data:G{v & 0xFFFF}"
        return f"{v:#010x}"

    out = [f"; {name}: lifted by a465.r3asm.lift", f".build {w[2]:#010x}", f".entry {label(w[3] & 0xFFFF)}"]
    out += [f".lib {lib.name} {lib.slot} {lib.build_time:#010x} {lib.signature:#010x} {lib.word1}" for lib in libs]
    out.append(".data")
    out += _lift_data(mod, addr_text)
    out.append(".code")
    for ins in insns:
        if ins.addr == 0:
            continue
        if ins.addr in calls or ins.addr in jumps:
            out.append(f"{label(ins.addr)}:")
        out += ["    " + line for line in _lift_insn(mod, ins, label, addr_text, name)]
    return "\n".join(out) + "\n"


def _lift_insn(mod, ins, label, addr_text, name):
    wd, op = ins.word, ins.op
    if ins.kind == "data":
        if not ins.text.startswith("; switch"):
            raise LiftError(f"{name}: word {ins.addr:#x} is not an instruction")
        rows = []
        for k in range(0, ins.words - 1, 3):
            tgt, lo, hi = mod.code[ins.addr + k:ins.addr + k + 3]
            if tgt >> 16:
                raise LiftError(f"{name}: switch row at {ins.addr + k:#x} has {tgt:#x}")
            rows.append(f"case {label(tgt)}, {_s32(lo)}, {_s32(hi)}")
        return rows + ["endcases"]
    form = r3dis.OPS[op][0]
    mn = MNEMONICS[op]
    a, b, c = r3dis.s8((wd >> 8) & 0xFF), r3dis.s8((wd >> 16) & 0xFF), r3dis.s8(wd >> 24)
    i16, u16 = r3dis.s16(wd >> 16), wd >> 16
    tag = f" tag {a}" if a else ""
    nxt = mod.code[ins.addr + 1:ins.addr + ins.words]
    if form == "ABC":
        return [f"{mn} {_fr(a)}, {_fr(b)}, {_fr(c)}"]
    if form == "ABc":
        return [f"{mn} {_fr(a)}, {_fr(b)}, {c}"]
    if form == "AB":
        return [f"{mn} {_fr(a)}, {_fr(b)}"]
    if form == "Ai":
        return [f"{mn} {_fr(a)}, {i16}"]
    if form in ("Aj", "switch"):
        return [f"{mn} {_fr(a)}, {label(u16)}"]
    if form == "Ar":
        return [f"{mn} {_fr(a)}, {_fr(i16)}"]
    if form == "Ag":
        return [f"{mn} {_fr(a)}, G{u16}"]
    if form == "lit":
        return [f"{mn} {_fr(a)}, {addr_text(nxt[0]) if op == 0x36 else hex(nxt[0])}"]
    if form == "litP":
        return [f"{mn} {addr_text(nxt[0]) if op == 0x37 else hex(nxt[0])}{tag}"]
    if form == "lit9":
        return [f"{mn} {_fr(a)}, " + ", ".join(hex(x) for x in nxt)]
    if form == "lit9P":
        return [f"{mn} " + ", ".join(hex(x) for x in nxt) + tag]
    if form == "iP":
        return [f"{mn} {i16}{tag}"]
    if form == "rP":
        return [f"{mn} {_fr(i16)}{tag}"]
    if form == "gP":
        return [f"{mn} G{u16}{tag}"]
    if form == "r":
        return [f"{mn} {_fr(i16)}"]
    if form == "r0":
        return [f"{mn} {_fr(i16) if i16 else 0}"]
    if form == "n":
        return [f"{mn} {-i16}"]
    if form == "enter":
        body = r3dis.HEADER_BYTES + 4 * (ins.addr + 1)
        init, relocs, _ = r3dis.init_stream(mod.raw[body:body + 4 * (ins.words - 1)])
        if relocs or len(init) != 4 * (-i16 - 1):
            raise LiftError(f"{name}: enter at {ins.addr:#x} has an initialiser this source cannot say")
        return [f"enter {-i16}" + (f" init {init.hex()}" if any(init) else "")]
    if form == "libcall":
        lib = mod.libs.get(a & 0x7F)
        if lib is None:
            raise LiftError(f"{name}: call into library slot {a & 0x7F}, which the table does not list")
        return [f"{mn} {lib}:{u16:#x}"]
    if form == "builtin":
        bname = r3dis.BUILTINS.get(u16, "").rstrip("*")
        return [f"{mn} {bname or hex(u16)}"]
    if form == "none":
        return [mn]
    if form in ("call", "j"):
        return [f"{mn} {label(u16)}"]
    raise LiftError(f"{name}: form {form}")          # pragma: no cover


def _lift_data(mod, addr_text):
    relocs = set(mod.data_relocs)
    if any(r % 4 for r in relocs):
        raise LiftError("a relocated data pointer is not word-aligned")
    out, n = [], 0
    while n < mod.data_words:
        head = mod.data_word(n)
        length, cap = head & 0xFFFF, head >> 16
        span = 1 + (cap + 3) // 4
        if (cap and length <= cap and n + span <= mod.data_words and 4 * n not in relocs
                and not any(4 * k in relocs for k in range(n + 1, n + span))
                and not any(mod.data[4 * n + 4 + length:4 * (n + span)])):
            text = mod.data[4 * n + 4:4 * n + 4 + length]
            default = max(4, (length + 3) // 4 * 4)
            out.append(f"G{n}: string {'' if cap == default else str(cap) + ' '}{quote_bytes(text)}")
            n += span
            continue
        if 4 * n in relocs:
            out.append(f"G{n}: ptr {addr_text(head)}")
            n += 1
            continue
        if head == 0:
            k = n
            while k < mod.data_words and mod.data_word(k) == 0 and 4 * k not in relocs:
                k += 1
            out.append(f"    zeros {k - n}")
            n = k
            continue
        out.append(f"G{n}: word {head:#x}")
        n += 1
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("file")
    ap.add_argument("-o", "--output", help="program file to write (assembling)")
    ap.add_argument("--lift", action="store_true", help="print source for a compiled program")
    args = ap.parse_args(argv)
    if args.lift:
        sys.stdout.write(lift(r3dis.load(args.file), Path(args.file).name))
        return 0
    image = assemble(Path(args.file).read_text(), args.file)
    if not args.output:
        ap.error("-o PROGRAM is needed when assembling")
    Path(args.output).write_bytes(image)
    print(f"{args.output}: {len(image)} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
