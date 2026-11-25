"""Disassembler for compiled RAPL-3 programs and libraries (the CROS controller's bytecode).

    python3 -m a465.r3dis sessions/controller-files/sbin/fastacid
    python3 -m a465.r3dis --syslib sessions/controller-files/lib/syslib.r sessions/controller-files/bin/cp

Worked out on 2026-11-24. The instruction set was read from the x86 code of the bytecode interpreter
(/sbin/r3interp of the 2001 CROS 3.1 firmware: its dispatch loop and the handler of every opcode), not
guessed from samples; it was then checked against the two stock example programs and their sources
(app/test/fib and sieve) and against bin/cp and bin/more of this unit (CROS 1.16.471, 1999). That
this unit's interpreter executes the same instruction set is an inference, supported by the stock
programs running on both and by this unit's syslib declaring every builtin under the same number
the 3.1 interpreter's own table gives it.

The machine
-----------
Every instruction is one little-endian 32-bit word: byte 0 the opcode, byte 1 operand A (signed), byte 2
operand B (signed), byte 3 operand C (signed); or A and a 16-bit immediate in bytes 2-3. A, B and C name
words of the current stack frame by their signed offset from the frame pointer: here L4 is the word at
fp-4 (a local or temporary) and A3 the word at fp+3. A call pushes its arguments last first, then the
number of argument words; the callee's frame then holds fp+0 the caller's frame, fp+1 the return
address, fp+2 that word count, and its first argument at fp+3 (A3), the second at A4, and so on.
G<n> is word n of the module's data segment, which starts where the code ends in the file and is
built from the run-length stream stored there (see data_segment()). Code addresses are word indexes:
word 0 is file byte 64.

Header word 4 holds the number of code words (low half) and data words (high half): on every program
and library of this unit the code then ends exactly at header word 6 and the data stream expands to
exactly that many words, and every code word decodes as an instruction or as data owned by one.

A pointer is (segment << 16) | word: segment 1 is the stack, 2 the heap, then data and text segments
of each loaded module in turn. A string is a word {u16 length, u16 capacity} followed by its bytes.

Builtins (opcodes 0x5b and 0x67) are the interpreter's own routines, numbered as in BUILTINS; 0x5b
leaves the routine's result on the stack, 0x67 pops it and raises an exception if it is negative.
Library routines (opcode 0x56) are called by library slot and vector number; the names come from the
library's own symbol records (library_routines()).
"""
import argparse
import os
import struct

HEADER_BYTES = 64
MAGIC = bytes.fromhex("1fabad10")

# Builtin routine numbers. All of these are declared under these numbers and names in this unit's
# /lib/syslib.r (1999-06-30), and the 3.1 interpreter's own name table gives the same numbers (it calls
# 0x1b get_module_name); the three marked * are in the 3.1 table only.
BUILTINS = {
    0x00: "exit", 0x01: "getpid", 0x02: "getppid", 0x03: "waitpid", 0x04: "execl", 0x05: "execv",
    0x06: "kill", 0x07: "split", 0x08: "msleep", 0x09: "argc", 0x0a: "argv", 0x0b: "sig_mask_set",
    0x0c: "signal", 0x0d: "trace", 0x0e: "time", 0x0f: "time_set", 0x10: "get_ps", 0x11: "sigfifo",
    0x12: "memstat", 0x13: "sem_acquire", 0x14: "sem_release", 0x15: "sem_test", 0x16: "memcopy",
    0x17: "memset", 0x18: "mtime", 0x19: "sysconf", 0x1a: "setprio", 0x1b: "module_name_get",
    0x1c: "get_error_string*", 0x1d: "get_signal_string*", 0x1f: "attach_process*",
    0x20: "open", 0x21: "close", 0x22: "read", 0x23: "write", 0x24: "seek", 0x25: "readdir",
    0x26: "access", 0x27: "sync", 0x28: "reads", 0x29: "chdir", 0x2a: "mkdir", 0x2b: "rcv",
    0x2c: "send", 0x2d: "dup", 0x2e: "dup2", 0x2f: "writes", 0x30: "pipe", 0x31: "socketpair",
    0x32: "readsa", 0x33: "writeread", 0x34: "unlink", 0x35: "ioctl", 0x36: "mknod", 0x37: "link",
    0x38: "rmdir", 0x39: "stat", 0x3a: "chmod", 0x3b: "mount", 0x3c: "unmount", 0x3d: "statfs",
    0x3e: "flock", 0x3f: "utime", 0x40: "print", 0x41: "fprint", 0x42: "snprint", 0x43: "printf",
    0x44: "fprintf", 0x45: "snprintf", 0x46: "str_scanf", 0x47: "freadline", 0x48: "v3_vars_load",
    0x49: "v3_vars_save", 0x4a: "getcounters", 0x4b: "ftime", 0x4c: "fstat", 0x4d: "v3_save_on_exit",
    0x4e: "malarm", 0x50: "environ", 0x51: "getenv", 0x52: "setenv", 0x53: "unsetenv",
    0x80: "str_len", 0x81: "str_chr_set", 0x82: "str_len_set", 0x83: "str_chr_get", 0x84: "str_to_int",
    0x85: "str_to_float", 0x86: "time_to_str", 0x87: "str_cksum", 0x88: "str_append", 0x89: "str_limit",
    0x8a: "str_limit_set", 0xb1: "mem_alloc", 0xb2: "mem_free", 0xb3: "heap_set", 0xb4: "heap_space",
    0xb5: "heap_size", 0xc0: "loc_class_get", 0xc1: "loc_class_set", 0xc2: "loc_machtype_get",
    0xc3: "loc_machtype_set", 0xc4: "loc_flags_get", 0xc5: "loc_flags_set", 0xc6: "loc_check",
    0xc7: "loc_re_check", 0xc8: "loc_pdata_get", 0xc9: "loc_pdata_set", 0xca: "loc_cdata_get",
    0xcb: "loc_cdata_set", 0x100: "call_ifunc", 0x104: "va_arg_type", 0x105: "va_arg_get",
    0x106: "int_to_ptr", 0x107: "ptr_to_int", 0x108: "pdp_get", 0x109: "pdp_set", 0x10a: "error_code",
    0x10b: "error_addr", 0x10c: "addr_decode", 0x110: "test_stub", 0x140: "sin", 0x141: "cos",
    0x142: "tan", 0x143: "asin", 0x144: "acos", 0x145: "atan2", 0x146: "pow", 0x147: "sqrt",
    0x148: "fabs", 0x149: "ln", 0x14a: "log",
}

# Routines of this unit's /lib/syslib.r (1999-06-30) by vector number, from its symbol records
# (library_routines() reproduces this from the file). Programs reach it as library slot 2.
SYSLIB = {
    0x01: "sysid_string", 0x02: "size_to_bytes", 0x03: "addr_to_file", 0x04: "addr_to_line",
    0x05: "error_line", 0x06: "error_file", 0x07: "str_error", 0x08: "str_signal", 0x09: "err_get_subsys",
    0x0a: "err_get_b2", 0x0b: "err_get_b1", 0x0c: "err_get_code", 0x0d: "err_compare", 0x0e: "str_subsys",
    0x0f: "err_compose", 0x10: "WIFEXITED", 0x11: "WIFSIGNALED", 0x12: "WEXITSTATUS", 0x13: "WTERMSIG",
    0x14: "delay", 0x15: "sigmask", 0x16: "MINOR", 0x17: "MAJOR", 0x18: "BUILD_DEV", 0x19: "str_sizeof",
    0x1a: "rad", 0x1b: "deg", 0x1c: "str_substr", 0x1d: "str_chr_find", 0x1e: "str_chr_rfind",
    0x1f: "str_edit", 0x20: "str_dup", 0x21: "readline", 0x22: "iabs", 0x23: "abort", 0x24: "rand",
    0x25: "srand", 0x26: "rand_in", 0x27: "build_cloc", 0x28: "build_ploc", 0x29: "assert",
}

# opcode -> (form, template). Forms say how the word (and any words after it) are decoded:
#   ABC  three frame operands        ABc  two frame operands and a signed 8-bit constant c
#   AB   two frame operands          Ai   frame operand A and signed 16-bit immediate i
#   Aj   A and a code address j      Ar   A and a frame operand given by the 16-bit field (R)
#   Ag   A and a data word given by the 16-bit field (G)
#   P    a push: the value, then A itself when A is not zero (a type tag for variadic builtins)
OPS = {
    0x00: ("ABC", "{A} = {B} || {C}"), 0x01: ("ABC", "{A} = {B} && {C}"),
    0x02: ("ABC", "{A} = {B} | {C}"), 0x03: ("ABC", "{A} = {B} ^ {C}"), 0x04: ("ABC", "{A} = {B} & {C}"),
    0x05: ("ABC", "{A} = {B} == {C}"), 0x06: ("ABc", "{A} = {B} == {c}"),
    0x07: ("ABC", "{A} = {B} != {C}"), 0x08: ("ABc", "{A} = {B} != {c}"),
    0x09: ("ABC", "{A} = {B} > {C}"), 0x0a: ("ABc", "{A} = {B} > {c}"),
    0x0b: ("ABC", "{A} = {B} >= {C}"), 0x0c: ("ABc", "{A} = {B} >= {c}"),
    0x0d: ("ABC", "{A} = {B} < {C}"), 0x0e: ("ABc", "{A} = {B} < {c}"),
    0x0f: ("ABC", "{A} = {B} <= {C}"), 0x10: ("ABc", "{A} = {B} <= {c}"),
    0x11: ("ABC", "{A} = {B} >>u {C}"), 0x12: ("ABc", "{A} = {B} >>u {c}"),
    0x13: ("ABC", "{A} = {B} << {C}"), 0x14: ("ABc", "{A} = {B} << {c}"),
    0x15: ("ABC", "{A} = {B} + {C}"), 0x16: ("ABc", "{A} = {B} + {c}"),
    0x17: ("ABC", "{A} = {B} - {C}"), 0x18: ("ABc", "{A} = {B} - {c}"),
    0x19: ("ABC", "{A} = {B} * {C}"), 0x1a: ("ABc", "{A} = {B} * {c}"),
    0x1b: ("ABC", "{A} = {B} / {C}"), 0x1c: ("ABc", "{A} = {B} / {c}"),
    0x1d: ("ABC", "{A} = {B} % {C}"), 0x1e: ("ABc", "{A} = {B} % {c}"),
    0x1f: ("AB", "{A} = -{B}"), 0x20: ("AB", "{A} = ~{B}"), 0x21: ("AB", "{A} = !{B}"),
    0x22: ("AB", "{A} = float({B})"),
    0x23: ("ABC", "{A} = {B} ||. {C}"), 0x24: ("ABC", "{A} = {B} &&. {C}"),
    0x25: ("ABC", "{A} = {B} ==. {C}"), 0x26: ("ABC", "{A} = {B} !=. {C}"),
    0x27: ("ABC", "{A} = {B} >. {C}"), 0x28: ("ABC", "{A} = {B} >=. {C}"),
    0x29: ("ABC", "{A} = {B} <. {C}"), 0x2a: ("ABC", "{A} = {B} <=. {C}"),
    0x2b: ("ABC", "{A} = {B} +. {C}"), 0x2c: ("ABC", "{A} = {B} -. {C}"),
    0x2d: ("ABC", "{A} = {B} *. {C}"), 0x2e: ("ABC", "{A} = {B} /. {C}"),
    0x2f: ("AB", "{A} = -.{B}"), 0x30: ("AB", "{A} = !.{B}"), 0x31: ("AB", "{A} = int({B})"),
    0x32: ("lit", "{A} = {lit}"), 0x33: ("Ai", "{A} = {i}"),
    0x34: ("litP", "push {lit}"), 0x35: ("iP", "push {i}"),
    0x36: ("lit", "{A} = addr {lit}"), 0x37: ("litP", "push addr {lit}"),
    0x38: ("lit9", "{A}..+8 = {lit}"), 0x39: ("lit9P", "push9 {lit}"),
    0x3a: ("ABc", "{A} = {B}@[{c}]"), 0x3b: ("ABc", "{B}@[{c}] = {A}"),
    0x3c: ("ABc", "{A}..+8 = {B}@[{c}..+8]"), 0x3d: ("ABc", "{B}@[{c}..+8] = {A}..+8"),
    0x3e: ("Aj", "if {A} goto {j}"), 0x3f: ("Aj", "ifnot {A} goto {j}"),
    0x40: ("switch", "switch {A} table {j}"),
    0x41: ("Ai", "check 0 <= {A} < {i}"),
    0x42: ("Aj", "if.{A} goto {j}"), 0x43: ("Aj", "ifnot.{A} goto {j}"),
    0x44: ("j", "goto {j}"), 0x45: ("call", "call {j}"),
    0x46: ("Ar", "{A} = &{R}"), 0x47: ("Ag", "{A} = &{G}"),
    0x48: ("Ar", "{A} = {R}"), 0x49: ("Ag", "{A} = {G}"),
    0x4a: ("Ar", "{R} = {A}"), 0x4b: ("Ag", "{G} = {A}"),
    0x4c: ("Ar", "{A}..+8 = {R}..+8"), 0x4d: ("Ag", "{A}..+8 = {G}..+8"),
    0x4e: ("Ar", "{R}..+8 = {A}..+8"), 0x4f: ("Ag", "{G}..+8 = {A}..+8"),
    0x50: ("rP", "push {R}"), 0x51: ("rP", "push9 {R}"),
    0x52: ("r", "pop {R}"), 0x53: ("r", "pop9 {R}"),
    0x54: ("enter", "enter {n}"), 0x55: ("n", "alloc0 {n}"),
    0x56: ("libcall", "call {lib}"),
    0x57: ("none", "return"), 0x58: ("r0", "return {R}"), 0x59: ("r0", "return9 {R}"),
    0x5a: ("none", "check pop >= 0"),
    0x5b: ("builtin", "{bname}"),
    0x5c: ("ABC", "memcopy({A}, {B}, {C} words)"), 0x5d: ("Ai", "memzero({A}, {i} words)"),
    0x5e: ("ABC", "{A} = strcmp({B}, {C}) == 0"), 0x5f: ("ABC", "{A} = strcmp({B}, {C}) != 0"),
    0x60: ("ABC", "{A} = strcmp({B}, {C}) > 0"), 0x61: ("ABC", "{A} = strcmp({B}, {C}) >= 0"),
    0x62: ("ABC", "{A} = strcmp({B}, {C}) < 0"), 0x63: ("ABC", "{A} = strcmp({B}, {C}) <= 0"),
    0x64: ("ABC", "strcopy({A}, {B}, {C})"),
    0x65: ("ABC", "{A} = {B}@[{C}]"), 0x66: ("ABC", "{B}@[{C}] = {A}"),
    0x67: ("builtin", "{bname}"),
    0x68: ("rP", "push &{R}"), 0x69: ("gP", "push &{G}"),
    0x6a: ("none", "push sp"), 0x6b: ("none", "check sp"),
    0x6c: ("r", "raise {R}"), 0x6d: ("j", "try handler {j}"),
    0x6e: ("none", "reraise"), 0x6f: ("none", "end try"), 0x70: ("none", "exception skip"),
    0x71: ("none", "handler"),
}


class R3Error(ValueError):
    """The bytes are not a compiled RAPL-3 file this module understands."""


def s8(b):
    return b - 256 if b > 127 else b


def s16(v):
    return v - 65536 if v > 32767 else v


def frame(off):
    """Name of the frame word at fp+off."""
    if off < 0:
        return f"L{-off}"
    return f"A{off}" if off >= 3 else ("fp+0", "fp+1", "fp+2")[off]


def header(data):
    if len(data) < HEADER_BYTES or data[:4] != MAGIC:
        raise R3Error("not a compiled RAPL-3 file (no 1f ab ad 10 magic)")
    return struct.unpack_from("<16I", data, 0)


def init_stream(stream, at=0):
    """Expand a run-length init stream (frame initialiser after 0x54, or a module's data segment).

    Returns (bytes, relocations, bytes consumed including the 0xff end). Codes: 0xfb + 4 bytes = a
    pointer to be relocated; 0xfc + 1 byte = that byte; 0xfd + n = n zero bytes; 0xfe + u16 n = n
    zero bytes; 0xff = end; any other byte stands for itself."""
    out, relocs, i = bytearray(), [], at
    while True:
        if i >= len(stream):
            raise R3Error("init stream runs off the end")
        b = stream[i]
        i += 1
        if b == 0xff:
            return bytes(out), relocs, i - at
        if b == 0xfb:
            relocs.append(len(out))
            out += stream[i:i + 4]
            i += 4
        elif b == 0xfc:
            out.append(stream[i])
            i += 1
        elif b == 0xfd:
            out += bytes(stream[i])
            i += 1
        elif b == 0xfe:
            out += bytes(stream[i] | stream[i + 1] << 8)
            i += 2
        else:
            out.append(b)


class Module:
    """A compiled program or library: its code words, its data segment and its library table."""

    def __init__(self, data, name="?"):
        self.raw, self.name = data, name
        w = header(data)
        self.build_time = w[2]
        self.code_words = w[4] & 0xFFFF
        self.data_words = w[4] >> 16
        self.code_end = w[6]
        if self.code_end != HEADER_BYTES + 4 * self.code_words:
            raise R3Error(f"code size {self.code_words} words does not end at {self.code_end}")
        self.code = [struct.unpack_from("<I", data, HEADER_BYTES + 4 * i)[0] for i in range(self.code_words)]
        self.lib_table = w[8]
        self.data, self.data_relocs, _ = init_stream(data, self.code_end)
        if len(self.data) != 4 * self.data_words:
            raise R3Error(f"data stream gives {len(self.data)} bytes, header says {self.data_words} words")
        self.libs = self._libs(w)

    def _libs(self, w):
        table, names = w[8], w[12]
        if not table or not names:
            return {}
        count = struct.unpack_from("<H", self.raw, table)[0]
        out = {}
        for i in range(count):
            off, slot = struct.unpack_from("<HH", self.raw, table + 4 + 16 * i)
            start = names + 4 + off
            out[slot] = self.raw[start:self.raw.index(b"\0", start)].decode("ascii", "replace")
        return out

    def data_word(self, n):
        return struct.unpack_from("<I", self.data, 4 * n)[0] if 0 <= n < self.data_words else None

    def string_at(self, n):
        """The string whose header is data word n, or None."""
        head = self.data_word(n)
        if head is None:
            return None
        length, cap = head & 0xFFFF, head >> 16
        if not cap or length > cap or 4 * n + 4 + length > len(self.data):
            return None
        return self.data[4 * n + 4:4 * n + 4 + length].decode("latin-1")


def library_routines(lib):
    """{number: name} of a library's own routines and {number: name} of the builtins it declares,
    read from its symbol records: kind byte (2 sub, 3 func, 4 command), 0x01, u16 name offset,
    8 bytes not decoded, u16 number, u16 0x80|slot for a routine of this library or 0 for a builtin.
    On this unit's robotlib.r it gives every entry of controller/robotlib-functions.txt."""
    w = header(lib)
    names = w[12] + 4
    size = struct.unpack_from("<I", lib, w[12])[0]
    own, builtins = {}, {}
    for i in range(w[9], w[12] - 16):
        if lib[i] not in (2, 3, 4) or lib[i + 1] != 1:
            continue
        off = struct.unpack_from("<H", lib, i + 2)[0]
        if off >= size or (off and lib[names + off - 1] != 0):
            continue
        number, owner = struct.unpack_from("<HH", lib, i + 12)
        name = lib[names + off:lib.index(b"\0", names + off)].decode("latin-1")
        if not name.isidentifier():
            continue
        if owner & 0x80 and owner & 0x7F:
            own.setdefault(number, name)
        elif owner == 0:
            builtins.setdefault(number, name)
    return own, builtins


class Insn:
    __slots__ = ("addr", "word", "op", "text", "words", "target", "kind", "comment", "pushes", "count")

    def __init__(self, addr, word):
        self.addr, self.word, self.op = addr, word, word & 0xFF
        self.text, self.words, self.target, self.kind, self.comment = "", 1, None, "", ""
        self.pushes, self.count = None, None


def lit_text(mod, value, relocated=False):
    """A 32-bit literal: a plain number (0x32, 0x34), or for 0x36 and 0x37 a pointer the loader
    relocates: library slot in bits 16-22, bit 23 set for its text, word offset in bits 0-15."""
    seg = (value >> 16) & 0x7F
    if relocated and seg:
        part = "text" if value & 0x800000 else "data"
        return f"<lib{seg} {part} {value & 0xFFFF:#x}>"
    if relocated:
        return f"<frame {s16(value & 0xFFFF)}>" if value & 0xFFFF else "<null>"
    return str(value - (1 << 32) if value & 0x80000000 else value) + (f" ({value:#x})" if value > 9 else "")


def decode(mod, libnames=None):
    """List of Insn for the module's code, with data words (literals, frame initialisers and switch
    tables) folded into the instruction that owns them. Decoding is linear, so a switch table is
    recognised only after its switch instruction (as in every file seen)."""
    libnames = libnames or {}
    tables = {}
    pc = 1                                 # word 0 is 00000044 in every file seen; not decoded
    out = {0: Insn(0, mod.code[0])}
    out[0].text, out[0].kind = f"; word 0: {mod.code[0]:08x}", "data"
    while pc < mod.code_words:
        w = mod.code[pc]
        ins = Insn(pc, w)
        if pc in tables:
            n = tables[pc]
            rows = []
            for k in range(n):
                tgt, lo, hi = mod.code[pc + 3 * k:pc + 3 * k + 3]
                rows.append(f"{s16(lo & 0xFFFF) if lo < 0x10000 else lo}..{s16(hi & 0xFFFF) if hi < 0x10000 else hi} -> {tgt & 0xFFFF:#06x}")
            ins.text, ins.kind, ins.words = "; switch table: " + ", ".join(rows) + "; end", "data", 3 * n + 1
            out[pc] = ins
            pc += ins.words
            continue
        op = ins.op
        a, b, c = s8((w >> 8) & 0xFF), s8((w >> 16) & 0xFF), s8(w >> 24)
        i16, u16 = s16(w >> 16), w >> 16
        if op not in OPS:
            ins.text, ins.kind = f"; data {w:08x}", "data"
            out[pc] = ins
            pc += 1
            continue
        form, tpl = OPS[op]
        f = dict(A=frame(a), B=frame(b), C=frame(c), c=c, i=i16, j=f"{u16:#06x}", R=frame(i16), G=f"G{u16}", n=-i16)
        tag = f" tag {a}" if a else ""
        if form in ("lit", "litP"):
            value = mod.code[pc + 1]
            f["lit"] = lit_text(mod, value, relocated=op in (0x36, 0x37))
            ins.words = 2
            if form == "litP":
                ins.pushes = [(f["lit"], 1 + (1 if a else 0), a)]
        elif form in ("lit9", "lit9P"):
            f["lit"] = "[" + ", ".join(f"{x:#x}" for x in mod.code[pc + 1:pc + 10]) + "]"
            ins.words = 10
            if form == "lit9P":
                ins.pushes = [("9 words", 9 + (1 if a else 0), a)]
        elif form == "iP":
            ins.pushes = [(str(i16), 1 + (1 if a else 0), a)]
        elif form == "rP":
            n = 9 if op == 0x51 else 1
            name = f["R"] if op != 0x68 else "&" + f["R"]
            ins.pushes = [(name, n + (1 if a else 0), a)]
        elif form == "gP":
            s = mod.string_at(u16)
            name = f"&G{u16}"
            if s is not None:
                name += " " + repr(s)
            ins.pushes = [(name, 1 + (1 if a else 0), a)]
            if s is not None:
                ins.comment = repr(s)
        elif form == "Ag":
            s = mod.string_at(u16)
            if s is not None:
                ins.comment = repr(s)
        elif form in ("Aj", "j", "call"):
            ins.target = u16
        elif form == "switch":
            ins.target = u16
            n = 0
            while u16 + 3 * n < mod.code_words and mod.code[u16 + 3 * n] != 0:
                n += 1
            tables[u16] = n
        elif form == "enter":
            body = mod.raw[HEADER_BYTES + 4 * (pc + 1):]
            init, relocs, used = init_stream(body)
            ins.words = 1 + (used + 3) // 4
            nz = sum(1 for x in init if x)
            f["n"] = -i16
            ins.comment = f"frame of {-i16} words; initialiser sets {len(init)} bytes" + (f" ({nz} non-zero)" if nz else " to zero")
        elif form == "libcall":
            slot = a & 0x7F
            lib = mod.libs.get(slot, f"lib{slot}")
            name = (libnames.get(lib, {}) or {}).get(u16)
            f["lib"] = f"{lib}:{u16:#x}" + (f" {name}" if name else "")
        elif form == "builtin":
            f["bname"] = ("builtin " if op == 0x5b else "builtin! ") + f"{u16:#x} {BUILTINS.get(u16, '?')}"
        elif form == "r0":
            f["R"] = frame(i16) if i16 else "0"
        ins.text = tpl.format(**f)
        if form in ("iP", "litP", "rP", "gP", "lit9P"):
            ins.text += tag
        if op in (0x5b, 0x67, 0x45, 0x56):
            ins.kind = "call"
        out[pc] = ins
        pc += ins.words
    insns = [out[k] for k in sorted(out)]
    _annotate_calls(insns)
    return insns


def _annotate_calls(insns):
    """Attach the arguments of each call: the pushes before it in straight-line code. The last push
    is the number of argument words; the ones before it are the arguments, last argument first."""
    pending = []
    targets = {i.target for i in insns if i.target is not None}
    for ins in insns:
        if ins.addr in targets:
            pending = []
        if ins.pushes:
            pending.extend(ins.pushes)
            continue
        if ins.kind == "call" and pending:
            text, words, _ = pending[-1]
            try:
                count = int(text)
            except ValueError:
                pending = []
                continue
            args, need, k = [], count, len(pending) - 2
            while need > 0 and k >= 0:
                t, wcount, tag = pending[k]
                args.append(t if not tag else f"{t}:{tag}")
                need -= wcount
                k -= 1
            if need == 0:
                ins.comment = "(" + ", ".join(args) + ")"
                ins.count = count
                pending = pending[:k + 1]
            else:
                pending = []
            continue
        if ins.op in (0x44, 0x57, 0x58, 0x59):      # control leaves straight-line code
            pending = []


def listing(mod, libnames=None):
    insns = decode(mod, libnames)
    targets = {}
    for ins in insns:
        if ins.target is not None:
            targets.setdefault(ins.target, set()).add("call" if ins.op == 0x45 else "jump")
        if ins.op == 0x54 and ins.kind != "data":
            targets.setdefault(ins.addr, set()).add("call")
    lines = [f"; {mod.name}: {mod.code_words} code words, {mod.data_words} data words, libraries {mod.libs}"]
    for ins in insns:
        if ins.addr in targets:
            kinds = targets[ins.addr]
            lines.append(f"{'func' if 'call' in kinds else 'loc'}_{ins.addr:04x}:")
        byte = HEADER_BYTES + 4 * ins.addr
        com = f"   ; {ins.comment}" if ins.comment else ""
        lines.append(f"  {ins.addr:04x} [{byte:05x}] {ins.word:08x}  {ins.text}{com}")
    return lines


def data_listing(mod):
    """The data segment: strings where a string header is found, else words."""
    lines, n = [], 0
    while n < mod.data_words:
        s = mod.string_at(n)
        cap = (mod.data_word(n) or 0) >> 16
        if s is not None and n + 1 + (cap + 3) // 4 <= mod.data_words:
            lines.append(f"  G{n:<5} string[{cap}] {s!r}")
            n += 1 + (cap + 3) // 4
            continue
        v = mod.data_word(n)
        if v:
            reloc = " (pointer)" if 4 * n in mod.data_relocs else ""
            lines.append(f"  G{n:<5} {v:#010x}{reloc}")
        n += 1
    return lines


def load(path):
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:2] == b"\x1f\x8b":          # the stock example programs are stored gzipped
        import gzip
        data = gzip.decompress(data)
    return data


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--syslib", help="this controller's /lib/syslib.r, to name its routines from its own symbols")
    ap.add_argument("--data", action="store_true", help="also list the data segment")
    args = ap.parse_args(argv)
    libnames = {"syslib": dict(SYSLIB)}
    if args.syslib:
        own, _ = library_routines(load(args.syslib))
        libnames["syslib"] = own
    for path in args.files:
        mod = Module(load(path), os.path.basename(path))
        print("\n".join(listing(mod, libnames)))
        if args.data:
            print("; data segment")
            print("\n".join(data_listing(mod)))


if __name__ == "__main__":
    main()
