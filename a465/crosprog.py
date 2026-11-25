"""Compiled RAPL-3 programs (and libraries) as the CROS controller stores them: header, check value, library stamps.

Worked out on 2026-11-24 from this unit's 1999 files (CROS 1.16.471) and from the 2001 firmware's
(CROS 3.1) programs, and proved by recomputation on every sample of both (see tests/test_crosprog.py).

Every file starts with a 64-byte header of sixteen little-endian 32-bit words:

    word 0   (byte 0)   0x10adab1f, the magic (bytes 1f ab ad 10)
    word 1   (byte 4)   0 in programs; 0x20000 in robotlib.r (meaning not known)
    word 2   (byte 8)   build time, Unix seconds
    word 3   (byte 12)  u16 and u16 0x0081 (meaning of the first not known; it varies per program)
    word 4   (byte 16)  u16 code length in 4-byte words (equals (word 6 - 64) / 4 in all 109 samples), then a u16
                        (0x13 in `home`; the runtime's debugger lists it as the size of a program's Data segment)
    word 6   (byte 24)  offset where the code ends and the data after it begins
    word 8   (byte 32)  offset of the library table (below); word 12 (byte 48) offset of the library names
    word 9, 10          two more section offsets, 0 in small programs (meaning not known)
    word 13  (byte 52)  1 in most programs; in a library it is that library's own signature (see stamp())
    word 14  (byte 56)  the check value: chosen so that words 0 to 14 add up to zero modulo 2**32
    word 15  (byte 60)  0

The library table, at word 8, lists the libraries a program links to: a u16 count and a u16 zero, then
one 16-byte record per library

    u16 offset of its name inside the names block, u16 slot (2, 3, ...), u32 build time of that library
    (its header word 2), u16 0, u16 (5 for syslib, 2 for the others; meaning not known), u32 signature
    (its header word 13)

The names block, at word 12, starts with a u32 giving the size of the names (padded to 4) and then holds
the NUL-terminated library names in the same order; the name offset in a record counts from after that u32.
A program built against another firmware
therefore differs in its own build time, in the check value (which covers that build time but not the
body), and in these stamps and signatures; nothing else that comes from the build.

A call into a library is the four bytes 56 83 <number> 00 in the code.

Known by recomputation only: the check and the stamps. That the controller itself compares the stamps
with the libraries it holds, and does not look at the program's build time, is an inference.
"""
import struct
from dataclasses import dataclass

MAGIC = bytes.fromhex("1fabad10")
HEADER_BYTES = 64
MASK = 0xFFFFFFFF
BUILD_TIME_AT, CODE_END_AT, LIB_TABLE_AT, LIB_NAMES_AT, SIGNATURE_AT, CHECK_AT = 8, 24, 32, 48, 52, 56
LIB_RECORD_BYTES = 16
CALL_OPCODE = bytes((0x56, 0x83))


class ProgramError(ValueError):
    """The bytes are not a compiled program this module understands."""


@dataclass
class LibRecord:
    name: str
    slot: int
    build_time: int         # the library's header word 2 as the program saw it when built
    signature: int          # the library's header word 13
    at: int                 # byte offset of this 16-byte record in the file


@dataclass
class Program:
    build_time: int
    check: int
    libs: list


def _words(data, count):
    return struct.unpack_from(f"<{count}I", data, 0)


def _need_header(data):
    if len(data) < HEADER_BYTES:
        raise ProgramError(f"expected at least {HEADER_BYTES} bytes, got {len(data)}")
    if data[:4] != MAGIC:
        raise ProgramError("missing the 1f ab ad 10 magic")


def compute_check(data):
    """The value that belongs at byte 56: minus the sum of header words 0 to 13, modulo 2**32."""
    _need_header(data)
    return -sum(_words(data, 14)) & MASK


def check_ok(data):
    _need_header(data)
    return struct.unpack_from("<I", data, CHECK_AT)[0] == compute_check(data)


def stamp(lib):
    """(build time, signature) of a library file: what a program records about it when it links to it."""
    _need_header(lib)
    return struct.unpack_from("<I", lib, BUILD_TIME_AT)[0], struct.unpack_from("<I", lib, SIGNATURE_AT)[0]


def parse(data):
    _need_header(data)
    table, names = struct.unpack_from("<I", data, LIB_TABLE_AT)[0], struct.unpack_from("<I", data, LIB_NAMES_AT)[0]
    if not (HEADER_BYTES <= table <= names <= len(data) - 4):
        raise ProgramError(f"library table at {table} and names at {names} do not fit a {len(data)}-byte file")
    count = struct.unpack_from("<H", data, table)[0]
    if table + 4 + count * LIB_RECORD_BYTES > names:
        raise ProgramError(f"{count} library records do not fit before the names at {names}")
    names += 4
    libs = []
    for i in range(count):
        at = table + 4 + i * LIB_RECORD_BYTES
        off, slot, built, _, _, signature = struct.unpack_from("<HHIHHI", data, at)
        end = data.find(b"\0", names + off)
        if end < 0:
            raise ProgramError(f"library name {i} is not terminated")
        libs.append(LibRecord(data[names + off:end].decode("ascii", "replace"), slot, built, signature, at))
    return Program(struct.unpack_from("<I", data, BUILD_TIME_AT)[0], struct.unpack_from("<I", data, CHECK_AT)[0], libs)


def calls(data):
    """(byte offset, call number) of every 56 83 <n> 00 instruction in the code, which runs from byte 64 to the code end."""
    _need_header(data)
    end = struct.unpack_from("<I", data, CODE_END_AT)[0]
    return [(i, data[i + 2]) for i in range(HEADER_BYTES, min(end, len(data)) - 3, 4)
            if data[i:i + 2] == CALL_OPCODE and data[i + 3] == 0]


def restamp(data, libs, build_time=None):
    """Copy of a program re-stamped for another firmware: libs maps library name to that firmware's library
    file bytes. Each library record gets that library's build time and signature, the program's own build time
    is replaced if one is given, and the check is recomputed. Raises if a library the program links to is missing."""
    prog = parse(data)
    out = bytearray(data)
    for lib in prog.libs:
        if lib.name not in libs:
            raise ProgramError(f"no {lib.name} given for the program's library record")
        built, signature = stamp(libs[lib.name])
        struct.pack_into("<I", out, lib.at + 4, built)
        struct.pack_into("<I", out, lib.at + 12, signature)
    if build_time is not None:
        struct.pack_into("<I", out, BUILD_TIME_AT, build_time)
    struct.pack_into("<I", out, CHECK_AT, compute_check(out))
    return bytes(out)


HOMEZC_OFFSET_WORDS = {0x19: 0x00260044, 0x1A: 0xFFF60150, 0x1B: 0x00010035, 0x1C: 0x00000069, 0x1D: 0x00020035,
                       0x1E: 0x00050035, 0x1F: 0x00440067, 0x20: 0x00000035, 0x21: 0x00600044, 0x5F: 0x001A0044}
HOMEZC_PLAIN_WORDS = {0x18: 0x0025FB3F, 0x19: 0xFFF81350, 0x5E: 0x0000006E, 0x5F: 0x00000035, 0x60: 0x00048356}
HOMEZC_OLD_TEXT = b"{}: invalid axis '{}'\n"
HOMEZC_NEW_TEXT = b"zero-cross offset: {}\n"


def homezc_offset_program(plain):
    """The homing program that prints `zero-cross offset: K` after it succeeds, made from the plain re-stamped
    `homezc` (the owner's hand patch of 2026-11-24, reproduced word for word).

    Code words are little-endian u32 at file offset 0x40 + 4 * index. The words the patch relies on are checked
    first (ProgramError if any differs, or if the file is not 640 bytes); then ten code words are replaced and
    the 22-byte text `{}: invalid axis '{}'\\n` becomes `zero-cross offset: {}\\n`. The header is not touched,
    so the check value stays as it was."""
    if len(plain) != 640:
        raise ProgramError(f"expected a 640-byte program, got {len(plain)} bytes")
    _need_header(plain)
    for index, want in HOMEZC_PLAIN_WORDS.items():
        have = struct.unpack_from("<I", plain, 0x40 + 4 * index)[0]
        if have != want:
            raise ProgramError(f"code word {index:#x} is {have:#010x}, expected {want:#010x}: not the program this patch is for")
    if plain.count(HOMEZC_OLD_TEXT) != 1:
        raise ProgramError(f"the text {HOMEZC_OLD_TEXT!r} is not in the program exactly once")
    out = bytearray(plain)
    for index, value in HOMEZC_OFFSET_WORDS.items():
        struct.pack_into("<I", out, 0x40 + 4 * index, value)
    at = bytes(out).index(HOMEZC_OLD_TEXT)
    out[at:at + len(HOMEZC_OLD_TEXT)] = HOMEZC_NEW_TEXT
    return bytes(out)
