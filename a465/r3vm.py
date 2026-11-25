"""A small interpreter for compiled RAPL-3 programs, to test programs before they go on the controller.

    vm = VM(image, argv=["homezc", "6"], libs={"robotlib": robot})
    code = vm.run()

Written 2026-11-24 for a465/jogprog.py. It runs one program module; builtins and library routines
are Python stand-ins that the caller supplies or overrides (FakeSyslib and FakeRobot below are
the defaults). It is for checking what a program does with its inputs, not a model of the
controller: every builtin and library routine here is a fake.

Where the semantics come from: the instruction set is a465/r3dis.py's (its OPS table). The machine
model, below, was read from the handlers of the 3.1 interpreter (/sbin/r3interp of the 2001
firmware, the file r3dis.py was written from; handler addresses from its dispatch table at file
offset 0x544c):

- the stack grows down; operands are words at fp+offset. A call (0x45, handler 0x41e0) pushes the
  return address and the caller's fp, then fp = sp; the caller has already pushed the arguments
  (last first) and their word count, so the first argument is at fp+3.
- `enter n` (0x3fdc) moves sp down n words and copies the initialiser into the words from sp
  upward. `return` (0x4358) sets sp = fp, pops fp, the return address and the word count, drops
  that many words; `return R` (0x4510) then pushes R (0 when R is 0).
- a push with a non-zero A byte pushes the value and then A (0x4ddd).
- a builtin (0x5b/0x67, 0x4960) pops the count and the arguments and pushes its result; 0x67 then
  pops the result and raises it if negative (0x4b6b). `check pop >= 0` (0x5a) pops and raises a
  negative value.
- `switch` (0x355c) walks rows {target, lo, hi} until a zero word and jumps to the first row with
  lo <= value <= hi; with no match execution goes on with the next instruction.
- 0x3a/0x3b/0x65/0x66 address the word `offset` words past a pointer (0x10c0). A pointer is
  (segment << 16) | word; a pointer to a frame word (0x46) is segment 1 and the word's index.
- `try handler` (0x6d, 0x5228) pushes six words of handler state (fp, sp, handler, mark, the
  address an exception came from, its code) and installs the new handler with sp and fp as they
  are; 0x6e (0x5324 -> 0x11c4) pops them back. Raising (0x125c) with a handler installed: if the
  handler is already running, sp = its saved sp, pop the record and try the next one out;
  otherwise (0x1170) note the address and code, and continue at the handler with the saved sp and
  fp. builtin error_code() (0x10a, dispatched by the table at 0x78d4 to 0x875c) returns that code.

Builtins provided (by number, see r3dis.BUILTINS): exit, msleep, argc, argv, time, mtime, open,
close, reads, print, fprint, printf, fprintf, str_len, str_chr_get, str_to_int, error_code. A
format "{}" prints one argument (tag 1 an int, tag 19 a string); other format specs raise, so a
test never passes on a guess about the controller's formatter.
"""
import struct

from a465 import r3dis

STACK, HEAP, DATA, TEXT = 1, 2, 3, 4
TAG_INT, TAG_STRING = 1, 19
EAGAIN = 35            # syslib's EAGAIN (its constant record); fastacid treats -35 as "no data yet"
O_NONBLOCK = 4         # syslib's O_NONBLOCK
MASK = 0xFFFFFFFF


def s32(v):
    v &= MASK
    return v - (1 << 32) if v & 0x80000000 else v


class R3VMError(RuntimeError):
    """The program did something this interpreter does not do, or that would fault."""


class ProgramExit(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


class Uncaught(R3VMError):
    """An exception was raised with no handler installed (the real interpreter ends the program)."""

    def __init__(self, code, pc):
        super().__init__(f"uncaught exception {code} at {pc:#06x}")
        self.code, self.pc = code, pc


class Deadline(Exception):
    """run() stopped because the fake clock passed its limit."""


class Input:
    """Bytes that arrive on a file at given times: [(time_ms, b"..."), ...]."""

    def __init__(self, arrivals=()):
        self.pending = sorted(((t, bytes(b)) for t, b in arrivals), key=lambda x: x[0])
        self.buffer = bytearray()

    def read(self, now, n):
        """Up to n bytes that have arrived by `now` (b"" if none); a subclass may return a
        negative int instead, which reads() then returns as its error."""
        while self.pending and self.pending[0][0] <= now:
            self.buffer += self.pending.pop(0)[1]
        out = bytes(self.buffer[:n])
        del self.buffer[:n]
        return out

    def exhausted(self):
        return not self.pending and not self.buffer


class FakeSyslib:
    """syslib routines by vector number: str_error (7), delay (0x14), iabs (0x22)."""

    def __init__(self, error_texts=None):
        self.error_texts = error_texts or {}
        self.calls = []

    def call(self, vm, number, args):
        self.calls.append((number, tuple(args)))
        if number == 0x07:                                 # str_error(n) -> static string
            n = s32(args[0])
            return vm.new_string(self.error_texts.get(n, f"error {n}").encode("latin-1"))
        if number == 0x14:                                 # delay(ms): sleeps; 0
            vm.advance(s32(args[0]))
            return 0
        if number == 0x22:                                 # iabs(x)
            return abs(s32(args[0]))
        raise R3VMError(f"syslib routine {number:#x} is not faked")


class FakeRobot:
    """robotlib: records every call as (number, args) and the clock at it in `times`; results come
    from `results`, a dict of number -> int, list of ints (used in turn, the last repeating) or
    callable(vm, args) -> int. Anything not listed returns 0. `latency_ms` is added per call."""

    def __init__(self, results=None, latency_ms=0):
        self.results = dict(results or {})
        self.calls, self.times = [], []
        self.latency_ms = latency_ms

    def call(self, vm, number, args):
        self.calls.append((number, tuple(s32(a) for a in args)))
        self.times.append(vm.clock)
        vm.advance(self.latency_ms)
        r = self.results.get(number, 0)
        if callable(r):
            return r(vm, args)
        if isinstance(r, list):
            return r.pop(0) if len(r) > 1 else r[0]
        return r


class VM:
    def __init__(self, image, argv=("program",), libs=None, files=None, stack_words=8192, clock_ms=0):
        self.mod = r3dis.Module(image, argv[0])
        w = struct.unpack_from("<16I", image, 0)
        self.entry = w[3] & 0xFFFF
        self.argv = list(argv)
        self.libs = {"syslib": FakeSyslib(), "robotlib": FakeRobot()}
        self.libs.update(libs or {})
        self.files = dict(files or {})       # path -> Input
        self.fds = {}                        # fd -> (path, flags)
        self.clock = clock_ms
        self.out = {1: bytearray(), 2: bytearray()}
        self.events = []                     # (clock, fd, text) per write
        self.segments = {STACK: [0] * stack_words, HEAP: [0], DATA: self._load_data(), TEXT: self.mod.code}
        self.argv_ptrs = [self.new_string(a.encode("latin-1")) for a in self.argv]
        self.sp, self.fp = stack_words, 0
        self.handler = self.try_sp = self.try_fp = self.mark = self.exc_from = self.exc_code = 0
        self.steps = 0
        self.pc = None
        self._inits = {}

    # -- memory -----------------------------------------------------------------------------
    def _load_data(self):
        data = bytearray(self.mod.data)
        for at in self.mod.data_relocs:
            v = struct.unpack_from("<I", data, at)[0]
            struct.pack_into("<I", data, at, self.relocate(v))
        return list(struct.unpack(f"<{len(data) // 4}I", bytes(data)))

    def relocate(self, v):
        """A pointer as stored in the file (bit 23 text, bits 16-22 slot, low half word) at run time."""
        slot, word = (v >> 16) & 0x7F, v & 0xFFFF
        if slot == 1:
            return (TEXT if v & 0x800000 else DATA) << 16 | word
        if slot == 0 and word == 0:
            return 0
        raise R3VMError(f"pointer {v:#x} into library slot {slot} is not supported")

    def _seg(self, ptr, off=0):
        seg, word = (ptr >> 16) & 0xFF, (ptr & 0xFFFF) + off
        if seg not in self.segments:
            raise R3VMError(f"pointer {ptr:#x} names no segment")
        mem = self.segments[seg]
        if not 0 <= word < len(mem):
            raise R3VMError(f"pointer {ptr:#x}+{off} is outside its segment")
        return mem, word

    def load(self, ptr, off=0):
        mem, word = self._seg(ptr, off)
        return mem[word]

    def store(self, ptr, off, value):
        if (ptr >> 16) & 0xFF == TEXT:
            raise R3VMError("store into the code")
        mem, word = self._seg(ptr, off)
        mem[word] = value & MASK

    def new_string(self, b):
        heap = self.segments[HEAP]
        at = len(heap)
        cap = max(4, (len(b) + 3) // 4 * 4)
        heap.append(len(b) | cap << 16)
        body = bytes(b) + bytes(cap - len(b))
        heap.extend(struct.unpack(f"<{cap // 4}I", body))
        return HEAP << 16 | at

    def string(self, ptr):
        head = self.load(ptr)
        length = head & 0xFFFF
        words = [self.load(ptr, 1 + k) for k in range((length + 3) // 4)]
        return struct.pack(f"<{len(words)}I", *words)[:length]

    def set_string(self, ptr, b):
        head = self.load(ptr)
        cap = head >> 16
        if len(b) > cap:
            raise R3VMError(f"{len(b)} bytes into a string of capacity {cap}")
        body = bytes(b) + bytes(-len(b) % 4)
        for k in range(len(body) // 4):
            self.store(ptr, 1 + k, struct.unpack_from("<I", body, 4 * k)[0])
        self.store(ptr, 0, len(b) | cap << 16)

    def advance(self, ms):
        if ms > 0:
            self.clock += ms

    # -- frame and stack ----------------------------------------------------------------------
    def frame(self, off):
        return self.segments[STACK][self.fp + off]

    def set_frame(self, off, value):
        self.segments[STACK][self.fp + off] = value & MASK

    def push(self, v):
        self.sp -= 1
        if self.sp < 0:
            raise R3VMError("stack overflow")
        self.segments[STACK][self.sp] = v & MASK

    def pop(self):
        v = self.segments[STACK][self.sp]
        self.sp += 1
        return v

    def pop_args(self):
        count = s32(self.pop())
        args = [self.pop() for _ in range(count)]
        return args

    def varargs(self, args):
        """[(tag, value)] from the words after a variadic routine's count of values."""
        out, k = [], 0
        while k < len(args):
            tag, value = s32(args[k]), args[k + 1]
            if tag not in (TAG_INT, TAG_STRING):
                raise R3VMError(f"variadic value with tag {tag} is not supported")
            out.append((tag, value))
            k += 2
        return out

    def format(self, fmt, values):
        out, k, i = bytearray(), 0, 0
        while i < len(fmt):
            ch = fmt[i:i + 1]
            if ch == b"{":
                j = fmt.index(b"}", i)
                if fmt[i:j + 1] != b"{}":
                    raise R3VMError(f"format spec {fmt[i:j + 1]!r} is not supported")
                tag, value = values[k]
                k += 1
                out += self.string(value) if tag == TAG_STRING else str(s32(value)).encode()
                i = j + 1
                continue
            out += ch
            i += 1
        if k != len(values):
            raise R3VMError(f"{len(values)} values for {k} format specs")
        return bytes(out)

    def write(self, fd, b):
        if fd not in self.out:
            raise R3VMError(f"write to fd {fd}")
        self.out[fd] += b
        self.events.append((self.clock, fd, b.decode("latin-1")))

    def stdout(self):
        return self.out[1].decode("latin-1")

    def stderr(self):
        return self.out[2].decode("latin-1")

    # -- builtins -----------------------------------------------------------------------------
    def builtin(self, number, args):
        f = getattr(self, f"b_{r3dis.BUILTINS.get(number, '').rstrip('*')}", None)
        if f is None:
            raise R3VMError(f"builtin {number:#x} {r3dis.BUILTINS.get(number, '?')} is not provided")
        return f(args)

    def b_exit(self, args):
        raise ProgramExit(s32(args[0]))

    def b_msleep(self, args):
        self.advance(s32(args[0]))
        return 0

    def b_argc(self, args):
        return len(self.argv)

    def b_argv(self, args):
        n = s32(args[0])
        if not 0 <= n < len(self.argv):
            raise R3VMError(f"argv({n}) with {len(self.argv)} arguments")
        return self.argv_ptrs[n]

    def b_time(self, args):
        return self.clock // 1000

    def b_mtime(self, args):              # word 0 milliseconds (syslib delay and TimeStamp use it so)
        self.store(args[0], 0, self.clock)
        self.store(args[0], 1, 0)
        return 0

    def b_open(self, args):               # open(&fd, path, flags, mode)
        path = self.string(args[1]).decode("latin-1")
        if path not in self.files:
            return -2                     # ENOENT is 2 in this unit's syslib
        fd = 3 + len(self.fds)
        self.fds[fd] = (path, s32(args[2]))
        self.store(args[0], 0, fd)
        return 0

    def b_close(self, args):
        self.fds.pop(s32(args[0]), None)
        return 0

    def b_reads(self, args):              # reads(fd, s, maxlen): bytes read, or -EAGAIN
        fd, s, maxlen = s32(args[0]), args[1], s32(args[2])
        if fd not in self.fds:
            raise R3VMError(f"reads from fd {fd}, which is not open")
        path, flags = self.fds[fd]
        got = self.files[path].read(self.clock, maxlen)
        if isinstance(got, int):
            return got
        if not got:
            if not flags & O_NONBLOCK:
                raise R3VMError("a blocking read with no data would wait forever here")
            return -EAGAIN
        self.set_string(s, got)
        return len(got)

    def b_print(self, args):
        self.write(1, self._concat(args[1:]))
        return 0

    def b_fprint(self, args):
        self.write(s32(args[0]), self._concat(args[2:]))
        return 0

    def _concat(self, rest):
        return b"".join(self.string(v) if t == TAG_STRING else str(s32(v)).encode() for t, v in self.varargs(rest))

    def b_printf(self, args):
        self.write(1, self.format(self.string(args[0]), self.varargs(args[2:])))
        return 0

    def b_fprintf(self, args):
        self.write(s32(args[0]), self.format(self.string(args[1]), self.varargs(args[3:])))
        return 0

    def b_str_len(self, args):
        return self.load(args[0]) & 0xFFFF

    def b_str_chr_get(self, args):
        b = self.string(args[0])
        i = s32(args[1])
        if not 0 <= i < len(b):
            raise R3VMError(f"str_chr_get index {i} of a {len(b)}-byte string")
        return b[i]

    def b_str_to_int(self, args):         # str_to_int(&dst, src): a negative value (this fake's
        text = self.string(args[1]).decode("latin-1")  # choice) when the text is not a number
        try:
            value = int(text, 10)
        except ValueError:
            return -1
        self.store(args[0], 0, value)
        return 0

    def b_error_code(self, args):
        return self.exc_code

    # -- exceptions ---------------------------------------------------------------------------
    def raise_(self, code, at):
        while True:
            if not self.handler:
                raise Uncaught(s32(code), at)
            if self.exc_from:
                self.sp = self.try_sp
                self.try_pop()
                continue
            self.exc_from = TEXT << 16 | at
            self.exc_code = code & MASK
            self.pc = self.handler & 0xFFFF
            self.sp, self.fp = self.try_sp, self.try_fp
            return

    def try_pop(self):
        if not self.handler:
            raise R3VMError("0x6e with no handler installed")
        self.exc_code = self.pop()
        self.exc_from = self.pop()
        self.mark = self.pop() & 0xFFFF
        self.handler = self.pop()
        self.try_sp = self.pop()
        self.try_fp = self.pop()

    # -- running ------------------------------------------------------------------------------
    def start(self):
        self.push(0)                          # main's argument count
        self.push(0)                          # return address 0: main returning ends the program
        self.push(0)                          # caller's fp
        self.fp = self.sp
        self.pc = self.entry

    def run(self, max_steps=5_000_000, until_ms=None):
        """Run to exit; returns the exit code. Raises Deadline if the clock passes until_ms."""
        if self.pc is None:
            self.start()
        try:
            while True:
                if until_ms is not None and self.clock > until_ms:
                    raise Deadline(self.clock)
                self.step()
                if self.steps > max_steps:
                    raise R3VMError(f"no exit after {max_steps} instructions")
        except ProgramExit as e:
            self.exit_code = e.code
            return e.code

    def step(self):
        self.steps += 1
        at = self.pc
        code = self.segments[TEXT]
        word = code[at]
        self.pc = at + 1
        op = word & 0xFF
        a, b, c = r3dis.s8((word >> 8) & 0xFF), r3dis.s8((word >> 16) & 0xFF), r3dis.s8(word >> 24)
        i16, u16 = r3dis.s16(word >> 16), word >> 16
        F, S = self.frame, self.set_frame
        if op in BINARY:
            S(a, BINARY[op](s32(F(b)), s32(F(c))))
        elif op in BINARY_C:
            S(a, BINARY_C[op](s32(F(b)), c))
        elif op == 0x1f:
            S(a, -s32(F(b)))
        elif op == 0x20:
            S(a, ~F(b))
        elif op == 0x21:
            S(a, int(not F(b)))
        elif op == 0x32:
            S(a, code[self.pc])
            self.pc += 1
        elif op == 0x33:
            S(a, i16)
        elif op == 0x34:
            self.push(code[self.pc])
            self.pc += 1
            self._tag(a)
        elif op == 0x35:
            self.push(i16)
            self._tag(a)
        elif op == 0x36:
            S(a, self.relocate(code[self.pc]))
            self.pc += 1
        elif op == 0x37:
            self.push(self.relocate(code[self.pc]))
            self.pc += 1
            self._tag(a)
        elif op == 0x3a:
            S(a, self.load(F(b), c))
        elif op == 0x3b:
            self.store(F(b), c, F(a))
        elif op == 0x3e:
            if F(a):
                self.pc = u16
        elif op == 0x3f:
            if not F(a):
                self.pc = u16
        elif op == 0x40:
            v, row = s32(F(a)), u16
            while code[row]:
                if s32(code[row + 1]) <= v <= s32(code[row + 2]):
                    self.pc = code[row] & 0xFFFF
                    break
                row += 3
        elif op == 0x41:
            if not 0 <= s32(F(a)) < i16:
                raise R3VMError(f"index {s32(F(a))} out of 0..{i16 - 1} at {at:#06x}")
        elif op == 0x44:
            self.pc = u16
        elif op == 0x45:
            self.push(TEXT << 16 | self.pc)
            self.push(self.fp)
            self.fp = self.sp
            self.pc = u16
        elif op == 0x46:
            S(a, STACK << 16 | (self.fp + i16))
        elif op == 0x47:
            S(a, DATA << 16 | u16)
        elif op == 0x48:
            S(a, F(i16))
        elif op == 0x49:
            S(a, self.segments[DATA][u16])
        elif op == 0x4a:
            S(i16, F(a))
        elif op == 0x4b:
            self.segments[DATA][u16] = F(a)
        elif op == 0x50:
            self.push(F(i16))
            self._tag(a)
        elif op == 0x52:
            S(i16, self.pop())
        elif op == 0x54:
            self.sp -= -i16
            if self.sp < 0:
                raise R3VMError("stack overflow")
            init, skip = self._frame_init(at)
            self.segments[STACK][self.sp:self.sp + len(init)] = init
            self.pc += skip
        elif op == 0x56:
            lib = self.mod.libs.get(a & 0x7F)
            if lib not in self.libs:
                raise R3VMError(f"call into library slot {a & 0x7F} ({lib}) with no stand-in")
            result = self.libs[lib].call(self, u16, self.pop_args())
            if result is not None:
                self.push(result)
        elif op in (0x57, 0x58):
            value = (F(i16) if i16 else 0) if op == 0x58 else None
            self.sp = self.fp
            self.fp = self.pop()
            ret = self.pop()
            count = s32(self.pop())
            self.sp += count
            if ret == 0:
                raise ProgramExit(0)
            self.pc = ret & 0xFFFF
            if value is not None:
                self.push(value)
        elif op == 0x5a:
            v = s32(self.pop())
            if v < 0:
                self.raise_(v, at)
        elif op in (0x5b, 0x67):
            result = self.builtin(u16, self.pop_args())
            if op == 0x67:
                if s32(result) < 0:
                    self.raise_(result, at)
            else:
                self.push(result)
        elif op in (0x5e, 0x5f):
            same = self.string(F(b)) == self.string(F(c))
            S(a, int(same if op == 0x5e else not same))
        elif op == 0x65:
            S(a, self.load(F(b), s32(F(c))))
        elif op == 0x66:
            self.store(F(b), s32(F(c)), F(a))
        elif op == 0x68:
            self.push(STACK << 16 | (self.fp + i16))
            self._tag(a)
        elif op == 0x69:
            self.push(DATA << 16 | u16)
            self._tag(a)
        elif op == 0x6c:
            v = s32(F(i16))
            self.raise_(v if v < 0 else -1, at)
        elif op == 0x6d:
            for v in (self.try_fp, self.try_sp, self.handler, self.mark, self.exc_from, self.exc_code):
                self.push(v)
            self.handler = TEXT << 16 | u16
            self.mark = self.exc_from = self.exc_code = 0
            self.try_sp, self.try_fp = self.sp, self.fp
        elif op == 0x6e:
            self.try_pop()
        else:
            raise R3VMError(f"opcode {op:#04x} ({r3dis.OPS.get(op, ('', '?'))[1]}) at {at:#06x} is not interpreted")

    def _frame_init(self, at):
        """(initial words, words to skip) of the enter at `at`: its run-length initialiser."""
        if at not in self._inits:
            body = self.mod.raw[r3dis.HEADER_BYTES + 4 * (at + 1):]
            init, relocs, used = r3dis.init_stream(body)
            if relocs:
                raise R3VMError("frame initialiser with pointers")
            init += bytes(-len(init) % 4)
            self._inits[at] = (list(struct.unpack(f"<{len(init) // 4}I", init)), (used + 3) // 4)
        return self._inits[at]

    def _tag(self, a):
        if a:
            self.push(a)


def _div(x, y):
    if y == 0:
        raise R3VMError("division by zero")
    q = abs(x) // abs(y)
    return q if (x < 0) == (y < 0) else -q


def _mod(x, y):
    return x - y * _div(x, y)


_OPS = {
    "||": lambda x, y: int(bool(x) or bool(y)), "&&": lambda x, y: int(bool(x) and bool(y)),
    "|": lambda x, y: x | y, "^": lambda x, y: x ^ y, "&": lambda x, y: x & y,
    "==": lambda x, y: int(x == y), "!=": lambda x, y: int(x != y), ">": lambda x, y: int(x > y),
    ">=": lambda x, y: int(x >= y), "<": lambda x, y: int(x < y), "<=": lambda x, y: int(x <= y),
    ">>u": lambda x, y: (x & MASK) >> (y & 31), "<<": lambda x, y: x << (y & 31),
    "+": lambda x, y: x + y, "-": lambda x, y: x - y, "*": lambda x, y: x * y, "/": _div, "%": _mod,
}
BINARY, BINARY_C = {}, {}
for _op, (_form, _tpl) in r3dis.OPS.items():
    if _op < 0x1f and _form in ("ABC", "ABc"):
        _sym = _tpl.split(" ")[3]
        (BINARY if _form == "ABC" else BINARY_C)[_op] = _OPS[_sym]
