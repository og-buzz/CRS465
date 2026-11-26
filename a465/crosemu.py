#!/usr/bin/env python3
"""Moved from the r3c spike (2026-11-26). Used by a465/r3c.py; the vendor binaries run ONLY here.

Run a CROS (C500C, CROS 3.1) i386 a.out program -- rc1 or rpp -- inside Unicorn, with every
system call answered by this file. Nothing the guest does reaches the host except:

  * reading files whose real path was named with --read (exact files, nothing else);
  * creating / writing / unlinking / chmod-ing files inside --outdir (which must already exist);
  * bytes written to guest fds 1 and 2, which are captured (and echoed to our stdout/stderr).

The guest's current directory is --outdir: relative paths resolve there.

What was worked out from the binary (see docs/RAPL3-COMPILER.md):
  * a.out QMAGIC (0xcc), MID 0x86 (i386), no symbols, no relocations: the header is part of the
    text segment, file offset 0 is loaded at 0x1000, entry 0x1020; data follows text, then bss.
  * startup: esp -> argc, argv[], NULL, envp[], NULL (the crt0 at 0x1020 computes environ that way).
  * system calls: `mov $N,%eax; int $0x40`, arguments on the caller's stack (cdecl, so [esp+4] is
    the first), result in eax, negative = -errno. sbrk() is a pure user-space bump of a pointer
    (no system call), so the heap is just memory above bss: we map 16 MB.
  * the CROS numbers used by rc1 / rpp, identified from the libc wrappers around each stub:
      1 exit   2 open(path, flags, perm3)   3 close   4 read   5 write   8 lseek(fd, off32, whence)
     13 unlink 19 stat(path, cros_stat*)    20 fstat(fd, cros_stat*)   21 chmod(path, perm3)
     26 dup2   33 getpid  42 time  52 kill  54 signal(sig, fn, &old)   59 sigprocmask
     62 strerror(buf, len, errno) -- the kernel writes the message text into buf.
  * open flags: 1 read, 2 write, 3 read/write, 0x8 append, 0x200 create, 0x400 truncate
    (from rc1's fopen mode parser at 0x25654: "r" -> 1, "w" -> 0x602, "a" -> 0x20a, "+" -> 3).
  * cros_stat (>= 0x58 bytes; caller passes a 0x60 buffer): u16 type at 0x40 (1 file, 2 dir,
    3 char dev, 4 symlink, 5 socket, 6 fifo; isatty() is type 3 or 6), u16 nlink 0x42,
    u16 perm 0x44 (rwx, 3 bits), u32 size 0x48, u32 mtime 0x4c, u16 uid? 0x50, u32 ino 0x54
    (from the stat conversion routine at 0x254b0).

usage:
  python3 -I -m a465.crosemu --exe BIN --outdir DIR [--read FILE ...] [--stdin FILE] [--time T]
                        [--log calls.jsonl] [--env NAME=VAL ...] -- ARGV0 [ARGS...]
Exit status: the guest's exit status; 125 if the guest died (abort, bad memory access, unknown call).
"""
import argparse
import errno as E
import json
import os
import struct
import sys
import time as _time

from unicorn import (Uc, UcError, UC_ARCH_X86, UC_MODE_32, UC_PROT_ALL, UC_HOOK_INTR,
                     UC_HOOK_MEM_INVALID)
from unicorn.x86_const import UC_X86_REG_EAX, UC_X86_REG_ESP, UC_X86_REG_EIP

PAGE = 0x1000
MEM_TOP = 0x01000000          # 16 MB address space for the guest
STACK_TOP = MEM_TOP - PAGE

SYSCALL_NAMES = {1: 'exit', 2: 'open', 3: 'close', 4: 'read', 5: 'write', 8: 'lseek', 13: 'unlink',
                 19: 'stat', 20: 'fstat', 21: 'chmod', 26: 'dup2', 33: 'getpid', 42: 'time',
                 52: 'kill', 54: 'signal', 59: 'sigprocmask', 62: 'strerror'}
O_READ, O_WRITE, O_APPEND, O_CREAT, O_TRUNC = 1, 2, 0x8, 0x200, 0x400

ERRTEXT = {E.ENOENT: 'No such file or directory', E.EACCES: 'Permission denied',
           E.EBADF: 'Bad file descriptor', E.EINVAL: 'Invalid argument', E.EIO: 'I/O error',
           E.EMFILE: 'Too many open files', E.ENOSPC: 'No space left on device'}


class GuestDied(Exception):
    pass


class HostFile:
    """An open guest fd backed by a host file object (or a capture buffer for 1/2)."""
    def __init__(self, kind, f=None, path=None, readable=False, writable=False, append=False):
        self.kind, self.f, self.path = kind, f, path
        self.readable, self.writable, self.append = readable, writable, append
        self.refs = 1


class Emu:
    def __init__(self, exe, outdir, reads, stdin_path, fixed_time, log_path, env, argv, echo=True):
        self.outdir = os.path.realpath(outdir)
        if not os.path.isdir(self.outdir):
            raise SystemExit(f'--outdir {outdir} must exist')
        self.reads = {os.path.realpath(p) for p in reads}
        self.fixed_time = fixed_time
        self.echo = echo                 # also copy guest fds 1/2 to our stdout/stderr
        self.log = open(log_path, 'w') if log_path else None
        self.calls = []
        self.exit_status = None
        self.died = None
        self.captured = {1: bytearray(), 2: bytearray()}
        self.signals = {}
        self.uc = Uc(UC_ARCH_X86, UC_MODE_32)
        self._load(exe)
        self.fds = {}
        stdin_data = open(stdin_path, 'rb').read() if stdin_path else b''
        self.stdin = HostFile('stdin', readable=True)
        self.stdin.buf, self.stdin.pos = stdin_data, 0
        self.fds[0] = self.stdin
        self.fds[1] = HostFile('out', writable=True)
        self.fds[1].which = 1
        self.fds[2] = HostFile('out', writable=True)
        self.fds[2].which = 2
        self._stack(argv, env)
        self.uc.hook_add(UC_HOOK_INTR, self._intr)
        self.uc.hook_add(UC_HOOK_MEM_INVALID, self._badmem)

    # ---------------------------------------------------------------- loading
    def _load(self, exe):
        img = open(exe, 'rb').read()
        midmag, a_text, a_data, a_bss, a_syms, a_entry, a_trsize, a_drsize = struct.unpack_from('<8I', img)
        magic, mid = midmag & 0xffff, (midmag >> 16) & 0x3ff
        if magic != 0o314 or mid != 0x86:
            raise SystemExit(f'{exe}: not an i386 QMAGIC a.out (midmag {midmag:#010x})')
        if a_trsize or a_drsize:
            raise SystemExit(f'{exe}: has relocations; not handled')
        if len(img) < a_text + a_data:
            raise SystemExit(f'{exe}: file shorter than text+data')
        self.hdr = dict(midmag=midmag, text=a_text, data=a_data, bss=a_bss, syms=a_syms, entry=a_entry)
        self.uc.mem_map(PAGE, MEM_TOP - PAGE, UC_PROT_ALL)   # page 0 left unmapped: NULL traps
        self.uc.mem_write(PAGE, img[:a_text + a_data])
        self.end_bss = PAGE + a_text + a_data + a_bss
        self.entry = a_entry

    def _stack(self, argv, env):
        strings = bytearray()
        offs = []
        for s in list(argv) + list(env):
            offs.append(len(strings))
            strings += s.encode('latin-1') + b'\0'
        base = (STACK_TOP - len(strings) - 64) & ~0xf
        self.uc.mem_write(base, bytes(strings))
        ptrs = [base + o for o in offs]
        av, ev = ptrs[:len(argv)], ptrs[len(argv):]
        words = [len(argv)] + av + [0] + ev + [0]
        sp = (base - 4 * len(words) - 16) & ~0xf
        self.uc.mem_write(sp, struct.pack(f'<{len(words)}I', *words))
        self.uc.reg_write(UC_X86_REG_ESP, sp)

    # ---------------------------------------------------------------- helpers
    def rd32(self, a):
        return struct.unpack('<I', self.uc.mem_read(a, 4))[0]

    def cstr(self, a, limit=4096):
        out = bytearray()
        while len(out) < limit:
            chunk = self.uc.mem_read(a + len(out), 64)
            z = chunk.find(b'\0')
            if z >= 0:
                out += chunk[:z]
                break
            out += chunk
        return out.decode('latin-1')

    def host_path(self, gpath):
        return os.path.realpath(os.path.join(self.outdir, gpath))

    def in_outdir(self, hp):
        return hp == self.outdir or hp.startswith(self.outdir + os.sep)

    def new_fd(self, hf):
        for fd in range(3, 32):
            if fd not in self.fds:
                self.fds[fd] = hf
                return fd
        return -E.EMFILE

    def cros_stat(self, kind, size=0, mtime=0, perm=7):
        b = bytearray(0x60)
        struct.pack_into('<HHH', b, 0x40, kind, 1, perm)
        struct.pack_into('<II', b, 0x48, size, mtime)
        return bytes(b)

    # ---------------------------------------------------------------- hooks
    def _badmem(self, uc, access, addr, size, value, _):
        eip = uc.reg_read(UC_X86_REG_EIP)
        self.died = f'invalid memory access type {access} at {addr:#x} (size {size}) eip={eip:#x}'
        return False

    def _intr(self, uc, intno, _):
        eip = uc.reg_read(UC_X86_REG_EIP)
        if intno != 0x40:
            self.died = f'unexpected interrupt {intno:#x} at eip {eip:#x}'
            uc.emu_stop()
            return
        num = uc.reg_read(UC_X86_REG_EAX)
        esp = uc.reg_read(UC_X86_REG_ESP)
        args = [self.rd32(esp + 4 * i) for i in range(1, 5)]
        rec = {'n': len(self.calls), 'eip': f'{eip:#x}', 'num': num, 'name': SYSCALL_NAMES.get(num, '?')}
        try:
            ret = self.dispatch(num, args, rec)
        except GuestDied as e:
            self.died = str(e)
            ret = None
        rec['ret'] = ret
        self.calls.append(rec)
        if self.log:
            self.log.write(json.dumps(rec) + '\n')
        if ret is None:
            uc.emu_stop()
        else:
            uc.reg_write(UC_X86_REG_EAX, ret & 0xffffffff)

    # ---------------------------------------------------------------- system calls
    def dispatch(self, num, a, rec):
        if num == 1:                                     # exit(status)
            self.exit_status = a[0] & 0xff
            rec['args'] = [a[0]]
            return None
        if num == 2:                                     # open(path, flags, perm)
            path, flags, perm = self.cstr(a[0]), a[1], a[2]
            rec['args'] = [path, hex(flags), perm]
            hp = self.host_path(path)
            rec['host'] = hp
            acc = flags & 3
            if acc == O_READ and not (flags & (O_CREAT | O_TRUNC)):
                if hp in self.reads:
                    if not os.path.isfile(hp):
                        return -E.ENOENT
                    return self.new_fd(HostFile('file', open(hp, 'rb'), hp, readable=True))
                rec['denied'] = 'not an allowed input'
                return -E.ENOENT
            if acc in (O_WRITE, 3):
                if not self.in_outdir(hp) or os.path.isdir(hp):
                    rec['denied'] = 'write outside --outdir'
                    return -E.EACCES
                exists = os.path.exists(hp)
                if not exists and not (flags & O_CREAT):
                    return -E.ENOENT
                mode = 'r+b' if exists else 'w+b'
                f = open(hp, mode)
                if flags & O_TRUNC:
                    f.truncate(0)
                return self.new_fd(HostFile('file', f, hp, readable=(acc == 3), writable=True,
                                            append=bool(flags & O_APPEND)))
            rec['denied'] = 'unhandled flags'
            return -E.EINVAL
        if num == 3:                                     # close(fd)
            fd = a[0]
            rec['args'] = [fd]
            hf = self.fds.pop(fd, None)
            if hf is None:
                return -E.EBADF
            hf.refs -= 1
            if hf.refs == 0 and hf.f:
                hf.f.close()
            return 0
        if num == 4:                                     # read(fd, buf, n)
            fd, buf, n = a[0], a[1], a[2]
            rec['args'] = [fd, hex(buf), n]
            hf = self.fds.get(fd)
            if hf is None or not hf.readable:
                return -E.EBADF
            if hf.kind == 'stdin':
                data = hf.buf[hf.pos:hf.pos + n]
                hf.pos += len(data)
            else:
                data = hf.f.read(n)
            self.uc.mem_write(buf, data)
            return len(data)
        if num == 5:                                     # write(fd, buf, n)
            fd, buf, n = a[0], a[1], a[2]
            data = bytes(self.uc.mem_read(buf, n))
            rec['args'] = [fd, hex(buf), n]
            hf = self.fds.get(fd)
            if hf is None or not hf.writable:
                return -E.EBADF
            if hf.kind == 'out':
                rec['text'] = data.decode('latin-1')[:200]
                self.captured[hf.which] += data
                if self.echo:
                    (sys.stdout if hf.which == 1 else sys.stderr).buffer.write(data)
                    (sys.stdout if hf.which == 1 else sys.stderr).flush()
            else:
                if hf.append:
                    hf.f.seek(0, 2)
                hf.f.write(data)
            return n
        if num == 8:                                     # lseek(fd, off, whence)
            fd, off, whence = a[0], struct.unpack('<i', struct.pack('<I', a[1]))[0], a[2]
            rec['args'] = [fd, off, whence]
            hf = self.fds.get(fd)
            if hf is None:
                return -E.EBADF
            if hf.kind in ('out',):
                return -E.EINVAL                         # "ESPIPE"-like; stdio copes
            if hf.kind == 'stdin':
                return -E.EINVAL
            if whence not in (0, 1, 2):
                return -E.EINVAL
            return hf.f.seek(off, whence)
        if num == 13:                                    # unlink(path)
            path = self.cstr(a[0])
            hp = self.host_path(path)
            rec['args'] = [path]
            rec['host'] = hp
            if not self.in_outdir(hp):
                rec['denied'] = 'outside --outdir'
                return -E.EACCES
            if not os.path.isfile(hp):
                return -E.ENOENT
            os.unlink(hp)
            return 0
        if num == 19:                                    # stat(path, buf)
            path = self.cstr(a[0])
            hp = self.host_path(path)
            rec['args'] = [path, hex(a[1])]
            if hp in self.reads or self.in_outdir(hp):
                if os.path.isdir(hp):
                    self.uc.mem_write(a[1], self.cros_stat(2))
                    return 0
                if os.path.isfile(hp):
                    st = os.stat(hp)
                    self.uc.mem_write(a[1], self.cros_stat(1, st.st_size, int(st.st_mtime)))
                    return 0
            return -E.ENOENT
        if num == 20:                                    # fstat(fd, buf)
            fd = a[0]
            rec['args'] = [fd, hex(a[1])]
            hf = self.fds.get(fd)
            if hf is None:
                return -E.EBADF
            if hf.kind in ('stdin', 'out'):
                self.uc.mem_write(a[1], self.cros_stat(3))   # character device (a tty)
            else:
                hf.f.flush()
                st = os.fstat(hf.f.fileno())
                self.uc.mem_write(a[1], self.cros_stat(1, st.st_size, int(st.st_mtime)))
            return 0
        if num == 21:                                    # chmod(path, perm3)
            path = self.cstr(a[0])
            hp = self.host_path(path)
            rec['args'] = [path, a[1]]
            if not self.in_outdir(hp):
                rec['denied'] = 'outside --outdir'
                return -E.EACCES
            return 0                                     # recorded, not applied
        if num == 26:                                    # dup2(old, new)
            old, new = a[0], a[1]
            rec['args'] = [old, new]
            hf = self.fds.get(old)
            if hf is None or not (0 <= new < 32):
                return -E.EBADF
            if new in self.fds and new != old:
                self.dispatch(3, [new, 0, 0, 0], {})
            self.fds[new] = hf
            hf.refs += 1
            return new
        if num == 33:                                    # getpid()
            return 42
        if num == 42:                                    # time()
            return self.fixed_time if self.fixed_time is not None else int(_time.time())
        if num == 52:                                    # kill(pid, sig) -- only abort() uses it
            rec['args'] = [a[0], a[1]]
            raise GuestDied(f'guest called kill({a[0]}, {a[1]}) (abort)')
        if num == 54:                                    # signal(sig, handler, &old)
            rec['args'] = [a[0], hex(a[1]), hex(a[2])]
            old = self.signals.get(a[0], 0)
            self.signals[a[0]] = a[1]
            if a[2]:
                self.uc.mem_write(a[2], struct.pack('<I', old))
            return 0
        if num == 59:                                    # sigprocmask(how, set, old)
            rec['args'] = [a[0], hex(a[1]), hex(a[2])]
            return 0
        if num == 62:                                    # strerror(buf, len, errno)
            buf, n, code = a[0], a[1], a[2]
            rec['args'] = [hex(buf), n, code]
            text = ERRTEXT.get(code, f'error {code}').encode()[:max(0, n - 1)] + b'\0'
            self.uc.mem_write(buf, text)
            return 0
        raise GuestDied(f'unknown system call {num} args {[hex(x) for x in a]}')

    # ---------------------------------------------------------------- run
    def run(self, max_seconds=120):
        try:
            self.uc.emu_start(self.entry, 0xffffffff, timeout=int(max_seconds * 1e6))
        except UcError as e:
            if self.died is None:
                self.died = f'unicorn error {e} at eip {self.uc.reg_read(UC_X86_REG_EIP):#x}'
        if self.exit_status is None and self.died is None:
            self.died = f'emulation stopped without exit (timeout?) at eip {self.uc.reg_read(UC_X86_REG_EIP):#x}'
        for hf in list(self.fds.values()):
            if hf.f:
                try:
                    hf.f.close()
                except Exception:
                    pass
        if self.log:
            self.log.close()
        return self.exit_status if self.died is None else 125


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--outdir', required=True)
    ap.add_argument('--read', action='append', default=[], help='host file the guest may read')
    ap.add_argument('--stdin', help='file to feed on guest fd 0 (default: empty)')
    ap.add_argument('--time', type=lambda s: int(s, 0), help='value time() returns (default: now)')
    ap.add_argument('--log', help='write one JSON line per system call here')
    ap.add_argument('--env', action='append', default=[], help='NAME=VALUE for the guest environment')
    ap.add_argument('--timeout', type=float, default=120)
    ap.add_argument('argv', nargs='+')
    o = ap.parse_args()
    emu = Emu(o.exe, o.outdir, o.read, o.stdin, o.time, o.log, o.env, o.argv)
    st = emu.run(o.timeout)
    summary = {}
    for c in emu.calls:
        summary[c['name']] = summary.get(c['name'], 0) + 1
    print(f'\n[crosemu] exit={emu.exit_status} died={emu.died} calls={len(emu.calls)} {summary}',
          file=sys.stderr)
    sys.exit(st if st is not None else 125)


if __name__ == '__main__':
    main()
