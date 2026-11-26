# RAPL-3 compiler: the vendor's `rc1` run on this PC

## Now in the repo (2026-11-26)

The owner decided to keep the compiler. The spike is now a tool:

- `scripts/a465-r3c SOURCE.r3 [-o OUT] [--libs ours|c500c] [--stamp-check] [rc1 options such as -s -Wall -x -l]`
  compiles on the PC and prints the compiler's messages; it exits non-zero on failure. `--libs ours` (the
  default) links this unit's `syslib.r` and `robotlib.r`; `--lib FILE` adds libraries, `--extra FILE`
  supplies a file the source `.include`s, `--time 0x...` pins the build time. It sends nothing to the
  controller (use `scripts/a465-putfile` for that).
- Code: `a465/r3c.py` (the driver: rpp, then rc1) and `a465/crosemu.py` (loader and system-call shim; reads
  only the files it is handed, writes only inside the work directory). Needs `python3-unicorn`.
- Vendor files: `vendor/crs/` (see its README: what each file is, sizes, SHA-256, and that they are CRS's,
  not ours). The flash image and its extractor are not in the repo.
- Tests: `tests/test_r3c.py` (skipped without unicorn): fib and sieve reproduce the vendor's builds byte
  for byte; hello and calc decode fully and run in `a465/r3vm.py`; a syntax error fails with a message;
  the shim's sandbox; our libraries' stamps.
- Compile the controller-check programs: `controller/programs/r3c-hello` and `r3c-fib` (both `-s`).

**Proven on the controller, 2026-11-26:** two programs built on the PC with `scripts/a465-r3c` against
this controller's libraries ran on the controller (CROS 1.16.471): `controller/programs/r3c-hello` (152
bytes) printed `hello from rc1`, and `r3c-fib` (376 bytes, the vendor's sample) printed its table
correctly through fib(23) before it was stopped with Ctrl-E (the last line is slow). So the controller
loads the compiler's output. The sections below are the spike's write-up as it was; where they say no vendor
file is in the repo or that the spike lives in the scratchpad, this section supersedes them.

Feasibility spike, 2026-11-25. Question: can the RAPL-3 compiler pass from a C500C (CROS 3.1.1249)
be run on this Linux PC, and is what it produces usable on our controller (C500, CROS 1.16.471)?
Nothing was sent to the controller and the serial port was not opened.

The spike's code, logs and outputs are outside the repo, in the session scratchpad
`/tmp/claude-1000/-home-buzz-dev/1aa7efb4-83d6-4b44-8584-24aebe5b3c9c/scratchpad/r3c-spike/`
(called SPIKE below; its `README.md` says how to run it, `reproduce.sh` re-runs everything in about
7 seconds and ends with `ALL PASS`). The scratchpad can be cleared by a reboot.

## Short answer

1. **What rc1 is.** A statically linked i386 a.out (FreeBSD QMAGIC layout) built with a BSD 4.4
   C library, calling the CROS kernel through its own gate `int $0x40` with CROS numbers, not
   FreeBSD's. It uses 17 system calls, all plain file I/O plus exit, time, signals and an
   error-text call. It is the second of three passes: `/bin/r3c` (driver, also x86) runs
   `/bin/rpp` (preprocessor, x86) and then `/bin/rc1`. rc1 needs nothing but the source text and
   the compiled library files named with `-L` (`syslib.r`, `robotlib.r`, ...). It has no built-in
   paths. All three binaries and the C500C's libraries are in the downloaded repository
   (`rc1` in `cros_system_bin/`, the rest inside `file_transfer/flash.bin`).
2. **Under emulation: yes.** It runs in Unicorn with a loader and a system-call shim written for
   this spike (no host pass-through). It printed its usage and version, compiled a one-line
   program and compiled every source tried: `tests/getrotspd.r3`, `trajectory/simple_test.r3`, both
   `simple_message` servers (with `.include` through rpp), the vendor's own `/app/test/fib.r3` and
   `sieve.r3`, and our test programs. A compile takes about 0.3 s.
   **Fidelity check:** with the vendor's libraries and build time, `fib` and `sieve` come out
   **byte-identical** to the vendor's own compiled `/app/test/fib` and `/app/test/sieve`, which are
   also on our controller.
3. **What it produces: a complete program file** in exactly the format our tools know (magic,
   16-word header, check word, library table); no link step. `a465/r3dis.py` decodes every word of
   every output, and every opcode in them also occurs in this unit's own programs. Compiled with
   `-L` of **this unit's** `/lib/syslib.r` and `/lib/robotlib.r`, the output carries this unit's
   stamps (syslib 0x377a8993 / 0x0a0d4875, robotlib 0x377a899b / 0xae952686) and this unit's
   robotlib routine numbers, so nothing needs changing. The other way round (compile against the
   C500C libraries, then re-stamp) is wrong for robotlib users: 39 robotlib routines have
   different numbers in the two firmware generations, and at least one has a different parameter
   list.
4. **`a465/r3vm.py` runs compiled programs correctly:** a printf one-liner and a program with a
   recursive function, a loop and float arithmetic gave the expected output.

Not established: that CROS 1.16.471 loads and runs these files. That needs one supervised run on
the controller (see "Next steps").

## The binary and the legal position

`rc1`, `rpp`, `r3c` and the `.r` libraries are CRS Robotics software (the strings of r3c and rpp
include "Copyright (c) 1996 CRS Robotics Corporation"). They are copies of a C500C's system files, found in
the public repository github.com/PetteriAimonen/crs-c500c-utils (checkout of 2025-05-16, commit
23d410c4); that repository's BSD licence covers its author's own files, not the vendor's. **None of
the vendor files is copied into this repo**; the spike reads them where they were downloaded. Using
them as a compiler for our own controller is the owner's decision; this note is not legal advice.
Our controller has no compiler on it (checked 2026-11-25), and we have no Robcomm3 or `r3c.exe`.

Hashes: rc1 `78d4e171...0e518f22` (sha256, 157396 bytes; identical to `/bin/rc1` at offset 0x10540
of `flash.bin`), rpp `a1043311...50a53726` (21184 bytes), r3c `e59b90b3...25842e30` (21208 bytes).
All dated 2001-06-15 in the flash file headers.

The repository's files were treated as data: nothing from it was run natively, no script of it was
run, and the only things executed were rc1 and rpp inside the emulator. The emulator package was the
only installation: `sudo apt-get install python3-unicorn` (Ubuntu 2.1.4, which brought
`libunicorn2t64`). `objdump` was already present.

## 1. What rc1 is

### a.out header (first 32 bytes of the file)

```
00000000: cc00 8600 0050 0200 b416 0000 0036 0000  .....P.......6..
00000010: 0000 0000 2010 0000 0000 0000 0000 0000  .... ...........
```

| Field | Value | Meaning |
|---|---|---|
| a_midmag | 0x008600cc | magic 0314 = QMAGIC (FreeBSD "compact demand paged"), machine 0x86 = i386, flags 0 |
| a_text | 0x25000 (151552) | includes the 32-byte header; file offset 0 is loaded at 0x1000 |
| a_data | 0x16b4 (5812) | at 0x26000 |
| a_bss | 0x3600 (13824) | 0x276b4-0x2acb4 |
| a_syms | 0 | no symbol table |
| a_entry | 0x1020 | first byte after the header |
| a_trsize, a_drsize | 0, 0 | no relocations |

The file is 157396 bytes: text + data (157364) and 32 zero bytes. `r3interp` has the same layout
(text 0x13000), and so do `rpp` (text 0x5000), `r3c` (0x5000) and `/sbin/zcat` in `flash.bin`.

**Statically linked:** no relocations, no dynamic-linker path or symbols among its strings, the
entry goes straight into crt0 (0x1020: computes environ from argc/argv, calls main at 0xc9fc, then
exit at 0x1eb78), and the system-call stubs are in the text. The C library is BSD 4.4 stdio and
vfprintf ("bug in vfprintf: bad base" at file offset 0x1eee7); the code is gcc-style
(`add $0xfffffff8,%esp` before calls).

### System calls

The convention, read from the stubs: `mov $N,%eax; int $0x40`; arguments are the caller's stack
(cdecl); the result is in eax and a negative value is minus an errno, which most stubs also store in
`errno` at 0x26004 (`or %eax,%eax; jns; neg; mov %eax,0x26004; neg; ret`). There is no `int $0x80`
and no FreeBSD `lcall $7,$0` in rc1, rpp or r3c. The numbers are CROS's own: 2 is open, 3 close,
4 read, 5 write (FreeBSD has 2 fork, 3 read, 4 write, 5 open). errno values look BSD (rc1's fopen
mode parser sets 22 for EINVAL).

Each stub was identified from the library routine that calls it (addresses are virtual, file offset
+ 0x1000):

| # | Call | Stub | Identified from |
|---|---|---|---|
| 1 | exit(status) | 0x25a74 | `_exit` at 0x25468, called last by exit() |
| 2 | open(path, flags, perm) | 0x25aa4 | wrapper 0x25630 passes `(mode >> 6) & 7` as perm; fopen. Flags from the mode parser at 0x25654: "r" 1, "w" 2+0x600, "a" 2+0x208, "+" 3, so 1 read, 2 write, 3 both, 0x8 append, 0x200 create, 0x400 truncate |
| 3 | close(fd) | 0x256e8 | freopen at 0x1fae3 (after dup2), stdio close 0x2460d |
| 4 | read(fd, buf, n) | 0x258c4 | stdio `__sread` 0x24540 |
| 5 | write(fd, buf, n) | 0x2588c | stdio `__swrite` 0x24578 (preceded by lseek(fd, 0, 2) when appending) |
| 8 | lseek(fd, off, whence) | 0x25ad4 | wrapper 0x258a4; 32-bit offset |
| 13 | unlink(path) | 0x1f638 | 0xa669: removes the object file when the error flag is set |
| 19 | stat(path, buf) | 0x25a8c | 0x255ec: stat then the conversion at 0x254b0 |
| 20 | fstat(fd, buf) | 0x25874 | 0x255b0 (same conversion); isatty 0x214e4 tests the type for 3 or 6 |
| 21 | chmod(path, perm) | 0x1f620 | 0xa67b: chmod(object file, 7) after a good write |
| 26 | dup2(old, new) | 0x25700 | freopen 0x1fad3 |
| 33 | getpid() | 0x25754 | abort() 0x1fd70: kill(getpid(), 6) |
| 42 | time() | 0x25718 | time(t) 0x1fc44; main calls time(0) at 0xca20 |
| 52 | kill(pid, sig) | 0x2576c | abort() |
| 54 | signal(sig, fn, &old) | 0x25abc | signal() 0x25730 returns the old handler |
| 59 | sigprocmask(how, set, old) | 0x25784 | abort(): (3, all but SIGABRT, 0) |
| 62 | strerror(buf, len, errno) | 0x25628 | strerror() 0x1f714: (0x27dd4, 64, e), returns the buffer. The kernel supplies the text; rc1 holds no error-message table |

CROS's stat record (the buffer is 0x60 bytes), from the conversion routine at 0x254b0: u16 type at
0x40 (1 file, 2 directory, 3 character device, 4 symlink, 5 socket, 6 fifo), u16 link count 0x42,
u16 permission 0x44 (3 bits rwx, which the conversion copies to owner, group and other), u32 size
0x48, u32 time 0x4c, u16 at 0x50 (copied to st_uid's place), u32 at 0x54 (copied to st_ino).

`rpp` uses calls 1, 2, 3, 4, 5, 8, 19, 20, 26, 62: a subset, so the same shim runs it. `r3c` uses
those plus 13, 31, 33, 37, 38, 39; 31, 37, 38 and 39 were not identified (r3c was not run; its
"Preprocessor fork failed" and "Compiler fork failed" messages suggest process creation).

**Memory:** `sbrk` (0x25480) only adds to a pointer that starts at the end of bss (0x2acb4) and
makes no system call and no check, so a CROS process simply owns the memory above its bss up to its
stack. getpagesize returns 0x1000 and getdtablesize 32.

### Strings: version, usage, options

`rc1 -V` prints `RAPL-3 Compiler Main Pass -- Version 1.0` (file offset 0xb8d8). Its getopt string
is `a:D:E:f:L:lO:o:sVvW:x?` (0xb6e9). `rc1 -?` prints (0xaeb8-0xb3f8):

```
Usage: rc1 [-options] [filename]
  where [filename] is the RAPL-3 source file name (stdin is
  used if [filename] is omitted), and [-options] are:
  -afilename  read the rest of the options from this file
  -Dall       equivalent to -Dexpr -Dsym -Dtype
  -Dexpr      dump internal expr trees (also turns on -Dsym)
  -Dsym       dump symbols tables in human readable form
  -Dtype      dump type tables in human readable form
  -Efilename  redirect error messages to a file
  -fcs        be case sensitive
  -figt       implicitly declared vars are global & teachable
  -fnat       no auto teachables
  -fnimp      no implicit declarations
  -fstackck   emit stack checking code
  -fstack=NNN allocate NNN words of run-time stack
  -Lmodname   load definitions from a RAPL-3 object module
  -l          include line numbers (even if stripping)
  -ofilename  set output file name
  -O0         don't perform any optimization
  -O1         optimize jumps and remove dead code
  -O2         optimize common subexpressions [unimplemented]
  -s          strip all symbols from the object module
  -v          be verbose
  -V          print version info
  -Wall       warn on any possible problem
  -Wmax       Warn on anything even remotely questionable
  -Wnone      don't give any warnings
  -x          exclude local symbols from the object module
  -?          print this message
```

Against the manual's `r3c` (`docs/text/Robocomm3.txt` lines 1280-1357 and 1500-1560;
`docs/text/RAPL-3LanguageReferenceGuide.txt` lines 3660-3700): the compiler options `-fnat`, `-fnimp`,
`-fstack=`, `-fstackck`, `-l`, `-L`, `-O0`/`-O1`, `-s`, `-Wall`/`-Wmax`/`-Wnone`, `-x`, `-o`, `-a`
and `-v` are rc1's. The manual's `-E` (preprocess only), `-Dname[=def]` (macro), `-nolibs`, `-n`,
`-tdir`, `-efile` and `-h` belong to the driver: in rc1, `-E` names the error file and `-D` turns on
internal dumps. rc1 also has `-fcs`, `-figt`, `-O2` and `-V`, which the manual does not list.

Other strings that matter: `RAPL3BASE` (0xb6df) is fetched with getenv at 0xca46 and stored at
0x29104, and never read again (the only reference to 0x29104 in the file is that store); there are
no `/lib`, `/bin` or `/conf` paths in rc1. Its default output is `r.out` (0xb929) and its input
`(stdin)`. Library loading prints "Loading module '%s' from '%s'" and refuses with "Bad magic
number in library module", "Library module '%s' has been stripped", "Library '%s' needs version.
%d.%d of '%s'; found version %d.%d", "Library '%s' signature incompatible with '%s'". The scanner
reads line markers with `%c %d "%60[^"]"` (0x18e6c): `. N "file"`, keeping at most 60 characters
of the file name.

### The driver and the preprocessor (from `flash.bin`)

`flash.bin` (917504 bytes) is the C500C's flash file system: each file has a 64-byte header (u16
0xf1a5, u16, u32 kind, u32 flags, u32 mtime, u32 size, name[44]) followed by its data, contiguous.
80 of its files are gzip files (the bytecode programs among them); the five i386 a.out
programs (`/bin/r3c`, `/bin/rc1`, `/bin/rpp`, `/sbin/r3interp`, `/sbin/zcat`), the four `/lib/*.r`
libraries and `/sbin/stpv3` are stored uncompressed. Extracted with `SPIKE/cros_flashfs.py`.

- `/bin/r3c` (x86, not bytecode; file header at `flash.bin` offset 0xb180, its strings at
  0xb238-0xc165): usage `%s [ option | filename ]...`, ".r3 C source; preprocess, compile,
  assemble", ".r ignored."; option string `vt:PEo:e:hD:W:lsxL:O:f:n:Na:P`. Its strings name `/conf/ldconf`, `/bin/rpp`, `/bin/rc1`,
  `-D%s=%s`, `__C500__`, `__RAPL3__`, `/rctXXXXX`, `-or.out`, "Preprocessor fork failed" and
  "Compiler fork failed". From those (r3c was not run): it reads the library list from ldconf, runs
  rpp and then rc1 with the options and one `-L` per library, with temporaries in `-t dir` or a pipe
  with `-P`; that `__C500__` and `__RAPL3__` are what it predefines for rpp is inferred.
- `/conf/ldconf` on the C500C: `syslib /lib/syslib.r`, `robotlib /lib/robotlib.r`,
  `v3access /lib/v3access.r`, `stp /lib/stp.r`, `acid /lib/acid.r`.
- `/bin/rpp` ("RAPL-3 Preprocessor"): `rpp [options] [infile] [outfile]`, options `-D`, `-E`, `-L`
  (no line information); handles `.include`, `.define`, `.undef`, `.ifdef`, `.ifndef`, `.else`,
  `.endif`, `.error`, `__LINE__`, `__FILE__`, and writes `. N "file"` markers.

### What rc1 needs, and what we have

| Needed | Have |
|---|---|
| rc1 | yes (downloaded) |
| rpp, for `.include` / `.define` | yes, in `flash.bin`; runs under the same shim |
| r3c, the driver | in `flash.bin`, not needed: `SPIKE/r3c_emu.py` does its job (rpp, then rc1 with `-L` per library) |
| This unit's `/lib/syslib.r`, `/lib/robotlib.r` | yes, `sessions/controller-files/lib/` |
| This unit's `/lib/stp.r` (45632 bytes) and `/lib/v3access.r` (9196 bytes) | **no**: they are on the controller (listing in `controller/backup-2026-11-23/listing.txt`) but not copied off. Needed only for programs that call teach-pendant or v3 routines |
| `/conf/ldconf` | not in our unit's `/conf` (listing of 2026-11-23); only r3c reads it |

## 2. Running it under emulation

`SPIKE/crosemu.py`: Unicorn x86-32; maps 0x1000-0x1000000 (page 0 unmapped so NULL faults), loads
text+data at 0x1000, zeroes bss, builds argc/argv/envp at the top of the stack and starts at 0x1020.
`int $0x40` is caught by an interrupt hook and handled by Python code; any other interrupt, unknown
call number or bad memory access stops the run. File policy: a read-only open succeeds only for files
named with `--read`; creating or writing, unlinking and chmod succeed only inside the output
directory, which is also the guest's current directory; stdout and stderr are captured. `--time`
fixes what time() returns. Every call is logged as a JSON line.

Results (all in `SPIKE/runs/`):

| Run | What | Result |
|---|---|---|
| 01 | `rc1 -V` | prints the version line, then (no file given) compiles empty stdin to `r.out`, 100 bytes |
| 02 | `rc1 -?` | the usage text above, exit 1 |
| 08 hello | `main` / `printf("hello from rc1\n")` / `end main` | 220-byte program; 152 bytes with `-s` (run 11) |
| 07 | `tests/getrotspd.r3`, `trajectory/simple_test.r3`, against both library sets | compile; 1996 / 1040 bytes against ours, 4 bytes more against the C500C's |
| 12 | `simple_message/robot_control_server.r3`, `robot_status_server.r3` (`.include "serial_framing.r3"`) | compile through rpp + rc1; 5732 / 2112 bytes |
| 04 | vendor `fib.r3`, C500C libraries, `-s`, time 0x3b2a12bd | **identical** to the vendor's `/app/test/fib` (376 bytes, sha256 3d6ac8fe...) |
| 05 | vendor `sieve.r3` (uses `.define`), rpp then rc1 `-s`, time 0x3b2a12bf | **identical** to the vendor's `/app/test/sieve` (sha256 1bb308e7...) |
| 06 | vendor `fib.r3` against **our** libraries | equals the vendor's fib re-stamped for our unit with `crosprog.restamp` |
| 08 bad | syntax errors | `bad.r3:3: Error: Syntax error before '('` etc., exit 1, no output file |
| 09 | `autohome()` (C500C only) against ours; `robotisdone()` (ours only) against the C500C's | compile error each way; each compiles against its own library |
| 10 | a library of our own (`libversion 1, 0`, `export` and `global` routines, `-x`) and a program using it (`mylib:plusone(41)`, `plustwo(40)`) | both build; the program's library table gets a slot-4 record for `mylib` |
| 11 | `-fstack=384/1000/5000` | stored in header word 5, high half (0x180, 0x3e8, 0x1388); 0 by default |
| 14 | `calc` with no option, `-x`, `-s`, `-l` | 572, 368, 360, 572 bytes; which of header words 9-11 are kept: see "Unknown" |
| 13 | missing `-L` file; `-E` error file; a library that links nothing | "No such file or directory on file ..." via call 62; error file via dup2; the last trips an rc1 assertion (`module_xref[i] != NULL` at modules.c:684) and abort |

The vendor's `fib` and `sieve` (gzip files dated 2001-06-15) are on our controller too, in
`/app/test` with their sources, and do not run there because this firmware refuses gzip programs
(`docs/CONTROLLER.md`); the files in `flash.bin` are identical to ours
(`controller/backup-2026-11-23/app/test/`).

### Every system call made (47 logs from `reproduce.sh`, 4437 calls)

| # | Call | Count | By | Answered |
|---|---|---|---|---|
| 1 | exit | 46 | rc1, rpp | stop; status 0 or 1 |
| 2 | open | 156 | rc1, rpp | fd for inputs (flags 0x1) and outputs (0x603 "w+", 0x602 "w"); one -ENOENT, the deliberately missing `/nonexistent/robotlib.r` (not an allowed input) |
| 3 | close | 128 | rc1, rpp | 0 |
| 4 | read | 2306 | rc1, rpp | data; 512-byte reads of libraries and sources, 1 byte from empty stdin in run 01 |
| 5 | write | 1170 | rc1, rpp | captured (stdout is written a byte at a time) or written to the output file |
| 8 | lseek | 363 | rc1 | host seek: the object file is written, then rc1 seeks back and rewrites the header |
| 20 | fstat | 212 | rc1, rpp | fds 0-2 as type 3, files as type 1 with their size |
| 21 | chmod | 22 | rc1 | (object file, 7); recorded, not applied |
| 26 | dup2 | 1 | rc1 | (3, 2) for `-E` |
| 33 | getpid | 1 | rc1 | 42, in abort() |
| 42 | time | 29 | rc1 | the `--time` value |
| 52 | kill | 1 | rc1 | (42, 6) in abort(): run stopped |
| 59 | sigprocmask | 1 | rc1 | 0, in abort() |
| 62 | strerror | 1 | rc1 | (0x27dd4, 64, 2): "No such file or directory" |

Never called in these runs: 13 unlink, 19 stat, 54 signal.

## 3. What it produces

A complete program: the same 64-byte header, check word, code, run-length data stream, library
table and names that `a465/crosprog.py` and `a465/r3dis.py` describe, plus, unless `-s` is given,
three more sections at header words 9-11 (with them, `rc1 -v` reports writing a type table,
symbols and strings; which word points at which was not checked).
No link step: library routines are bound by number at compile time from the `-L` files.

Header words, this unit's own programs next to rc1 output (`hello`, `calc` built against our
libraries without `-s`; `fib` built against ours with `-s`; the vendor's 2001 `fib`):

```
word  calrdy(ours) home(ours)  hello      calc       fib(ours)  fib(2001)
w0    10adab1f     10adab1f    10adab1f   10adab1f   10adab1f   10adab1f   magic
w1    0            0           0          0          0          0          (library version in a library)
w2    377a89a7     377a89a8    692600c0   692600c0   3b2a12bd   3b2a12bd   build time = time()
w3    00810001     00810001    00810001   00810014   00810014   00810014   low half: entry word
w4    00100041     00130086    00050008   000f0033   00140031   00140031   data words << 16 | code words
w5    0            0           0          0          0          0          -fstack << 16
w6    144          258         60         10c        104        104        code end
w7    0            0           0          0          0          0
w8    188          2a4         78         148        158        158        library table
w9    0            0           8c         15c        0          0          \
w10   0            0           90         160        0          0           > sections left out by -s
w11   0            0           a8         1cc        0          0          /
w12   1ac          2c8         b8         20c        16c        16c        library names
w13   1            1           1          1          1          1
w14   b746c67f     b743c2ed    85a650c3   859c4af1   b3933e16   b3933e16   check (crosprog.check_ok: all true)
```

Library tables, the vendor's 2001 `fib` and the same source compiled against our syslib:

```
2001 fib  00000158: 0100 0000 | 0000 0200 b012 2a3b 0000 0500 7548 0d0a | 0800 0000 syslib..
ours      00000158: 0100 0000 | 0000 0200 9389 7a37 0000 0500 7548 0d0a | 0800 0000 syslib..
```

and `getrotspd` compiled against ours next to this unit's `calrdy`:

```
calrdy     0x188: 0200 0000 | 0000 0200 9389 7a37 0000 0500 7548 0d0a | 0700 0300 9b89 7a37 0000 0200 8626 95ae
getrotspd  0x554: 0200 0000 | 0000 0200 9389 7a37 0000 0500 7548 0d0a | 0700 0300 9b89 7a37 0000 0200 8626 95ae
```

**Same bytecode generation.** Every word of every output decodes with `r3dis` (no stray words), and
the 38 opcodes the outputs use all occur among the 152 used by this unit's own programs and
libraries (`SPIKE/verify.py`). Independently, five of this unit's programs are byte-identical to
the 2001 firmware's builds once those are re-stamped with our stamps and build time: `calrdy`,
`cp`, `home`, `joint`, `more`. Of the others, `/diag/xzero` differs in one robotlib call number
(below), nine differ in size (changed sources), and `ash` and `v3make` link v3access, of which we
have no copy from our unit, so they were not compared. So the 1999 compiler and the 2001 rc1 emit
the same code for the same source, as far as these programs show.

**Stamps and signatures.** Compiled with `-L` of our libraries, a program records syslib build
0x377a8993 signature 0x0a0d4875 and robotlib build 0x377a899b signature 0xae952686, the values our
programs carry. The C500C's libraries: syslib build 0x3b2a12b0, signature 0x0a0d4875 (the same as
ours); robotlib build 0x3b2a12b3, signature 0x51b97334 (different); v3access 0x3b2a12b0 / 0x0e2fd1e7;
acid 0x3b2a12c1 / 0x96ef8dfa.

**Routine numbers** (from each library's own symbol records, `r3dis.library_routines`):

- syslib: the 81 routines (0x01-0x51) have the same numbers and names in both, and the 129 builtins
  they declare are the same. The C500C's has two more records named `SIG14` (0x108, 0x109).
- robotlib: ours 189 routines, the C500C's 204. 147 have the same number; 39 are renumbered
  (our 0x59-0x67 are one lower in the C500C's, which has no `robotisdone`, our 0x58; our 0xa1-0xab
  are one higher, the C500C's 0xa1 being `force_pwd_set`; our 0xb1-0xbd are one lower); 3 are only ours
  (`gen_force_param`, `init_kin_and_servo`, `robotisdone`) and 18 only the C500C's (`autohome`,
  jerk, radius and recovery routines and others). Example: `here` is 0x62 on our unit, 0x61 on the
  C500C, where our 0x61 is `accel_set`. `getrotspd` and `simple_test` compile to 0x62 against ours,
  0x61 against the C500C's.
- Parameter lists differ too: `robotisfinished()` compiles to a call with one argument word against
  the C500C's robotlib and none against ours.
- The 2001 `/diag/xzero`, re-stamped for our unit, differs from ours in exactly one word: it calls
  robotlib 0x59 (`axes_get` on the C500C, `robotislistening` on ours) where ours calls 0x5a.

**What a program built this way needs to load on CROS 1.16.471:** built against our unit's own
library files, nothing known. Built against the C500C's libraries, re-stamping (`crosprog.restamp`)
is right only if every robotlib routine it calls has the same number and parameters in both
libraries; for syslib-only programs that holds. Re-stamping writes our robotlib signature, so it
also defeats the check that would otherwise refuse a mismatched program ("signature
incompatible"). Always compile against this unit's libraries.

Header details learnt on the way (not yet in `a465/crosprog.py`'s docstring, which this spike did
not touch): header word 1 of a library is its `libversion` (major << 16 | minor: our `libversion 1,
0` gives 0x10000; syslib 0x50000, robotlib 0x20000); the u16 pair after a library record's build
time matches that library's version word as (minor, major): 0, 5 for syslib; 0, 2 for robotlib;
0, 1 for our test library; header word 5's high half is `-fstack`; `-s` zeroes words 9-11 and `-x`
zeroes word 11 only.
`r3dis.library_routines` lists a library's `global` routines but not its `export` ones (our test
library's `plusone` is missing from its result).

## 4. End to end in our interpreter

`a465/r3vm.py` runs the compiled programs:

| Program | r3vm output |
|---|---|
| `hello` (printf one-liner) | `hello from rc1\n`, exit 0, 6 instructions |
| `calc` (recursive fib, for loop, int to float, `print`) | `sum of fib(1..10) = 143\nquarter = 35.75\ndone\n`, exit 0, 2813 instructions |

The vendor's `fib` cannot be run in r3vm because it uses the format specs `{2}`, `{5}`, `{9.8}`,
which r3vm deliberately refuses.

## What works, what does not, what is unknown

Works (measured): rc1 and rpp run under Unicorn with a 17-call shim and no host access beyond the
named files; byte-identical reproduction of two vendor builds; compiling against this unit's
libraries gives this unit's stamps and routine numbers; libraries of our own and programs that use
them; `.include` and `.define`; error and warning messages with file and line; r3vm runs the output.

Does not work or needs care: a file name longer than 60 characters is cut in rc1's line markers
(`SPIKE/r3c_emu.py` copies the source into the work directory and uses the short name); a library
that links no other library makes rc1 assert when a program uses it (a one-routine library; the
same library with a printf in it, so that it links syslib, works); calling a routine the given libraries lack gives "Implicit declaration" and
"Syntax error before '('", not a clear "undefined" message.

Unknown:
- Whether CROS 1.16.471 loads and runs rc1 output. Evidence for: same format and opcodes; rc1's
  output for a syslib-only program equals a 2001 build re-stamped, and re-stamped 2001 builds
  (`homezc`) have run on this unit (2026-11-24).
- Whether a program built with no `-s` or `-x` (all three sections, words 9-11) loads on this
  unit. Run 14: `-s` zeroes words 9-11; `-x` keeps 9 and 10 and zeroes 11; no option keeps all
  three. On this unit, `/bin/pendant` and both libraries have words 9 and 10 set and 11 zero (the
  `-x` shape) and every other program has all three zero (the `-s` shape); none has word 11 set.
  So `-s` and `-x` output have a precedent on this unit; a full build has not.
- r3c's system calls 31, 37, 38, 39, and the values r3c gives `__C500__` and `__RAPL3__`.
- Whether the controller checks a program's library build times as well as the signatures.

## Recommendation

| Option | What it takes | Rough size |
|---|---|---|
| **A. Use the vendor's rc1 (and rpp) under emulation as the compiler** | Turn the spike into a tool: `scripts/a465-r3c` with our library list, `-o`, `-s`, build-time pinning, errors shown with file and line, the binaries' location given by the user (not in the repo), tests that skip when the binaries are absent; copy `stp.r` and `v3access.r` off the controller; one supervised controller run. | small to medium: a session or two. The emulator and driver exist (about 490 lines) |
| B. Write our own compiler, with rc1 as the reference | A RAPL-3 front end for the whole language (types including strings, locations, structs, unions, arrays, pointers; implicit declarations; teachables; try/except/retry/resume; varargs and var parameters; goto, case, with), code generation onto the known instruction set (`a465/r3asm.py`), the data stream, the type and symbol sections (not yet decoded), reading `-L` libraries, writing libraries, and a preprocessor. rc1 would serve as an oracle for byte-for-byte differential tests. | large: thousands of lines, many sessions; the bulk of a project |
| C. Neither | Keep writing programs in our assembler (`.r3s`). | nothing now; every program stays hand-written bytecode |

Recommended: **A**. It already produces vendor-identical output, takes RAPL-3 exactly as the manuals
describe it, and costs little more to finish. B makes sense only if the owner decides the vendor
binary must not be used (licence) or wants no Unicorn dependency; even then rc1 is the way to test it.

## Next steps (if A is chosen)

1. Copy `/lib/stp.r` and `/lib/v3access.r` off the controller with `scripts/a465-getfile`.
2. First controller run, supervised: send `SPIKE/runs/06-fib-ourlibs/fib` (the vendor's `fib.r3`
   built against our libraries, 376 bytes, no robot motion) to `/tmp` and run it; it should print the
   Fibonacci table that `/app/test/fib.r3` describes. Then `calc` built with `-x` and with no option
   (`SPIKE/runs/14-symopts-x/calc`, `SPIKE/runs/14-symopts-none/calc`), to answer the
   symbol-section question. Until that is answered, build with `-s` or `-x`.
3. Wrap the driver as a repo tool; keep the vendor files outside the repo.
