# vendor/crs: CRS Robotics files kept for the RAPL-3 compiler

**These are CRS Robotics Corporation's files, not ours. They are not covered by this repository's
licence.** They are kept here by the owner's decision of 2026-11-26 ("keep the compiler, stick it in the
repo"), only so that `scripts/a465-r3c` can compile RAPL-3 programs for our own controller. The compiler
binaries are never run natively: only inside `a465/crosemu.py`, an i386 emulator (Unicorn) that answers
every system call itself. See `docs/RAPL3-COMPILER.md`.

Nothing else from the source flash image is here: no kernel, no other programs, no flash image.

## Where they came from

- `c500c-3.1/`: the system files of a CRS C500C controller (CROS 3.1.1249), found in the public repository
  github.com/PetteriAimonen/crs-c500c-utils (checkout of 2025-05-16, commit 23d410c4), inside its
  `file_transfer/flash.bin`, and unpacked with the spike's flash-file-system extractor (not kept in this
  repo; the file headers are 64 bytes followed by contiguous data). `fib` and `sieve` are the
  vendor-compiled programs gunzipped (the image stores them gzipped); they are the reference outputs
  that `tests/test_r3c.py` reproduces byte for byte. All were dated 2001-06-15 in the image.
- `c500-1.16.471/lib/`: the four library files copied off our own controller (a C500, CROS 1.16.471)
  in November 2026 (`/lib/syslib.r`, `robotlib.r`, `v3access.r`, `stp.r`). Programs compiled against
  these carry this unit's stamps (syslib signature 0x0a0d4875, robotlib 0xae952686). The driver links
  `syslib.r` and `robotlib.r` by default; the others can be added with `--lib`.

## Files (bytes, SHA-256)

| File | Bytes | SHA-256 |
|---|---:|---|
| c500c-3.1/bin/rc1 (the compiler proper) | 157396 | 78d4e171d70f27f8f182688a31b8dc8b6e2c98d610df6d38765422ba0e518f22 |
| c500c-3.1/bin/rpp (preprocessor) | 21184 | a1043311ef4d6dcb22f51f20030fabfdfea8b54cc745d755d42e3caa50a53726 |
| c500c-3.1/bin/r3c (the driver on the controller; NOT used, kept for reference) | 21208 | e59b90b35524620d0da01a5e70178667a3714aa4cd35081acb48b4a225842e30 |
| c500c-3.1/lib/syslib.r | 35780 | 4b77916dab8ae9ae0b8db26b6d56d283fae77a6fe6801fbe154bdaaa949412ac |
| c500c-3.1/lib/robotlib.r | 56908 | 24af47f1d9d1dc27871ea9ad574e83b6418075dd85dbaaf0e90b213e78dc1de7 |
| c500c-3.1/lib/v3access.r | 11988 | 85826d6b6d4f26133a018dbb299a5d4d8f3536c4d89d871447cba04ffaf25f97 |
| c500c-3.1/lib/acid.r | 4036 | dc04109066c5412cf9954bf5377282f82d27870fc9a52343634be268d75dba88 |
| c500c-3.1/conf/ldconf (the library list r3c reads) | 210 | 5d11fead41165da6075ef303b63b2be7d47b69c501712c39e86c62bbfe454ee9 |
| c500c-3.1/app/test/fib.r3 | 503 | 8bd56c9449810f3eeca29130ad3890a6f19970f83e104df8014296ab3a416278 |
| c500c-3.1/app/test/fib (vendor-compiled, build time 0x3b2a12bd) | 376 | 3d6ac8fe3a78764e73c181d6752d25048beadc89978e6c4ed177dea4b0345a08 |
| c500c-3.1/app/test/sieve.r3 | 680 | bb8db4fa91a4c03310fcf8fe7fd11c19caefb41be81c94fee745dc26626e7ed5 |
| c500c-3.1/app/test/sieve (vendor-compiled, build time 0x3b2a12bf) | 376 | 1bb308e7a55c50756848541dcfafbf0744b2aa40444716953f214dfa062d9d2f |
| c500-1.16.471/lib/syslib.r | 32428 | 9375f7e8eddaffc85601ab8a4a4895f368461fca1ae8526c2068f548fdbaa515 |
| c500-1.16.471/lib/robotlib.r | 43388 | 85029e28d11f1e9c917885cfc718fc5fbd5daf1533ff1ad69bb12cf182abe870 |
| c500-1.16.471/lib/v3access.r | 9196 | 4ee813d265e119d91774b53ebd3072ce21e7da30f0625060bcf330e2a5fff909 |
| c500-1.16.471/lib/stp.r | 45632 | 04e919ee926c1e7e4a598dda4cd1a1696f88df5984fa4c4ff0d43d7b7725d6bf |

(`c500c-3.1/lib/` has no `stp.r`: the C500C image does not contain one, although `ldconf` lists it.)
