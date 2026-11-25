"""a465/r3dis.py: the disassembly of the two stock example programs matches their RAPL-3 sources, and the
facts aci.py rests on are where its docstring says they are in fastacid.

fib and sieve (with fib.r3 and sieve.r3) are in controller/backup-2026-11-23/app/test/. The other files
are this unit's own copies under sessions/controller-files/ (git-ignored) and those tests skip without them.
"""
from pathlib import Path

import pytest

from a465 import r3dis

REPO = Path(__file__).resolve().parent.parent
TEST_APP = REPO / "controller" / "backup-2026-11-23" / "app" / "test"
FILES = REPO / "sessions" / "controller-files"


def program(path):
    if not path.exists():
        pytest.skip(f"{path.relative_to(REPO)} is not here")
    return r3dis.Module(r3dis.load(path), path.name)


def by_address(mod, libnames=None):
    return {ins.addr: ins for ins in r3dis.decode(mod, libnames)}


def text(mod):
    return "\n".join(r3dis.listing(mod, {"syslib": r3dis.SYSLIB}))


def test_fib_matches_its_source():
    mod = program(TEST_APP / "fib")
    code = by_address(mod)
    assert (mod.code_words, mod.data_words) == (49, 20)
    assert code[1].text == "enter 7"                              # func int fib( int n )
    assert code[3].text == "L6 = A3 <= 2"                         #   if n <= 2
    assert code[5].text == "L6 = 1" and code[6].text == "return L6"   # return 1
    assert code[7].text == "L6 = A3 - 1" and code[0xC].text == "L5 = A3 - 2"
    assert code[0xA].text == code[0xF].text == "call 0x0001"      #   fib(n - 1) + fib(n - 2)
    assert code[0x11].text == "L4 = L6 + L5"
    assert "'Recursive Fibonacci Number Calculation:\\n'" in code[0x1A].comment and "print" in code[0x1A].text
    assert code[0x24].text == "L2 = L4 /. L3"                     # <float>n / <float>last
    assert "printf" in code[0x2B].text and code[0x2B].comment.startswith("(&G11 'fib({2}) = {5} (ratio = {9.8})\\n', 3, L8:1, L7:1, L2:2")
    assert code[0x2E].text == "L4 = L8 > 24"                      # for i = 1 to 24


def test_sieve_matches_its_source():
    mod = program(TEST_APP / "sieve")
    code = by_address(mod)
    assert mod.data_words == 8205
    assert code[0xF].text == "check 0 <= L8 < 8191"               # int[size + 1] flag, size 8190
    assert code[0x11].text == "L6@[L8] = L5"                      # flag[k] = TRUE
    assert code[0x1C].text == "L6 = L11 << 1" and code[0x1D].text == "L9 = L6 + 3"   # prime = (j << 1) + 3
    assert code[0x2E].text == "L6 = L12 > 25"                     # for i = 1 to ntimes
    assert [mod.string_at(n) for n in (8191, 8196, 8198, 8203)] == [" iterations.\n", "\n\n", " primes found.\n\n", "\n"]


def test_more_reads_a_key_and_quits_on_q():
    mod = program(FILES / "bin" / "more")
    listing = text(mod)
    assert "builtin 0x20 open   ; (&L42, A3, 1, 0)" in listing     # O_RDONLY is 1 in this syslib
    assert "builtin! 0x28 reads   ; (0, &L41, 1)" in listing
    assert "L3 = L5 == 113" in listing and "L5 = L4 == 81" in listing   # 'q' or 'Q'


def test_cp_copies_with_reads_and_writes():
    listing = text(program(FILES / "bin" / "cp"))
    assert "builtin 0x20 open   ; (&L8, A3, 1538, 2)" in listing  # O_WRONLY|O_CREAT|O_TRUNC
    assert "builtin 0x28 reads   ; (L9, &G0 '', 1024)" in listing and "builtin 0x2f writes   ; (L8, &G0 '', 0)" in listing


def test_syslib_symbols_give_the_routine_names_used():
    lib = FILES / "lib" / "syslib.r"
    if not lib.exists():
        pytest.skip("syslib.r is not here")
    own, builtins = r3dis.library_routines(r3dis.load(lib))
    assert {k: own[k] for k in r3dis.SYSLIB} == r3dis.SYSLIB
    assert {number: r3dis.BUILTINS[number] for number in builtins} == builtins
    assert sorted(n for n, name in r3dis.BUILTINS.items() if name.endswith("*")) == sorted(set(r3dis.BUILTINS) - set(builtins))


def test_fastacid_facts_the_protocol_rests_on():
    mod = program(FILES / "sbin" / "fastacid")
    code = by_address(mod, {"syslib": r3dis.SYSLIB})
    assert (code[0x401].text, code[0x402].text) == ("L5 = 1", "G8 = L5")          # the controller address
    assert sum(1 for ins in code.values() if ins.text.endswith("G8 = L5") or "&G8 " in ins.text) == 1
    assert code[0x11C].text == "switch L4 table 0x011e"
    assert code[0x11E].text.startswith("; switch table: 5..5 -> 0x012e, 26..26 -> 0x0137, 1..1 -> 0x013c, 7..7 -> 0x0141, 22..22 -> 0x0143")
    assert {n: mod.string_at(n) for n in (1309, 1338, 1349, 1638, 1640, 1642)} == {
        1309: "\x1e", 1338: "\x15", 1349: "\x18", 1638: "\x06", 1640: "\x01", 1642: "\x14"}
    assert code[0x221].comment == "(L8, &G165 '', 16, 5000)"                     # readsat: the header
    assert code[0x30D].comment == "(L8, &G165 '', L18, 5000)"                    # readsat: the rest
    assert code[0x52C].comment == "(&G19, &G428 '', 1538, 7)"                     # the received file's open()
    assert code[0x253].text == "switch L14 table 0x0255" and "1..1 -> 0x025c, 2..2 -> 0x02db" in code[0x255].text
