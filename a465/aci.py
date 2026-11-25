"""Put a file onto the controller through its serial console, in the packet protocol of /sbin/fastacid.

Worked out on 2026-11-24 by disassembling this unit's /sbin/fastacid (CROS 1.16.471, 9372 bytes) with
a465/r3dis.py; nothing here was tried on the controller yet. Numbers in brackets are fastacid code
addresses (word index; the file byte is 64 + 4 * word) where the statement is read. The interpreter's
semantics behind them come from the 2001 interpreter (see a465/r3dis.py).

How fastacid sits on the line
-----------------------------
fastacid runs as three processes [main 0x03d3]: one reads the serial line a byte at a time and passes
it to the shell's input fifo (sio-to-fifo, [0x00fb]); one copies the shell's output to the line
(fifo-to-sio, [0x0186]); the parent handles packets [0x0477]. Two semaphores decide who may read the
line (key 0xfa5ac1dc) and who may write it (key 0xfa5ac1d0).

Typed bytes are passed on except [switch at 0x011c]: 0x05 (sends SIGINT to the shell), 0x1a and 0x01
(signal the robot server; if that fails fastacid opens /dev/estop [0x0070]), 0x07 BEL (swallowed, arms
packet mode) and 0x16 SYN (swallowed; right after a BEL it hands the line to the packet handler). So a
packet is told apart from typing by the two bytes BEL SYN in a row, and nothing else.

One packet
----------
    PC:         07 16                       BEL SYN
    controller: 1e                          RS: the handler has the line and is waiting [0x0211]
    PC:         the packet (below)
    controller: 06 ACK / 15 NAK / 18 CAN    (or nothing at all: see the failure list)

The handler reads the 16 bytes of a header with a 5 s timeout [0x021b], then, for a data packet, the
rest with another 5 s [0x030d]; the timeouts count only time spent waiting, in 50 ms steps [0x01da].
Every packet needs its own BEL SYN: after each one the line goes back to the shell. send_file never
sends a packet before it has seen the RS, and waits SETTLE after each answer before the next BEL SYN
so that the handler is back waiting for the line (if it were not, sio-to-fifo would take the line
straight back, no RS would come, and only BEL SYN would have been sent).

A command packet is exactly 16 bytes:

    0       controller address: must be 1, fastacid sets it as a constant [0x0401]
    1       1 = command
    2-3     port, u16: must be 0 or the reply is CAN [0x02a9]
    4       command: 4 receive a file, 5 send a file, 6 next block, 7 next directory entry, 1 close/abort
    5       not used
    6-7     u16 count: for command 4 the number of data packets that will follow the name packet
    8-11    u32, only written to fastacid's log (send_file sends 0)
    12-13   u16 sum of bytes 0-11 [0x027c-0x0292]
    14      not used
    15      bit 0 must be 0, or the reply is NAK [0x028d-0x0291]

A data packet is a 12-byte header, the data, zero to three pad bytes and a check word:

    0       controller address (1)
    1       2 = data
    2-3     port, u16: 0
    4       packet number: 0 for the name packet, then 1, 2, ... [0x04b4]
    5-6     not used
    7       4 on the last packet; anything else means more follow [0x05a5]
    8-9     n, u16: the number of data bytes (at most 1032: fastacid's buffer is 1048)
    10-11   -n, u16 (two's complement); a mismatch is answered NAK after a 300 ms flush [0x02db-0x02e6]
    12..    the n data bytes, then pad bytes up to a multiple of 4
    last 4  u32 sum of every byte before it (header, data and pad) [0x0343-0x0360]

so the whole packet is 16 + n rounded up to 4 bytes. Multi-byte fields are little-endian.

Sending one file [file_transfer_command 0x05e8, file_transfer_data 0x049e]
------------------------------------------------------------------------
1.  command 4 with count = number of data packets N (not counting the name packet). ACK is sent
    first, for any command; then transfer mode is set, the expected packet number becomes 0 and the
    count is kept.
2.  data packet 0 holding the remote path. fastacid first refuses (error -12) unless memstat() minus
    256 x count is at least 3072 [0x0511-0x051d]; memstat() counts free memory in 4-byte words here
    (the shell's `mem` prints it times 4), so about 12 KB plus 1 KB per data packet must be free. It
    makes the missing directories on the path [0x00b8] and opens it O_WRONLY|O_CREAT|O_TRUNC
    [0x052c]: an existing file is emptied here. ACK, or CAN if either fails (then text on the
    console and an ACK as well).
3.  data packets 1..N with the file's bytes, byte 7 = 4 on packet N (send_file puts 3 in the others,
    as fastacid does in the blocks it sends, and never sends one with no data: whether fastacid's read
    of 0 more bytes returns at once is not in the program). Each is written as it comes and
    the count goes down by one. On packet N the file is closed and, if the count is not then 0,
    deleted ("deleted incomplete file" [0x05b5]). ACK each.
Packet numbers are one byte: at most 255 data packets. The controller sends nothing on its own.

Command 1 closes a file being transferred and, when it was being received, deletes it [0x061c].
Sending a file to the PC is command 5, a name packet (answered ACK, SOH, the size as 4 bytes, DC4),
then command 6 for each block: ACK, then 02 seq -seq 03|04, u16 n, u16 -n, n bytes, u32 sum of the n
bytes [0x0640]. Command 7 lists a directory opened that way [0x069b]. There is no delete command.

Failure cases, from the code
----------------------------
NAK: command checksum or byte 15 wrong; data length check wrong; data sum wrong. Nothing changes on
the controller and the same packet may be sent again.
CAN: port not 0; command 4/5 not given first ("not in data transfer mode"); packet number not the
one expected; the file will not open; a write fails (CAN then ACK); a data packet with no file open.
Nothing at all: header or data not complete within its 5 s, a controller address other than 1, a
packet type other than 1 or 2 (it then reads and drops what comes in the next 300 ms). In these cases fastacid also keeps the write semaphore it took at
[0x020c], which stops the shell's output reaching the line; whether its next packet can then still
run depends on how CROS treats a semaphore taken twice by one process, which the program does not
show. So after a silent failure send nothing more and check the console by hand.
Command 1 before any file was opened since fastacid started closes descriptor 0 instead (its file
variable starts at 0, not -1), which is not safe; send_file only sends it after a name packet was
accepted.
"""
import re
import struct
import time

BEL, SYN, RS = 0x07, 0x16, 0x1E
ACK, NAK, CAN = 0x06, 0x15, 0x18
SOH, DC4 = 0x01, 0x14
PACKET_MODE = bytes((BEL, SYN))
NAMES = {ACK: "ACK", NAK: "NAK", CAN: "CAN", RS: "RS", BEL: "BEL", SYN: "SYN", SOH: "SOH", DC4: "DC4"}

CONTROLLER = 1                      # fastacid's own address, a constant [0x0401]
COMMAND, DATA = 1, 2                # packet types (header byte 1)
ABORT, RECEIVE, SEND, NEXT_BLOCK, NEXT_ENTRY = 1, 4, 5, 6, 7
MORE, LAST = 3, 4                   # header byte 7 of a data packet: 4 marks the last one
HEADER_BYTES, DATA_HEADER_BYTES = 16, 12
MAX_DATA = 1032                     # 16 + n rounded up to 4 must fit fastacid's 1048-byte buffer
MAX_DATA_PACKETS = 255              # the packet number is one byte
CHUNK = 256                         # data bytes per packet used by default
XON_XOFF = (0x11, 0x13)

RS_TIMEOUT = 6.0                    # longer than fastacid's own 5 s wait for a header
REPLY_TIMEOUT = 8.0                 # longer than its 5 s wait for the rest of a data packet
SETTLE = 0.3                        # pause after each reply, so the handler is waiting again
AFTER_CAN = 1.5                     # how long to keep listening after a CAN for the text and ACK that can follow


class TransferError(RuntimeError):
    """The transfer stopped. `silent` is True when the controller gave no answer to a packet."""

    def __init__(self, message, silent=False):
        super().__init__(message)
        self.silent = silent


def u16(value):
    return struct.pack("<H", value & 0xFFFF)


def command_packet(command, count=0, value=0, address=CONTROLLER, port=0):
    head = bytes((address, COMMAND)) + u16(port) + bytes((command, 0)) + u16(count) + struct.pack("<I", value & 0xFFFFFFFF)
    return head + u16(sum(head)) + bytes((0, 0))


def data_packet(number, data, last, address=CONTROLLER, port=0):
    if len(data) > MAX_DATA:
        raise ValueError(f"{len(data)} data bytes do not fit one packet (at most {MAX_DATA})")
    if not 0 <= number <= MAX_DATA_PACKETS:
        raise ValueError(f"packet number {number} does not fit one byte")
    body = (bytes((address, DATA)) + u16(port) + bytes((number, 0, 0, LAST if last else MORE)) + u16(len(data))
            + u16(-len(data)) + data + bytes(-len(data) % 4))
    return body + struct.pack("<I", sum(body) & 0xFFFFFFFF)


def describe(packet):
    """One line saying what a packet built here is."""
    if len(packet) < HEADER_BYTES:
        return f"{len(packet)} bytes, not a whole packet"
    if packet[1] == COMMAND:
        command, count, value = packet[4], struct.unpack_from("<H", packet, 6)[0], struct.unpack_from("<I", packet, 8)[0]
        what = {ABORT: "close/abort", RECEIVE: "receive a file", SEND: "send a file", NEXT_BLOCK: "next block",
                NEXT_ENTRY: "next directory entry"}.get(command, "?")
        extra = f", {count} data packets to follow the name" if command == RECEIVE else ""
        return f"command {command} ({what}{extra}; logged value {value})"
    n = struct.unpack_from("<H", packet, 8)[0]
    return f"data packet {packet[4]}{' (last)' if packet[7] == LAST else ''}: {n} bytes"


def plan(data, remote_path, chunk=CHUNK):
    """The packets that put `data` at `remote_path`, in order: (label, packet bytes)."""
    if not 1 <= chunk <= MAX_DATA:
        raise ValueError(f"chunk must be 1 to {MAX_DATA} bytes")
    name = remote_path.encode("ascii")
    if not remote_path.startswith("/") or len(name) > MAX_DATA:
        raise ValueError("the remote path must be absolute and fit one packet")
    if not data:
        # A data packet with no bytes leaves fastacid reading 0 more bytes from a non-blocking line;
        # whether that returns at once or waits out the 5 s is not in the program.
        raise ValueError("an empty file cannot be sent: fastacid's handling of a packet with no data is not known")
    pieces = [data[i:i + chunk] for i in range(0, len(data), chunk)]
    if len(pieces) > MAX_DATA_PACKETS:
        raise ValueError(f"{len(pieces)} data packets needed; fastacid counts at most {MAX_DATA_PACKETS} (use a bigger chunk)")
    # Bytes 8-11 of the command are only logged; 0 keeps every byte the command sum covers below 0x80.
    out = [("start", command_packet(RECEIVE, count=len(pieces))),
           ("name", data_packet(0, name, last=False))]
    for i, piece in enumerate(pieces, 1):
        out.append((f"data {i}", data_packet(i, piece, last=i == len(pieces))))
    return out


def hexs(data):
    return " ".join(f"{b:02x}" for b in data)


def shown(data):
    """Console bytes for a person: printable text kept, everything else as <xx>."""
    return "".join(chr(b) if 32 <= b < 127 else {10: "\\n", 13: "\\r"}.get(b, f"<{NAMES.get(b, f'{b:02x}')}>") for b in data)


class Link:
    """The serial port with a read-ahead buffer, a clock that tests can replace, and a report line per event."""

    def __init__(self, port, report=print, clock=time.monotonic, sleep=time.sleep):
        self.port, self.report, self.clock, self.sleep = port, report, clock, sleep
        self.ahead = b""

    def send(self, data, what):
        self.port.write(data)
        if hasattr(self.port, "flush"):
            self.port.flush()
        if len(data) <= 32:
            self.report(f"-> {hexs(data)}   {what}")
            return
        self.report(f"-> ({len(data)} bytes)   {what}")
        for at in range(0, len(data), 32):
            self.report(f"   {at:04x}: {hexs(data[at:at + 32])}")

    def _more(self):
        chunk = self.port.read(4096)
        if chunk:
            self.ahead += chunk
            return True
        self.sleep(0.01)
        return False

    def wait_byte(self, wanted, timeout):
        """(the first byte found in `wanted`, the bytes before it), or (None, all bytes seen) after `timeout`."""
        end = self.clock() + timeout
        while True:
            for i, b in enumerate(self.ahead):
                if b in wanted:
                    before, self.ahead = self.ahead[:i], self.ahead[i + 1:]
                    return b, before
            if self.clock() >= end:
                seen, self.ahead = self.ahead, b""
                return None, seen
            self._more()

    def drain(self, quiet=0.2, limit=3.0):
        """Whatever arrives until the line has been quiet for `quiet` seconds."""
        start = last = self.clock()
        while self.clock() - start < limit:
            if self._more():
                last = self.clock()
            elif self.clock() - last > quiet:
                break
        seen, self.ahead = self.ahead, b""
        return seen

    def shell(self, text, done, limit):
        """Type a line to the controller's shell and collect its answer until `done(answer)`."""
        self.port.write(text.encode() + b"\r")
        end = self.clock() + limit
        while self.clock() < end and not done(self.ahead):
            self._more()
        got, self.ahead = self.ahead, b""
        return got


def exchange(link, packet, label, rs_timeout=None, reply_timeout=None):
    """BEL SYN, wait for RS, send the packet, wait for its answer. Returns ACK, NAK or CAN; raises
    TransferError if no RS (nothing but BEL SYN was sent) or no answer (silent=True)."""
    rs_timeout = RS_TIMEOUT if rs_timeout is None else rs_timeout
    reply_timeout = REPLY_TIMEOUT if reply_timeout is None else reply_timeout
    early = link.drain(quiet=0.05, limit=0.5)
    if early:
        link.report(f"<- {shown(early)}   (console output, not part of the transfer)")
    link.send(PACKET_MODE, "BEL SYN: ask fastacid for packet mode")
    got, before = link.wait_byte({RS}, rs_timeout)
    if before:
        link.report(f"<- {shown(before)}   (console output)")
    if got is None:
        raise TransferError(f"{label}: no RS within {rs_timeout:.0f} s after BEL SYN, so the packet was not sent"
                            " (is fastacid running? `ps` should list it three times)")
    link.report("<- 1e   RS: ready for the header")
    link.send(packet, f"{label}: {describe(packet)}")
    got, before = link.wait_byte({ACK, NAK, CAN}, reply_timeout)
    if before:
        link.report(f"<- {shown(before)}   (console output)")
    if got is None:
        raise TransferError(f"{label}: no answer within {reply_timeout:.0f} s. fastacid gives no answer when a"
                            " packet arrives incomplete or garbled, and may then hold back the shell's output",
                            silent=True)
    link.report(f"<- {got:02x}   {NAMES[got]}")
    if got == CAN:
        after = link.drain(quiet=0.3, limit=AFTER_CAN)
        if after:
            link.report(f"<- {shown(after)}   (after the CAN)")
    else:
        link.sleep(SETTLE)
    return got


def send_file(port, local_path, remote_path, chunk=CHUNK, report=print, clock=time.monotonic, sleep=time.sleep,
              rs_timeout=None, reply_timeout=None, retries=0):
    """Put the file at `local_path` onto the controller as `remote_path`, reporting every step.

    `port` is an open pyserial port (38400 8N1, timeout=0) or anything with its read() and write().
    A NAK is answered by sending the same packet again up to `retries` times. Stops at the first
    other unexpected answer; if the name packet had been accepted it then sends command 1 so that
    fastacid closes and deletes the partial file, except after a silent failure, when it sends nothing.
    Returns the number of bytes sent."""
    with open(local_path, "rb") as source:
        data = source.read()
    packets = plan(data, remote_path, chunk)
    link = port if isinstance(port, Link) else Link(port, report, clock, sleep)
    report(f"{local_path}: {len(data)} bytes, byte sum {sum(data):#x}; {len(packets)} packets to {remote_path}")
    file_open = False
    for index, (label, packet) in enumerate(packets):
        tries = 0
        while True:
            try:
                got = exchange(link, packet, label, rs_timeout, reply_timeout)
            except TransferError as err:
                if file_open and not err.silent:
                    abort(link, rs_timeout, reply_timeout)
                raise
            if got == NAK and tries < retries:
                tries += 1
                report(f"   NAK: sending {label} again ({tries} of {retries})")
                continue
            break
        if got != ACK:
            why = {NAK: "NAK (the controller found the packet damaged; nothing was changed by it)",
                   CAN: "CAN (the controller refused it: see /log/acid.log on the controller)"}[got]
            if file_open:
                abort(link, rs_timeout, reply_timeout)
            raise TransferError(f"{label}: {why}")
        file_open = index < len(packets) - 1 and index >= 1      # from the name packet until the last one
    report(f"done: {len(data)} bytes in {len(packets) - 2} data packets; fastacid has closed {remote_path}")
    return len(data)


def abort(link, rs_timeout=None, reply_timeout=None):
    """Command 1: fastacid closes the file it is receiving and deletes it. Only for after the name packet was accepted."""
    link.report("   aborting: command 1 makes fastacid close and delete the partial file")
    try:
        got = exchange(link, command_packet(ABORT), "abort", rs_timeout, reply_timeout)
        link.report("   partial file deleted" if got == ACK else f"   abort answered {NAMES[got]}")
    except TransferError as err:
        link.report(f"   abort failed: {err}")


CKSUM = r"0x([0-9a-fA-F]+)\s+(\d+)\s+"


def remote_cksum(link, path, limit=60.0):
    """(byte sum, size) from the controller's own `cksum`, or None if it printed no sum for the path.
    The system shell's `$` prompt must be showing."""
    if re.search(r"\s", path):
        raise ValueError("a path with spaces cannot be passed to the shell")
    link.drain(quiet=0.1, limit=1.0)
    # The prompt comes back only after the answer (the controller adds the whole file up first), and
    # the echoed command line ends with a line end, not with "$ ".
    answer = link.shell(f"cksum {path}", lambda g: g.endswith(b"$ ") and b"\n" in g, limit)
    found = re.search(CKSUM + re.escape(path), answer.decode("latin-1"))
    return (int(found.group(1), 16), int(found.group(2))) if found else None


def at_system_prompt(link):
    link.port.write(b"\r")
    return link.drain(quiet=0.8, limit=5.0).rstrip(b" ").endswith(b"$")


def free_bytes(link):
    """What the controller's `mem` says is free, in bytes, or None if it did not say."""
    link.drain(quiet=0.1, limit=1.0)
    answer = link.shell("mem", lambda g: g.endswith(b"$ ") and b"\n" in g, 10.0)
    found = re.search(r"(\d+) bytes free", answer.decode("latin-1"))
    return int(found.group(1)) if found else None


def memory_needed(data_packets):
    """Free memory, in bytes, fastacid wants before it opens the file: 3072 + 256 words per data packet."""
    return 4 * (3072 + 256 * data_packets)


def risky_bytes(data):
    """How many XON/XOFF bytes the data holds: they arrive only if the console has software flow control off."""
    return sum(data.count(b) for b in XON_XOFF)

