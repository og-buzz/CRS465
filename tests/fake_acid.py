"""A stand-in for the controller's console with /sbin/fastacid behind it, for testing a465/aci.py.

Written from the disassembly of this unit's fastacid (a465/r3dis.py), not from aci.py: the console
byte handling of its sio-to-fifo process, the packet handler, file_transfer_command() and the
receiving half of file_transfer_data(). Code addresses in brackets are fastacid's. It also stands in
for the bits of the system shell aci.py uses: the `$` prompt and `cksum`.

Time is a FakeClock shared with the code under test, so fastacid's 5 s and 300 ms timeouts run
without waiting. Things fastacid does that matter for safety are recorded: bytes that reached the
shell, control bytes that would have signalled the shell or the robot server, and the write
semaphore left held after a silent failure.
"""
import struct


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, 0.001)


class FakeAcid:
    def __init__(self, clock=None):
        self.clock = clock or FakeClock()
        self.files = {}                 # path -> bytes on the controller
        self.dirs = {"/", "/tmp", "/conf", "/app", "/log"}
        self.out = bytearray()          # bytes on their way to the PC
        self.typed = bytearray()        # bytes that reached the shell's input fifo
        self.signals = []               # what typed control bytes did [0x011c]
        self.packets = []               # every packet the handler read completely
        self.log = []                   # what fastacid would write to /log/acid.log
        self.running = True             # fastacid present; if not, BEL and SYN go to the shell like any byte
        self.dead = False               # the packet handler has died or is stuck: BEL SYN gets no RS
        self.out_sem_held = False       # the write semaphore kept after a silent failure [0x020c]
        self.reentrant_sem = False      # what CROS does on a second sem_acquire by the owner is not known
        self.unwritable = set()         # paths whose open() fails
        self.damage = None              # function(packet bytes as received) -> bytes, to corrupt the line
        self.drop_reply = False         # swallow the next reply byte (a lost byte on the line)
        self.chatter = b""              # shell output that is on its way when packet mode starts
        self.writes = []                # (path, number of bytes) per data packet written
        self.fail_writes = False        # writes() into the file fails (full file system, say)
        self.free_words = 21664         # what memstat() returns: `mem` said 86656 bytes free on 2026-11-23
        self._bel = False
        self._state = "console"
        self._buf = bytearray()
        self._deadline = None
        self._line = bytearray()
        # fastacid's transfer state: G29 mode, G30 next packet number, G31 receiving, G954 packets left,
        # G19 the open file ("fd0" until something is opened: the variable starts at 0, not -1), G691 the name
        self.mode, self.expect, self.receiving, self.left = False, 0, False, 0
        self.fd, self.name = "fd0", None

    # --- the pyserial methods aci.py uses ---
    def write(self, data):
        self._tick()
        for byte in bytes(data):
            self._byte(byte)
        return len(data)

    def read(self, count=1):
        self._tick()
        data, self.out = bytes(self.out[:count]), self.out[count:]
        return data

    def flush(self):
        pass

    # --- the console: sio-to-fifo [0x00fb] ---
    def _byte(self, b):
        if self._state in ("header", "payload", "flush", "drain"):
            self._buf.append(b)
            self._deadline = self.clock() + (0.3 if self._state in ("flush", "drain") else 5.0)
            self._packet_progress()
            return
        if self._state == "stuck":
            self.typed.append(b)        # sio-to-fifo is blocked; bytes pile up unread
            return
        if not self.running:
            self._shell(b)
            return
        bel, self._bel = self._bel, False
        if b == 0x05:
            self.signals.append("SIGINT to the shell")
        elif b == 0x1A:
            self.signals.append("SIGABRT to the robot server")
        elif b == 0x01:
            self.signals.append("SIGALRM to the robot server")
        elif b == 0x07:
            self._bel = True
        elif b == 0x16:
            if bel:
                self._packet_mode()
        else:
            self._shell(b)

    def _packet_mode(self):
        if self.dead:
            return
        if self.out_sem_held and not self.reentrant_sem:
            self._state = "stuck"       # the handler would block on its own semaphore, holding the line
            return
        self.out += self.chatter
        self.chatter = b""
        self.out_sem_held = True
        self.out.append(0x1E)           # RS [0x0211]
        self._state, self._buf, self._deadline = "header", bytearray(), self.clock() + 5.0

    def _tick(self):
        if self._state in ("header", "payload", "flush", "drain") and self.clock() > self._deadline:
            if self._state == "flush":
                self._reply(0x15)       # NAK after the flush [0x02f3]
            elif self._state != "drain":
                self.log.append(f"{self._state} rcv failed ({len(self._buf)})")
            self._state = "console"     # the semaphore taken at [0x020c] stays held unless a reply released it

    # --- the packet handler [0x020a] ---
    def _packet_progress(self):
        buf = self._buf
        if self._state == "header" and len(buf) == 16:
            kind = buf[1]
            if kind == 1:
                self._state = "console"
                self._command(self._received(bytes(buf)))
            elif kind == 2:
                n, neg = struct.unpack_from("<HH", buf, 8)
                if (-n) & 0xFFFF != neg:
                    self.log.append("data packet with bad length received -- flush and send NAK")
                    self._state = "flush"
                else:
                    self._need = 16 + ((n + 3) & ~3)
                    self._state = "payload"
            else:
                self.log.append(f"Unknown packet type {kind} -- flushing input...")
                self._state = "drain"   # 300 ms of whatever comes, then nothing [0x03c3]
        elif self._state == "payload" and len(buf) == self._need:
            self._state = "console"
            self._data(self._received(bytes(buf)))

    def _received(self, packet):
        if self.damage:
            packet = self.damage(packet)
        self.packets.append(packet)
        return packet

    def _reply(self, byte):
        if self.drop_reply:
            self.drop_reply = False
        else:
            self.out.append(byte)
        self.out_sem_held = False

    def _command(self, p):
        if p[0] != 1:
            self.log.append(f"got cmd packet for controller {p[0]} -- ignored")
            return                      # no answer, semaphore kept
        total = 0
        for i in range(12):
            total += p[i]
        if total != p[12] | p[13] << 8 or p[15] & 1:
            self.log.append("got bad cmd packet -- sent NAK")
            return self._reply(0x15)
        if p[2] | p[3] << 8:
            return self._reply(0x18)
        command, count = p[4], p[6] | p[7] << 8
        self._reply(0x06)               # ACK before anything else [0x05fd]
        if command == 4:
            self.mode, self.receiving, self.expect, self.left = True, True, 0, count
        elif command == 5:
            self.mode, self.receiving, self.expect = True, False, 0
        elif command == 1:
            if self.fd == "fd0":
                self.dead = True        # close() of descriptor 0 by the checked builtin: see aci.py
            elif self.fd is not None:
                if self.receiving:
                    del self.files[self.fd]
                    self.log.append("deleted transfer file")
                self.fd = None
            self.mode = self.receiving = False

    def _data(self, p):
        if p[0] != 1:
            self.log.append(f"got data packet for controller {p[0]} -- ignored")
            return
        total = 0
        for b in p[:-4]:
            total += b
        if total != struct.unpack_from("<I", p, len(p) - 4)[0]:
            self.log.append("got bad data packet -- sent NAK")
            return self._reply(0x15)
        if p[2] | p[3] << 8:
            return self._reply(0x18)
        number, n, flag = p[4], p[8] | p[9] << 8, p[7]
        data = p[12:12 + n]
        if not self.mode:
            self.log.append("not in data transfer mode -- sent CAN")
            return self._reply(0x18)
        if number != self.expect:
            self.log.append("packet out of order -- sent CAN")
            return self._reply(0x18)
        if number == 0:
            name = data.decode("latin-1")
            self.name = name
            parent = name.rsplit("/", 1)[0] or "/"
            if self.receiving:
                if name in self.unwritable or self.free_words - 256 * self.left < 3072:     # [0x0511]
                    self.out += b"\x18"                     # CAN [0x054d] ...
                    self.out += f"\nacid: Transfer of file '{name}' aborted.\nacid: File may be busy or space low.\n".encode()
                    self.fd = None
                else:
                    path = ""
                    for part in parent.strip("/").split("/"):
                        if part:
                            path += "/" + part
                            self.dirs.add(path)             # mkdir along the path [0x00b8]
                    self.files[name] = b""                  # O_WRONLY|O_CREAT|O_TRUNC [0x052c]
                    self.fd = name
        else:
            if self.fd == "fd0":
                self.log.append("data written to descriptor 0")     # G19 still 0: writes() goes there
                self.left -= 1
            elif self.fd is not None and self.fail_writes:
                self.out += b"\x18"                                 # CAN [0x0579], then on to the ACK
                self.log.append(f"unable to write packet {number}")
            elif self.fd is not None:
                self.files[self.fd] += data                 # writes(G19, G165, 12) [0x0574]
                self.writes.append((self.fd, len(data)))
                self.left -= 1
            else:
                self.out += b"\x18"
                self.log.append("Got packet without open file -- sent CAN")
            if flag == 4:
                if self.fd not in (None, "fd0"):
                    if self.left:
                        del self.files[self.fd]
                        self.log.append("deleted incomplete file")
                    self.fd = None
                self.mode = False
        self.expect += 1
        self._reply(0x06)                                   # ACK [0x05c7]

    # --- the system shell, as far as aci.py needs it ---
    def _shell(self, b):
        self.typed.append(b)
        if b == 0x0D:
            line = self._line.decode("latin-1").strip()
            self._line = bytearray()
            self.out += b"\r\n"
            words = line.split()
            if words == ["mem"]:
                self.out += f"{self.free_words * 4} bytes free.\r\n8% fragmentation.\r\n".encode()
            if words[:1] == ["cksum"] and len(words) == 2:
                body = self.files.get(words[1])
                if body is None:
                    self.out += f"cksum: {words[1]}: file not found\r\n".encode()
                else:
                    self.out += f"0x{sum(body):08x} {len(body)} {words[1]}\r\n".encode()
            self.out += b"$ "
        else:
            self._line.append(b)
            self.out.append(b)          # echo
