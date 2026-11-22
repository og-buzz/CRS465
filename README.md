# crs-a465

Notes, manuals, scripts and terminal sessions for the CRS A465 six-axis robot arm
and its controller, worked from a Linux dev box.

## Status (2026-11-22)

- Manuals gathered and searchable (see [docs/SOURCES.md](docs/SOURCES.md)).
- `scripts/a465-term` written and tested against a simulated controller only.
- **Nothing here has been tried on the real robot yet.** The dev box has no RS-232
  port and no USB-to-RS232 adapter attached.

Open questions:

1. Which controller does this arm have? Everything below assumes a **C500C**
   (DB-9 "Console" port on the front panel). A C500 or C500B has a DB-25
   "Communication" port inside the front panel door, defaults to 38400 baud, and
   is not covered by the manuals collected here.
2. Is the Robcomm3 install media available? See "What a plain terminal cannot do".

## Connecting

Hardware (C500C Controller User Guide p.34-35; Robcomm3 guide ch.7 "Installing the Cable"):

- USB-to-RS232 adapter with real RS-232 levels (not a 3.3 V/5 V TTL adapter).
- **Straight-through** DB-9 cable, male end into the controller, **not** null-modem.
  A DB-9 male adapter is wired like a PC serial port, so it can also plug straight in.
- Plug into the port labeled **Console** on the controller front panel.
- **Never connect to the rear-panel SYSIO DB-25.** It carries live 24 V and will
  damage the computer (Robcomm3 guide, ch.7 warning).

Factory console settings (C500C guide p.35, "Setting the Baud Rate"):

| Setting | Value |
|---|---|
| Baud | 57600 (C500/C500B: 38400) |
| Data / parity / stop | 8 / none / 1 |
| Handshake | none |
| Emulation | TTY or ANSI |

Then:

```sh
scripts/a465-term --list     # show attached USB serial ports
scripts/a465-term            # connect (uses the port if exactly one is attached)
scripts/a465-term --port /dev/serial/by-id/<adapter>
```

Exit with `Ctrl-]`. Each run writes what appeared on screen to `sessions/<timestamp>.log`.
The Enter key sends CR and local echo is off; if typing is invisible use `--echo`,
if Enter does nothing try `--eol crlf`. Neither has been checked on the robot.

Input can also be piped, which sends the lines and records the reply:

```sh
echo 'ls' | scripts/a465-term
```

### What to expect

From the C500C guide p.40 and the A465 guide p.55:

- After power-up the front panel shows `C500C CROS OK` and the terminal shows a
  `$` prompt (system shell) or `>>`.
- `ash <name>` starts the application shell; the prompt becomes `<name>>`.
- If a teach pendant is attached it takes control at start-up. Hand control to the
  terminal from the pendant (C500C guide: `ESC` then `F1`; A465 guide: `Shift`+`ESC`).
- Start-up speed is always 10%.

### Before applying arm power

The A465 guide (p.55, "Verifying Encoder Feedback") requires checking encoder
feedback first, with arm power **off**: at the ash prompt type `w1`, push each
joint by hand and confirm the pulse counts go both up and down, then `Ctrl+E`.
An axis without feedback can run away when arm power is applied.

## What a plain terminal cannot do

Per the Robcomm3 guide (ch.7 "Limiting Other Functions"): a terminal emulator gives
shell access only. **File transfer to the controller is only possible with
Robcomm3's file transfer utility**, and RAPL-3 programs are compiled on the PC
(from within Robcomm3 or its command-line compiler). Robcomm3 is Windows software.
The controller does have an on-board editor (Terminal Editor guide, UMI-R3-166).

## Layout

| Path | Contents |
|---|---|
| `docs/SOURCES.md` | What each manual is and where it came from |
| `docs/manuals.tsv` | Download URL and sha256 for each manual |
| `docs/manuals/`, `docs/text/` | PDFs and extracted text. Not committed; run `scripts/fetch-manuals.sh` |
| `scripts/a465-term` | Serial terminal with session logging (needs `python3-serial`) |
| `scripts/fetch-manuals.sh` | Re-download and verify the manuals, extract text |
| `sessions/` | Terminal session logs |
| `programs/` | RAPL-3 programs and controller files |

Search the manuals with e.g. `grep -n -i 'siocfg' docs/text/*.txt`.
