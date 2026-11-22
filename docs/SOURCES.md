# Manual sources

Downloaded 2026-11-22. The PDFs are CRS Robotics documents hosted by third parties,
so they are not committed. `docs/manuals.tsv` holds the URL and sha256 of each file;
`scripts/fetch-manuals.sh` re-downloads them and rebuilds `docs/text/`.

All of these describe the **C500C** controller generation (CROS / RAPL-3).

| File (under `docs/manuals/`) | Doc no. | Pages | Covers |
|---|---|---|---|
| `jpa-kapsi-crs-set/A465-User-Guide-for-C500C.pdf` | UMI-33-465 | 90 | A465 arm: specs, installation, commissioning, homing, maintenance |
| `yorku-c500c-set/C500CControllerUserGuide.pdf` | UMI-33-CON | 80 | Controller: front panel, serial ports, console defaults, power-up, E-stop/SYSIO wiring |
| `yorku-c500c-set/CROSandSystemShell.pdf` | UMI-R3-171 | 68 | CROS operating system and the `$` system shell commands (incl. `siocfg`) |
| `yorku-c500c-set/ApplicationShellASH.pdf` | UMI-R3-161 | 90 | `ash` application shell: moving the arm, teaching locations, running programs |
| `yorku-c500c-set/RAPL-3LanguageReferenceGuide.pdf` | UMI-R3-210 | 360 | RAPL-3 language and library reference |
| `yorku-c500c-set/Robocomm3.pdf` | UMI-R3-151 | 64 | Robcomm3 (Windows): editor, compiler, file transfer, terminal; ch.7 covers using another terminal |
| `yorku-c500c-set/Teach-Pendant.pdf` | UMI-R3-141 | 42 | Teach pendant operation |
| `yorku-c500c-set/Application-Development.pdf` | UMI-R3-111 | 42 | Application development workflow |
| `jpa-kapsi-crs-set/Application-Development-Guide.pdf` | UMI-R3-101 | 8 | Cover and introduction to the Application Development Guide set |
| `yorku-c500c-set/Application-Environment.pdf` | UMI-R3-131 | 60 | System components, peripherals, coordinate spaces and motion types, safe operation |
| `yorku-c500c-set/Terminal-Editor-for-C500C.pdf` | UMI-R3-166 | 16 | On-controller text editor usable from a terminal |
| `yorku-c500c-set/Firmware-Download-Utility-for-the-C500C-Controller.pdf` | UMI-R3-156 | 4 | Re-flashing CROS firmware |
| `yorku-c500c-set/Servo-Gripper-User-Guide.pdf` | UMI-33-640 | 54 | Servo gripper option |
| `jpa-kapsi-crs-set/Force-Sensor-C500C.pdf` | UMI-R3-510 | 48 | Force sensor option |
| `jpa-kapsi-crs-set/Track-User-Guide.pdf` | UMI-33-TRACK | 76 | Linear track option |
| `jpa-kapsi-crs-set/c500_coremodule_4dxe.pdf` |  | 3 | Ampro CoreModule 486 board datasheet (filed by the hoster as the C500 CPU module; not confirmed) |
| `jpa-kapsi-crs-set/c500_coremodule_486dxi.pdf` |  | 2 | Ampro CoreModule 486 board datasheet (filed by the hoster as the C500 CPU module; not confirmed) |
| `makerforums-a465-datasheet/A465-datasheet.pdf` |  | 2 | Two-page A465 brochure: payload, reach, dimensions |

Hosts:

- `yorku-c500c-set/`: York University course archive, <https://www.eecs.yorku.ca/course_archive/2011-12/W/4421/doc/a255/>
- `jpa-kapsi-crs-set/`: personal collection, <http://jpa.kapsi.fi/stuff/other/crs_robot/>
- `makerforums-a465-datasheet/`: forum attachment, see `manuals.tsv`

## Not collected

- Anything for the older C500 / C500B controller (RAPL-II). Needed only if the arm has one.
- Robcomm3 itself (Windows software; no download source found).
- `cros_executable.bin` and `cros_filesystem.bin` on the jpa.kapsi.fi page. Unverified
  binaries of unknown origin, left alone.
- An A255 arm guide and service manual and F3 arm guides exist on the same hosts (different arms).

## Leads not yet followed

- RoboDK forum thread where a user describes moving files to a CRS controller over the
  serial console with a home-made uudecode tool instead of Robcomm3:
  <https://robodk.com/forum/Thread-CRS-F3-newbie-Also-try-Thermo-CRS-CRS-A465-or-KUKA-KR3?page=2>
