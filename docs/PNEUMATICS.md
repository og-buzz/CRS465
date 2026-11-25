# Gripper air: fittings to find

Status 2026-11-24: nothing ordered. This is a shopping list with the open questions first.

## What is on the arm

- Two air ports on the underside of the outer link, about 80 mm (3 1/4 in.) behind the wrist axis
  (A465 user guide, "Connecting the Cable or Hose").
- They take barbs for **1/16 in. ID hose** (owner's check, 2026-11-24). The guide agrees: the CRS
  pneumatic gripper "is supplied with two 1/16 in. [1.6 mm] barbed fittings".
- **The two ports sit very close together: about 8 mm centre to centre** (owner's estimate, 2026-11-24,
  taking the holes as 4 mm). Owner: "I don't think the spacing is correct for a hex fitting, they're
  really tight together." See "What fits at 8 mm centres" below.
- The holes are threaded. The thread is not in the manual. Owner's caliper reading of the holes,
  2026-11-24: **about 3.92 mm** ("could be some error"). That fits **10-32 UNF**, whose internal thread
  has a bore of 3.96 to 4.17 mm (and is often drilled 5/32 in. = 3.97 mm). An M5 x 0.8 hole would read
  4.13 to 4.33 mm. Calipers tend to read a little small in a small threaded hole. Not yet confirmed
  with a screw.
- OPEN sends air out of the port nearest the wrist; CLOSE uses the one further back. If the gripper
  works backwards, swap the two hoses.
- Supply: 1/8 NPT port at the rear of the base. Dry, clean, filtered, non-lubricated air, 100 psi
  maximum.

## What is on the gripper

- SMC **MHS2-32D** (two-finger parallel, 32 mm bore, double acting).
- Both ports (open and close) are **M5 x 0.8** female
  ([SMC MHS2 drawings](https://www.smcpneumatics.com/americansmc/MHS/1_MHS2_Series_Air_Grippers.pdf),
  [SMC MHS catalog](https://www.smcworld.com/catalog/BEST-5-4-en/mpv/4-p0583-0657-mhs_en/data/4-p0583-0657-mhs_en.pdf),
  [Motion listing](https://www.motion.com/products/sku/03301545)).

## Measure before ordering

1. Centre-to-centre distance of the two arm ports, and the widest part of the barbs that are there now.
   That decides whether a hex-bodied fitting can go in at all.
2. Confirm the thread in the arm ports with a 10-32 machine screw by hand: it should run in freely.
   An M5 screw (5.0 mm across) should not start in a 10-32 hole (4.83 mm). The caliper reading above
   and the arm's other inch sizes (1/8 NPT, 10-24 screws) both point to 10-32.
3. Whether the barbs on the arm are usable as they are. If so, only the gripper end and hose are needed.

## Candidates

The simplest plan is 1/16 in. ID hose end to end, with a barb in each of the gripper's two M5 ports.

| Need | Candidate | Link | Check |
|---|---|---|---|
| M5 x 0.8 male to 1/16 in. barb, 2 off (gripper end) | McMaster-Carr straight adapter, M5 x 0.8 thread, 1/16 in. barb | [barbed hose fittings](https://www.mcmaster.com/products/hose-fittings/fitting-connection~barbed-2/for-use-with~lubricated-air) | Listed in plastic. Look on the brass page below for a metal one. |
| Same, in brass | McMaster-Carr miniature brass barbed tube fittings | [miniature brass barbed](https://www.mcmaster.com/products/miniature-brass-barbed-tube-fittings) | Their brass M5 barbs seen so far were for 3 mm and 4 mm tube; confirm a 1/16 in. one exists. |
| 10-32 male to 1/16 in. barb (arm end, if the ports are 10-32 and spares are wanted) | Clippard 1/16 in. ID hose barb, nickel-plated brass | [Clippard ST0-2](https://clippard.com/part/ST0-2-PKG) | Body size against the port spacing. |
| Same | Parker 10-32 x 0.062 in. barb, brass, about $7 each | [Zoro](https://www.zoro.com/parker-10-32-mnpt-x-0062-barb-brass-male-connector-27-1/i/G2460997/) | The title says "MNPT" but the detail table says UNF. Body size. |
| Same | McMaster-Carr 5454K61 (10-32 thread, 1/16 in. tube ID, pack of 10) | [10-32 hose fittings](https://www.mcmaster.com/products/hose-fittings/thread-size~10-32/) | The search result did not make the fitting style clear. Open the product page. |
| Small-bodied fittings, if hex bodies collide | Beswick miniature fittings: 10-32 and M5 threads, O-ring seal, barbs down to 1/16 in. ID, straight and adjustable elbow | [Beswick catalog (PDF)](https://www.beswick.com/wp-content/uploads/2021/05/Beswick-Catalog-119-Interactive-Version_High-Res.pdf), [10-32 to barb](https://www.beswick.com/catalog/product-detail/MH) | Find the 1/16 in. barb and its hex size in the catalog. |
| Hose | 1/16 in. ID (usually 1/8 in. OD) polyurethane | McMaster-Carr or Clippard | |

Not found: a brass M5-to-1/16 in. barb as a standard Clippard part.

## What fits at 8 mm centres

Two fittings side by side each have to be less than 8 mm across at their widest point, and there is
no room for a socket around either.

| Body | Across flats | Across corners | Side by side at 8 mm? |
|---|---|---|---|
| 1/4 in. hex | 6.35 mm | 7.3 mm | Yes, with 0.7 mm to spare in any position |
| 7 mm hex | 7.0 mm | 8.1 mm | Only with the two hexes turned flat to flat |
| 8 mm hex | 8.0 mm | 9.2 mm | No |

So the small barbs with a 1/4 in. hex are what this spacing was made for. Check the hex size of any
candidate above against this table.

**Push-to-connect.** The owner would prefer these ("barbs or better, push-to-connects"; no threaded
John Guest was found in this size). What was checked:

- SMC KQ2H03-32A, 5/32 in. tube to 10-32 UNF male: an **8 mm hex** per its listing, so two will not go
  side by side ([Zoro](https://www.zoro.com/smc-push-fit-adapter-fitting-push-to-connect-x-munf-532-in-tube-od-10-32-pipe-size-brass-kq2h03-32a/i/G5000931)).
- SMC KQ2S04-M5A, 4 mm tube to M5, hexagon socket head (tightened with a 2 mm key from inside, so no
  wrench room is needed): body diameter not found; it is the "øD" column on the
  [MISUMI page](https://my.misumi-ec.com/vona2/detail/221006292240/). It has an M5 thread, so it is
  only an option if the ports are M5.
- McMaster-Carr lists push-to-connect fittings for 5/32 in. tube with a 10-32 male thread in elbow and
  banjo styles ([banjo-style elbows](https://www.mcmaster.com/banjo-style-elbows),
  [tube fittings](https://www.mcmaster.com/products/tube-fittings/thread-size~10-32/)); hex sizes not
  checked.

Ways to get push-to-connect at this spacing, none tried:

- **Stagger the heights.** Put one fitting on a short 10-32 extension so its body sits above the
  other's. An 8 mm hex body (4.6 mm from centre to corner) then only has to clear the neighbour's
  4 mm tube, whose edge is 6 mm away.
- **Keep small barbs at the arm** and change to push-to-connect a short way down the hose with an
  inline union.
- **One straight and one elbow or banjo**, so the bodies are at different heights.
- **A smaller tube size** (2 mm or 1/8 in.), where push-to-connect bodies are smaller; check that the
  gripper still moves fast enough.
