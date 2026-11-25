# Gripper air: fittings to find

Status 2026-11-24: nothing ordered. This is a shopping list with the open questions first.

## What is on the arm

- Two air ports on the underside of the outer link, about 80 mm (3 1/4 in.) behind the wrist axis
  (A465 user guide, "Connecting the Cable or Hose").
- They take barbs for **1/16 in. ID hose** (owner's check, 2026-11-24). The guide agrees: the CRS
  pneumatic gripper "is supplied with two 1/16 in. [1.6 mm] barbed fittings".
- **The two ports sit very close together.** Owner: "I don't think the spacing is correct for a hex
  fitting, they're really tight together." A fitting with a hex body may not fit next to its neighbour.
- The thread the barbs screw into is **not in the manual and has not been measured.**
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
2. The thread in the arm ports. Try a 10-32 machine screw by hand: it runs in freely in a 10-32 port
   and feels loose in an M5 x 0.8 port. Both have a bore of about 4.0 to 4.2 mm. The manual's other
   sizes on this arm are inch sizes (1/8 NPT, 10-24 screws), which points to 10-32, but that is an
   inference.
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

## If hex bodies do not fit side by side

Ideas to check against the measured spacing, none tried:

- one straight barb and one elbow, so the bodies are at different heights;
- a barb with a round or slotted body instead of a hex;
- an M5 or 10-32 extension under one of the two fittings, to stagger them.
