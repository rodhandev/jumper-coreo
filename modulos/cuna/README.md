# Phone cradle for Jumper

A printable cradle that turns Jumper into a desk phone stand that still dances. The phone sits in landscape
behind the visor, leaning back 30°, so the eyes stay in view and it rises only ~65 mm.

- **Holds:** a MagSafe ring (N52, 56 × 46 × 1.5 mm, in a 1.6 mm recess) and a lip that takes the weight. For a
  phone without MagSafe, stick a MagSafe-compatible ring on it.
- **Releases:** press the tab behind the plate. The printed spring tongue inside the ring pushes the phone 3 mm off
  the magnet, and the phone lifts out diagonally. No springs or screws.
- **Mounts:** the underside is cut from the shell's own surface, so it sits flush. Stick it on with two strips of
  VHB tape; there are no holes in the robot.
- **USB-C:** the plate is 130 mm wide and the phone 160, so both short ends, where the port is, stay free for a
  cable.

| File | What it is |
|---|---|
| `cuna.scad` | The design (OpenSCAD + [BOSL2](https://github.com/BelfrySCAD/BOSL2), BSD-2-Clause), in Jumper's `base_link` frame, mm |
| `cuna.stl` | Built from it: 64.6 cm³, one closed piece |
| `caparazon.py` | Writes `caparazon.stl`, the shell's top under the cradle, from a jumper checkout |
| `montar.py` | Adds the cradle and a 200 g phone to a Jumper MJCF, with their masses, for training and video |
| `construir.sh` | Fetches BOSL2 at a fixed commit and rebuilds everything |

```sh
bash construir.sh <jumper checkout>
python montar.py <jumper>/assets/jumper/jumper.xml jumper-cuna.xml
```

## Checked in simulation (15 Oct 2026)

- **Mass:** cradle 68 g (PLA; its 3–4 mm walls print almost solid) + ring ~15 g + phone 200 g = **283 g**. The robot
  goes from 2.54 to 2.83 kg.
- **The claws clear it:** the trained romantic dance, the wave, a sketch dance and the stock `demo` dance were run
  kinematically with the cradle and phone on; no leg or claw comes within 10 mm.
- **The dance survives it untrained:** the romantic policy, trained without the phone, played with it on tracks its
  joints at 0.025 rad RMS (0.023 without) and tilts at most 9.4° (8.1° without), with no falls.
- **The magnet has margin:** over that run the phone accelerates at most 1.8 m/s². It always presses on the plate
  (0.67–1.47 N) and never lifts off the lip; the largest force the magnet must hold is 0.19 N sideways, against
  ~4 N of friction. Jumping would need more (see below).

Not checked yet: a real print, fit on a physical shell, and the tongue's spring force.

## Design notes

Before drawing we looked for an existing design: gravity holders (they open when the robot bounces), printed-spring
clamps, push-push latches (the heart-cam is in an expired patent, US6056333), MagSafe mounts, and two holders we were
sent: a rack-and-gear self-closing stand and an under-desk tray with a printed trigger. None had a licence that lets us
share a derivative, so this is drawn from scratch. The tongue takes the idea of the under-desk tray's trigger, and the
rack-and-gear stand's self-centring arms are the plan for a version for jumps.
