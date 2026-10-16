# Phone holders for Jumper

Two holders that turn Jumper into a desk companion carrying your phone, and the phone itself, so the robot can be
trained and filmed with its real mass on. They are showpieces: one piece each, no magnet, latch or spring, curved to
blend into the shell, and printable.

| Version | The phone | Good for |
|---|---|---|
| `vertical` | stands in portrait right behind the visor, leaning back 6°, in a sleeve whose front opens in one curve and whose base flows onto the shell | the screen faces you: notifications, a video call, the agent's replies |
| `plana` | lies in portrait along the shell, screen up, held at its four corners by curved hooks on a bed shaped to the shell | the lowest centre of mass and nothing for a claw to reach: walking and jumping |

The phone (`telefono.scad`) has the size and mass of today's common 6.1" phones: 147.6 × 71.6 × 7.8 mm, 172 g,
rounded outline, a camera pill and a three-lens block, no brand.

| File | What it is |
|---|---|
| `cuna-vertical.scad`, `cuna-plana.scad` | the holders (OpenSCAD + [BOSL2](https://github.com/BelfrySCAD/BOSL2), BSD-2-Clause), in Jumper's `base_link` frame, mm; each also places the phone (`-D parte=…`) |
| `telefono.scad` | the phone, one part per colour |
| `stl/<version>-<part>.stl` | built from them: `cuna` (one closed piece), `cuerpo`, `pantalla`, `isla`, `camara` |
| `caparazon.py` | writes `caparazon.stl`, the shell's top under both holders, from a jumper checkout |
| `montar.py` | adds a holder and the phone to a Jumper MJCF, with their masses, for training and video |
| `construir.sh` | fetches BOSL2 at a fixed commit and rebuilds everything |

```sh
bash construir.sh <jumper checkout>
python montar.py <jumper>/assets/jumper/jumper.xml jumper-telefono.xml --version vertical   # or plana
```

## Checked in simulation (15 Oct 2026)

- **Mass:** holder 48 g (`vertical`, 46 cm³) or 63 g (`plana`, 60 cm³) as PLA, plus the 172 g phone. The robot goes
  from 2.54 to 2.76 or 2.78 kg.
- **The claws clear them** (`puente/factible.py`, robot posed at every row of the romantic dance, the mix-up dance,
  the wave and the stock `demo` dance): with `vertical` no leg comes within 30 mm of the holder or the phone; with
  `plana` the closest is a palm at 12 mm, while it scratches its head.

Not checked yet: the trained dances with these holders on (the policy was trained with the earlier cradle and the
earlier violin; it has to be trained again before the final video).

## Design notes

Before drawing we looked for existing designs (gravity holders, printed-spring clamps, push-push latches, MagSafe
mounts, and two holders we were sent). None had a licence that lets us share a derivative, so these are drawn from
scratch. An earlier version had a MagSafe ring and a printed release tongue; it was dropped for these, which only
need to hold the phone on the robot (15 Oct).
