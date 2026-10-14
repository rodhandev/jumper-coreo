# coreo 🦀 — behaviour scripts to motion clips for Jumper

**English** · [Español](#español)

Write what the crab robot does and when, in a text script on the music's beat; `coreo`
turns it into a 22-joint motion clip that [Jumper](https://github.com/KingKongRobotics/jumper)'s
imitation training (`jumper.dance` machinery) learns to perform.

```text
pulso 100                                         # bpm: times are in beats; "2.5s" is seconds
@0   cara sonrisa                                 # face screen expression
@1   patas apoyo_atras en 2                       # legs lean back, arriving in 2 beats
@1   tenaza_izq arriba en 2                       # left claw up
@3   alternar tenaza_izq arriba_b arriba veces 6 cada 0.5   # wave
@6   sonido celebracion                           # sound cue for the video
@16  fin
```

What it does for you:

- **Named postures per limb group** (`tenaza_izq`, `tenaza_der`, `tenazas`, `patas`), seeded
  from frames of Jumper's own gesture clips, so the library starts from poses a policy
  already tracks.
- **Mirrors the right claw** from the left as an offset from `HOME` (the robot is not exactly
  symmetric, so mirroring angles would be wrong).
- **Minimum-jerk interpolation**; overlapping orders chain instead of jumping.
- **Refuses what the robot cannot do**, with the line: unknown postures, joints past their
  range, joint speed over 12 rad/s, base tilt over 17° (both measured on Jumper's trained
  dances).

## Pipeline

```bash
cargo run --release -- compilar guiones/saludo.guion -o salidas/saludo.json   # Rust: script -> joint table
cd ../jumper && . .venv/bin/activate                                           # Jumper's environment
python ../coreo/puente/clip_a_npz.py . ../coreo/salidas/saludo.json tasks/jumper/coreo/media/saludo.npz
python ../coreo/puente/ver_clip.py . tasks/jumper/coreo/media/saludo.npz <model.xml> preview.mp4
python scripts/train.py --task jumper.coreo                                    # learn it
```

| Piece | Role |
|---|---|
| `src/` | script parser, postures, mirroring, interpolation, checks (Rust, no runtime deps beyond serde/toml) |
| `puente/clip_a_npz.py` | base pose from Jumper's resting-floor solver (`tools/import_wbc_gestures.py`), writes the training `.npz` |
| `puente/ver_clip.py` | kinematic preview video, face screen following the script |
| `puente/compatibilidad.py` | which claw posture goes with which leg stance without tipping over |
| `puente/exportar_robot.py`, `extraer_posturas.py` | regenerate `robot.json` and `posturas/base.toml` from Jumper |

The `jumper.coreo` task lives in [rodhandev/jumper](https://github.com/rodhandev/jumper) (branch `coreo-task`).

## Español

Se escribe en un guion de texto qué hace el robot y cuándo, sobre el pulso de la música; `coreo`
lo convierte en un clip de 22 articulaciones que el entrenamiento por imitación de Jumper aprende.
Las posturas tienen nombre por grupo de extremidades, la tenaza derecha se deduce de la izquierda,
el movimiento es de mínimo tirón y lo imposible se rechaza con su línea y su motivo.

Made by [Zymbiotek](https://zymbiotek.com) for the KingKong Robotics Crab Robot Open-source Challenge.
#Jumper #CrabRobot #OpenSourceRobot

License: Apache-2.0 (see `LICENSE` and `NOTICE`).
