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
- **Automatic balance**: the claws are the front legs, so lifting both tips the robot over
  (at HOME the centre of mass is 17 mm ahead of the middle feet). Wherever a claw is up,
  the body is shifted over the four legs by IK, ahead of time and smoothly — the script
  does not have to know. Two claws up from standing: 50° of tilt without it, 0.1° with it.
- **On the real beat of a track**: `musica pulsos.json` (from `zym-compas pulsos`) instead of
  `pulso`, and `@8` is the track's 8th beat even if the tempo drifts.
- **Its own sound set**: `coreo sonidos <dir>` synthesises 31 classified cues (greeting, celebration, thinking,
  error, …) in a crab timbre — claw clacks and chirps — with no third-party samples, so they are Apache-2.0 like
  the rest and free to reuse.
- **Refuses what the robot cannot do**, with the line: unknown postures, joints past their
  range, joint speed over 12 rad/s, base tilt over 17° (both measured on Jumper's trained
  dances).

## Pipeline

```bash
cd ../jumper && . .venv/bin/activate                     # Jumper's environment (MuJoCo, its tools)
python ../coreo/puente/construir.py . ../coreo/guiones/saludo.guion [--musica track.wav]
#   [1] compile (Rust)  [2] balance  [3] .npz  [4] preview video -> ../coreo/salidas/
cp ../coreo/salidas/saludo.npz tasks/jumper/coreo/media/   # one clip in media/ at a time
python scripts/train.py --task jumper.coreo               # learn it
```

| Piece | Role |
|---|---|
| `src/` | script parser, postures, mirroring, interpolation, checks (Rust, no runtime deps beyond serde/toml) |
| `puente/construir.py` | the whole pipeline in one command |
| `puente/equilibrar.py` | automatic balance (IK of the four legs while a claw is up) |
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
