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
- **An automatic dancer**: `coreo bailar pulsos.json --estilo romantico|rapido|clasico` writes the whole script
  from a track's beats — a step from the style's library (`pasos/`: violin, kisses, sway, claw taps, bow, wave,
  head-scratch…) on every bar, never the same twice in a row — and the script stays text you can tune.
- **Claw detours**: where going straight from one claw pose to the next would sweep a claw through the other
  one or through the shell, `puente/desviar.py` lifts both claws over the top along a smooth bump, as small as
  works; a pose that collides while held is reported as the posture's fault instead.
- **Postures by position** (`puente/crear_postura.py`): «claw tip here», «body 15 mm left and 5° rolled»; it solves
  the IK and refuses what does not reach or runs into the body. Whole clips are checked for self-collision too.
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
#   [1] compile (Rust)  [2] claw detours  [2b] balance  [3] .npz  [4] preview video -> ../coreo/salidas/
cp ../coreo/salidas/saludo.npz tasks/jumper/coreo/media/   # one clip in media/ at a time
python scripts/train.py --task jumper.coreo               # learn it
```

| Piece | Role |
|---|---|
| `src/` | script parser, postures, mirroring, interpolation, checks (Rust, no runtime deps beyond serde/toml) |
| `puente/construir.py` | the whole pipeline in one command |
| `puente/desviar.py` | claw detours over the top where a transition self-collides |
| `puente/equilibrar.py` | automatic balance (IK of the four legs while a claw is up) |
| `puente/clip_a_npz.py` | base pose from Jumper's resting-floor solver (`tools/import_wbc_gestures.py`), writes the training `.npz` |
| `puente/ver_clip.py` | kinematic preview video, face screen following the script |
| `puente/esqueleto.py` | fast preview on the CPU: flat outlines of each part, claws coloured by screen side (`construir.py --esqueleto`) |
| `puente/factible.py` | can the real robot do it: joint speed, arm torque against the servo's measured curve, lag, self-contact, clearance to the phone |
| `puente/golpe.py` | the other end of a stroke (strum, key, drum hit): a tip move from a posture with the arm in the same fold, so the stroke stays short |
| `puente/exportar_blender.py`, `blender/escena.py` | the clip to Blender (meshes, poses, eye and phone pictures per frame) and the desk set that films it |
| `puente/instrumento.py` | which instrument is playing, second by second (YAMNet), and which step plays along; `coreo bailar --instrumentos` uses it |
| `puente/sonido_movimiento.py` | the clip's own movement sound: servo whir by speed, feet landing, pincer clicks, air on fast moves, the body settling |
| `puente/compatibilidad.py` | which claw posture goes with which leg stance without tipping over |
| `puente/exportar_robot.py`, `extraer_posturas.py` | regenerate `robot.json` and `posturas/base.toml` from Jumper |

The `jumper.coreo` task lives in [rodhandev/jumper](https://github.com/rodhandev/jumper) (branch `coreo-task`).

## Listening to the instrument

Jumper mimes the instrument it hears: `violin`, `guitarra`, `piano`, `tambor`, `viento` (trumpet, sax, flute),
`arpa` (also xylophone, marimba), `sacudir` (maracas, tambourine), `dj` (electronic music) and `cantar` (voice).
An instrument without a step of its own takes the closest gesture (a cello is bowed, an organ has keys); when
nothing is clearly ahead the robot sways. `puente/instrumento.py` lists every AudioSet class each step stands for.

It listens with [YAMNet](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet) (Apache-2.0,
4 MB as TFLite, small enough for the robot's RK3576). The model is fetched, not committed:

```sh
mkdir -p recursos/modelos && cd recursos/modelos     # next to the coreo checkout
curl -LO https://storage.googleapis.com/download.tensorflow.org/models/tflite/task_library/audio_classification/android/lite-model_yamnet_classification_tflite_1.tflite
mv lite-model_yamnet_classification_tflite_1.tflite yamnet.tflite
curl -LO https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet/yamnet_class_map.csv
uv pip install ai-edge-litert
python puente/instrumento.py track.wav --json tramos.json
coreo bailar track.pulsos.json --estilo romantico --instrumentos tramos.json -o baile.guion
```

Every instrument step is checked with `puente/factible.py`: within the servo's curve, no claw into the other or
the body, the body tilted at most 7°. Both front claws are also feet, so the side-reaching gestures (harp,
maracas) play one claw at a time.

## Filming on a desk

The published video is filmed in Blender, on a desk built from [Poly Haven](https://polyhaven.com) assets (CC0, no
attribution needed, free to hand on): the HDRI `poly_haven_studio` (a room with a window and a desk) and the glTF
models `WoodenTable_01`, `desk_lamp_arm_01`, `potted_plant_04`, `book_encyclopedia_set_01`, `alarm_clock_01`.
MuJoCo stays the source of the motion; Blender lights and films it.

```sh
python puente/exportar_blender.py <jumper> clip.npz model.xml export/ --guion clip.json --caras <faces dir>
blender -b -P blender/escena.py -- export/ <assets dir> still.png --frame 1             # one still
blender -b -P blender/escena.py -- export/ <assets dir> frames/ --gpu                   # the whole clip
#   --telefono-mesa X,Y,GIRO[,INCLINACION]: the phone on the desk beside the robot instead of in a holder
```

Fetch the assets with Poly Haven's API (`https://api.polyhaven.com/files/<id>`: the 2k `.exr`, the 1k glTF).

## Español

Se escribe en un guion de texto qué hace el robot y cuándo, sobre el pulso de la música; `coreo`
lo convierte en un clip de 22 articulaciones que el entrenamiento por imitación de Jumper aprende.
Las posturas tienen nombre por grupo de extremidades, la tenaza derecha se deduce de la izquierda,
el movimiento es de mínimo tirón y lo imposible se rechaza con su línea y su motivo.

Made by [Zymbiotek](https://zymbiotek.com) for the KingKong Robotics Crab Robot Open-source Challenge.
#Jumper #CrabRobot #OpenSourceRobot

License: Apache-2.0 (see `LICENSE` and `NOTICE`).
