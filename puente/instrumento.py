"""Which instrument is playing, second by second, and which coreo step plays along: listen, then dance.

Rodhan (16-oct): «the robot should mime the instrument that sounds: guitar if it hears guitar, violin if violin,
drum if drum… make a list of general instruments as a base; an instrument we do not have gets the closest one».
YAMNet (Google, Apache-2.0: AudioSet's 521 classes, 4 MB as TFLite, small enough for the robot's RK3576) scores
each 0.975 s of audio; the AudioSet classes are grouped into the nine instrument steps below, and the winner, held
for at least MIN_S, is the step to dance. When no step is clearly ahead the robot sways to the beat (`mecerse`).

    python instrumento.py <pista.wav> [--modelo yamnet.tflite] [--clases yamnet_class_map.csv] [--json out.json]

Writes the segments ([start s, end s, step, score]) and prints them; with `--pulsos <pulsos.json>` (zym-compas
pulsos) it prints them as guion lines on the track's beats, ready to paste.
"""

import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MODELOS = HERE.parents[1] / "recursos" / "modelos"   # outside the repo: YAMNet is fetched, not committed
SR, WIN, HOP = 16000, 15600, 8000                    # YAMNet's input: 0.975 s at 16 kHz; a window every 0.5 s
#: YAMNet scores each class on its own and «Music» takes most of it: on our tracks an instrument scores 0.02-0.05
#: (measured 16-oct). So a step wins when its classes add up to UMBRAL and it beats the runner-up by MARGEN.
UMBRAL, MARGEN, MIN_S = 0.04, 1.3, 2.0

#: step -> the AudioSet classes it stands for. What has no step of its own goes to the closest gesture: a cello is
#: bowed (violin), an organ has keys (piano), a xylophone is struck up and down (arpa), a rap is sung (cantar).
PASOS = {
    "violin": ["Violin, fiddle", "Cello", "Double bass", "Bowed string instrument", "String section", "Orchestra",
               "Pizzicato"],
    "guitarra": ["Guitar", "Electric guitar", "Bass guitar", "Acoustic guitar", "Steel guitar, slide guitar",
                 "Tapping (guitar technique)", "Strum", "Banjo", "Sitar", "Mandolin", "Ukulele",
                 "Plucked string instrument", "Zither"],
    "piano": ["Piano", "Electric piano", "Organ", "Electronic organ", "Hammond organ", "Synthesizer", "Sampler",
              "Harpsichord", "Keyboard (musical)"],
    "tambor": ["Drum kit", "Drum machine", "Drum", "Snare drum", "Rimshot", "Drum roll", "Bass drum", "Timpani",
               "Tabla", "Cymbal", "Hi-hat", "Percussion", "Wood block"],
    "viento": ["Trumpet", "Trombone", "Brass instrument", "French horn", "Saxophone", "Flute", "Clarinet",
               "Wind instrument, woodwind instrument", "Harmonica", "Accordion", "Bagpipes", "Didgeridoo"],
    "arpa": ["Harp", "Glockenspiel", "Marimba, xylophone", "Vibraphone", "Mallet percussion", "Steelpan", "Chime"],
    "sacudir": ["Maraca", "Tambourine", "Rattle (instrument)", "Cowbell", "Jingle bell"],
    "dj": ["Electronic music", "Electronica", "Electronic dance music", "Techno", "House music", "Dubstep",
           "Drum and bass", "Hip hop music", "Scratching (performance technique)", "Trance music", "Disco"],
    "cantar": ["Singing", "Choir", "Child singing", "Synthetic singing", "Rapping", "Vocal music", "A capella",
               "Humming", "Yodeling"],
}


def arg(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def audio(path):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-ac", "1", "-ar", str(SR), "-f", "f32le", "-"],
                         check=True, capture_output=True).stdout
    return np.frombuffer(raw, np.float32)


def puntuar(x, modelo, clases):
    """Per window (every HOP samples): the score of each step = the sum of its classes."""
    from ai_edge_litert.interpreter import Interpreter

    names = [r["display_name"] for r in csv.DictReader(open(clases))]
    cols = {p: [names.index(c) for c in cs] for p, cs in PASOS.items()}
    it = Interpreter(str(modelo))
    it.allocate_tensors()
    i, o = it.get_input_details()[0]["index"], it.get_output_details()[0]["index"]
    filas = []
    for a in range(0, max(len(x) - WIN, 0) + 1, HOP):
        it.set_tensor(i, x[a:a + WIN].astype(np.float32))
        it.invoke()
        s = it.get_tensor(o)[0]
        filas.append([float(s[cols[p]].sum()) for p in PASOS])
    return np.array(filas)


def tramos(S):
    """Smooth over 2 s, take the winner per window, and merge runs; runs shorter than MIN_S join a neighbour."""
    k = int(MIN_S * SR / HOP)
    sm = np.apply_along_axis(lambda c: np.convolve(c, np.ones(k) / k, mode="same"), 0, S)
    nombres = list(PASOS)
    orden = np.sort(sm, axis=1)
    gan = [nombres[j] if sm[n, j] >= UMBRAL and sm[n, j] >= MARGEN * orden[n, -2] else "mecerse"
           for n, j in enumerate(sm.argmax(1))]
    t = lambda n: n * HOP / SR  # noqa: E731
    out = []
    for n, g in enumerate(gan):
        if out and out[-1][2] == g:
            out[-1][1] = t(n + 1)
        else:
            out.append([t(n), t(n + 1), g])
    while len(out) > 1 and min(b - a for a, b, _ in out) < MIN_S:   # absorb the shortest run into a neighbour
        j = min(range(len(out)), key=lambda q: out[q][1] - out[q][0])
        v = j - 1 if j > 0 else j + 1
        a, b = min(out[j][0], out[v][0]), max(out[j][1], out[v][1])
        out[min(j, v)] = [a, b, out[v][2]]
        del out[max(j, v)]
        while len(out) > 1 and any(out[q][2] == out[q + 1][2] for q in range(len(out) - 1)):
            q = next(q for q in range(len(out) - 1) if out[q][2] == out[q + 1][2])
            out[q][1] = out[q + 1][1]
            del out[q + 1]
    for seg in out:   # the mean score of the winning step over the run, for the record
        a, b = int(seg[0] * SR / HOP), max(int(seg[1] * SR / HOP), int(seg[0] * SR / HOP) + 1)
        seg.append(round(float(sm[a:b, nombres.index(seg[2])].mean()), 3) if seg[2] in PASOS else 0.0)
    return [[round(a, 2), round(b, 2), g, s] for a, b, g, s in out]


def main():
    pista = Path(sys.argv[1])
    modelo = Path(arg("--modelo", MODELOS / "yamnet.tflite"))
    clases = Path(arg("--clases", MODELOS / "yamnet_class_map.csv"))
    if not modelo.exists():
        raise SystemExit(f"{modelo} missing: fetch YAMNet (see coreo/README.md, «Listening to the instrument»)")
    segs = tramos(puntuar(audio(pista), modelo, clases))
    for a, b, g, s in segs:
        print(f"{a:6.1f}-{b:6.1f} s  {g:9} {s:.2f}")
    if arg("--json"):
        Path(arg("--json")).write_text(json.dumps({"pista": str(pista), "tramos": segs}, indent=1))
    if arg("--pulsos"):
        p = json.loads(Path(arg("--pulsos")).read_text())
        beats = p["pulsos"] if isinstance(p, dict) else p
        print("# guion (pulsos de la pista):")
        for a, _, g, _ in segs:
            n = min(range(len(beats)), key=lambda q: abs(beats[q] - a))
            print(f"@{n + 1} paso {g}")


if __name__ == "__main__":
    main()
