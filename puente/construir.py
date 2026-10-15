"""One command from a guion to a trainable clip and its preview.

    guion --coreo compilar--> clip.json --desviar--> clip-d.json --equilibrar--> clip-eq.json
          --clip_a_npz--> clip.npz --ver_clip--> clip.mp4

    python construir.py <jumper checkout> <guion> [--dest <dir>] [--sin-desvio] [--sin-equilibrio]
                        [--modelo <xml>] [--caras <dir>] [--musica <pista>] [--sin-video]
                        [--tam 1080x1920] [--camara DIST,AZIMUT,ELEVACION]

Run from the jumper checkout's venv. Every step is the script of the same name in this
directory, run as is; a step that refuses (a posture that does not exist, a joint too
fast, a base that tips) stops the build with its own message.
"""

import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
COREO = HERE.parent
#: The same limit the Rust side checks (src/robot.rs, VEL_MAX), re-checked here
#: because the detour rewrites the claws and the balance pass the legs.
VEL_MAX = 12.0


def arg(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def run(*cmd):
    r = subprocess.run([str(c) for c in cmd], text=True, capture_output=True)
    for line in (r.stdout + r.stderr).splitlines():
        if "warn" not in line.lower():
            print("  " + line)
    if r.returncode:
        raise SystemExit(r.returncode)


def main() -> None:
    repo, guion = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    dest = Path(arg("--dest", COREO / "salidas")).resolve()
    dest.mkdir(parents=True, exist_ok=True)
    name = guion.stem
    py = sys.executable

    print(f"[1] compilar {guion.name}")
    run("cargo", "run", "--release", "--offline", "-q", "--manifest-path", COREO / "Cargo.toml",
        "--", "compilar", guion, "-o", dest / f"{name}.json")
    clip = dest / f"{name}.json"
    if "--sin-desvio" not in sys.argv:
        # Before balancing: the detour lifts the claws, and the balance has to see them lifted.
        print("[2] desviar")
        run(py, HERE / "desviar.py", repo, clip, dest / f"{name}-d.json")
        clip = dest / f"{name}-d.json"
    if "--sin-equilibrio" not in sys.argv:
        print("[2b] equilibrar")
        run(py, HERE / "equilibrar.py", repo, clip, dest / f"{name}-eq.json")
        clip = dest / f"{name}-eq.json"
        c = json.loads(clip.read_text())
        rows = c["q"]
        v = max(abs(b - a) * c["hz"] for r0, r1 in zip(rows, rows[1:]) for a, b in zip(r0, r1))
        if v > VEL_MAX:
            raise SystemExit(f"  the balanced clip moves a joint at {v:.1f} rad/s (limit {VEL_MAX})")
    print("[3] clip_a_npz")
    run(py, HERE / "clip_a_npz.py", repo, clip, dest / f"{name}.npz")
    if "--sin-video" not in sys.argv:
        print("[4] ver_clip")
        model = arg("--modelo", repo / "assets" / "jumper" / "jumper.xml")
        extra = ["--guion", clip]
        extra += ["--caras", arg("--caras")] if arg("--caras") else []
        extra += ["--musica", arg("--musica")] if arg("--musica") else []
        extra += [x for k in ("--tam", "--camara") if arg(k) for x in (k, arg(k))]
        run(py, HERE / "ver_clip.py", repo, dest / f"{name}.npz", model, dest / f"{name}.mp4", *extra)
    print(f"listo: {dest / (name + '.npz')}")


if __name__ == "__main__":
    main()
