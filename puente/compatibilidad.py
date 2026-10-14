"""Which claw posture goes with which leg stance: final tilt after moving there from HOME.

For each (claws, legs) pair, coreo's own interpolation (minimum jerk, 1 s) from HOME,
then the repo's resting-floor solver; prints the tilt it ends at, or the worst along
the way. The table is what a guion can combine without tipping the robot over.

    python compatibilidad.py <jumper checkout> <coreo dir>
"""
import json, subprocess, sys, tempfile
from pathlib import Path

import numpy as np

repo, coreo = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_gestures as G  # noqa: E402

out = subprocess.run([coreo / "target/debug/coreo", "posturas"], capture_output=True, text=True).stdout
cat = {l.split(":")[0]: l.split(": ")[1].split(", ") for l in out.strip().splitlines()}
claws = [("tenazas", p) for p in cat["tenaza (tenaza_izq, tenaza_der, tenazas)"] + cat["tenazas, sólo las dos juntas"]]
claws += [("tenaza_izq", p) for p in cat["tenaza (tenaza_izq, tenaza_der, tenazas)"] if p != "reposo"]
legs = cat["patas"]
R = G._Robot()
tmp = Path(tempfile.mkdtemp())
print(f"{'':28s}" + "".join(f"{l:>13s}" for l in legs))
for grupo, c in claws:
    row = f"{grupo + ' ' + c:28s}"
    for l in legs:
        g = tmp / "p.guion"
        g.write_text(f"@0 {grupo} {c} en 1s\n@0 patas {l} en 1s\n@2s fin\n")
        r = subprocess.run([coreo / "target/debug/coreo", "compilar", g, "-o", tmp / "p.json"],
                           capture_output=True, text=True)
        if r.returncode:
            row += f"{'(error)':>13s}"
            continue
        clip = json.loads((tmp / "p.json").read_text())
        q = np.asarray(clip["q"])[:, [clip["joints"].index(n) for n in R.names]]
        try:
            pose, _ = G.solve_resting_pose(R, q)
            tilt = np.degrees(np.hypot(pose[:, 3], pose[:, 4])).max()
            row += f"{tilt:>12.0f}{'°' if tilt <= 17 else '✗'}"
        except ValueError:
            row += f"{'(cae)':>13s}"
    print(row)
