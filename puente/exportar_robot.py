"""Write coreo/robot.json: what the Rust side needs to know about the robot.

Joint names in the entity's order, HOME, joint ranges from the compiled MJCF,
the limb groups and the mirror rule (symmetry.py). Run from the jumper checkout's
venv; regenerate when the model changes upstream.
"""

import json
import sys
from pathlib import Path

import mujoco

from tasks.jumper.common.constants import HOME, get_spec
from tasks.jumper.common.mdp.symmetry import _SIGN_KEEP

model = get_spec(None).compile()
names = list(HOME)
rng = {}
for n in names:
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    rng[n] = [float(v) for v in model.jnt_range[j]]

out = {
    "joints": names,
    "home": HOME,
    "range": rng,
    "groups": {
        "tenaza_izq": [n for n in names if n.startswith("LF_")],
        "tenaza_der": [n for n in names if n.startswith("RF_")],
        "pata_media_izq": [n for n in names if n.startswith("LM_")],
        "pata_media_der": [n for n in names if n.startswith("RM_")],
        "pata_trasera_izq": [n for n in names if n.startswith("LR_")],
        "pata_trasera_der": [n for n in names if n.startswith("RR_")],
    },
    "mirror_keep_sign": list(_SIGN_KEEP),
    "mujoco": mujoco.__version__,
}
dest = Path(sys.argv[1])
dest.write_text(json.dumps(out, indent=1) + "\n")
print(f"wrote {dest}: {len(names)} joints")
