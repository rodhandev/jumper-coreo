"""The sound of a clip's movement, made from the clip itself: nothing placed by hand.

Rodhan (16-oct): «it should also make movement sounds… the actuators' own are not enough; another way to give
the robot an aesthetic for the ear». Five layers, each from something the clip already says:

- whir: a soft servo hum per moving joint, its pitch and loudness following the joint's speed (filtered noise
  plus a tone that rises with speed), very low: the robot sounds alive while it moves, silent when still;
- tap: a dry click each time a foot lands (its height crosses down to the floor);
- snip: a short bright click when a pincer starts to open or close (J4 starts moving);
- whoosh: air on a fast arm move (claw speed past WHOOSH m/s), a noise sweep shaped by that speed;
- settle: a soft low thump when the body comes down and stops (vertical speed turns from falling to rising).

All synthesised here (Apache-2.0, like the rest of coreo), mixed LAYER_DB under the expressive sounds.

    python sonido_movimiento.py <jumper checkout> <clip.npz> <out.wav> [--modelo <xml>] [--capas whir,tap,...]
"""

import sys
import wave
from pathlib import Path

import mujoco
import numpy as np

SR = 48000
LAYER_DB = -12.0          # under the expressive (family A) sounds
WHOOSH = 0.9              # m/s at the claw tip
FLOOR = 0.008             # m: a foot this close to the floor is down

repo, clip, dest = Path(sys.argv[1]).resolve(), Path(sys.argv[2]), Path(sys.argv[3])
arg = lambda n, d=None: sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d  # noqa: E731
capas = set(arg("--capas", "whir,tap,snip,whoosh,settle").split(","))
sys.path[:0] = [str(repo / "tools"), str(repo)]
import import_wbc_dances as dances  # noqa: E402
from tasks.jumper.common.constants import FEET  # noqa: E402
from tasks.jumper.common.dance.motion import LEG_JOINTS  # noqa: E402

z = np.load(clip)
dt = float(z["dt"])
names = [n for leg in sorted(LEG_JOINTS) for n in LEG_JOINTS[leg]]
q = np.stack([z[k] for k in dances._keys()], axis=1).astype(np.float64)
base = np.stack([z[f"body_{a}"] for a in dances.AXES], axis=1).astype(np.float64)
m = mujoco.MjModel.from_xml_path(str(arg("--modelo", repo / "assets/jumper/jumper.xml")))
d = mujoco.MjData(m)
free = m.jnt_qposadr[next(j for j in range(m.njnt) if m.jnt_type[j] == 0)]
adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f) for f in FEET]
tips = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_finger_tip_link") for s in ("LF", "RF")]

# kinematics per row: foot heights, claw tip positions
fz, tp = [], []
for k in range(len(q)):
    x, y, zz, r, p, yw = base[k]
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, [r, p, yw], "xyz")
    d.qpos[free:free + 7] = (x, y, zz, *quat)
    d.qpos[adr] = q[k]
    mujoco.mj_kinematics(m, d)
    fz.append([d.xpos[b][2] for b in feet])
    tp.append([d.xpos[b].copy() for b in tips])
fz, tp = np.array(fz), np.array(tp)
fz -= fz.min(axis=0)   # each foot's lowest point in the clip is the floor
n = int(len(q) * dt * SR) + SR // 2
out = np.zeros(n)
rng = np.random.default_rng(3)
t_row = np.arange(len(q)) * dt


def up(x):
    """A per-row curve at audio rate."""
    return np.interp(np.arange(n) / SR, t_row, x)


def put(t, s, g=1.0):
    a = int(t * SR)
    b = min(a + len(s), n)
    if 0 <= a < n:
        out[a:b] += g * s[:b - a]


def env(dur, a=0.002, decay=0.02):
    t = np.arange(int(dur * SR)) / SR
    return np.minimum(t / a, 1) * np.exp(-t / decay)


def bandnoise(length, f, bw):
    """Noise in a band around f (FFT filter): the air and the hum."""
    s = np.fft.rfft(rng.standard_normal(length))
    fr = np.fft.rfftfreq(length, 1 / SR)
    s *= np.exp(-((fr - f) / bw) ** 2)
    y = np.fft.irfft(s, length)
    return y / (np.abs(y).max() + 1e-9)


qd = np.abs(np.gradient(q, dt, axis=0))
if "whir" in capas:
    # one hum for the whole robot: the sum of joint speeds drives level and pitch (servos in a box sound as one)
    v = np.clip(qd.sum(axis=1) / 25.0, 0, 1)
    v = np.convolve(v, np.ones(5) / 5, mode="same")
    V = up(v)
    f = 180 + 520 * V                          # 180 Hz idle hum up to 700 Hz at speed
    tono = np.sin(2 * np.pi * np.cumsum(f) / SR) + 0.3 * np.sin(4 * np.pi * np.cumsum(f) / SR)
    aire = bandnoise(n, 1400, 900)
    out += 0.10 * V ** 1.5 * (0.6 * tono + 0.4 * aire)
if "tap" in capas:
    down = fz < FLOOR
    for j in range(fz.shape[1]):
        for k in np.where(down[1:, j] & ~down[:-1, j])[0] + 1:
            fp = 2600 + 400 * j
            s = env(0.05, 0.0005, 0.008) * (0.6 * rng.standard_normal(int(0.05 * SR))
                                              + 0.8 * np.sin(2 * np.pi * fp * np.arange(int(0.05 * SR)) / SR))
            put(k * dt, s, 0.5)
if "snip" in capas:
    for col in [names.index(f"{s}_J4_joint") for s in ("LF", "RF")]:
        moving = np.abs(np.gradient(q[:, col], dt)) > 0.3   # coreo's curves are smooth: no sudden reversal
        for k in np.where(moving[1:] & ~moving[:-1])[0] + 1:
            put(k * dt, env(0.03, 0.0003, 0.004) * np.sin(2 * np.pi * 4200 * np.arange(int(0.03 * SR)) / SR), 0.6)
if "whoosh" in capas:
    sp = np.linalg.norm(np.gradient(tp, dt, axis=0), axis=2).max(axis=1)
    over = sp > WHOOSH
    for k in np.where(over[1:] & ~over[:-1])[0] + 1:
        e = k
        while e < len(sp) and over[e]:
            e += 1
        dur = max((e - k) * dt + 0.15, 0.2)
        L = int(dur * SR)
        y = bandnoise(L, 900 + 900 * min(sp[k:e].max() / 2, 1), 700)
        put(k * dt, y * np.sin(np.pi * np.arange(L) / L) ** 2, 0.35)
if "settle" in capas:
    vz = np.gradient(np.convolve(base[:, 2], np.ones(5) / 5, mode="same"), dt)
    falling = vz < -0.015
    for k in np.where(~falling[1:] & falling[:-1])[0] + 1:
        L = int(0.18 * SR)
        tt = np.arange(L) / SR
        put(k * dt, env(0.18, 0.004, 0.05) * np.sin(2 * np.pi * (90 - 120 * tt) * tt), 0.8)

out *= 10 ** (LAYER_DB / 20) / (np.abs(out).max() + 1e-9) * 0.9
with wave.open(str(dest), "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes((np.clip(out, -1, 1) * 32767).astype("<i2").tobytes())
print(f"{dest}: {n / SR:.1f} s, layers {', '.join(sorted(capas))}")
