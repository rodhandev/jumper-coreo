#!/usr/bin/env bash
# Build cuna.stl: BOSL2 at a fixed commit, the shell's top from the jumper checkout, OpenSCAD, binary STL.
#   bash construir.sh <jumper checkout>      (from a Python env with mujoco and trimesh)
set -euo pipefail
cd "$(dirname "$0")"
JUMPER=${1:?jumper checkout}
BOSL2_COMMIT=d6d184505210c8b7bee76c4195905a34e7ded1a5   # 2026-10-08, BSD-2-Clause
if [ ! -d vendor/BOSL2 ]; then
  git clone -q https://github.com/BelfrySCAD/BOSL2.git vendor/BOSL2
fi
git -C vendor/BOSL2 checkout -q "$BOSL2_COMMIT"
python caparazon.py "$JUMPER" caparazon.stl
OPENSCADPATH="$PWD/vendor" openscad -o cuna.stl cuna.scad 2>&1 | grep -iE "warn|error" && exit 1 || true
# OpenSCAD writes ASCII STL; MuJoCo reads binary
python -c "import trimesh; m = trimesh.load('cuna.stl'); assert m.is_watertight and len(m.split()) == 1; m.export('cuna.stl'); print(f'cuna.stl: {m.volume / 1000:.1f} cm3, one closed piece')"
