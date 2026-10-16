#!/usr/bin/env bash
# Build both holders and their phone: BOSL2 at a fixed commit, the shell's top from the jumper checkout, OpenSCAD,
# binary STL. Writes stl/<version>-<part>.stl for version vertical|plana, part cuna|cuerpo|pantalla|isla|camara.
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
mkdir -p stl
for v in vertical plana; do
  for p in cuna cuerpo pantalla isla camara; do
    OPENSCADPATH="$PWD/vendor" openscad -D "parte=\"$p\"" -o "stl/$v-$p.stl" "cuna-$v.scad" 2>&1 | grep -iE "warn|error" && exit 1 || true
  done
done
# OpenSCAD writes ASCII STL; MuJoCo reads binary. The holder must be one closed piece.
python - <<'EOF'
import glob, trimesh
for f in sorted(glob.glob("stl/*.stl")):
    m = trimesh.load(f)
    if f.endswith("-cuna.stl"):
        assert m.is_watertight and len(m.split()) == 1, f"{f}: not one closed piece"
    m.export(f)
    print(f"{f}: {m.volume / 1000:.1f} cm3")
EOF
