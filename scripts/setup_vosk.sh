#!/usr/bin/env bash
# One-time, workspace-local dependency installation. No robot or microphone access.
set -euo pipefail
hri_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
hri_ws_dir="$(cd -- "$hri_repo_dir/../.." && pwd)"
hri_venv="$hri_ws_dir/.venvs/vosk"
hri_model_name=vosk-model-small-es-0.42
mkdir -p "$hri_ws_dir/models" "$hri_ws_dir/.cache/pip"
if [[ ! -x "$hri_venv/bin/python" ]]; then
  python3 -m venv --system-site-packages "$hri_venv"
fi
if ! "$hri_venv/bin/python" -c 'import vosk' >/dev/null 2>&1; then
  "$hri_venv/bin/python" -m pip install --cache-dir "$hri_ws_dir/.cache/pip" 'vosk==0.3.45'
fi
if [[ ! -f "$hri_ws_dir/models/$hri_model_name/am/final.mdl" ]]; then
  hri_archive="$(mktemp /tmp/hri-vosk-model-XXXXXX.zip)"
  trap 'rm -f -- "$hri_archive"' EXIT
  curl -fL --retry 2 "https://alphacephei.com/vosk/models/$hri_model_name.zip" -o "$hri_archive"
  echo "09b239888f633ef2f0b4e09736e3d9936acfd810bc65d53fad45261762c6511f  $hri_archive" | sha256sum -c -
  "$hri_venv/bin/python" - "$hri_archive" "$hri_ws_dir/models" <<'PY'
import sys, zipfile
with zipfile.ZipFile(sys.argv[1]) as archive:
    if archive.testzip() is not None:
        raise SystemExit('Damaged Vosk archive')
    archive.extractall(sys.argv[2])
PY
fi
printf 'Vosk preparado: %s\n' "$hri_ws_dir/models/$hri_model_name"
