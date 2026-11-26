#!/bin/bash
# Start the PhotographyRig app from this checkout, whatever its path.
# Uses ./venv (create with: python3 -m venv --system-site-packages venv, so the
# system picamera2/libcamera are visible) and MediaMTX from ./mediamtx/mediamtx,
# $MEDIAMTX_BIN, or `mediamtx` on PATH (~/.local/bin is added below).
set -e

RIG_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
cd "$RIG_DIR"   # main.py serves index.html and ./captures from the CWD

if [ ! -x venv/bin/python ]; then
    echo "start_rig.sh: no venv at $RIG_DIR/venv" >&2
    exit 1
fi
# shellcheck disable=SC1091
source venv/bin/activate
export PATH="$HOME/.local/bin:$PATH"

PYTHONUNBUFFERED=1 exec python main.py "$@"
