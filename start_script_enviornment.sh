#!/usr/bin/env bash
# Open an interactive shell in dev_tests/ with this checkout's venv active.
RIG_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
source "$RIG_DIR/venv/bin/activate"
export PATH="$HOME/.local/bin:$PATH"
cd "$RIG_DIR/dev_tests"
# hand control to an interactive login shell so you get a prompt
exec bash -li
