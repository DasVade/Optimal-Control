#!/usr/bin/env bash
# Persistent, isolated project environment. Reuse it on subsequent invocations.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ENV="${OPTIMAL_CONTROL_ENV:-$HOME/.venvs/optimal-control}"
BASE_PYTHON="${OPTIMAL_CONTROL_BASE_PYTHON:-$HOME/miniconda3/envs/hw2/bin/python}"
if [[ ! -x "$PROJECT_ENV/bin/python" ]]; then
  "$BASE_PYTHON" -m venv "$PROJECT_ENV"
fi
"$PROJECT_ENV/bin/python" -c 'import sys; assert sys.version_info[:2] == (3,11), "The WSL lock was verified with Python 3.11"'
"$PROJECT_ENV/bin/python" -m pip install -r "$PROJECT_ROOT/requirements-wsl-lock.txt"
"$PROJECT_ENV/bin/python" -m pip install --no-deps -e "$PROJECT_ROOT"
"$PROJECT_ENV/bin/python" -c 'import pydrake, importlib.metadata; print("Drake", importlib.metadata.version("drake"))'
printf 'Persistent interpreter: %s/bin/python\n' "$PROJECT_ENV"
