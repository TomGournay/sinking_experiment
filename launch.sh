#!/bin/sh
set -eu

cd "$(dirname "$0")"
export PYTHONPATH="$PWD/INSLIB/python${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m uvicorn backend:app \
    --host 0.0.0.0 \
    --port 8080 \
    --ws none
