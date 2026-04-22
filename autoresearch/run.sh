#!/bin/bash
# Autoresearch runner for Go2 CPU optimization
# Usage: bash run.sh [--profile-only] [--skip-profile] [--timeout SECONDS]
cd "$(dirname "$0")"
python3 eval.py "$@" 2>&1 | tee results.txt
