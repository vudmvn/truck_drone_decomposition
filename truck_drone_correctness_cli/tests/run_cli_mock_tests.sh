#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$(mktemp)"
trap 'rm -f "$OUT"' EXIT
c++ -std=c++17 -Wall -Wextra -pedantic -DBBCP_WITH_ORTOOLS \
    -I"$ROOT/tests/stubs" "$ROOT/src/main.cpp" \
    "$ROOT/src/SlaveSolverFactory.cpp" -o "$OUT"
"$OUT" --list-variants
"$OUT" data/instances/toy_6.txt --assignment explicit --slave milp --truck sec
"$OUT" data/instances/toy_6.txt --assignment explicit --slave cp --truck mtz
"$OUT" data/instances/toy_6.txt --assignment slave-assign --slave cp --truck sec
if "$OUT" data/instances/toy_6.txt --assignment slave-assign --slave milp; then
    echo 'ERROR: invalid variant unexpectedly accepted' >&2; exit 1
fi
if "$OUT" data/instances/toy_6.txt --assignment slave-assign --slave cp --local-search on; then
    echo 'ERROR: unsupported local search unexpectedly accepted' >&2; exit 1
fi
python3 "$ROOT/tests/test_math_invariants.py"
echo 'MOCK CLI AND MATH CHECKS PASSED. Real solver linking not performed.'
