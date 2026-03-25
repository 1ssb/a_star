#!/usr/bin/env bash
# test_gallery.sh — Run all 10 gallery scenarios and report pass/fail
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BINARY="$SCRIPT_DIR/astar"
GALLERY_CFG="$SCRIPT_DIR/gallery/planner.cfg"
GALLERY_DIR="$SCRIPT_DIR/gallery"

if [ ! -x "$BINARY" ]; then
    echo "Building astar..."
    make -C "$SCRIPT_DIR" >/dev/null 2>&1
fi

pass=0
fail=0
total=0

for query in "$GALLERY_DIR"/*.cfg; do
    name=$(basename "$query" .cfg)
    # Skip the static planner config
    [ "$name" = "planner" ] && continue

    total=$((total + 1))
    printf "  %-30s " "$name"

    output=$("$BINARY" --config "$GALLERY_CFG" --query "$query" 2>&1) || true

    if echo "$output" | grep -q "^Path found:"; then
        waypoints=$(echo "$output" | grep "^Path found:" | sed 's/.*: \([0-9]*\) waypoints.*/\1/')
        controls=$(echo "$output" | grep "^Wrote" | grep "controls" | sed 's/Wrote \([0-9]*\) controls.*/\1/')
        violations=""
        if echo "$output" | grep -q "safety violations"; then
            violations=" [SAFETY VIOLATION]"
        fi
        printf "PASS  (%s wp, %s ctrl)%s\n" "$waypoints" "$controls" "$violations"
        pass=$((pass + 1))
    else
        printf "FAIL  (no path found)\n"
        fail=$((fail + 1))
    fi
done

echo ""
echo "Results: $pass/$total passed, $fail failed"
exit $fail
