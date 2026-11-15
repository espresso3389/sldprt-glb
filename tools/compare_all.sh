#!/bin/sh
# Compare every reference CADIR with `sldprt-glb dump`.
# usage: tools/compare_all.sh <sldprt-glb binary> <data dir> [areas]
set -u
bin=$1; data=$2; areas=${3:-tessellations,ownership,appearance,presentation,brep,assembly}
tmp=$(mktemp -d)
pass=0; fail=0
for ref in "$data"/ref/*.cadir.json; do
  stem=$(basename "$ref" .cadir.json)
  src=$(ls "$data"/in/"$stem".* 2>/dev/null | head -1)
  [ -n "$src" ] || continue
  if ! "$bin" dump "$src" > "$tmp/$stem.json" 2> "$tmp/$stem.err"; then
    echo "== $stem: dump failed: $(head -c 300 "$tmp/$stem.err")"; fail=$((fail+1)); continue
  fi
  if out=$(python3 "$(dirname "$0")/compare_cadir.py" "$tmp/$stem.json" "$ref" --only "$areas"); then
    pass=$((pass+1))
  else
    echo "== $stem"; echo "$out" | head -20; fail=$((fail+1))
  fi
done
echo "pass=$pass fail=$fail"
rm -rf "$tmp"
