#!/bin/sh
# Convert every reference input with sldprt-glb and compare the GLB (and the
# exit status) with the reference result.
# usage: tools/glb_all.sh <sldprt-glb binary> <data dir> [out.tsv]
# <data dir>/in/<stem>.SLDPRT|.SLDASM are the inputs; <data dir>/ref/ holds
# <stem>.cadir.json, <stem>.glb and timings.tsv (column 4: expected exit status).
# Extra converter options (e.g. --generator / --extras-key matching the
# reference) come from $SWGLB_ARGS, parsed with shell quoting rules.
# out.tsv: stem, exit status, seconds, comparison (identical/different/...)
set -u
bin=$1; data=$2; tsv=${3:-/dev/null}
eval "set -- ${SWGLB_ARGS:-}"
tmp=$(mktemp -d)
identical=0; different=0; status_mismatch=0
: > "$tsv"
for ref in "$data"/ref/*.cadir.json; do
  stem=$(basename "$ref" .cadir.json)
  src=$(ls "$data"/in/"$stem".* 2>/dev/null | head -1)
  [ -n "$src" ] || continue
  ext=${src##*.}
  out="$tmp/$stem.glb"
  start=$(date +%s.%N)
  SWGLB_DEBUG=1 "$bin" "$@" "$src" "$out" > "$tmp/$stem.out" 2> "$tmp/$stem.err"; rc=$?
  end=$(date +%s.%N)
  secs=$(echo "$end - $start" | bc)
  ref_rc=$(awk -F'\t' -v s="$stem" '$1 == s { print $4 }' "$data/ref/timings.tsv")
  refglb="$data/ref/$stem.glb"
  if [ "$rc" != "${ref_rc:-0}" ]; then
    result="status $rc vs ${ref_rc:-?}: $(head -c 200 "$tmp/$stem.err")"
    status_mismatch=$((status_mismatch+1))
  elif [ "$rc" != 0 ]; then
    result="both-failed"; identical=$((identical+1))
  elif cmp -s "$out" "$refglb"; then
    result=identical; identical=$((identical+1))
  else
    result="different"; different=$((different+1))
    echo "== $stem"; python3 "$(dirname "$0")/compare_glb.py" "$out" "$refglb" | head -15
  fi
  printf '%s\t%s\t%s\t%s\n' "$stem" "$rc" "$secs" "$result" >> "$tsv"
  [ "${result#status}" != "$result" ] && echo "== $stem: $result"
done
echo "identical=$identical different=$different status_mismatch=$status_mismatch"
rm -rf "$tmp"
