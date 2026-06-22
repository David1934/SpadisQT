#!/usr/bin/env bash
#
# gen_diagrams.sh — render all SpadisQT Graphviz diagram sources to PNG + SVG.
#
# The .dot files under tools/diagrams/ are the single source of truth for every
# diagram used by the README and the docs/. Never hand-edit the generated images;
# edit the .dot source and re-run this script.
#
# Usage:
#   tools/gen_diagrams.sh            # render to docs/images/ (+ refresh README images)
#   tools/gen_diagrams.sh <out_dir>  # render to a custom output directory
#
# Requires: graphviz (the `dot` binary).  Install: sudo apt-get install -y graphviz
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"
SRC_DIR="$HERE/diagrams"
OUT_DIR="${1:-$REPO_ROOT/docs/images}"

# Diagrams that the README and the porting guides embed from vx_images/.
# They are refreshed there too so existing relative links keep working.
README_IMAGES=("arch_overview" "data_flow" "deployment")
VX_DIR="$REPO_ROOT/vx_images"

if ! command -v dot >/dev/null 2>&1; then
    echo "ERROR: graphviz 'dot' not found. Install with: sudo apt-get install -y graphviz" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

shopt -s nullglob
sources=("$SRC_DIR"/*.dot)
if [ ${#sources[@]} -eq 0 ]; then
    echo "ERROR: no .dot sources found in $SRC_DIR" >&2
    exit 1
fi

echo "Rendering ${#sources[@]} diagram(s) from $SRC_DIR -> $OUT_DIR"
for src in "${sources[@]}"; do
    name="$(basename "$src" .dot)"
    dot -Tpng "$src" -o "$OUT_DIR/$name.png"
    dot -Tsvg "$src" -o "$OUT_DIR/$name.svg"
    echo "  [ok] $name.png / $name.svg"

    # Mirror the README/guide diagrams into vx_images/ so existing links resolve.
    for ri in "${README_IMAGES[@]}"; do
        if [ "$name" = "$ri" ]; then
            mkdir -p "$VX_DIR"
            cp -f "$OUT_DIR/$name.png" "$VX_DIR/$name.png"
            echo "       -> mirrored to vx_images/$name.png"
        fi
    done
done

echo "Done."
