#!/usr/bin/env bash
#
# Mirror this repo's Markdown documentation into a GitHub Wiki working tree.
#
# The wiki is treated as a *generated artifact*: on every run the wiki tree is
# wiped (except .git) and rebuilt from the repo, so renamed/deleted docs are
# reflected and hand-edited wiki pages do not drift. Edit the docs in the repo,
# never the wiki directly.
#
# Strategy: mirror the doc folders preserving their relative paths so the
# relative `*.md` links and `images/` references already in the docs keep
# resolving inside the wiki. The root README becomes the wiki landing page
# (Home), which GitHub requires to be named `Home.md`.
#
# Usage: sync_docs_to_wiki.sh <repo_root> <wiki_dir>
#
set -euo pipefail

SRC="${1:?repo root}"
WIKI="${2:?wiki working dir}"

# Doc trees to mirror verbatim (path-preserving), plus image dirs they reference.
TREES=(docs documents vx_images)

# 1. Clean the wiki tree (keep .git) — the action fully owns the wiki content.
find "$WIKI" -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} +

# 2. Landing page: root README -> Home.md (+ localized variant alongside it).
cp "$SRC/README.md" "$WIKI/Home.md"
[ -f "$SRC/README_zh_CN.md" ] && cp "$SRC/README_zh_CN.md" "$WIKI/README_zh_CN.md"

# 3. Mirror the doc trees, preserving relative structure.
for d in "${TREES[@]}"; do
    if [ -d "$SRC/$d" ]; then
        mkdir -p "$WIKI/$d"
        cp -r "$SRC/$d/." "$WIKI/$d/"
    fi
done

# 4. Never publish internal-only files to the public wiki.
rm -f "$WIKI/CLAUDE.md" "$WIKI/docs/CLAUDE.md"

# 5. Fix up links that don't survive the move into the wiki namespace:
#    - the root README is published as Home, so `../README.md` must point at Home;
#    - links into repo dirs that are NOT mirrored (e.g. tools/) are rewritten to
#      absolute GitHub URLs. Set REPO_BLOB_URL=https://github.com/OWNER/REPO/blob/BRANCH.
REPO_BLOB_URL="${REPO_BLOB_URL:-}"
while IFS= read -r f; do
    sed -i -E 's#\]\(\.\./README\.md#](../Home.md#g' "$f"
    if [ -n "$REPO_BLOB_URL" ]; then
        sed -i -E "s#\]\(\.\./tools/#](${REPO_BLOB_URL}/tools/#g" "$f"
        sed -i -E "s#\]\(tools/#](${REPO_BLOB_URL}/tools/#g" "$f"
    fi
done < <(find "$WIKI" -name '*.md')

# 6. Generate a navigation sidebar from the mirrored Markdown.
#    Page title = first level-1 heading if present, else the file name.
page_title() {
    local f="$1" t
    t="$(grep -m1 -E '^# ' "$f" 2>/dev/null | sed -E 's/^#[[:space:]]+//')"
    [ -n "$t" ] && printf '%s' "$t" || basename "$f" .md
}

sidebar="$WIKI/_Sidebar.md"
{
    echo "### SpadisQT"
    echo "- [Home](Home)"
    echo "- [README (中文)](README_zh_CN)"
    for d in docs documents; do
        [ -d "$WIKI/$d" ] || continue
        echo ""
        echo "**${d}/**"
        # Stable ordering; link uses the wiki page path (no .md suffix).
        while IFS= read -r f; do
            rel="${f#"$WIKI"/}"          # e.g. docs/architecture.md
            page="${rel%.md}"            # e.g. docs/architecture
            echo "- [$(page_title "$f")]($page)"
        done < <(find "$WIKI/$d" -maxdepth 1 -name '*.md' | sort)
    done
} > "$sidebar"

echo "Wiki tree rebuilt under: $WIKI"
