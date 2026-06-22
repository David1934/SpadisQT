#!/usr/bin/env python3
"""Publish the repo's Markdown docs to a GitHub Wiki working tree.

GitHub Wikis use a **flat** page namespace: a `.md` file inside a subdirectory of
the wiki repo is NOT served as a page. So we cannot mirror `docs/`, `documents/`
as subfolders — every page must live at the wiki root. This script:

  1. Maps each source doc to a unique top-level wiki page (slug).
  2. Copies the referenced assets (images, PDF) to fixed wiki-root locations.
  3. Rewrites every internal Markdown link to the flattened slug / asset path,
     resolving `../` relative to each file. Links into non-mirrored repo dirs
     (e.g. tools/) become absolute GitHub URLs (REPO_BLOB_URL).
  4. Generates a _Sidebar.md for navigation.

The wiki is a generated artifact — edit docs in the repo, never the wiki.

Usage: sync_docs_to_wiki.py <repo_root> <wiki_dir>
Env:   REPO_BLOB_URL=https://github.com/OWNER/REPO/blob/BRANCH  (for off-wiki links)
"""
import os
import re
import shutil
import sys

SRC = os.path.abspath(sys.argv[1])
WIKI = os.path.abspath(sys.argv[2])
REPO_BLOB_URL = os.environ.get("REPO_BLOB_URL", "").rstrip("/")
# GitHub Wiki does NOT resolve relative subdirectory asset paths from a page, so
# images/PDFs must be referenced by their absolute raw wiki URL. The files are
# still committed into the wiki repo (images/, vx_images/, root pdf) so the URL
# resolves. e.g. https://raw.githubusercontent.com/wiki/OWNER/REPO
WIKI_RAW_BASE = os.environ.get("WIKI_RAW_BASE", "").rstrip("/")

# Internal files that must never be published to the public wiki.
EXCLUDE = {"CLAUDE.md", "docs/CLAUDE.md"}

LINK_RE = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\))")


def discover_pages():
    """Return {repo_relpath: wiki_slug} for every Markdown page to publish."""
    sources = ["README.md", "README_zh_CN.md"]
    for d in ("docs", "documents"):
        dpath = os.path.join(SRC, d)
        if not os.path.isdir(dpath):
            continue
        for f in sorted(os.listdir(dpath)):
            if f.endswith(".md"):
                sources.append(f"{d}/{f}")
    pages = {}
    for rel in sources:
        if rel in EXCLUDE or not os.path.isfile(os.path.join(SRC, rel)):
            continue
        pages[rel] = slug_for(rel)
    return pages


def slug_for(rel):
    """Flatten a repo doc path to a unique top-level wiki page slug."""
    if rel == "README.md":
        return "Home"
    name = os.path.basename(rel)[:-3]  # strip .md
    # docs/README* would collide with the root README (Home); give it its own name.
    if os.path.dirname(rel) == "docs" and name.startswith("README"):
        return "Developer-Docs" + name[len("README"):]
    return name


def asset_dest(resolved):
    """Where a referenced asset lands in the wiki (or None if not an asset)."""
    if resolved.startswith("docs/images/"):
        return resolved[len("docs/"):]            # -> images/<file>
    if resolved.startswith("vx_images/"):
        return resolved                            # -> vx_images/<file>
    if resolved.startswith("documents/") and resolved.lower().endswith(".pdf"):
        return os.path.basename(resolved)          # -> <file>.pdf at root
    return None


def rewrite_target(srcrel, target, pages):
    """Map one link target (as seen in srcrel) to its wiki destination."""
    if re.match(r"^[a-z][a-z0-9+.-]*://", target) or target.startswith("#"):
        return target  # external URL or pure in-page anchor
    path, sep, anchor = target.partition("#")
    if not path:
        return target
    resolved = os.path.normpath(os.path.join(os.path.dirname(srcrel), path))
    if resolved in pages:                       # internal doc page -> slug
        return pages[resolved] + (sep + anchor if sep else "")
    dest = asset_dest(resolved)
    if dest is not None:                        # asset -> absolute raw wiki URL
        url = f"{WIKI_RAW_BASE}/{dest}" if WIKI_RAW_BASE else dest
        return url + (sep + anchor if sep else "")
    if REPO_BLOB_URL and not resolved.startswith(".."):
        return f"{REPO_BLOB_URL}/{resolved}" + (sep + anchor if sep else "")
    return target


def rewrite_links(srcrel, text, pages):
    return LINK_RE.sub(
        lambda m: m.group(1) + rewrite_target(srcrel, m.group(2), pages) + m.group(3),
        text,
    )


def main():
    pages = discover_pages()

    # Wipe the wiki tree (keep .git) — the action fully owns wiki content.
    for entry in os.listdir(WIKI):
        if entry == ".git":
            continue
        p = os.path.join(WIKI, entry)
        shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)

    # Copy referenced assets to their flat wiki locations.
    if os.path.isdir(os.path.join(SRC, "docs/images")):
        shutil.copytree(os.path.join(SRC, "docs/images"), os.path.join(WIKI, "images"))
    if os.path.isdir(os.path.join(SRC, "vx_images")):
        shutil.copytree(os.path.join(SRC, "vx_images"), os.path.join(WIKI, "vx_images"))
    for f in os.listdir(os.path.join(SRC, "documents")):
        if f.lower().endswith(".pdf"):
            shutil.copy2(os.path.join(SRC, "documents", f), os.path.join(WIKI, f))

    # Render each page to a flat slug with rewritten links.
    for rel, slug in pages.items():
        with open(os.path.join(SRC, rel), encoding="utf-8") as fh:
            text = fh.read()
        text = rewrite_links(rel, text, pages)
        with open(os.path.join(WIKI, slug + ".md"), "w", encoding="utf-8") as fh:
            fh.write(text)

    write_sidebar(pages)
    print(f"Wiki rebuilt: {len(pages)} pages under {WIKI}")


def page_title(rel):
    with open(os.path.join(SRC, rel), encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("# "):
                return line[2:].strip()
    return slug_for(rel)


def write_sidebar(pages):
    lines = ["### SpadisQT", "- [Home](Home)", "- [README (中文)](README_zh_CN)"]
    for label, d in (("docs/", "docs"), ("documents/", "documents")):
        group = [r for r in pages if r.startswith(d + "/")]
        if not group:
            continue
        lines.append("")
        lines.append(f"**{label}**")
        for rel in sorted(group):
            lines.append(f"- [{page_title(rel)}]({pages[rel]})")
    with open(os.path.join(WIKI, "_Sidebar.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
