#!/usr/bin/env python3
"""Regenerate per-PDF TOC and text dumps for grep/Read-tool lookup.

For every <name>.pdf in this directory, writes:
  <name>.toc.md   — markdown outline with 1-based PDF page numbers (Read tool format)
  <name>.txt      — pdftotext -layout dump with `=== PAGE N ===` markers between pages

Run: `python3 resources/build_index.py` after adding/replacing PDFs.
Requires: PyMuPDF (`pip install pymupdf`), poppler `pdftotext` on PATH.
"""
import os
import subprocess
import sys

import fitz

HERE = os.path.dirname(os.path.abspath(__file__))


def write_toc(pdf_path: str, out_path: str) -> int:
    doc = fitz.open(pdf_path)
    toc = doc.get_toc()
    title = (doc.metadata or {}).get("title", "") or os.path.basename(pdf_path)
    lines = [
        f"# {os.path.basename(pdf_path)}",
        "",
        f"**Title:** {title}",
        f"**Pages:** {doc.page_count}",
        "",
        "Page numbers are 1-based PDF pages (pass directly to the Read tool's `pages` parameter).",
        "",
        "---",
        "",
    ]
    for lvl, heading, page in toc:
        indent = "  " * (lvl - 1)
        lines.append(f"{indent}- p{page}: {heading}")
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return len(toc)


def write_text(pdf_path: str, out_path: str) -> int:
    raw = subprocess.check_output(
        ["pdftotext", "-layout", pdf_path, "-"]
    ).decode("utf-8", errors="replace")
    pages = raw.split("\f")
    if pages and pages[-1] == "":
        pages = pages[:-1]
    with open(out_path, "w") as f:
        for i, body in enumerate(pages, 1):
            f.write(f"=== PAGE {i} ===\n")
            f.write(body.rstrip("\n"))
            f.write("\n\n")
    return len(pages)


def main() -> int:
    pdfs = sorted(p for p in os.listdir(HERE) if p.lower().endswith(".pdf"))
    if not pdfs:
        print("No PDFs found in", HERE, file=sys.stderr)
        return 1
    for pdf in pdfs:
        base = pdf[:-4]
        pdf_path = os.path.join(HERE, pdf)
        toc_path = os.path.join(HERE, base + ".toc.md")
        txt_path = os.path.join(HERE, base + ".txt")
        n_toc = write_toc(pdf_path, toc_path)
        n_pages = write_text(pdf_path, txt_path)
        print(f"{pdf}: {n_toc} TOC entries, {n_pages} pages -> {base}.toc.md, {base}.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
