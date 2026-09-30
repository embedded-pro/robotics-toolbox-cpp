#!/usr/bin/env python3
"""Validate that algorithm documentation files follow the project template.

Checks every .md file under doc/ (excluding README.md, TEMPLATE.md, and backups)
for the required section headings defined in the template, in template order,
and checks that relative links in every .md file under doc/ resolve.

Exit codes:
    0 — all files pass, or no algorithm documentation files found
    1 — doc directory is missing, or one or more files fail a check
"""

import pathlib
import re
import sys

REQUIRED_SECTIONS = [
    "Overview & Motivation",
    "Mathematical Theory",
    "Complexity Analysis",
    "Step-by-Step Walkthrough",
    "Pitfalls & Edge Cases",
    "Variants & Generalizations",
    "Applications",
    "Connections to Other Algorithms",
    "References & Further Reading",
]

SKIP_NAMES = {"README.md", "TEMPLATE.md"}
SKIP_DIRS = {".backup"}

DOC_ROOT = pathlib.Path(__file__).resolve().parent.parent / "doc"


def extract_h2_headings(text: str) -> list[str]:
    return re.findall(r"^## (.+)$", text, re.MULTILINE)


LINK_PATTERN = re.compile(r"\]\(([^)\s]+)\)")


def validate_file(path: pathlib.Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    headings = extract_h2_headings(text)
    problems = [f"missing: ## {s}" for s in REQUIRED_SECTIONS if s not in headings]

    present = [h for h in headings if h in REQUIRED_SECTIONS]
    expected = [s for s in REQUIRED_SECTIONS if s in present]
    if present != expected:
        problems.append("sections out of template order: " + " → ".join(present))

    return problems


def broken_links(path: pathlib.Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    problems = []
    for target in LINK_PATTERN.findall(text):
        if target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        relative = target.split("#", 1)[0]
        if relative and not (path.parent / relative).exists():
            problems.append(f"broken link: {target}")
    return problems


def main() -> int:
    if not DOC_ROOT.is_dir():
        print(f"ERROR: doc directory not found at {DOC_ROOT}", file=sys.stderr)
        return 1

    md_files = sorted(
        p
        for p in DOC_ROOT.rglob("*.md")
        if p.name not in SKIP_NAMES
        and not any(part in SKIP_DIRS for part in p.parts)
    )

    if not md_files:
        print("WARNING: no algorithm documentation files found.")
        return 0

    failures: list[tuple[pathlib.Path, list[str]]] = []
    for path in md_files:
        problems = validate_file(path)
        if problems:
            failures.append((path, problems))

    for path in sorted(DOC_ROOT.rglob("*.md")):
        if path.name == "TEMPLATE.md" or any(part in SKIP_DIRS for part in path.parts):
            continue
        problems = broken_links(path)
        if problems:
            failures.append((path, problems))

    if failures:
        print(f"FAIL: {len(failures)} file(s) do not follow the template:\n")
        for path, missing in failures:
            rel = path.relative_to(DOC_ROOT)
            print(f"  {rel}")
            for problem in missing:
                print(f"    - {problem}")
            print()
        return 1

    print(f"OK: {len(md_files)} file(s) validated successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
