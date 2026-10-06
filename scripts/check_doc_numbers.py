#!/usr/bin/env python3
"""Fail when a summary quotes a number that is not in the generated tables.

    python3 -B scripts/check_doc_numbers.py

Checked: every number with a decimal point or a percent sign in the summary
sections of README.md (section 1) and docs/results.md (the summary and the
claims table).  Each must occur in docs/results/tables.md, which
scripts/report.py writes from the data.  Constants of the method that are not
results (pass marks, tick length, thresholds) are listed in ALLOWED.
"""

import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
TABLES = os.path.join("docs", "results", "tables.md")
#: (file, start marker, end marker): the text between the markers is checked
SECTIONS = (
    ("README.md", "## 1. ", "## 2. "),
    (os.path.join("docs", "results.md"), "## Summary", "## 1. "),
    (os.path.join("docs", "results.md"), "## 12. The claims", "## 13. "),
)
#: not results: pass marks, settings and the chance level
ALLOWED = {"0.5", "0.3", "0.05", "0.02", "0.03", "1.0", "0.25", "126.5", "1.8", "0.7", "70%", "1.5", "0.1"}
NUMBER = re.compile(r"(?<![\w.])\d[\d,]*\.\d+%?|(?<![\w.])\d+%")


def section(text, start, end):
    if start not in text:
        return None
    body = text[text.index(start):]
    return body[: body.index(end)] if end in body else body


def numbers(text):
    # code spans and links are not prose
    text = re.sub(r"`[^`]*`", " ", text)
    text = re.sub(r"\]\([^)]*\)", "]", text)
    return sorted({match.group(0).replace(",", "") for match in NUMBER.finditer(text)})


def check(root=ROOT):
    """Return ``(checked, missing)``; ``missing`` is a list of ``(file, number)``."""

    with open(os.path.join(root, TABLES), encoding="utf-8") as stream:
        tables = stream.read().replace(",", "")
    known = set(NUMBER.findall(tables))
    checked, missing = 0, []
    for name, start, end in SECTIONS:
        path = os.path.join(root, name)
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as stream:
            body = section(stream.read(), start, end)
        if body is None:
            continue
        for number in numbers(body):
            checked += 1
            if number not in known and number not in ALLOWED:
                missing.append((name, number))
    return checked, missing


def main():
    checked, missing = check()
    for name, number in missing:
        print("%s quotes %s, which is not in %s" % (name, number, TABLES))
    print("%d numbers checked, %d not found" % (checked, len(missing)))
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
