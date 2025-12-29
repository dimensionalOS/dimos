#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Markdown reference lookup tool.

Finds markdown links like [`service/spec.py`](...) and fills in the correct
file path from the codebase.

Usage:
    python reference_lookup.py --root /repo/root [options] markdownfile.md
"""

import argparse
from collections import defaultdict
import os
from pathlib import Path
import re
import sys


def load_gitignore_patterns(root: Path) -> list[str]:
    """Load patterns from .gitignore file."""
    gitignore = root / ".gitignore"
    if not gitignore.exists():
        return []

    patterns = []
    with open(gitignore) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                patterns.append(line)
    return patterns


def should_ignore(path: Path, root: Path, patterns: list[str]) -> bool:
    """Check if path should be ignored based on gitignore patterns."""
    rel_path = path.relative_to(root)
    path_str = str(rel_path)
    name = path.name

    # Always ignore these
    if name in {".git", ".venv", "venv", "node_modules", "__pycache__", ".mypy_cache"}:
        return True

    for pattern in patterns:
        # Handle directory patterns (ending with /)
        if pattern.endswith("/"):
            dir_pattern = pattern[:-1]
            if name == dir_pattern or path_str.startswith(dir_pattern + "/"):
                return True
        # Handle glob patterns
        elif "*" in pattern:
            import fnmatch

            if fnmatch.fnmatch(name, pattern) or fnmatch.fnmatch(path_str, pattern):
                return True
        # Simple name match
        elif name == pattern or path_str == pattern or path_str.startswith(pattern + "/"):
            return True

    return False


def build_file_index(root: Path) -> dict[str, list[Path]]:
    """
    Build an index mapping filename suffixes to full paths.

    For /dimos/protocol/service/spec.py, creates entries for:
    - spec.py
    - service/spec.py
    - protocol/service/spec.py
    - dimos/protocol/service/spec.py
    """
    index: dict[str, list[Path]] = defaultdict(list)
    patterns = load_gitignore_patterns(root)

    for dirpath, dirnames, filenames in os.walk(root):
        current = Path(dirpath)

        # Filter out ignored directories
        dirnames[:] = [d for d in dirnames if not should_ignore(current / d, root, patterns)]

        for filename in filenames:
            filepath = current / filename
            if should_ignore(filepath, root, patterns):
                continue

            rel_path = filepath.relative_to(root)
            parts = rel_path.parts

            # Add all suffix combinations
            for i in range(len(parts)):
                suffix = "/".join(parts[i:])
                index[suffix].append(rel_path)

    return index


def find_symbol_line(file_path: Path, symbol: str) -> int | None:
    """Find the first line number where symbol appears."""
    try:
        with open(file_path, encoding="utf-8", errors="replace") as f:
            for line_num, line in enumerate(f, start=1):
                if symbol in line:
                    return line_num
    except OSError:
        pass
    return None


def extract_other_backticks(line: str, file_ref: str) -> list[str]:
    """Extract other backticked terms from a line, excluding the file reference."""
    pattern = r"`([^`]+)`"
    matches = re.findall(pattern, line)
    return [m for m in matches if m != file_ref and not m.endswith(".py") and "/" not in m]


def generate_link(
    rel_path: Path,
    root: Path,
    doc_path: Path,
    link_mode: str,
    github_url: str | None,
    github_ref: str,
    line_fragment: str = "",
) -> str:
    """Generate the appropriate link format."""
    if link_mode == "absolute":
        return f"/{rel_path}{line_fragment}"
    elif link_mode == "relative":
        doc_dir = (
            doc_path.parent.relative_to(root) if doc_path.is_relative_to(root) else doc_path.parent
        )
        target = root / rel_path
        try:
            rel_link = os.path.relpath(target, root / doc_dir)
        except ValueError:
            rel_link = str(rel_path)
        return f"{rel_link}{line_fragment}"
    elif link_mode == "github":
        if not github_url:
            raise ValueError("--github-url required when using --link-mode=github")
        return f"{github_url.rstrip('/')}/blob/{github_ref}/{rel_path}{line_fragment}"
    else:
        raise ValueError(f"Unknown link mode: {link_mode}")


def process_markdown(
    content: str,
    root: Path,
    doc_path: Path,
    file_index: dict[str, list[Path]],
    link_mode: str,
    github_url: str | None,
    github_ref: str,
) -> tuple[str, list[str], list[str]]:
    """
    Process markdown content, replacing file links.

    Returns (new_content, changes, errors).
    """
    # Pattern: [`filename`](link)
    pattern = r"\[`([^`]+)`\]\(([^)]*)\)"
    changes = []
    errors = []

    def replace_match(match: re.Match) -> str:
        file_ref = match.group(1)
        current_link = match.group(2)
        full_match = match.group(0)

        # Skip anchor-only links (e.g., [`Symbol`](#section))
        if current_link.startswith("#"):
            return full_match

        # Skip if the reference doesn't look like a file path (no extension or path separator)
        if "." not in file_ref and "/" not in file_ref:
            return full_match

        # Look up in index
        candidates = file_index.get(file_ref, [])

        if len(candidates) == 0:
            errors.append(f"No file matching '{file_ref}' found in codebase")
            return full_match
        elif len(candidates) > 1:
            errors.append(f"'{file_ref}' matches multiple files: {[str(c) for c in candidates]}")
            return full_match

        resolved_path = candidates[0]

        # Determine line fragment
        line_fragment = ""

        # Check if current link has a line fragment to preserve
        if "#" in current_link:
            line_fragment = "#" + current_link.split("#", 1)[1]
        else:
            # Look for other backticked symbols on the same line
            line_start = content.rfind("\n", 0, match.start()) + 1
            line_end = content.find("\n", match.end())
            if line_end == -1:
                line_end = len(content)
            line = content[line_start:line_end]

            symbols = extract_other_backticks(line, file_ref)
            if symbols:
                # Try to find the first symbol in the target file
                full_file_path = root / resolved_path
                for symbol in symbols:
                    line_num = find_symbol_line(full_file_path, symbol)
                    if line_num is not None:
                        line_fragment = f"#L{line_num}"
                        break

        new_link = generate_link(
            resolved_path, root, doc_path, link_mode, github_url, github_ref, line_fragment
        )
        new_match = f"[`{file_ref}`]({new_link})"

        if new_match != full_match:
            changes.append(f"  {file_ref}: {current_link} -> {new_link}")

        return new_match

    new_content = re.sub(pattern, replace_match, content)
    return new_content, changes, errors


def collect_markdown_files(paths: list[str]) -> list[Path]:
    """Collect markdown files from paths, expanding directories recursively."""
    result = []
    for p in paths:
        path = Path(p)
        if path.is_dir():
            result.extend(path.rglob("*.md"))
        elif path.exists():
            result.append(path)
    return sorted(set(result))


USAGE = """\
doclinks - Update markdown file links to correct codebase paths

Finds [`filename.py`](...) patterns and resolves them to actual file paths.
Also auto-links symbols: `Configurable` on same line adds #L<line> fragment.

Usage:
  doclinks --root <repo> [options] <paths...>

Examples:
  # Single file
  doclinks --root . docs/guide.md

  # Recursive directory
  doclinks --root . docs/

  # GitHub links
  doclinks --root . --link-mode github \\
    --github-url https://github.com/org/repo docs/

  # Relative links (from doc location)
  doclinks --root . --link-mode relative docs/

  # CI check (exit 1 if changes needed)
  doclinks --root . --check docs/

  # Dry run (show changes without writing)
  doclinks --root . --dry-run docs/

Options:
  --root PATH          Repository root (required)
  --link-mode MODE     absolute (default), relative, or github
  --github-url URL     Base GitHub URL (for github mode)
  --github-ref REF     Branch/ref for GitHub links (default: main)
  --dry-run            Show changes without modifying files
  --check              Exit with error if changes needed
  -h, --help           Show this help
"""


def main():
    if len(sys.argv) == 1:
        print(USAGE)
        sys.exit(0)

    parser = argparse.ArgumentParser(
        description="Update markdown file links to correct codebase paths",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        add_help=False,
    )
    parser.add_argument("paths", nargs="*", help="Markdown files or directories to process")
    parser.add_argument("--root", type=Path, help="Repository root path")
    parser.add_argument("-h", "--help", action="store_true", help="Show help")
    parser.add_argument(
        "--link-mode",
        choices=["absolute", "relative", "github"],
        default="absolute",
        help="Link format (default: absolute)",
    )
    parser.add_argument("--github-url", help="Base GitHub URL (required for github mode)")
    parser.add_argument("--github-ref", default="main", help="GitHub branch/ref (default: main)")
    parser.add_argument(
        "--dry-run", action="store_true", help="Show changes without modifying files"
    )
    parser.add_argument(
        "--check", action="store_true", help="Exit with error if changes needed (CI mode)"
    )

    args = parser.parse_args()

    if args.help:
        print(USAGE)
        sys.exit(0)

    if not args.root:
        print("Error: --root is required\n", file=sys.stderr)
        print(USAGE)
        sys.exit(1)

    if not args.paths:
        print("Error: at least one path is required\n", file=sys.stderr)
        print(USAGE)
        sys.exit(1)

    if args.link_mode == "github" and not args.github_url:
        print("Error: --github-url is required when using --link-mode=github\n", file=sys.stderr)
        sys.exit(1)

    root = args.root.resolve()
    if not root.is_dir():
        print(f"Error: {root} is not a directory", file=sys.stderr)
        sys.exit(1)

    print(f"Building file index from {root}...")
    file_index = build_file_index(root)
    print(f"Indexed {sum(len(v) for v in file_index.values())} file path entries")

    all_errors = []
    any_changes = False

    markdown_files = collect_markdown_files(args.paths)
    if not markdown_files:
        print("No markdown files found", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(markdown_files)} markdown file(s)")

    for md_path in markdown_files:
        md_path = md_path.resolve()

        print(
            f"\nProcessing {md_path.relative_to(root) if md_path.is_relative_to(root) else md_path}..."
        )
        content = md_path.read_text()

        new_content, changes, errors = process_markdown(
            content, root, md_path, file_index, args.link_mode, args.github_url, args.github_ref
        )

        if errors:
            all_errors.extend(errors)
            for err in errors:
                print(f"  Error: {err}", file=sys.stderr)

        if changes:
            any_changes = True
            print("  Changes:")
            for change in changes:
                print(change)

            if not args.dry_run and not args.check:
                md_path.write_text(new_content)
                print("  Updated")
        else:
            print("  No changes needed")

    if all_errors:
        print(f"\n{len(all_errors)} error(s) encountered", file=sys.stderr)
        sys.exit(1)

    if args.check and any_changes:
        print("\nChanges needed (--check mode)", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
