#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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
import subprocess
import sys
from typing import Any


def find_git_root() -> Path | None:
    """Find the git repository root from current directory."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=True,
        )
        return Path(result.stdout.strip())
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def get_git_tracked_files(root: Path) -> list[Path]:
    """
    Get list of tracked files from git ls-files.

    Returns list of Path objects relative to root.
    Only includes files tracked by git, respecting .gitignore.

    Args:
        root: Repository root directory

    Returns:
        List of Path objects relative to root, sorted.
        Returns empty list if not in git repo or on error.
    """
    try:
        result = subprocess.run(
            ["git", "ls-files", "--full-name", "--cached", "--others", "--exclude-standard"],
            capture_output=True,
            text=True,
            check=True,
            cwd=root,
        )
        if not result.stdout.strip():
            return []

        paths = [Path(line) for line in result.stdout.strip().split("\n") if line]
        return sorted(paths)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []


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
    tracked_files = get_git_tracked_files(root)

    for rel_path in tracked_files:
        parts = rel_path.parts

        # Add all suffix combinations
        for i in range(len(parts)):
            suffix = "/".join(parts[i:])
            index[suffix].append(rel_path)

    return index


def build_doc_index(root: Path) -> dict[str, list[Path]]:
    """
    Build an index mapping lowercase doc names to .md file paths.

    For docs/concepts/modules.md, creates entry:
    - "modules" -> [Path("docs/concepts/modules.md")]

    Also indexes directory index files:
    - "modules" -> [Path("docs/modules/index.md")] (if modules/index.md exists)
    """
    index: dict[str, list[Path]] = defaultdict(list)
    tracked_files = get_git_tracked_files(root)

    for rel_path in tracked_files:
        if rel_path.suffix != ".md":
            continue

        stem = rel_path.stem.lower()

        # For index.md files, also index by parent directory name
        if stem == "index":
            parent_name = rel_path.parent.name.lower()
            if parent_name:
                index[parent_name].append(rel_path)
        else:
            index[stem].append(rel_path)

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


def split_by_ignore_regions(content: str) -> list[tuple[str, bool]]:
    """
    Split content into regions, marking which should be processed.

    Returns list of (text, should_process) tuples.
    Regions between <!-- doclinks-ignore-start --> and <!-- doclinks-ignore-end --> are skipped.
    """
    ignore_start = re.compile(r"<!--\s*doclinks-ignore-start\s*-->", re.IGNORECASE)
    ignore_end = re.compile(r"<!--\s*doclinks-ignore-end\s*-->", re.IGNORECASE)

    regions = []
    pos = 0
    in_ignore = False

    while pos < len(content):
        if not in_ignore:
            # Look for start of ignore region
            match = ignore_start.search(content, pos)
            if match:
                # Add content before ignore marker (to be processed)
                if match.start() > pos:
                    regions.append((content[pos : match.start()], True))
                # Add the marker itself (not processed)
                regions.append((content[match.start() : match.end()], False))
                pos = match.end()
                in_ignore = True
            else:
                # No more ignore regions, add rest of content
                regions.append((content[pos:], True))
                break
        else:
            # Look for end of ignore region
            match = ignore_end.search(content, pos)
            if match:
                # Add ignored content including end marker
                regions.append((content[pos : match.end()], False))
                pos = match.end()
                in_ignore = False
            else:
                # Unclosed ignore region, add rest as ignored
                regions.append((content[pos:], False))
                break

    return regions


def process_markdown(
    content: str,
    root: Path,
    doc_path: Path,
    file_index: dict[str, list[Path]],
    link_mode: str,
    github_url: str | None,
    github_ref: str,
    doc_index: dict[str, list[Path]] | None = None,
) -> tuple[str, list[str], list[str]]:
    """
    Process markdown content, replacing file and doc links.

    Regions between <!-- doclinks-ignore-start --> and <!-- doclinks-ignore-end -->
    are skipped.

    Returns (new_content, changes, errors).
    """
    changes = []
    errors = []

    # Pattern 1: [`filename`](link) - code file links
    code_pattern = r"\[`([^`]+)`\]\(([^)]*)\)"

    # Pattern 2: [Text](.md) - doc file links
    doc_pattern = r"\[([^\]]+)\]\(\.md\)"

    def replace_code_match(match: re.Match[str]) -> str:
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

    def replace_doc_match(match: re.Match[str]) -> str:
        """Replace [Text](.md) with resolved doc path."""
        if doc_index is None:
            return match.group(0)

        link_text = match.group(1)
        full_match = match.group(0)
        lookup_key = link_text.lower()

        # Look up in doc index
        candidates = doc_index.get(lookup_key, [])

        if len(candidates) == 0:
            errors.append(f"No doc matching '{link_text}' found")
            return full_match
        elif len(candidates) > 1:
            errors.append(f"'{link_text}' matches multiple docs: {[str(c) for c in candidates]}")
            return full_match

        resolved_path = candidates[0]
        new_link = generate_link(resolved_path, root, doc_path, link_mode, github_url, github_ref)
        new_match = f"[{link_text}]({new_link})"

        if new_match != full_match:
            changes.append(f"  {link_text}: .md -> {new_link}")

        return new_match

    # Split by ignore regions and only process non-ignored parts
    regions = split_by_ignore_regions(content)
    result_parts = []

    for region_content, should_process in regions:
        if should_process:
            # Process code links first, then doc links
            processed = re.sub(code_pattern, replace_code_match, region_content)
            processed = re.sub(doc_pattern, replace_doc_match, processed)
            result_parts.append(processed)
        else:
            result_parts.append(region_content)

    new_content = "".join(result_parts)
    return new_content, changes, errors


def collect_markdown_files(paths: list[str]) -> list[Path]:
    """Collect markdown files from paths, expanding directories recursively."""
    result: list[Path] = []
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

Supports doc-to-doc linking: [Modules](.md) resolves to modules.md or modules/index.md.

Usage:
  doclinks [options] <paths...>

Examples:
  # Single file (auto-detects git root)
  doclinks docs/guide.md

  # Recursive directory
  doclinks docs/

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
  --root PATH          Repository root (default: git root)
  --link-mode MODE     absolute (default), relative, or github
  --github-url URL     Base GitHub URL (for github mode)
  --github-ref REF     Branch/ref for GitHub links (default: main)
  --dry-run            Show changes without modifying files
  --check              Exit with error if changes needed
  --watch              Watch for changes and re-process (requires watchdog)
  -h, --help           Show this help
"""


def main() -> None:
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
    parser.add_argument("--watch", action="store_true", help="Watch for changes and re-process")

    args = parser.parse_args()

    if args.help:
        print(USAGE)
        sys.exit(0)

    # Auto-detect git root if --root not provided
    if args.root:
        root = args.root.resolve()
    else:
        root = find_git_root()
        if root is None:
            print("Error: --root not provided and not in a git repository\n", file=sys.stderr)
            sys.exit(1)

    if not args.paths:
        print("Error: at least one path is required\n", file=sys.stderr)
        print(USAGE)
        sys.exit(1)

    if args.link_mode == "github" and not args.github_url:
        print("Error: --github-url is required when using --link-mode=github\n", file=sys.stderr)
        sys.exit(1)

    if not root.is_dir():
        print(f"Error: {root} is not a directory", file=sys.stderr)
        sys.exit(1)

    print(f"Building file index from {root}...")
    file_index = build_file_index(root)
    doc_index = build_doc_index(root)
    print(
        f"Indexed {sum(len(v) for v in file_index.values())} file paths, {len(doc_index)} doc names"
    )

    def process_file(md_path: Path, quiet: bool = False) -> tuple[bool, list[str]]:
        """Process a single markdown file. Returns (changed, errors)."""
        md_path = md_path.resolve()
        if not quiet:
            rel = md_path.relative_to(root) if md_path.is_relative_to(root) else md_path
            print(f"\nProcessing {rel}...")

        content = md_path.read_text()
        new_content, changes, errors = process_markdown(
            content,
            root,
            md_path,
            file_index,
            args.link_mode,
            args.github_url,
            args.github_ref,
            doc_index=doc_index,
        )

        if errors:
            for err in errors:
                print(f"  Error: {err}", file=sys.stderr)

        if changes:
            if not quiet:
                print("  Changes:")
                for change in changes:
                    print(change)
            if not args.dry_run and not args.check:
                md_path.write_text(new_content)
                if not quiet:
                    print("  Updated")
            return True, errors
        else:
            if not quiet:
                print("  No changes needed")
            return False, errors

    # Watch mode
    if args.watch:
        try:
            from watchdog.events import FileSystemEventHandler
            from watchdog.observers import Observer
        except ImportError:
            print(
                "Error: --watch requires watchdog. Install with: pip install watchdog",
                file=sys.stderr,
            )
            sys.exit(1)

        watch_paths = args.paths if args.paths else [str(root / "docs")]

        class MarkdownHandler(FileSystemEventHandler):  # type: ignore[misc]
            def on_modified(self, event: Any) -> None:
                if not event.is_directory and event.src_path.endswith(".md"):
                    process_file(Path(event.src_path))

            def on_created(self, event: Any) -> None:
                if not event.is_directory and event.src_path.endswith(".md"):
                    process_file(Path(event.src_path))

        observer = Observer()
        handler = MarkdownHandler()

        for watch_path in watch_paths:
            p = Path(watch_path)
            if p.is_file():
                p = p.parent
            print(f"Watching {p} for changes...")
            observer.schedule(handler, str(p), recursive=True)

        observer.start()
        try:
            while True:
                import time

                time.sleep(1)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()
        return

    # Normal mode
    markdown_files = collect_markdown_files(args.paths)
    if not markdown_files:
        print("No markdown files found", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(markdown_files)} markdown file(s)")

    all_errors = []
    any_changes = False

    for md_path in markdown_files:
        changed, errors = process_file(md_path)
        if changed:
            any_changes = True
        all_errors.extend(errors)

    if all_errors:
        print(f"\n{len(all_errors)} error(s) encountered", file=sys.stderr)
        sys.exit(1)

    if args.check and any_changes:
        print("\nChanges needed (--check mode)", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
