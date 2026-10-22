# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Exact SDK API pins must survive automatic shared-source refreshes."""

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
loader = SourceFileLoader("relock_shared_flakes", str(ROOT / "bin/relock-shared-flakes"))
spec = spec_from_loader(loader.name, loader)
relock = module_from_spec(spec)
loader.exec_module(relock)


class ExactSdkVersionTests(unittest.TestCase):
    def stale(self, requirement: str, revision_matches: bool = True) -> list[str]:
        revision = "a" * 40
        manifest_revision = revision if revision_matches else "b" * 40
        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "Cargo.toml"
            manifest.write_text(
                '[dependencies]\ndimos-module = { git = "'
                + relock.GIT_URL
                + '", rev = "'
                + manifest_revision
                + '"'
                + requirement
                + " }\n"
            )
            with (
                patch.object(
                    relock,
                    "_pinned_crates",
                    return_value=[
                        ("dimos-module", "0.1.0", revision),
                        ("dimos-module-macros", "0.1.0", revision),
                    ],
                ),
                patch.object(
                    relock,
                    "_crate_dirs",
                    return_value={
                        "dimos-module": "native/rust/dimos-module",
                        "dimos-module-macros": "native/rust/dimos-module-macros",
                    },
                ),
                patch.object(relock, "_git", return_value='[package]\nversion = "0.2.0"'),
                patch.object(relock, "tree_at", side_effect=lambda rev, _: rev),
            ):
                return relock._stale_pin(manifest, manifest.with_name("Cargo.lock"))

    def test_exact_legacy_api_keeps_its_sdk_and_transitive_macro_revision(self):
        self.assertEqual(self.stale(', version = "=0.1.0"'), [])

    def test_unconstrained_dependencies_still_refresh_changed_sources(self):
        self.assertEqual(len(self.stale("")), 2)

    def test_exact_constraint_does_not_hide_a_mismatched_lock_revision(self):
        failures = self.stale(', version = "=0.1.0"', revision_matches=False)
        self.assertEqual(len(failures), 2)
        self.assertTrue(all("Cargo.lock pins" in error for error in failures))


if __name__ == "__main__":
    unittest.main()
