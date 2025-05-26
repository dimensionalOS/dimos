import tempfile
from pathlib import Path

import pytest

from dimos.utils import testing


class TestRealRepoIntegration:
    """Integration tests using the actual dimos repository."""

    def test_testfile_lfs_download(self):
        """Test testFile with actual LFS files that need to be downloaded."""
        import hashlib
        import os
        import subprocess

        repo_root = testing._get_repo_root()
        test_data_dir = repo_root / "tests" / "data"

        # Ensure we start with LFS pointer files, not actual content
        binary_file = test_data_dir / "binary_test_file.bin"

        # Remove actual file if it exists
        if binary_file.exists():
            binary_file.unlink()

        # Restore as LFS pointer file (without downloading content)
        env = os.environ.copy()
        env["GIT_LFS_SKIP_SMUDGE"] = "1"
        subprocess.run(
            ["git", "checkout", "HEAD", "--", "tests/data/binary_test_file.bin"],
            cwd=repo_root,
            env=env,
            check=True,
            capture_output=True,
        )

        # Verify we have a pointer file (small ASCII text file)
        assert binary_file.exists()
        assert binary_file.stat().st_size < 200  # Pointer files are small

        # Test with the binary LFS file
        result = testing.testFile("binary_test_file.bin")

        assert isinstance(result, Path)
        assert result.exists()
        assert result.name == "binary_test_file.bin"
        # Should be 10KB (10240 bytes) as we created it
        assert result.stat().st_size == 10240

        # Verify the file hash matches expected SHA256
        expected_sha256 = "01c779312c35758412ab3b27623aee38239398d79e75a8cf4786b19c495d0cba"
        with result.open("rb") as f:
            actual_sha256 = hashlib.sha256(f.read()).hexdigest()
        assert actual_sha256 == expected_sha256, f"File hash mismatch: expected {expected_sha256}, got {actual_sha256}"

    def test_testfile_with_existing_files(self):
        """Test testFile with any existing files in tests/data directory."""
        repo_root = testing._get_repo_root()
        test_data_dir = repo_root / "tests" / "data"

        # List existing files in tests/data
        existing_files = [f for f in test_data_dir.iterdir() if f.is_file()] if test_data_dir.exists() else []

        if not existing_files:
            pytest.skip("No existing test data files found")

        # Test with the first existing file
        test_file = existing_files[0]

        result = testing.testFile(test_file.name)

        assert isinstance(result, Path)
        assert result.exists()
        assert result.name == test_file.name
        assert result == test_file

    def test_lfs_pointer_detection_with_real_files(self):
        """Test LFS pointer detection with any existing files."""
        repo_root = testing._get_repo_root()
        test_data_dir = repo_root / "tests" / "data"

        if not test_data_dir.exists():
            pytest.skip("No tests/data directory found")

        # Check all files in tests/data directory
        existing_files = [f for f in test_data_dir.iterdir() if f.is_file()]

        if not existing_files:
            pytest.skip("No existing test data files found")

        for test_file in existing_files:
            # Test LFS pointer detection
            is_pointer = testing._is_lfs_pointer_file(test_file)
            # Just verify the function runs without error
            assert isinstance(is_pointer, bool)


class TestErrorConditions:
    """Test error conditions and edge cases."""

    def test_not_in_git_repo(self):
        """Test behavior when not in a git repository."""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Change to a non-git directory
            import os

            original_cwd = os.getcwd()
            try:
                os.chdir(temp_dir)
                # Clear cache to force fresh detection
                testing._get_repo_root.cache_clear()

                with pytest.raises(RuntimeError, match="Not in a Git repository"):
                    testing._get_repo_root()
            finally:
                os.chdir(original_cwd)
                # Clear cache again to reset for other tests
                testing._get_repo_root.cache_clear()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
