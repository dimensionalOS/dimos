"""MkDocs hooks for DimOS documentation build process."""

from pathlib import Path
import subprocess
import sys


def on_post_build(config):
    """
    Generate LLM-optimized context files after the main build completes.

    Converts the generated llms.txt file into Claude-optimized XML formats
    using the llm-ctx tool.
    """
    site_dir = Path(config["site_dir"])
    llms_txt = site_dir / "llms.txt"

    if not llms_txt.exists():
        print("Warning: llms.txt not found, skipping llms-ctx generation", file=sys.stderr)
        return

    print("Generating llms-ctx files...")

    try:
        # Generate llms-ctx.txt (without optional sections)
        with open(site_dir / "llms-ctx.txt", "w") as f:
            subprocess.run(["llms_txt2ctx", str(llms_txt)], stdout=f, check=True, text=True)

        # Generate llms-ctx-full.txt (with optional sections)
        with open(site_dir / "llms-ctx-full.txt", "w") as f:
            subprocess.run(
                ["llms_txt2ctx", str(llms_txt), "--optional", "true"],
                stdout=f,
                check=True,
                text=True,
            )

        print(f"  ✓ Generated {site_dir}/llms-ctx.txt")
        print(f"  ✓ Generated {site_dir}/llms-ctx-full.txt")

    except subprocess.CalledProcessError as e:
        print(f"Error generating llms-ctx files: {e}", file=sys.stderr)
    except FileNotFoundError:
        print(
            "Error: llms_txt2ctx command not found. Install with: pip install llm-ctx",
            file=sys.stderr,
        )
