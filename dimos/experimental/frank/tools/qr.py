# Copyright 2026 Dimensional Inc.
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

"""A printable QR sticker for FRANK's chat app.

    uv run python dimos/experimental/frank/tools/qr.py https://abc123.ngrok.app
    uv run python dimos/experimental/frank/tools/qr.py https://abc123.ngrok.app --print
    uv run python dimos/experimental/frank/tools/qr.py --ngrok --print

Needs either the `qrcode` package or the `qrencode` CLI for the code itself; the page
layout is Pillow. Neither installed is a hard error — install one, don't work around it.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont
import requests

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[3]

NGROK_API = "http://127.0.0.1:4040/api/tunnels"
PAGE = (2480, 3508)  # A4 at 300 dpi
HEADLINE = "Scan to meet FRANK"


class QrError(Exception):
    """No way to make a QR code, or no URL to put in one."""


def ngrok_url() -> str:
    """The public https URL of the first ngrok tunnel on this laptop."""
    try:
        tunnels = requests.get(NGROK_API, timeout=5).json().get("tunnels", [])
    except (requests.RequestException, ValueError) as exc:
        raise QrError(
            f"ngrok's local API isn't answering at {NGROK_API} — is `ngrok http 7790` running?"
        ) from exc
    for tunnel in tunnels:
        url = tunnel.get("public_url", "")
        if url.startswith("https://"):
            return url
    raise QrError("ngrok is running but has no https tunnel")


def qr_image(url: str, size: int) -> Image.Image:
    """The QR code alone, as a square image `size` px on a side."""
    try:
        import qrcode

        code = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=2)
        code.add_data(url)
        code.make(fit=True)
        img = code.make_image(fill_color="black", back_color="white").convert("L")
    except ImportError:
        if shutil.which("qrencode"):
            with tempfile.NamedTemporaryFile(suffix=".png") as tmp:
                subprocess.run(["qrencode", "-o", tmp.name, "-m", "2", "-s", "10", url], check=True)
                img = Image.open(tmp.name).convert("L")
        elif os.environ.get("FRANK_QR_REEXEC"):
            raise QrError("qrcode package unavailable even via `uv run --with qrcode`") from None
        else:
            # Not in the venv: re-run this script through uv with the package added. uv caches it,
            # so this costs a moment once and nothing after.
            env = {**os.environ, "FRANK_QR_REEXEC": "1"}
            cmd = [
                "uv",
                "run",
                "--with",
                "qrcode",
                "python",
                os.path.abspath(__file__),
                *sys.argv[1:],
            ]
            raise SystemExit(subprocess.run(cmd, env=env, cwd=REPO_ROOT).returncode)
    return img.resize((size, size), Image.NEAREST)


def _font(size: int) -> ImageFont.ImageFont:
    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _centered(draw: ImageDraw.ImageDraw, text: str, font, y: int, width: int) -> None:
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    draw.text(((width - (right - left)) // 2 - left, y - top), text, font=font, fill="black")


def make_page(url: str, path: str = "qr.png") -> str:
    """Full-page sticker: headline, big QR, the URL underneath. Returns the path."""
    page = Image.new("L", PAGE, "white")
    draw = ImageDraw.Draw(page)
    qr_size = int(PAGE[0] * 0.72)
    code = qr_image(url, qr_size)

    _centered(draw, HEADLINE, _font(190), int(PAGE[1] * 0.13), PAGE[0])
    page.paste(code, ((PAGE[0] - qr_size) // 2, int(PAGE[1] * 0.27)))
    _centered(draw, url, _font(80), int(PAGE[1] * 0.27) + qr_size + 120, PAGE[0])

    page.save(path, dpi=(300, 300))
    return path


def send_to_printer(path: str) -> None:
    """Hand the PNG to the default printer via `lp`."""
    if not shutil.which("lp"):
        raise QrError("`lp` isn't installed, so there's nothing to print with")
    subprocess.run(["lp", path], check=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("url", nargs="?", help="the public URL to encode")
    parser.add_argument("--ngrok", action="store_true", help="read the URL from ngrok's local API")
    parser.add_argument(
        "--out",
        default=str(HERE.parent / "qr.png"),
        help="where to write the page (default: dimos/experimental/frank/qr.png)",
    )
    parser.add_argument(
        "--print", dest="do_print", action="store_true", help="also send it to the default printer"
    )
    args = parser.parse_args(argv)

    try:
        url = ngrok_url() if args.ngrok else args.url
        if not url:
            raise QrError("give a URL, or --ngrok to read one from the running tunnel")
        path = make_page(url, args.out)
        print(path)
        if args.do_print:
            send_to_printer(path)
            print(f"sent {path} to the default printer")
    except (QrError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
