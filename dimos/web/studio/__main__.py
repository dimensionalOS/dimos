"""Run DimOS Studio as a local application."""

import argparse
from threading import Timer
import webbrowser

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local DimOS Studio workbench")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--no-open", action="store_true")
    args = parser.parse_args()

    url = f"http://{args.host}:{args.port}"
    if not args.no_open:
        Timer(1.0, lambda: webbrowser.open(url)).start()
    uvicorn.run("dimos.web.studio.app:app", host=args.host, port=args.port, reload=False)


if __name__ == "__main__":
    main()
