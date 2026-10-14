"""python -m mujoManipulation.server  — then open http://127.0.0.1:8000

No mjpython needed: the server never opens a passive viewer, so on macOS this
is the one way to watch the arm from a plain `python`.
"""

import argparse

import uvicorn

from .app import DEFAULT_XML, create_app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--xml", default=DEFAULT_XML, help="model to load")
    parser.add_argument("--stream-hz", type=float, default=60.0)
    args = parser.parse_args()

    print(f"model  : {args.xml}")
    print(f"open   : http://{args.host}:{args.port}")
    uvicorn.run(
        create_app(args.xml, args.stream_hz),
        host=args.host,
        port=args.port,
        log_level="warning",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
