"""
Usage:
python webbuilder.py
python webbuilder.py aircraft/example
python webbuilder.py aircraft/example --port 8765

Browser front end for plane-first airframes (the same projects as guibuilder.py):
serve the built React app from web/dist, open one aircraft project and run
the shared libs on the server; recompute is the Editor's F5.
"""

from __future__ import annotations

import argparse
import asyncio
import webbrowser
from pathlib import Path

from aiohttp import web

from libs.project import PROJECTS_DIR, open_project
from webapp.server import close_websockets, create_app
from webapp.state import AppState

DEFAULT_PROJECT = PROJECTS_DIR / "example"
DEFAULT_PORT = 8765
WEB_DIST = Path(__file__).resolve().parent / "web" / "dist"
SHUTDOWN_TIMEOUT_S = 1.0  # caps how long a non-responsive client can hold up Ctrl+C


async def serve(app: web.Application, host: str, port: int) -> None:
    """Run until cancelled (Ctrl+C); cleanup closes the event sockets and releases the port before returning."""
    runner = web.AppRunner(app, shutdown_timeout=SHUTDOWN_TIMEOUT_S)
    await runner.setup()
    try:
        site = web.TCPSite(runner, host, port)
        await site.start()
        await asyncio.Event().wait()
    finally:
        await close_websockets(app)  # never wait on a connected browser
        await runner.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plane-first airframe builder, browser front end")
    parser.add_argument("project", type=Path, nargs="?", default=DEFAULT_PROJECT, help="aircraft project directory (aircraft/<name>)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="port on 127.0.0.1")
    parser.add_argument("--no-browser", action="store_true", help="do not open the default browser")
    args = parser.parse_args()

    if not (WEB_DIST / "index.html").is_file():
        raise SystemExit(f"web frontend not built: {WEB_DIST} is missing. Run 'npm install && npm run build' in web/.")

    project = open_project(args.project)
    state = AppState(project)
    state.recompute(run_aero=True)
    app = create_app(state, WEB_DIST)

    url = f"http://127.0.0.1:{args.port}/"
    print(f"quadrocketbuilder browser front end: {url}  (project {project.name})")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        asyncio.run(serve(app, "127.0.0.1", args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
