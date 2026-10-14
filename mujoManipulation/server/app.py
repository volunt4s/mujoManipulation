"""FastAPI app: three GETs and a socket.

    GET  /            the page
    GET  /scene       the static description — geoms, materials, mesh table
    GET  /scene.bin   the geometry, once (3.12 MB, 1.72 MB gzipped)
    WS   /ws          one state frame per screen refresh

The socket is a one-way push for now; `receive` is where W3's commands land.
The handler never touches the simulation directly — it reads the latest
published frame and sends it, so a client that stalls or disconnects cannot
affect the physics.
"""

import asyncio
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware

from .loop import SimulationThread

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(os.path.dirname(_HERE), "web")
DEFAULT_XML = os.path.join(
    os.path.dirname(_HERE), "assets", "panda", "franka_panda.xml"
)


def create_app(xml_path: str = DEFAULT_XML, stream_hz: float = 60.0) -> FastAPI:
    sim = SimulationThread(xml_path, stream_hz=stream_hz)

    async def lifespan(app: FastAPI):
        sim.start()
        yield
        sim.stop()

    app = FastAPI(title="mujoManipulation", lifespan=lifespan)
    # The geometry blob is float data, so gzip only takes it 3.12 -> 1.72 MB.
    # Worth having, not worth tuning.
    app.add_middleware(GZipMiddleware, minimum_size=1024)
    app.state.sim = sim

    @app.get("/")
    async def index():
        return FileResponse(os.path.join(WEB_DIR, "index.html"))

    @app.get("/scene")
    async def scene():
        return JSONResponse(sim.scene.description())

    @app.get("/scene.bin")
    async def scene_blob():
        return Response(
            sim.scene.blob(),
            media_type="application/octet-stream",
            headers={"Cache-Control": "no-cache"},
        )

    @app.websocket("/ws")
    async def stream(ws: WebSocket):
        await ws.accept()
        period = 1.0 / sim.stream_hz
        # Sleep until the next deadline rather than for a fixed period after
        # the send: serialising a frame takes a few milliseconds, and sleeping
        # a whole period on top of that costs about 15% of the frame rate.
        deadline = asyncio.get_running_loop().time()
        try:
            while True:
                frame = sim.frame()
                if frame is not None:
                    await ws.send_json(frame)
                deadline += period
                now = asyncio.get_running_loop().time()
                if deadline < now:
                    deadline = now          # fell behind; do not burst
                await asyncio.sleep(deadline - now)
        except WebSocketDisconnect:
            pass

    app.mount("/web", StaticFiles(directory=WEB_DIR), name="web")
    return app


app = create_app()
