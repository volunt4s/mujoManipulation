"""FastAPI app: three GETs and a socket.

    GET  /            the page
    GET  /scene       the static description — geoms, materials, mesh table
    GET  /scene.bin   the geometry, once (3.12 MB, 1.72 MB gzipped)
    GET  /catalogue   the rungs, their equations and their break-it variants
    WS   /ws          state frames out, commands in

The socket handler never touches the simulation directly. Outbound it reads the
latest published frame, so a client that stalls or disconnects cannot affect the
physics; inbound it drops commands on a queue that the sim thread drains between
steps, so a controller is never swapped mid-update.

There is one simulation, shared by every connected client — move a control in
one browser and every other sees it. That is right for a tool you run on your
own machine and wrong for a public link, where it becomes strangers fighting
over the gains; the split into a simulation per connection belongs with
deployment, and `SimulationThread` is already the unit it would be split on.
"""

import asyncio
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware

from .loop import SimulationThread
from .reference import DURATION_RANGE, GOALS, MOTIONS
from .scenarios import catalogue

_HERE = os.path.dirname(os.path.abspath(__file__))
WEB_DIR = os.path.join(os.path.dirname(_HERE), "web")
DEFAULT_XML = os.path.join(
    os.path.dirname(_HERE), "assets", "panda", "franka_panda.xml"
)


def create_app(xml_path: str = DEFAULT_XML, stream_hz: float = 60.0,
               use_cache: bool = True) -> FastAPI:
    sim = SimulationThread(xml_path, stream_hz=stream_hz, use_cache=use_cache)

    async def lifespan(app: FastAPI):
        sim.start()
        yield
        sim.stop()

    app = FastAPI(title="mujoManipulation", lifespan=lifespan)

    @app.middleware("http")
    async def no_cache(request, call_next):
        # Everything here is served off disk and edited constantly. A cached
        # module is a confusing afternoon, and there is no production traffic
        # to protect.
        response = await call_next(request)
        response.headers["Cache-Control"] = "no-store"
        return response

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

    @app.get("/catalogue")
    async def catalogue_route():
        return JSONResponse({
            "scenarios": catalogue(),
            "goals": GOALS,
            "motions": MOTIONS,
            "duration_range": DURATION_RANGE,
            "active": sim.state(),
        })

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

        async def receive():
            # Commands are validated by the sim thread, which owns the switch
            # positions; this only has to not die on malformed JSON.
            try:
                while True:
                    message = await ws.receive_json()
                    if isinstance(message, dict) and message.get("op"):
                        sim.submit(message)
            except (WebSocketDisconnect, RuntimeError, ValueError):
                pass

        inbound = asyncio.create_task(receive())
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
        finally:
            inbound.cancel()

    app.mount("/web", StaticFiles(directory=WEB_DIR), name="web")
    return app


app = create_app()
