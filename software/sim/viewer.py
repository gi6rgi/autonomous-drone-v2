"""3D-просмотрщик симуляции в браузере: HTTP отдаёт страницу, WebSocket шлёт позу дрона, маркер и кадр камеры.

Страница: http://localhost:8766  (three.js из CDN). Клик по земле переносит маркер, клавиша L — команда на посадку.
"""
import asyncio
import base64
import http.server
import json
import threading
from pathlib import Path

import cv2
import websockets

HTML = Path(__file__).parent / "viewer.html"


class Viewer:
    def __init__(self, http_port: int = 8766, ws_port: int = 8765):
        self.http_port, self.ws_port = http_port, ws_port
        self._clients: set = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self.commands: list = []                   # входящие команды со страницы: {"marker": [n, e]} / {"land": true}
        self._lock = threading.Lock()

    # ------------------------------------------------------------ запуск в фоне
    def start(self):
        threading.Thread(target=self._http, daemon=True).start()
        threading.Thread(target=self._ws_thread, daemon=True).start()

    def _http(self):
        html = HTML.read_bytes()

        class H(http.server.BaseHTTPRequestHandler):
            def do_GET(self_inner):
                self_inner.send_response(200)
                self_inner.send_header("Content-Type", "text/html; charset=utf-8")
                self_inner.end_headers()
                self_inner.wfile.write(html)

            def log_message(self_inner, *a):
                pass

        http.server.ThreadingHTTPServer(("127.0.0.1", self.http_port), H).serve_forever()

    def _ws_thread(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_main())

    async def _ws_main(self):
        async def handler(ws):
            self._clients.add(ws)
            try:
                async for msg in ws:
                    try:
                        with self._lock:
                            self.commands.append(json.loads(msg))
                    except Exception:
                        pass
            finally:
                self._clients.discard(ws)

        async with websockets.serve(handler, "127.0.0.1", self.ws_port, max_size=8_000_000):
            await asyncio.Future()

    # ------------------------------------------------------------ отправка
    def send(self, payload: dict):
        if not self._loop or not self._clients:
            return
        data = json.dumps(payload)

        async def _go():
            dead = []
            for c in list(self._clients):
                try:
                    await c.send(data)
                except Exception:
                    dead.append(c)
            for c in dead:
                self._clients.discard(c)

        asyncio.run_coroutine_threadsafe(_go(), self._loop)

    def send_frame(self, frame_bgr, quality: int = 70):
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            self.send({"frame": base64.b64encode(buf.tobytes()).decode()})

    def pop_commands(self) -> list:
        with self._lock:
            cmds, self.commands = self.commands, []
        return cmds
