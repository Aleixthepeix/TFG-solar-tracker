#!/usr/bin/env python3
"""
mjpeg_server.py — Servidor MJPEG desde webcam local.

Emula exactamente el stream que sirve el ESP32, permitiendo probar
solar_monitor.py sin hardware.

Uso:
    python mjpeg_server.py              # webcam 0, puerto 81
    python mjpeg_server.py --port 8080
    python mjpeg_server.py --cam 1 --port 8080

URL de acceso desde solar_monitor:
    http://127.0.0.1:81/stream          (mismo PC)
    http://<IP_local>:81/stream         (otro dispositivo en la red)

Dependencias: opencv-python  (ya instalado)
"""

import argparse
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

# ── Estado global compartido entre clientes ───────────────────────────
_frame_lock  = threading.Lock()
_latest_jpeg = b''          # último frame JPEG codificado
_cam_ok      = False        # True cuando la cámara está activa


def capture_loop(cam_index: int, quality: int, fps: int):
    """Captura frames de la webcam en un hilo dedicado."""
    global _latest_jpeg, _cam_ok

    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f'[ERROR] No se pudo abrir la cámara {cam_index}')
        return

    _cam_ok = True
    interval = 1.0 / fps
    print(f'[cam] Cámara {cam_index} abierta — capturando a {fps} fps')

    while True:
        t0 = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            time.sleep(interval)
            continue

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            with _frame_lock:
                _latest_jpeg = buf.tobytes()

        elapsed = time.monotonic() - t0
        wait = interval - elapsed
        if wait > 0:
            time.sleep(wait)

    cap.release()


# ── Handler HTTP ──────────────────────────────────────────────────────

BOUNDARY = 'mjpg'
CTYPE    = f'multipart/x-mixed-replace;boundary={BOUNDARY}'


class MJPEGHandler(BaseHTTPRequestHandler):

    def log_message(self, fmt, *args):
        pass   # silenciar logs por request para no saturar la consola

    def do_GET(self):
        if self.path != '/stream':
            self.send_error(404, 'Solo existe el endpoint /stream')
            return

        if not _cam_ok:
            self.send_error(503, 'Cámara no disponible')
            return

        self.send_response(200)
        self.send_header('Content-Type', CTYPE)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()

        client = self.client_address
        print(f'[stream] Cliente conectado: {client[0]}:{client[1]}')

        try:
            while True:
                with _frame_lock:
                    jpg = _latest_jpeg

                if not jpg:
                    time.sleep(0.033)
                    continue

                part = (
                    f'\r\n--{BOUNDARY}\r\n'
                    f'Content-Type: image/jpeg\r\n'
                    f'Content-Length: {len(jpg)}\r\n'
                    f'\r\n'
                ).encode()

                self.wfile.write(part + jpg)
                self.wfile.flush()

        except (BrokenPipeError, ConnectionResetError):
            pass   # cliente desconectado — salir limpiamente
        finally:
            print(f'[stream] Cliente desconectado: {client[0]}:{client[1]}')


# ── Punto de entrada ──────────────────────────────────────────────────

def local_ip() -> str:
    """Devuelve la IP local en la red (no 127.0.0.1)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser(description='Servidor MJPEG desde webcam')
    parser.add_argument('--cam',     type=int, default=0,  help='Índice de cámara (default: 0)')
    parser.add_argument('--port',    type=int, default=81, help='Puerto HTTP (default: 81)')
    parser.add_argument('--quality', type=int, default=80, help='Calidad JPEG 0-100 (default: 80)')
    parser.add_argument('--fps',     type=int, default=30, help='FPS de captura (default: 30)')
    args = parser.parse_args()

    # Arrancar hilo de captura
    t = threading.Thread(target=capture_loop,
                         args=(args.cam, args.quality, args.fps),
                         daemon=True)
    t.start()
    time.sleep(0.5)   # esperar a que la cámara abra

    # Arrancar servidor HTTP
    server = ThreadingHTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    ip = local_ip()
    print(f'\nServidor MJPEG activo:')
    print(f'  http://127.0.0.1:{args.port}/stream       (este PC)')
    print(f'  http://{ip}:{args.port}/stream   (red local)')
    print(f'\nCtrl+C para detener.\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nServidor detenido.')


if __name__ == '__main__':
    main()
