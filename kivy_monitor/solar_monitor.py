#!/usr/bin/env python3
"""
solar_monitor.py — Interfaz Kivy para el seguidor solar

Muestra en tiempo real:
  · Feed de cámara con máscara de umbral y centroide del sol superpuestos
  · Actuación calculada del controlador P para cada eje (theta / phi)
  · Consola serie para enviar comandos esp_console al ESP32

La fuente de vídeo puede ser:
  - Stream MJPEG del ESP32  →  http://<IP>:81/stream  (recomendado)
  - Webcam local            →  índice 0 (fallback o prueba sin hardware)

La conexión al ESP32 por puerto serie es opcional: la interfaz funciona
sin hardware conectado.

Dependencias:  pip install kivy opencv-python numpy pyserial
"""

import threading
import queue
import time
import json
import sqlite3
import os
import urllib.request
from urllib.parse import urlparse

import cv2
import numpy as np

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.uix.progressbar import ProgressBar
from kivy.uix.scrollview import ScrollView
from kivy.uix.spinner import Spinner
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock
from kivy.core.window import Window

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    from prueba_prediccion_hora_solar import convertir as solar_convertir
    HAS_SOLAR = True
except ImportError:
    HAS_SOLAR = False

# ════════════════════════════════════════════════════════════════════
# Parámetros del algoritmo  — idénticos a config.h
# ════════════════════════════════════════════════════════════════════

VIS_THRESHOLD_DEF = 200
VIS_MIN_AREA      = 20
FRAME_W           = 320
FRAME_H           = 240
MAX_CONSOLE_LINES = 400


# ════════════════════════════════════════════════════════════════════
# Persistencia de ajustes entre sesiones (SQLite clave-valor)
# ════════════════════════════════════════════════════════════════════

class Settings:
    _DB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'solar_monitor.db')

    def __init__(self):
        self._db = sqlite3.connect(self._DB)
        self._db.execute(
            'CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT)')
        self._db.commit()

    def get(self, key, default=''):
        row = self._db.execute(
            'SELECT value FROM kv WHERE key=?', (key,)).fetchone()
        return row[0] if row else default

    def set(self, key, value):
        self._db.execute(
            'INSERT OR REPLACE INTO kv VALUES (?,?)', (key, str(value)))
        self._db.commit()

    def close(self):
        self._db.close()


# ════════════════════════════════════════════════════════════════════
# Algoritmo de centroide  — traducción directa de vis_capture() en C
# ════════════════════════════════════════════════════════════════════

def compute_centroid(gray: np.ndarray, thr: int):
    """
    Equivale a vis_capture() en vision.c.
    Devuelve (dx, dy, area) o None si no se detecta el sol.
    """
    ys, xs = np.where(gray >= thr)
    n = len(xs)
    if n < VIS_MIN_AREA:
        return None
    cx = int(xs.sum() / n)
    cy = int(ys.sum() / n)
    return cx - FRAME_W // 2, cy - FRAME_H // 2, n


# ════════════════════════════════════════════════════════════════════
# Hilo de captura de vídeo (webcam local o stream MJPEG del ESP32)
# ════════════════════════════════════════════════════════════════════

class CameraThread(threading.Thread):
    """
    Captura frames de la fuente de vídeo activa:
      - source=0                       → webcam local (índice OpenCV)
      - source='http://…/snapshot'     → polling JPEG ESP32 (recomendado)
      - source='http://…/stream'       → stream MJPEG continuo

    Con /snapshot se hace un GET por frame (cada segundo), dejando el
    httpd del ESP32-CAM libre entre peticiones para que el Motor pueda
    también acceder.  Con /stream se abre una conexión persistente
    (puede interferir si el httpd usa una sola tarea).
    """

    def __init__(self, frame_q: queue.Queue, thr_ref: list,
                 source=0, status_q: queue.Queue = None):
        super().__init__(daemon=True)
        self.frame_q  = frame_q
        self.thr_ref  = thr_ref
        self.source   = source
        self.status_q = status_q
        self._stop_ev = threading.Event()

    def _notify(self, msg: str):
        if self.status_q and not self.status_q.full():
            self.status_q.put_nowait(msg)

    def _run_snapshot(self):
        """Polling de /snapshot: un GET por frame, sin conexión persistente."""
        req = urllib.request.Request(
            self.source, headers={'Connection': 'close'})
        fails = 0
        while not self._stop_ev.is_set():
            try:
                with urllib.request.urlopen(req, timeout=2.0) as r:
                    data = r.read()
                img = cv2.imdecode(np.frombuffer(data, np.uint8),
                                   cv2.IMREAD_GRAYSCALE)
                if img is None:
                    raise ValueError('JPEG inválido')
                img    = cv2.resize(img, (FRAME_W, FRAME_H))
                result = compute_centroid(img, self.thr_ref[0])
                if not self.frame_q.full():
                    self.frame_q.put_nowait((img, result))
                if fails > 0:
                    self._notify('ok')
                fails = 0
            except Exception:
                fails += 1
                if fails == 1:
                    self._notify('sin señal')
                elif fails == 2:
                    self._notify('reconectando…')
            self._stop_ev.wait(timeout=1.0)

    def _run_stream(self):
        """Stream MJPEG continuo vía OpenCV VideoCapture."""
        while not self._stop_ev.is_set():
            cap = cv2.VideoCapture()
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 3000)
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000)
            cap.open(self.source)

            if not cap.isOpened():
                cap.release()
                self._notify('sin señal')
                if self._stop_ev.wait(timeout=2.0):
                    break
                self._notify('reconectando…')
                continue

            self._notify('ok')
            consecutive_fails = 0

            while not self._stop_ev.is_set():
                ret, frame = cap.read()
                if not ret:
                    consecutive_fails += 1
                    if consecutive_fails == 1:
                        self._notify('sin señal')
                    if consecutive_fails > 30:
                        break
                    if self._stop_ev.wait(timeout=0.033):
                        break
                    continue
                if consecutive_fails > 0:
                    self._notify('ok')
                consecutive_fails = 0
                gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray   = cv2.resize(gray, (FRAME_W, FRAME_H))
                result = compute_centroid(gray, self.thr_ref[0])
                if not self.frame_q.full():
                    self.frame_q.put_nowait((gray, result))

            cap.release()
            if not self._stop_ev.is_set():
                self._notify('reconectando…')
                if self._stop_ev.wait(timeout=2.0):
                    break

    def _run_webcam(self):
        while not self._stop_ev.is_set():
            cap = cv2.VideoCapture()
            cap.open(self.source)
            if not cap.isOpened():
                cap.release()
                self._notify('sin señal')
                if self._stop_ev.wait(timeout=1.0):
                    break
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            self._notify('webcam')
            while not self._stop_ev.is_set():
                ret, frame = cap.read()
                if not ret:
                    if self._stop_ev.wait(timeout=0.033):
                        break
                    continue
                gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray   = cv2.resize(gray, (FRAME_W, FRAME_H))
                result = compute_centroid(gray, self.thr_ref[0])
                if not self.frame_q.full():
                    self.frame_q.put_nowait((gray, result))
            cap.release()

    def run(self):
        if isinstance(self.source, str):
            if 'snapshot' in self.source:
                self._run_snapshot()
            else:
                self._run_stream()
        else:
            self._run_webcam()

    def stop(self):
        self._stop_ev.set()


# ════════════════════════════════════════════════════════════════════
# Hilo de polling del centroide JSON (proceso_cam_v1 → GET /centroid)
# ════════════════════════════════════════════════════════════════════

class CentroidThread(threading.Thread):
    """
    Consulta http://host/centroid cada segundo y deposita el JSON en
    result_q.  El ESP32 (proceso_cam_v1) actualiza el centroide a 1 FPS;
    este hilo mantiene la cola con el dato más reciente.

    JSON esperado:
      {dx, dy, cx, cy, area, valid, frame_w, frame_h}
    donde dx/dy son el desplazamiento del centroide respecto al centro
    del frame en píxeles del frame original (p.ej. 160×120 para QQVGA).
    """

    def __init__(self, host: str, result_q: queue.Queue):
        super().__init__(daemon=True)
        self.host     = host
        self.result_q = result_q
        self._stop_ev = threading.Event()

    def run(self):
        url = f'http://{self.host}/status'
        req = urllib.request.Request(url, headers={'Connection': 'close'})
        while not self._stop_ev.is_set():
            try:
                with urllib.request.urlopen(req, timeout=2.0) as r:
                    data = json.loads(r.read())
                while not self.result_q.empty():
                    try:
                        self.result_q.get_nowait()
                    except queue.Empty:
                        break
                self.result_q.put_nowait(data)
            except Exception:
                pass
            self._stop_ev.wait(timeout=5.0)

    def stop(self):
        self._stop_ev.set()


# ════════════════════════════════════════════════════════════════════
# Hilo de comandos HTTP al ESP32-Motor (POST /cmd, GET /status)
# ════════════════════════════════════════════════════════════════════

class MotorHttpThread(threading.Thread):
    """
    Envía comandos al ESP32-Motor vía HTTP y recibe respuestas.
      - cmd_q  → cola de entrada: strings de comando (p.ej. 'ctrl pause')
      - resp_q → cola de salida:  strings de respuesta para la consola
                 Los mensajes de estado llevan prefijo '__status__' y
                 contienen JSON {theta, phi, paused} para los widgets.
    """

    def __init__(self, host: str, cmd_q: queue.Queue, resp_q: queue.Queue):
        super().__init__(daemon=True)
        self.host     = host
        self.cmd_q    = cmd_q
        self.resp_q   = resp_q
        self._stop_ev = threading.Event()

    def _post_cmd(self, cmd: str) -> str:
        url  = f'http://{self.host}/cmd'
        data = cmd.encode()
        req  = urllib.request.Request(url, data=data, method='POST')
        req.add_header('Content-Type', 'text/plain')
        try:
            with urllib.request.urlopen(req, timeout=3.0) as r:
                return r.read().decode(errors='replace').strip()
        except Exception as e:
            return f'[error HTTP] {e}'

    def run(self):
        status_url = f'http://{self.host}/status'
        while not self._stop_ev.is_set():
            # Enviar todos los comandos pendientes antes del poll de estado
            while True:
                try:
                    cmd = self.cmd_q.get_nowait()
                except queue.Empty:
                    break
                resp = self._post_cmd(cmd)
                if not self.resp_q.full():
                    self.resp_q.put_nowait(f'[motor] {resp}')

            # Polling de estado del Motor
            try:
                with urllib.request.urlopen(status_url, timeout=2.0) as r:
                    data = json.loads(r.read())
                if not self.resp_q.full():
                    self.resp_q.put_nowait(f'__status__{json.dumps(data)}')
            except Exception:
                pass

            self._stop_ev.wait(timeout=1.0)

    def stop(self):
        self._stop_ev.set()


# ════════════════════════════════════════════════════════════════════
# Hilo de comunicación serie con el ESP32
# ════════════════════════════════════════════════════════════════════

class SerialThread(threading.Thread):
    """
    Lee líneas del ESP32 (esp_console) y las pone en out_q.
    Escribe comandos tomados de in_q al ESP32.
    """

    def __init__(self, port: str, out_q: queue.Queue, in_q: queue.Queue):
        super().__init__(daemon=True)
        self.port  = port
        self.out_q = out_q
        self.in_q  = in_q
        self._stop_ev = threading.Event()

    def run(self):
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.1)
        except Exception as e:
            self.out_q.put_nowait(f'[Error] No se pudo abrir {self.port}: {e}')
            return

        while not self._stop_ev.is_set():
            try:
                if ser.in_waiting:
                    line = ser.readline().decode(errors='replace').rstrip()
                    if line and not self.out_q.full():
                        self.out_q.put_nowait(line)
            except Exception:
                break

            try:
                cmd = self.in_q.get_nowait()
                ser.write((cmd + '\n').encode())
            except queue.Empty:
                pass

            time.sleep(0.01)

        ser.close()

    def stop(self):
        self._stop_ev.set()


# ════════════════════════════════════════════════════════════════════
# Widget: visualización de un actuador lineal
# ════════════════════════════════════════════════════════════════════

class ActuatorWidget(BoxLayout):
    """Muestra la posición absoluta del actuador en mm."""

    def __init__(self, title: str, **kw):
        super().__init__(orientation='vertical', padding=[8, 4], spacing=3, **kw)

        with self.canvas.before:
            Color(0.16, 0.16, 0.2, 1)
            self._bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self._sync_bg, size=self._sync_bg)

        self.add_widget(Label(
            text=title, font_size=13, bold=True,
            size_hint_y=None, height=22, color=(0.85, 0.85, 0.85, 1)))

        self.pos_bar = ProgressBar(max=300, size_hint_y=None, height=14)
        self.add_widget(self.pos_bar)

        self.pos_lbl = Label(
            text='sin datos', font_size=12,
            size_hint_y=None, height=20, color=(0.4, 1, 0.85, 1))
        self.add_widget(self.pos_lbl)

    def update_pos(self, mm: float):
        if mm < 0:
            self.pos_bar.value = 0
            self.pos_lbl.text = 'sin datos'
        else:
            self.pos_bar.value = mm
            self.pos_lbl.text = f'{mm:.1f} mm  ({mm / 300 * 100:.0f}%)'

    def _sync_bg(self, *_):
        self._bg.pos  = self.pos
        self._bg.size = self.size


# ════════════════════════════════════════════════════════════════════
# Aplicación Kivy
# ════════════════════════════════════════════════════════════════════

class SolarMonitorApp(App):

    def build(self):
        Window.size = (980, 700)
        Window.clearcolor = (0.11, 0.11, 0.13, 1)
        self.title = 'Solar Tracker Monitor'

        # Estado compartido entre hilos
        self._thr_ref      = [VIS_THRESHOLD_DEF]
        self._cam_q        = queue.Queue(maxsize=2)
        self._cam_status_q = queue.Queue(maxsize=4)
        self._serial_out_q = queue.Queue(maxsize=100)
        self._serial_in_q  = queue.Queue(maxsize=20)
        self._serial_th    = None

        # Centroide del ESP32-CAM (proceso_cam_v1 → /status)
        self._centroid_q    = queue.Queue(maxsize=4)
        self._centroid_th   = None
        self._esp_centroid  = None
        self._esp_cent_time = 0.0

        # Comandos y estado del ESP32-Motor (bucle_control_v2 → /cmd, /status)
        self._motor_cmd_q   = queue.Queue(maxsize=10)
        self._motor_resp_q  = queue.Queue(maxsize=20)
        self._motor_th      = None
        self._motor_status  = None   # último {theta, phi, paused} del Motor

        # Arrancar con webcam local; el usuario puede cambiar a ESP32
        self._cam_th = CameraThread(
            self._cam_q, self._thr_ref,
            source=0, status_q=self._cam_status_q)
        self._cam_th.start()

        # ── Raíz ─────────────────────────────────────────────────
        root = BoxLayout(orientation='vertical', spacing=6, padding=6)

        root.add_widget(Label(
            text='Solar Tracker Monitor', font_size=16, bold=True,
            size_hint_y=None, height=28, color=(0.5, 0.85, 1, 1)))

        # ── Fila central: cámara + actuadores ────────────────────
        mid = BoxLayout(orientation='horizontal', spacing=8)
        root.add_widget(mid)

        # Panel de cámara
        cam_col = BoxLayout(orientation='vertical', spacing=4,
                            size_hint_x=0.57)
        mid.add_widget(cam_col)

        self.cam_img = Image(allow_stretch=True, keep_ratio=True)
        cam_col.add_widget(self.cam_img)

        # Métricas dx / dy / area
        info = BoxLayout(orientation='horizontal',
                         size_hint_y=None, height=26)
        self.lbl_dx   = Label(text='dx: —',   font_size=13,
                               color=(0.6, 1, 0.6, 1))
        self.lbl_dy   = Label(text='dy: —',   font_size=13,
                               color=(0.6, 1, 0.6, 1))
        self.lbl_area = Label(text='área: —', font_size=13,
                               color=(0.6, 1, 0.6, 1))
        self.lbl_src  = Label(text='[py]', font_size=11,
                               size_hint_x=0.4, color=(0.55, 0.55, 0.55, 1))
        for w in (self.lbl_dx, self.lbl_dy, self.lbl_area, self.lbl_src):
            info.add_widget(w)
        cam_col.add_widget(info)

        # Slider de umbral
        thr_row = BoxLayout(orientation='horizontal',
                            size_hint_y=None, height=32, spacing=6)
        thr_row.add_widget(Label(text='Umbral', size_hint_x=0.22,
                                  font_size=12, color=(0.8, 0.8, 0.8, 1)))
        self.thr_slider = Slider(min=50, max=255,
                                  value=VIS_THRESHOLD_DEF,
                                  size_hint_x=0.58)
        self.thr_slider.bind(value=self._on_thr)
        thr_row.add_widget(self.thr_slider)
        self.thr_lbl = Label(text=str(VIS_THRESHOLD_DEF),
                              size_hint_x=0.2, font_size=12,
                              color=(1, 0.75, 0.3, 1))
        thr_row.add_widget(self.thr_lbl)
        cam_col.add_widget(thr_row)

        # ── Panel de fuente de vídeo (ESP32 / webcam) ─────────────
        cam_col.add_widget(self._build_stream_panel())

        # Panel derecho: actuadores + conexión serie
        right = BoxLayout(orientation='vertical', spacing=6,
                          size_hint_x=0.43)
        mid.add_widget(right)

        self.theta_w = ActuatorWidget('THETA  (elevación)')
        self.phi_w   = ActuatorWidget('PHI    (acimut)')
        right.add_widget(self.theta_w)
        right.add_widget(self.phi_w)

        right.add_widget(self._build_ctrl_panel())
        right.add_widget(self._build_conn_panel())

        # ── Panel consola ─────────────────────────────────────────
        root.add_widget(self._build_console())

        # Restaurar estado de la sesión anterior
        self._settings = Settings()
        saved_url = self._settings.get('stream_url')
        if saved_url:
            self.url_input.text = saved_url
        saved_thr = self._settings.get('threshold')
        if saved_thr:
            v = int(saved_thr)
            self._thr_ref[0] = v
            self.thr_slider.value = v
            self.thr_lbl.text = str(v)
        saved_port = self._settings.get('serial_port')
        if saved_port and saved_port in self.port_spin.values:
            self.port_spin.text = saved_port
        saved_motor_url = self._settings.get('motor_url')
        if saved_motor_url:
            self.motor_url_input.text = saved_motor_url

        Clock.schedule_interval(self._update, 1 / 30)
        return root

    # ── Subpanel: fuente de vídeo ─────────────────────────────────

    def _build_stream_panel(self):
        box = BoxLayout(orientation='vertical', spacing=4, padding=6,
                        size_hint_y=None, height=100)
        with box.canvas.before:
            Color(0.14, 0.17, 0.22, 1)
            bg = Rectangle(pos=box.pos, size=box.size)
        box.bind(pos=lambda *_: setattr(bg, 'pos', box.pos),
                 size=lambda *_: setattr(bg, 'size', box.size))

        # Fila 1: label + estado de conexión de cámara
        hdr = BoxLayout(orientation='horizontal',
                        size_hint_y=None, height=22)
        hdr.add_widget(Label(
            text='Fuente de vídeo', font_size=12, bold=True,
            size_hint_x=0.5, color=(0.7, 0.85, 1, 1)))
        self.cam_status_lbl = Label(
            text='● webcam', font_size=12,
            size_hint_x=0.5, color=(0.4, 1, 0.4, 1))
        hdr.add_widget(self.cam_status_lbl)
        box.add_widget(hdr)

        # Fila 2: URL completa
        url_row = BoxLayout(orientation='horizontal',
                            size_hint_y=None, height=30, spacing=4)
        url_row.add_widget(Label(text='URL:', size_hint_x=0.12,
                                  font_size=12, color=(0.8, 0.8, 0.8, 1)))
        self.url_input = TextInput(
            hint_text='http://192.168.4.1/snapshot', multiline=False,
            font_size=12, size_hint_x=0.88,
            background_color=(0.1, 0.1, 0.12, 1),
            foreground_color=(1, 1, 1, 1))
        self.url_input.bind(on_text_validate=self._connect_esp32_stream)
        url_row.add_widget(self.url_input)
        box.add_widget(url_row)

        # Fila 3: botones de fuente
        btn_row = BoxLayout(orientation='horizontal',
                            size_hint_y=None, height=30, spacing=4)
        esp_btn = Button(
            text='Conectar ESP32', font_size=12,
            background_color=(0.18, 0.42, 0.55, 1))
        esp_btn.bind(on_press=self._connect_esp32_stream)
        btn_row.add_widget(esp_btn)

        cam_btn = Button(
            text='Webcam local', font_size=12,
            background_color=(0.25, 0.25, 0.35, 1))
        cam_btn.bind(on_press=self._connect_webcam)
        btn_row.add_widget(cam_btn)
        box.add_widget(btn_row)

        return box

    # ── Panel de control del barrido ─────────────────────────────

    def _build_ctrl_panel(self):
        box = BoxLayout(orientation='vertical', spacing=4, padding=6,
                        size_hint_y=None, height=116)
        with box.canvas.before:
            Color(0.12, 0.18, 0.14, 1)
            bg = Rectangle(pos=box.pos, size=box.size)
        box.bind(pos=lambda *_: setattr(bg, 'pos', box.pos),
                 size=lambda *_: setattr(bg, 'size', box.size))

        self.ctrl_state_lbl = Label(
            text='● EN ESPERA — pulsa Iniciar barrido',
            font_size=12, bold=True,
            size_hint_y=None, height=22,
            color=(1, 0.75, 0.2, 1))
        box.add_widget(self.ctrl_state_lbl)

        btn_row = BoxLayout(orientation='horizontal',
                            size_hint_y=None, height=32, spacing=4)
        self.iniciar_btn = Button(
            text='▶ Iniciar barrido', font_size=12,
            background_color=(0.18, 0.55, 0.28, 1))
        self.iniciar_btn.bind(on_press=lambda _: self._send_cmd('ctrl resume'))
        btn_row.add_widget(self.iniciar_btn)

        pausar_btn = Button(
            text='⏸ Pausar', font_size=12,
            background_color=(0.42, 0.28, 0.10, 1))
        pausar_btn.bind(on_press=lambda _: self._send_cmd('ctrl pause'))
        btn_row.add_widget(pausar_btn)

        rebarrer_btn = Button(
            text='↺ Re-barrer', font_size=12,
            background_color=(0.28, 0.18, 0.42, 1))
        rebarrer_btn.bind(on_press=lambda _: self._send_cmd('scan'))
        btn_row.add_widget(rebarrer_btn)

        box.add_widget(btn_row)

        home_row = BoxLayout(orientation='horizontal',
                             size_hint_y=None, height=32, spacing=4)
        self.home_btn = Button(
            text='⌂ Calibrar (homing)', font_size=12,
            background_color=(0.45, 0.20, 0.10, 1))
        self.home_btn.bind(on_press=self._send_home)
        home_row.add_widget(self.home_btn)
        box.add_widget(home_row)
        return box

    # ── Subpaneles restantes ──────────────────────────────────────

    def _build_conn_panel(self):
        box = BoxLayout(orientation='vertical', spacing=4, padding=6,
                        size_hint_y=None, height=170)
        with box.canvas.before:
            Color(0.17, 0.17, 0.22, 1)
            bg = Rectangle(pos=box.pos, size=box.size)
        box.bind(pos=lambda *_: setattr(bg, 'pos', box.pos),
                 size=lambda *_: setattr(bg, 'size', box.size))

        # ── Motor WiFi ───────────────────────────────────────────────
        box.add_widget(Label(
            text='Motor WiFi', font_size=11, bold=True,
            size_hint_y=None, height=18, color=(0.7, 0.85, 1, 1),
            halign='left', text_size=(None, None)))

        motor_url_row = BoxLayout(orientation='horizontal',
                                  size_hint_y=None, height=28, spacing=4)
        motor_url_row.add_widget(Label(text='URL:', size_hint_x=0.15,
                                        font_size=11, color=(0.8, 0.8, 0.8, 1)))
        self.motor_url_input = TextInput(
            hint_text='http://192.168.4.2', multiline=False,
            font_size=11, size_hint_x=0.85,
            background_color=(0.1, 0.1, 0.12, 1),
            foreground_color=(1, 1, 1, 1))
        self.motor_url_input.bind(on_text_validate=self._connect_motor)
        motor_url_row.add_widget(self.motor_url_input)
        box.add_widget(motor_url_row)

        motor_btn_row = BoxLayout(orientation='horizontal',
                                   size_hint_y=None, height=28, spacing=4)
        self.motor_conn_btn = Button(
            text='Conectar Motor', font_size=11,
            background_color=(0.18, 0.55, 0.28, 1))
        self.motor_conn_btn.bind(on_press=self._toggle_motor)
        motor_btn_row.add_widget(self.motor_conn_btn)
        self.motor_status_lbl = Label(
            text='○ sin Motor', font_size=11,
            size_hint_x=0.5, color=(0.55, 0.55, 0.55, 1))
        motor_btn_row.add_widget(self.motor_status_lbl)
        box.add_widget(motor_btn_row)

        # ── Puerto Serie (debug local) ───────────────────────────────
        box.add_widget(Label(
            text='Puerto Serie', font_size=11, bold=True,
            size_hint_y=None, height=18, color=(0.7, 0.85, 1, 1),
            halign='left', text_size=(None, None)))

        port_row = BoxLayout(orientation='horizontal',
                             size_hint_y=None, height=28, spacing=4)
        ports = self._list_ports()
        self.port_spin = Spinner(
            text=ports[0] if ports else '(sin puertos)',
            values=ports, size_hint_x=0.55, font_size=11)
        self.conn_btn = Button(
            text='Conectar', size_hint_x=0.25, font_size=11,
            background_color=(0.18, 0.55, 0.28, 1))
        self.conn_btn.bind(on_press=self._toggle_serial)
        refresh_btn = Button(
            text='↻', size_hint_x=0.2, font_size=11,
            background_color=(0.2, 0.2, 0.3, 1))
        refresh_btn.bind(on_press=self._refresh_ports)
        port_row.add_widget(self.port_spin)
        port_row.add_widget(self.conn_btn)
        port_row.add_widget(refresh_btn)
        box.add_widget(port_row)

        self.conn_status = Label(
            text='○  Sin conexión', font_size=11,
            size_hint_y=None, height=18, color=(0.55, 0.55, 0.55, 1))
        box.add_widget(self.conn_status)

        return box

    def _build_console(self):
        col = BoxLayout(orientation='vertical', spacing=4,
                        size_hint_y=None, height=222)

        btn_row = BoxLayout(orientation='horizontal',
                            size_hint_y=None, height=32, spacing=4)
        quick = [
            ('pause',   'ctrl pause',  (0.18, 0.28, 0.42, 1)),
            ('resume',  'ctrl resume', (0.18, 0.42, 0.28, 1)),
            ('stop',    'stop',        (0.55, 0.18, 0.18, 1)),
            ('pos',     'pos',         (0.18, 0.28, 0.42, 1)),
            ('scan',    'scan',        (0.35, 0.28, 0.18, 1)),
            ('help',    'help',        (0.22, 0.22, 0.30, 1)),
        ]
        for label, cmd, color in quick:
            b = Button(text=label, font_size=12, background_color=color)
            b.bind(on_press=lambda _, c=cmd: self._send_cmd(c))
            btn_row.add_widget(b)
        col.add_widget(btn_row)

        # Fila 2: comandos adicionales
        btn_row2 = BoxLayout(orientation='horizontal',
                             size_hint_y=None, height=28, spacing=4)
        setpos_btn = Button(text='setpos theta', font_size=11,
                            background_color=(0.20, 0.20, 0.30, 1))
        setpos_btn.bind(on_press=lambda _: self._send_cmd(
            f'setpos theta {self._motor_status["theta"]:.1f}'
            if self._motor_status else 'setpos theta 150'))
        btn_row2.add_widget(setpos_btn)
        kin_btn = Button(text='kin theta', font_size=11,
                         background_color=(0.20, 0.20, 0.30, 1))
        kin_btn.bind(on_press=lambda _: self._send_cmd('kin theta 45'))
        btn_row2.add_widget(kin_btn)
        solar_btn = Button(text='☀ solar', font_size=11,
                           background_color=(0.50, 0.35, 0.00, 1))
        solar_btn.bind(on_press=self._send_solar)
        btn_row2.add_widget(solar_btn)
        col.add_widget(btn_row2)

        sv = ScrollView(size_hint_y=None, height=110)
        self.console_out = TextInput(
            text='', readonly=True, font_size=12,
            background_color=(0.07, 0.07, 0.09, 1),
            foreground_color=(0.5, 1, 0.5, 1),
            size_hint_y=None)
        self.console_out.bind(
            minimum_height=self.console_out.setter('height'))
        sv.add_widget(self.console_out)
        col.add_widget(sv)
        self._console_sv = sv

        in_row = BoxLayout(orientation='horizontal',
                           size_hint_y=None, height=32, spacing=4)
        self.cmd_in = TextInput(
            hint_text='Escribe un comando  (Intro para enviar)',
            multiline=False, font_size=12, size_hint_x=0.84,
            background_color=(0.1, 0.1, 0.12, 1),
            foreground_color=(1, 1, 1, 1))
        self.cmd_in.bind(
            on_text_validate=lambda i: self._send_cmd(i.text))
        send_b = Button(
            text='↵', font_size=15, size_hint_x=0.16,
            background_color=(0.18, 0.42, 0.18, 1))
        send_b.bind(on_press=lambda _: self._send_cmd(self.cmd_in.text))
        in_row.add_widget(self.cmd_in)
        in_row.add_widget(send_b)
        col.add_widget(in_row)

        return col

    # ── Callbacks de fuente de vídeo ──────────────────────────────

    def _switch_source(self, source):
        """Para el hilo actual, drena la cola y arranca uno nuevo."""
        self._cam_th.stop()
        self._cam_th.join(timeout=2.0)
        while True:
            try:
                self._cam_q.get_nowait()
            except queue.Empty:
                break
        self._cam_th = CameraThread(
            self._cam_q, self._thr_ref,
            source=source, status_q=self._cam_status_q)
        self._cam_th.start()

    def _connect_esp32_stream(self, *_):
        url = self.url_input.text.strip()
        if not url:
            self._log('[Stream] Introduce la URL del stream')
            return
        self._log(f'[Stream] Conectando a {url}')
        self._settings.set('stream_url', url)

        # Derivar host para el polling de /centroid (puerto 80)
        parsed = urlparse(url)
        host   = parsed.hostname or ''
        if host and host not in ('127.0.0.1', 'localhost'):
            self._start_centroid_thread(host)
        else:
            self._stop_centroid_thread()

        self._switch_source(url)

    def _connect_webcam(self, *_):
        self._log('[Stream] Cambiando a webcam local')
        self._stop_centroid_thread()
        self._switch_source(0)

    def _connect_motor(self, *_):
        url = self.motor_url_input.text.strip() or 'http://192.168.4.2'
        parsed = urlparse(url)
        host   = parsed.netloc or parsed.path  # soporta "192.168.4.2" sin scheme
        if not parsed.scheme:
            url  = f'http://{host}'
            host = host
        self._settings.set('motor_url', url)
        self._disconnect_motor()
        self._motor_th = MotorHttpThread(
            host, self._motor_cmd_q, self._motor_resp_q)
        self._motor_th.start()
        self.motor_conn_btn.text             = 'Desconectar Motor'
        self.motor_conn_btn.background_color = (0.55, 0.18, 0.18, 1)
        self.motor_status_lbl.text           = f'● {host}'
        self.motor_status_lbl.color          = (0.4, 1, 0.4, 1)
        self._log(f'[Motor] Conectado a {url}')

    def _disconnect_motor(self):
        if self._motor_th and self._motor_th.is_alive():
            self._motor_th.stop()
            self._motor_th.join(timeout=2.0)
        self._motor_th     = None
        self._motor_status = None
        if hasattr(self, 'motor_conn_btn'):
            self.motor_conn_btn.text             = 'Conectar Motor'
            self.motor_conn_btn.background_color = (0.18, 0.55, 0.28, 1)
            self.motor_status_lbl.text           = '○ sin Motor'
            self.motor_status_lbl.color          = (0.55, 0.55, 0.55, 1)

    def _toggle_motor(self, *_):
        if self._motor_th and self._motor_th.is_alive():
            self._disconnect_motor()
            self._log('[Motor] Desconectado')
        else:
            self._connect_motor()

    def _start_centroid_thread(self, host: str):
        self._stop_centroid_thread()
        self._centroid_th = CentroidThread(host, self._centroid_q)
        self._centroid_th.start()
        self._log(f'[Status] Polling en http://{host}/status')

    def _stop_centroid_thread(self):
        if self._centroid_th and self._centroid_th.is_alive():
            self._centroid_th.stop()
            self._centroid_th.join(timeout=2.0)
        self._centroid_th  = None
        self._esp_centroid = None
        self._esp_cent_time = 0.0

    # ── Callbacks de UI ───────────────────────────────────────────

    def _on_thr(self, _, val):
        self._thr_ref[0] = int(val)
        self.thr_lbl.text = str(int(val))

    def _list_ports(self):
        if not HAS_SERIAL:
            return ['(instala pyserial)']
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports if ports else ['(sin puertos)']

    def _refresh_ports(self, *_):
        ports = self._list_ports()
        self.port_spin.values = ports
        self.port_spin.text   = ports[0] if ports else '(sin puertos)'

    def _toggle_serial(self, *_):
        if self._serial_th and self._serial_th.is_alive():
            self._serial_th.stop()
            self._serial_th = None
            self.conn_btn.text = 'Conectar'
            self.conn_btn.background_color = (0.18, 0.55, 0.28, 1)
            self.conn_status.text  = '○  Sin conexión'
            self.conn_status.color = (0.55, 0.55, 0.55, 1)
        else:
            port = self.port_spin.text
            self._serial_th = SerialThread(
                port, self._serial_out_q, self._serial_in_q)
            self._serial_th.start()
            self.conn_btn.text = 'Desconectar'
            self.conn_btn.background_color = (0.55, 0.18, 0.18, 1)
            self.conn_status.text  = f'●  {port}'
            self.conn_status.color = (0.4, 1, 0.4, 1)
            self._log(f'Conectado a {port}')
            self._settings.set('serial_port', port)

    def _send_cmd(self, text: str):
        text = text.strip()
        if not text:
            return
        self._log(f'> {text}')
        if self._motor_th and self._motor_th.is_alive():
            # Motor WiFi conectado → HTTP (campo de pruebas)
            try:
                self._motor_cmd_q.put_nowait(text)
            except queue.Full:
                pass
        elif self._serial_th and self._serial_th.is_alive():
            # Fallback: puerto serie USB (debug local)
            try:
                self._serial_in_q.put_nowait(text)
            except queue.Full:
                pass
        else:
            self._log('[!] Sin conexión al Motor (WiFi ni Serie)')
        self.cmd_in.text = ''

    def _send_home(self, *_):
        self.home_btn.disabled = True
        self.home_btn.text = '⟳ Calibrando…'
        self._send_cmd('home')

    def _send_solar(self, *_):
        import datetime as _dt
        if not HAS_SOLAR:
            self._log('[!] prueba_prediccion_hora_solar.py no encontrado')
            return
        r = solar_convertir(_dt.datetime.now())
        elev = r['elevacion_deg']
        acim = r['angulo_acimutal_deg']
        self._log(
            f'[Solar] {r["fecha_hora"]}  θ(elev)={elev:.2f}°  φ(acim)='
            + (f'{acim:.2f}°' if acim is not None else '—')
            + f'  ({r["huso"]})'
        )
        if not r['sol_visible']:
            self._log('[Solar] Sol bajo el horizonte — sin alineación')
            return
        if acim is None:
            self._log('[Solar] Acimut no disponible')
            return
        self._send_cmd(f'solar {elev:.2f} {acim:.2f}')

    def _log(self, line: str):
        self.console_out.text += line + '\n'
        lines = self.console_out.text.split('\n')
        if len(lines) > MAX_CONSOLE_LINES:
            self.console_out.text = '\n'.join(lines[-MAX_CONSOLE_LINES:])
        Clock.schedule_once(
            lambda _: setattr(self._console_sv, 'scroll_y', 0), 0.05)

    # ── Actualización de UI a 30 fps ──────────────────────────────

    def _update(self, _dt):
        # Actualizar etiqueta de estado de la cámara
        while True:
            try:
                status = self._cam_status_q.get_nowait()
            except queue.Empty:
                break
            color_map = {
                'ok':           (0.4, 1, 0.4, 1),
                'webcam':       (0.4, 1, 0.4, 1),
                'conectando':   (1, 0.85, 0.2, 1),
                'reconectando…':(1, 0.6,  0.1, 1),
                'sin señal':    (1, 0.3,  0.3, 1),
            }
            if status == 'webcam':
                self.cam_status_lbl.text = '● webcam local'
            elif status == 'ok':
                self.cam_status_lbl.text = '● ESP32 stream'
            else:
                self.cam_status_lbl.text = f'○ {status}'
            self.cam_status_lbl.color = color_map.get(status, (0.8, 0.8, 0.8, 1))

        # Obtener el dato de centroide más reciente del ESP32
        while True:
            try:
                c = self._centroid_q.get_nowait()
                self._esp_centroid  = c
                self._esp_cent_time = time.monotonic()
            except queue.Empty:
                break
        # Invalidar si lleva más de 5 s sin actualizarse (ESP32 desconectado)
        if time.monotonic() - self._esp_cent_time > 5.0:
            self._esp_centroid = None

        # Procesar respuestas del Motor HTTP
        while True:
            try:
                msg = self._motor_resp_q.get_nowait()
            except queue.Empty:
                break
            if msg.startswith('__status__'):
                try:
                    self._motor_status = json.loads(msg[len('__status__'):])
                except Exception:
                    pass
            else:
                self._log(msg)

        # Volcar salida serie a la consola
        while True:
            try:
                self._log(self._serial_out_q.get_nowait())
            except queue.Empty:
                break

        # Tomar el frame más reciente
        try:
            gray, py_result = self._cam_q.get_nowait()
        except queue.Empty:
            return

        # Actualizar panel de control según estado del Motor
        if self._motor_status is not None:
            paused = self._motor_status.get('paused', True)
            th     = self._motor_status.get('theta', 0.0)
            ph     = self._motor_status.get('phi',   0.0)
            if paused:
                self.ctrl_state_lbl.text  = f'⏸ PAUSADO — θ={th:.1f}mm  φ={ph:.1f}mm'
                self.ctrl_state_lbl.color = (1, 0.75, 0.2, 1)
                self.iniciar_btn.text     = '▶ Iniciar barrido'
                self.iniciar_btn.background_color = (0.18, 0.55, 0.28, 1)
            else:
                self.ctrl_state_lbl.text  = f'● ACTIVO — θ={th:.1f}mm  φ={ph:.1f}mm'
                self.ctrl_state_lbl.color = (0.4, 1, 0.4, 1)
                self.iniciar_btn.text     = '▶ Activo'
                self.iniciar_btn.background_color = (0.12, 0.35, 0.18, 1)
        elif self._motor_th is not None:
            self.ctrl_state_lbl.text  = '○ Conectando al Motor...'
            self.ctrl_state_lbl.color = (0.7, 0.7, 0.7, 1)
        else:
            self.ctrl_state_lbl.text  = '○ Sin Motor — conecta WiFi o Serie'
            self.ctrl_state_lbl.color = (0.5, 0.5, 0.5, 1)

        # Posición del actuador: Motor directo (WiFi) > CAM relayed > sin datos
        if self._motor_status:
            theta_mm = self._motor_status.get('theta', -1.0)
            phi_mm   = self._motor_status.get('phi',   -1.0)
        elif self._esp_centroid:
            theta_mm = self._esp_centroid.get('theta', -1.0)
            phi_mm   = self._esp_centroid.get('phi',   -1.0)
        else:
            theta_mm = phi_mm = -1.0
        self.theta_w.update_pos(theta_mm)
        self.phi_w.update_pos(phi_mm)

        # Reflejar estado paused/homing del Motor en el label del panel
        if self._motor_status is not None:
            homing = self._motor_status.get('homing', False)
            paused = self._motor_status.get('paused', True)
            self.motor_status_lbl.color = (1, 0.75, 0.2, 1) if paused else (0.4, 1, 0.4, 1)
            if homing:
                self.ctrl_state_lbl.text  = '⟳ CALIBRANDO… (~3 min) — no toques los motores'
                self.ctrl_state_lbl.color = (1, 0.55, 0.1, 1)
                self.iniciar_btn.disabled = True
                self.home_btn.disabled    = True
                self.home_btn.text        = '⟳ Calibrando…'
            elif paused:
                self.ctrl_state_lbl.text  = '⏸ EN ESPERA — pulsa Iniciar barrido'
                self.ctrl_state_lbl.color = (1, 0.75, 0.2, 1)
                self.iniciar_btn.disabled         = False
                self.iniciar_btn.text             = '▶ Iniciar barrido'
                self.iniciar_btn.background_color = (0.18, 0.55, 0.28, 1)
                self.home_btn.disabled = False
                self.home_btn.text     = '⌂ Calibrar (homing)'
            else:
                self.ctrl_state_lbl.text  = '● ACTIVO'
                self.ctrl_state_lbl.color = (0.4, 1, 0.4, 1)
                self.iniciar_btn.disabled         = False
                self.iniciar_btn.text             = '▶ En curso'
                self.iniciar_btn.background_color = (0.15, 0.30, 0.18, 1)
                self.home_btn.disabled = False
                self.home_btn.text     = '⌂ Calibrar (homing)'

        # ── Seleccionar fuente de centroide ──────────────────────
        # Preferencia: ESP32 (datos reales del hardware) > Python (local)
        c = self._esp_centroid
        use_esp = c is not None and c.get('valid', False)

        if use_esp:
            fw   = c.get('frame_w', FRAME_W)
            fh   = c.get('frame_h', FRAME_H)
            # Escalar coords del frame original (p.ej. 160×120) al display
            dx   = int(c['dx'] * FRAME_W / fw)
            dy   = int(c['dy'] * FRAME_H / fh)
            area = int(c['area'])
            src  = 'ESP32'
            dot_color = (0, 140, 255)   # naranja
        elif py_result:
            dx, dy, area = py_result
            src  = 'py'
            dot_color = (255, 90, 0)    # azul
        else:
            dx = dy = area = None
            src = '—'
            dot_color = None

        # ── Construir imagen de visualización ────────────────────
        _, mask = cv2.threshold(
            gray, self._thr_ref[0] - 1, 255, cv2.THRESH_BINARY)
        display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cx0, cy0 = FRAME_W // 2, FRAME_H // 2
        cv2.drawMarker(display, (cx0, cy0),
                       (0, 200, 0), cv2.MARKER_CROSS, 20, 1)

        if dx is not None:
            cx, cy = cx0 + dx, cy0 + dy
            cv2.line(display, (cx0, cy0), (cx, cy),
                     dot_color, 1, cv2.LINE_AA)
            cv2.circle(display, (cx, cy), 8, dot_color, 2)
            cv2.drawMarker(display, (cx, cy),
                           dot_color, cv2.MARKER_CROSS, 18, 2)
            cv2.putText(display,
                        f'dx={dx:+d}  dy={dy:+d}  area={area}  [{src}]',
                        (4, FRAME_H - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 220, 220), 1)

            self.lbl_dx.text   = f'dx: {dx:+d} px'
            self.lbl_dy.text   = f'dy: {dy:+d} px'
            self.lbl_area.text = f'área: {area} px'
            if use_esp:
                elapsed = time.monotonic() - self._esp_cent_time
                self.lbl_src.text  = f'[ESP32 {elapsed:.0f}s]'
                self.lbl_src.color = (0.3, 0.8, 1, 1)
            else:
                self.lbl_src.text  = '[py]'
                self.lbl_src.color = (0.6, 0.6, 0.6, 1)
        else:
            self.lbl_dx.text   = 'dx: —'
            self.lbl_dy.text   = 'dy: —'
            self.lbl_area.text = 'sin sol'
            self.lbl_src.text  = f'[{src}]'
            self.lbl_src.color = (0.5, 0.5, 0.5, 1)

        # ── Convertir frame OpenCV → textura Kivy ────────────────
        buf     = cv2.flip(display, 0).tobytes()
        texture = Texture.create(size=(FRAME_W, FRAME_H), colorfmt='bgr')
        texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        self.cam_img.texture = texture

    def on_stop(self):
        self._cam_th.stop()
        self._stop_centroid_thread()
        self._disconnect_motor()
        if self._serial_th:
            self._serial_th.stop()
        self._settings.set('threshold', self._thr_ref[0])
        self._settings.close()


if __name__ == '__main__':
    SolarMonitorApp().run()
