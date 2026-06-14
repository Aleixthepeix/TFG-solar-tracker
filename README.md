# TFG — Seguidor Solar

Sistema de seguimiento solar de dos ejes basado en visión artificial. Detecta el centroide del sol en tiempo real y orienta un panel mediante dos actuadores lineales controlados con cinemática inversa.

## Arquitectura

```
┌─────────────────┐        WiFi (HTTP)       ┌─────────────────────┐
│   ESP32-CAM     │ ◄──────────────────────► │     ESP32-Motor     │
│ proceso_cam_v1  │  GET /centroid           │  bucle_control_v2   │
│                 │  POST /position          │                     │
│ · Captura QQVGA │                          │ · 2× actuador lineal│
│ · Umbral + cent.│                          │ · Homing + barrido  │
│ · MJPEG stream  │                          │ · Cinemática inversa│
└────────┬────────┘                          └──────────┬──────────┘
         │ MJPEG / snapshot                             │ HTTP /status
         │                                             │ HTTP /cmd
         ▼                                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                    kivy_monitor (PC)                            │
│  · Feed de cámara con centroide superpuesto                     │
│  · Posición absoluta de los actuadores (mm)                     │
│  · Consola de comandos (WiFi o Serie)                           │
└─────────────────────────────────────────────────────────────────┘
```

## Estructura del repositorio

```
TFG-solar-tracker/
├── bucle_control_v2/   # Firmware ESP32 — control de motores
├── proceso_cam_v1/     # Firmware ESP32-CAM — visión artificial
└── kivy_monitor/       # Monitor en Python/Kivy para PC
```

---

## bucle_control_v2

Firmware ESP-IDF para el ESP32 que controla dos actuadores lineales paso a paso.

**Funcionalidades:**
- Barrido raster (PHI × THETA) hasta encontrar el sol
- Seguimiento con cinemática inversa: error en píxeles → ángulo → mm de actuador
- Homing automático por finales de carrera
- Servidor HTTP en `:80` — `POST /cmd` y `GET /status`

**Dependencia externa — FastAccelStepper:**

Esta librería no está incluida en el repo. Clónala antes de compilar:
```bash
git clone https://github.com/gin66/FastAccelStepper components/FastAccelStepper
```

**Compilar y flashear:**
```bash
cd bucle_control_v2
idf.py build
idf.py -p <PUERTO> flash monitor
```

---

## proceso_cam_v1

Firmware ESP-IDF para el módulo ESP32-CAM (AI-Thinker).

**Funcionalidades:**
- Captura en QQVGA (160×120) en escala de grises
- Detección del centroide solar por umbralización
- Servidor HTTP: `GET /centroid`, `GET /snapshot`, `GET /stream`, `GET /status`
- Almacena la última posición absoluta recibida del ESP32-Motor

**Compilar y flashear:**
```bash
cd proceso_cam_v1
idf.py build
idf.py -p <PUERTO> flash monitor
```

> La primera vez, `idf.py build` descarga automáticamente las dependencias (`managed_components/`).

---

## kivy_monitor

Interfaz gráfica en Python para monitorizar el sistema en tiempo real.

**Requisitos:**
```bash
cd kivy_monitor
pip install -r requirements.txt
```

**Ejecutar:**
```bash
python solar_monitor.py
```

**Funcionalidades:**
- Feed de cámara con máscara de umbral y centroide superpuestos
- Posición absoluta de los actuadores en mm (via `GET /status` al ESP32-Motor)
- Panel de control: iniciar barrido, pausar, re-barrer, homing
- Alineación por predicción solar (`prueba_prediccion_hora_solar.py`)
- Consola de comandos por WiFi (HTTP) o puerto serie USB

---

## Comunicación entre nodos

| Endpoint | Nodo | Descripción |
|---|---|---|
| `GET /centroid` | ESP32-CAM | JSON con `{dx, dy, cx, cy, area, valid}` |
| `GET /snapshot` | ESP32-CAM | Imagen JPEG |
| `GET /stream` | ESP32-CAM | Stream MJPEG continuo |
| `GET /status` | ESP32-CAM | Estado completo + posición relayed del motor |
| `POST /cmd` | ESP32-Motor | Ejecuta un comando de texto |
| `GET /status` | ESP32-Motor | JSON con `{theta, phi, paused, homing}` en mm |

**Comandos disponibles en el ESP32-Motor:**

```
home                          → calibración por finales de carrera
goto  <theta|phi> <mm>        → mover eje a posición absoluta
angle <theta|phi> <grados>    → mover eje a ángulo (usa cinemática inversa)
solar <elev_deg> <acim_deg>   → apuntar al sol por coordenadas solares
ctrl  <pause|resume>          → pausar / reanudar el bucle de control
scan                          → reiniciar barrido desde posición actual
stop  [theta|phi]             → parar eje(s) inmediatamente
pos                           → posición actual en mm (JSON)
status                        → estado completo (JSON)
setpos <theta|phi> <mm>       → fijar contador de posición manualmente
```
