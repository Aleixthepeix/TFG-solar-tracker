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

## Configuración manual obligatoria antes de compilar

Hay tres parámetros que dependen del hardware instalado y deben ajustarse a mano antes de flashear.

### 1. GPIOs de los finales de carrera — `bucle_control_v2/main/config.h`

```c
#define CFG_THETA_ENDSTOP_GPIO  -1   /* TODO: asignar GPIO real al instalar */
#define CFG_PHI_ENDSTOP_GPIO    -1   /* TODO: asignar GPIO real al instalar */
```

El valor `-1` deshabilita la detección hardware y el homing se realiza por tiempo.
Sustituir por el número de GPIO al que está conectado cada interruptor NC (activo en LOW, pull-up interno).
Cableado esperado: pin NC del final de carrera → GPIO, pin COM → GND.

### 2. Umbral de luminosidad — `proceso_cam_v1/main/vision.h`

```c
#define CAM_THRESHOLD   200     /* 0-255: pixels >= umbral contribuyen al centroide */
```

Valor entre 0 y 255. Solo los píxeles con intensidad ≥ `CAM_THRESHOLD` se incluyen en el cálculo del centroide solar.
Debe ajustarse según las condiciones de iluminación del entorno: un valor demasiado bajo detecta falsas fuentes de luz; demasiado alto puede no detectar el sol en días nublados.
El umbral es ajustable en tiempo real desde el slider del `kivy_monitor` solo para el cliente en PC.

### 3. FOV de la cámara — `bucle_control_v2/main/config.h`

```c
#define CFG_CAM_FOV_H_DEG   65.0f   /* campo de visión horizontal [°] */
#define CFG_CAM_FOV_V_DEG   50.0f   /* campo de visión vertical   [°] */
```

Estos valores determinan la conversión píxel → grado que usa la cinemática inversa durante el seguimiento.
Los valores por defecto (65° × 50°) son una estimación para el módulo AI-Thinker con OV2640 en QQVGA.
Calibrar apuntando la cámara a una referencia conocida y midiendo cuántos píxeles de desplazamiento corresponden a un giro de ángulo conocido.

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

## Código de desarrollo — Plataforma Stewart

> Código previo al sistema de control final. No forma parte del bucle distribuido ESP32↔ESP32-CAM.

El directorio [`stewart/`](stewart/) contiene los scripts desarrollados durante la fase de estudio y prototipado con una **plataforma de Gough-Stewart** de 6 actuadores. Hay dos líneas de trabajo:

- **`bucle_control_v1.py`** — primer prototipo del seguimiento solar: visión artificial (OpenCV) + cinemática inversa + control proporcional, todo en un único script de PC. Usa una plataforma triangular simplificada y visualiza el movimiento en 3D con Matplotlib en tiempo real.

- **`mapeo_espacio_trabajo_ruta_solar.py`** — conecta con SolidWorks vía API COM (`win32com`) para barrer el espacio de trabajo angular de la plataforma real (−30° a +30° en φ y θ), verificar las restricciones cinemáticas en el ensamblaje CAD y superponer las trayectorias solares del solsticio y equinoccio, identificando qué porción de la ruta solar queda dentro del espacio de trabajo.

El resto de scripts son simulaciones, pruebas de umbralización y utilidades de análisis. Ver [`stewart/README.md`](stewart/README.md) para la descripción detallada de cada archivo.

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
