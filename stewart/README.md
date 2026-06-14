# Código de desarrollo — Plataforma de Gough-Stewart

Este directorio contiene el código desarrollado durante la fase de estudio y prototipado del TFG, **anterior al sistema de control final** (`bucle_control_v2` + `proceso_cam_v1`).

Incluye tres líneas de trabajo:
1. **Bucle de control visual inicial** — prototipo del seguimiento solar con imagen real (vídeo/webcam).
2. **Validación con imagen simulada** — banco de pruebas en bucle cerrado sin hardware ni vídeo real.
3. **Mapeo del espacio de trabajo** — análisis cinemático del ensamblaje real conectado a SolidWorks.

---

## 1. Bucle de control visual inicial

### `bucle_control_v1.py`

Primer prototipo del bucle de seguimiento solar. Integra visión artificial y cinemática inversa en un único script de PC.

**Flujo:**
1. Captura frames de vídeo (webcam o archivo `.mp4`)
2. Convierte a escala de grises y umbraliza (valor fijo 240) para aislar el sol
3. Calcula el centroide del área más brillante con momentos de imagen (OpenCV)
4. Convierte el vector de error en píxeles a incrementos angulares (control proporcional, Kp = 0.2)
5. Ejecuta la cinemática inversa sobre una **plataforma Stewart simplificada de 4 vértices** y visualiza el resultado en 3D con Matplotlib en tiempo real

> **Nota:** el script referencia `sunrise_to_sunset.mp4` con ruta absoluta local (línea 115). Para ejecutarlo con webcam, comenta la línea 115 y descomenta la línea 114. El vídeo no está incluido en el repositorio por su tamaño.

**Dependencias:** `opencv-python`, `numpy`, `matplotlib`

---

## 2. Validación con imagen simulada

### `imagen_simulada_prueba.py`

Banco de pruebas en **bucle cerrado completamente sintético**: no requiere webcam, vídeo real ni hardware. Sirve para verificar que la cinemática inversa y el controlador reaccionan correctamente ante un input de sol virtual.

**Flujo:**
1. El usuario posiciona el sol virtual moviendo dos sliders de OpenCV (φ y θ en grados, rango −30° a +30°)
2. `SimuladorCamara.generar_frame()` genera un frame sintético: dibuja un círculo blanco desplazado del centro en función del **error** entre la posición del sol en el "cielo" y la orientación actual de la plataforma, replicando lo que vería una cámara montada sobre ella
3. El pipeline de visión idéntico al de `bucle_control_v1.py` (umbralización + centroide por momentos) detecta el punto brillante y calcula el vector de error en píxeles
4. El controlador proporcional (Kp = 0.2) convierte el error en incrementos angulares y actualiza la orientación acumulada de la plataforma
5. La cinemática inversa recalcula las longitudes de los actuadores y el visualizador 3D de Matplotlib muestra la plataforma moviéndose en tiempo real

**Lo que valida:** al mover los sliders, la plataforma debe converger hacia la posición del sol virtual y estabilizarse cuando el error es cero (sol centrado en imagen). Esto confirma que la cinemática inversa y el signo del control son correctos antes de probar con hardware real.

**Dependencias:** `opencv-python`, `numpy`, `matplotlib`

---

## 3. Mapeo del espacio de trabajo y rutas solares

### `mapeo_espacio_trabajo_ruta_solar.py`

Clase `SimuladorStewart` que opera sobre el **ensamblaje CAD real** de la plataforma hexagonal (6 actuadores) conectándose a SolidWorks mediante su API COM (`win32com`).

**Funcionalidades:**

#### `cinematica_inversa(phi, theta, psi=0)`
Calcula las longitudes de los 6 actuadores para una orientación dada de la plataforma (rotaciones Rx·Ry·Rz respecto al foco C). Devuelve los valores en metros para inyectarlos directamente como cotas de SolidWorks.

#### `ejecutar_simulacion()`
Anima el ensamblaje en SolidWorks interpolando suavemente desde la posición neutra hasta (φ=14°, θ=14°) en 10 pasos con easing cúbico.

#### `mapear_espacio_trabajo()`
Barre una cuadrícula de ángulos (φ, θ) de −30° a +30° en pasos de 5° y para cada punto:
- Verifica que las longitudes de actuador estén dentro de los límites físicos (30.2 mm – 619.46 mm)
- Inyecta las cotas en SolidWorks y fuerza la reconstrucción
- Detecta errores críticos de geometría (mates sin resolver, sobre-definición, fuera de rango…)
- Clasifica cada punto: ✅ válido / ❌ fuera de carrera / ⚠️ error CAD

Sobre el mapa resultante superpone las **trayectorias solares** del solsticio de verano, equinoccio y solsticio de invierno calculadas astronómicamente para la latitud del emplazamiento (39.866°), permitiendo comparar visualmente qué fracción de la ruta solar queda dentro del espacio de trabajo de la plataforma.

> **Requisito:** SolidWorks debe estar abierto con el ensamblaje de la plataforma antes de ejecutar. Solo funciona en Windows.

**Dependencias:** `pywin32`, `numpy`, `matplotlib`

