# Código de desarrollo — Arquitectura alternativa de dos ejes

Este directorio contiene los scripts desarrollados durante la fase de diseño de la **arquitectura alternativa** propuesta cuando el análisis del espacio de trabajo mostró que la plataforma de Gough-Stewart no cubría la trayectoria solar requerida.

La solución adoptada es un **mecanismo de dos ejes independientes** (elevación y azimut), cada uno accionado por un único actuador lineal (carros de 0–300 mm) a través de un sistema de barras articuladas. Esta arquitectura es la que se implementó finalmente en hardware (`bucle_control_v2`).

Incluye tres scripts:
1. **`solve_theta_elev.py`** — cinemática directa del eje de elevación.
2. **`solve_phi.py`** — cinemática directa del eje de azimut.
3. **`bucle_imagen_simulada.py`** — simulación completa en bucle cerrado con imagen sintética.

---

## 1. Cinemática directa del eje de elevación

### `solve_theta_elev.py`

Deriva analíticamente la relación entre la **posición del carro** (actuador lineal, 0–300 mm) y el **ángulo de elevación** θ que adopta la plataforma.

La función `solve_theta(A)` resuelve la geometría de una **cadena cinemática plana** de 5 parámetros (ρ₁–ρ₅) mediante la ley del coseno aplicada sucesivamente a los triángulos que forman las barras de la transmisión:

```
θ = π − θ_A − θ_B − θ_C
```

donde θ_A y θ_B son ángulos de los brazos extremos y θ_C el ángulo interior del triángulo central (ley del coseno con hipotenusas `a` y `b`).

El script calcula y representa θ vs `s` para todo el recorrido del actuador, mostrando la función de transferencia del mecanismo de elevación.

**Dependencias:** `numpy`, `matplotlib`

---

## 2. Cinemática directa del eje de azimut

### `solve_phi.py`

Misma lógica para el **eje de azimut** φ, cuyo mecanismo de transmisión tiene una geometría diferente al de elevación.

La función `solve_phi(A)` encadena dos pasos:
1. Calcula las coordenadas del punto C a partir del ángulo del brazo OB (ley del coseno sobre el triángulo OA₂B), desplazado 47.7° respecto a OC.
2. Calcula el ángulo α dentro del triángulo QCD (ley del coseno) y lo combina con la dirección del vector QC para obtener φ:

```
φ = arctan2(QCy, QCx) − α
```

El script representa φ vs `s` e indica con una línea vertical el punto de mediodía (φ = 0°, s ≈ 189.7 mm).

**Dependencias:** `numpy`, `matplotlib`

---

## 3. Simulación completa en bucle cerrado

### `bucle_imagen_simulada.py`

Script de validación integral que combina **cinemática inversa numérica**, **modelo de cámara por proyección gnomónica** y **control proporcional** en un único bucle visual, sin necesidad de hardware real.

### Arquitectura del bucle

```
Sol virtual (az, el)
        │
        ▼
SimuladorCamara.generar_frame(plat_az, plat_el)
  → proyección gnomónica: dibuja círculo blanco donde el sol
    proyecta en la imagen según la orientación actual de la plataforma
        │
        ▼
Detección (umbralización + momentos OpenCV)
  → sun_x, sun_y en píxeles
        │
        ▼
Error en imagen → incremento angular (Kp = 0.3)
  delta_az = (vector_x / width)  * FOV_H * Kp / sin(el)   ← corrección de elevación
  delta_el = (vector_y / height) * FOV_V * Kp
        │
        ▼
Cinemática inversa (método de Brent, scipy)
  A_theta = cinematica_inversa(plat_el, 'theta')   → posición actuador elevación [mm]
  A_phi   = cinematica_inversa(plat_az, 'phi')     → posición actuador azimut    [mm]
        │
        ▼
Panel de actuadores (barras de progreso OpenCV)
```

### Detalles técnicos destacables

**Proyección gnomónica** (`proyeccion_gnomica`): construye un sistema de referencia local de la cámara (ejes ex, ey, ez) a partir del vector de orientación de la plataforma, y proyecta el vector unitario del sol en el plano imagen usando perspectiva pura. Esto es más fiel a la óptica real que el desplazamiento lineal usado en los prototipos anteriores.

**Corrección de elevación en azimut**: el incremento `delta_az` se divide por `sin(el)` para compensar la compresión del ángulo azimutal al acercarse al cenit. Sin esta corrección el control se vuelve inestable cerca de la vertical.

**Cinemática inversa numérica**: `cinematica_inversa(angulo, metodo)` usa el método de Brent (`scipy.optimize.brentq`) para encontrar la posición `A` del actuador que produce el ángulo deseado. Si los extremos del intervalo tienen el mismo signo hace un barrido previo de 500 puntos para localizar el cambio de signo.

**Modo dual**: `MODO_SIMULACION = True` usa la imagen sintética con sliders (azimut −120°…+120°, elevación 0°…90°); `MODO_SIMULACION = False` lee la webcam real (sin necesidad de cambiar otra línea).

**Lo que valida**: al mover los sliders, la plataforma virtual debe perseguir la posición del sol y converger. Confirma que ambas funciones de cinemática directa, la inversión numérica y el signo del control son correctos antes de pasar al hardware.

**Dependencias:** `opencv-python`, `numpy`, `scipy`
