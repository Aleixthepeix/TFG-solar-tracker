import cv2
import numpy as np
from scipy.optimize import brentq

# ─────────────────────────────────────────────
#  CONFIGURACIÓN DE SIMULACIÓN
# ─────────────────────────────────────────────
MODO_SIMULACION = True

# Posición del sol en el cielo en coordenadas esféricas
#   azimut  : 0° = sur, +120° = oeste, -120° = este  (rango -120..+120)
#   elevacion: 0° = cenit, 90° = horizonte            (rango 0..90)
sol_azimut    =  30.0   # grados
sol_elevacion =  45.0   # grados

# Límites del actuador (mm)
A_MIN, A_MAX = 0.0, 300.0


# ─────────────────────────────────────────────
#  CINEMÁTICA DIRECTA
# ─────────────────────────────────────────────
def solve_theta(A):
    rho1, rho2, rho3, rho4, rho5 = 316.57, 120, 200, 400, 221.08

    theta_A = np.arctan(rho5 / (rho4 - A))
    theta_B = np.arctan(rho2 / rho3)

    a = np.sqrt(rho5**2 + (rho4 - A)**2)
    b = np.sqrt(rho2**2 + rho3**2)

    theta_C = np.arccos((a**2 + b**2 - rho1**2) / (2 * a * b))
    theta   = np.pi - theta_A - theta_B - theta_C
    return np.degrees(theta)


def solve_phi(A):
    rho3, rho5, rho6 = 262.5719, 120 * np.sqrt(3), 120
    OB, OC, OQ       = 165.0094, 240, 120
    OA = 422.4261 - A

    thetaOB = np.pi - np.arccos((OA**2 + OB**2 - rho3**2) / (2 * OA * OB))
    thetaOC = thetaOB - np.radians(47.7)

    Cx = OC * np.cos(thetaOC)
    Cy = OC * np.sin(thetaOC)
    Qx, Qy = OQ, 0.0

    QCx    = Cx - Qx
    QCy    = Cy - Qy
    distQC = np.sqrt(QCx**2 + QCy**2)
    ang_QC = np.arctan2(QCy, QCx)

    cos_alpha = np.clip(
        (rho6**2 + distQC**2 - rho5**2) / (2 * rho6 * distQC), -1, 1)
    alpha = np.arccos(cos_alpha)
    phi   = ang_QC - alpha

    return np.degrees(phi)


# ─────────────────────────────────────────────
#  CINEMÁTICA INVERSA
# ─────────────────────────────────────────────
def cinematica_inversa(angulo, metodo, a_min=A_MIN, a_max=A_MAX):
    def f(A):
        return (solve_theta(A) if metodo == 'theta' else solve_phi(A)) - angulo

    fa, fb = f(a_min), f(a_max)

    if fa * fb >= 0:
        A_grid  = np.linspace(a_min, a_max, 500)
        f_grid  = np.array([f(a) for a in A_grid])
        cambios = np.where(np.diff(np.sign(f_grid)))[0]
        if len(cambios) == 0:
            return None
        A_lo = A_grid[cambios[0]]
        A_hi = A_grid[cambios[0] + 1]
    else:
        A_lo, A_hi = a_min, a_max

    return brentq(f, A_lo, A_hi, xtol=1e-9)


# ─────────────────────────────────────────────
#  GEOMETRÍA ESFÉRICA
#  Convenio:
#    elevacion: 0° = cenit (+Z), 90° = horizonte
#    azimut   : 0° = sur (+Y), +az = oeste, -az = este
#  Vector unitario apuntando al sol desde la plataforma:
#    x = sin(el)*sin(az)   → E-O  (O positivo)
#    y = sin(el)*cos(az)   → S-N  (S positivo)
#    z = cos(el)           → cenit
# ─────────────────────────────────────────────
def esf_a_vec(az_deg, el_deg):
    az = np.radians(az_deg)
    el = np.radians(el_deg)
    x =  np.sin(el) * np.sin(az)   # E-O
    y =  np.sin(el) * np.cos(az)   # S-N
    z =  np.cos(el)                 # cenit
    return np.array([x, y, z])


def proyeccion_gnomica(vec_sol, vec_plat, fov_h, fov_v, w, h):
    """
    Proyecta vec_sol en el plano imagen de una cámara orientada según vec_plat.
    Devuelve (px, py) en píxeles, o None si el sol está detrás de la cámara.

    Se construye un sistema de referencia local de la cámara:
      - eje_z apunta a donde mira la cámara (vec_plat)
      - eje_x apunta a la derecha  (este → oeste = azimut creciente)
      - eje_y apunta hacia arriba  (elevacion decreciente en imagen)
    """
    ez = vec_plat / np.linalg.norm(vec_plat)

    # "arriba" del mundo: cenit global, o N si la cámara apunta al cenit
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(ez, up)) > 0.999:
        up = np.array([0.0, 1.0, 0.0])   # fallback al sur

    ex = np.cross(ez, up)
    ex /= np.linalg.norm(ex)
    ey = np.cross(ex, ez)
    ey /= np.linalg.norm(ey)

    # Coordenadas del vector sol en el sistema cámara
    dot_z = np.dot(vec_sol, ez)
    if dot_z <= 0:
        return None   # sol detrás de la cámara

    # Proyección gnomónica (perspectiva)
    dot_x = np.dot(vec_sol, ex)
    dot_y = np.dot(vec_sol, ey)

    tan_x = dot_x / dot_z
    tan_y = dot_y / dot_z

    # Escala: tan(FOV/2) cubre la mitad de la imagen
    scale_x = (w / 2) / np.tan(fov_h / 2)
    scale_y = (h / 2) / np.tan(fov_v / 2)

    px = int(w / 2 + tan_x * scale_x)
    py = int(h / 2 - tan_y * scale_y)   # Y imagen invertida respecto a "arriba"

    return px, py


# ─────────────────────────────────────────────
#  SIMULADOR DE CÁMARA
# ─────────────────────────────────────────────
class SimuladorCamara:
    def __init__(self, w=640, h=480):
        self.w, self.h = w, h
        self.fov_h = np.radians(60)
        self.fov_v = np.radians(45)

    def generar_frame(self, plat_az_deg, plat_el_deg):
        frame = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        vec_sol  = esf_a_vec(sol_azimut,  sol_elevacion)
        vec_plat = esf_a_vec(plat_az_deg, plat_el_deg)

        resultado = proyeccion_gnomica(
            vec_sol, vec_plat, self.fov_h, self.fov_v, self.w, self.h)

        if resultado is not None:
            px, py = resultado
            if 0 < px < self.w and 0 < py < self.h:
                cv2.circle(frame, (px, py), 20, (255, 255, 255), -1)

        return frame   # sin blur — el loop lo aplica


# ─────────────────────────────────────────────
#  CALLBACKS DE TRACKBARS
# ─────────────────────────────────────────────
def on_change_azimut(val):
    global sol_azimut
    sol_azimut = val - 120          # 0:240 → -120:+120 °

def on_change_elevacion(val):
    global sol_elevacion
    sol_elevacion = val             # 0:90 → 0:90 °


# ─────────────────────────────────────────────
#  PANEL DE ACTUADORES
# ─────────────────────────────────────────────
BAR_W = 320
BAR_H = 180

def dibujar_panel_actuadores(A_theta_val, A_phi_val):
    panel = np.zeros((BAR_H, BAR_W, 3), dtype=np.uint8)

    for x in range(0, BAR_W, 40):
        cv2.line(panel, (x, 0), (x, BAR_H), (30, 30, 30), 1)
    for y in range(0, BAR_H, 30):
        cv2.line(panel, (0, y), (BAR_W, y), (30, 30, 30), 1)

    title_color  = (200, 200, 200)
    bar_bg_color = (50, 50, 60)
    val_color    = (255, 255, 255)

    def barra(label, valor, y_top, color_lleno):
        if valor is None:
            valor = 0.0
        valor = float(np.clip(valor, A_MIN, A_MAX))
        ratio = valor / A_MAX

        x0, x1 = 20, BAR_W - 20
        bar_y0, bar_y1 = y_top + 28, y_top + 56

        cv2.rectangle(panel, (x0, bar_y0), (x1, bar_y1), bar_bg_color, -1)
        cv2.rectangle(panel, (x0, bar_y0), (x1, bar_y1), (80, 80, 90), 1)

        fill_x = int(x0 + ratio * (x1 - x0))
        if fill_x > x0:
            mid_x = (x0 + fill_x) // 2
            cv2.rectangle(panel, (x0, bar_y0), (fill_x, bar_y1), color_lleno, -1)
            overlay = panel.copy()
            cv2.rectangle(overlay, (x0, bar_y0), (mid_x, bar_y1),
                          tuple(min(c + 60, 255) for c in color_lleno), -1)
            cv2.addWeighted(overlay, 0.3, panel, 0.7, 0, panel)

        for mark in [0, 100, 200, 300]:
            mx = int(x0 + (mark / A_MAX) * (x1 - x0))
            cv2.line(panel, (mx, bar_y1), (mx, bar_y1 + 5), (120, 120, 120), 1)
            cv2.putText(panel, str(mark), (mx - 10, bar_y1 + 16),
                        cv2.FONT_HERSHEY_PLAIN, 0.7, (100, 100, 100), 1)

        cv2.putText(panel, label, (x0, y_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, title_color, 1)
        cv2.putText(panel, f"{valor:6.1f} mm", (BAR_W - 100, y_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, val_color, 1)

        cv2.line(panel, (fill_x, bar_y0 - 4), (fill_x, bar_y1 + 4),
                 (255, 255, 255), 2)

    cv2.putText(panel, "ACTUADORES LINEALES", (BAR_W // 2 - 95, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 200, 255), 1)
    cv2.line(panel, (20, 22), (BAR_W - 20, 22), (60, 80, 120), 1)

    barra("A_theta  (elevacion)", A_theta_val, 30,  (0, 180, 220))
    barra("A_phi    (azimut)",    A_phi_val,   100, (0, 140, 255))

    return panel


# ─────────────────────────────────────────────
#  LOOP PRINCIPAL
# ─────────────────────────────────────────────
def main():
    global plat_az, plat_el, sol_azimut, sol_elevacion

    FOV_H = np.radians(60)
    FOV_V = np.radians(45)
    Kp    = 0.3

    # ── Plataforma arranca apuntando al sol → error inicial = 0 ──────────
    plat_az = sol_azimut
    plat_el = sol_elevacion

    # Cinemática inversa inicial
    # theta controla elevacion, phi controla azimut
    A_theta_val = cinematica_inversa(plat_el, 'theta')
    A_phi_val   = cinematica_inversa(plat_az, 'phi')

    if MODO_SIMULACION:
        sim = SimuladorCamara(640, 480)
        cap = None

        cv2.namedWindow("Control Simulacion", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Control Simulacion", 440, 110)
        # Azimut: trackbar 0..240 → sol_azimut = val-120 ∈ [-120, +120]
        cv2.createTrackbar("Azimut  [°+120]",  "Control Simulacion",
                           int(sol_azimut    + 120), 240, on_change_azimut)
        # Elevacion: trackbar 0..90 → sol_elevacion = val ∈ [0, 90]
        cv2.createTrackbar("Elevacion [°]",    "Control Simulacion",
                           int(sol_elevacion),       90,  on_change_elevacion)
    else:
        sim = None
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("Error: No se pudo acceder a la cámara.")
            return

    print("Rastreando punto brillante... Presiona 'ESC' para salir.")

    while True:
        # ── Frame ─────────────────────────────────────────────────────────
        if MODO_SIMULACION:
            frame = sim.generar_frame(plat_az, plat_el)
        else:
            ret, frame = cap.read()
            if not ret:
                break

        height, width = frame.shape[:2]
        c_cam_x, c_cam_y = width // 2, height // 2

        # ── Detección ─────────────────────────────────────────────────────
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        umbral  = 200 if MODO_SIMULACION else 240
        _, thresh = cv2.threshold(blurred, umbral, 255, cv2.THRESH_BINARY)

        M = cv2.moments(thresh)

        if M["m00"] > 0:
            sun_x = int(M["m10"] / M["m00"])
            sun_y = int(M["m01"] / M["m00"])

            vector_x = sun_x - c_cam_x   # + → sol a la derecha (oeste, az+)
            vector_y = sun_y - c_cam_y   # + → sol abajo (el+, más horizonte)

            # Conversión píxel → ángulo con corrección de elevación:
            # Al acercarse al cenit (el→0) el azimut se comprime por sin(el),
            # así que dividimos por sin(el) para descomprimir.
            sin_el = max(np.sin(np.radians(plat_el)), 0.05)  # evitar /0

            delta_az = ( vector_x * (np.degrees(FOV_H) / width)  * Kp) / sin_el
            delta_el = ( vector_y * (np.degrees(FOV_V) / height) * Kp)

            plat_az = np.clip(plat_az + delta_az, -120.0, 120.0)
            plat_el = np.clip(plat_el + delta_el,    0.0,  90.0)

            A_theta_val = cinematica_inversa(plat_el, 'theta')
            A_phi_val   = cinematica_inversa(plat_az, 'phi')

            # ── Overlays ──────────────────────────────────────────────────
            cv2.line(frame, (c_cam_x - 10, c_cam_y), (c_cam_x + 10, c_cam_y), (0, 0, 255), 2)
            cv2.line(frame, (c_cam_x, c_cam_y - 10), (c_cam_x, c_cam_y + 10), (0, 0, 255), 2)
            cv2.circle(frame, (sun_x, sun_y), 15, (0, 255, 0), 2)
            cv2.line(frame, (c_cam_x, c_cam_y), (sun_x, sun_y), (255, 0, 0), 2)

            cv2.putText(frame, f"Vector: [{vector_x:+4d}, {vector_y:+4d}] px",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Sol:  az={sol_azimut:+6.1f}  el={sol_elevacion:5.1f} deg",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 100), 1)
            cv2.putText(frame, f"Plat: az={plat_az:+6.1f}  el={plat_el:5.1f} deg",
                        (20, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 1)
        else:
            cv2.putText(frame, "SOL NO DETECTADO", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # ── Panel actuadores ───────────────────────────────────────────────
        panel = dibujar_panel_actuadores(A_theta_val, A_phi_val)

        cv2.imshow("Tracking Solar", frame)
        cv2.imshow("Actuadores Lineales", panel)
        if not MODO_SIMULACION:
            cv2.imshow("Mascara", thresh)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    if cap:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()