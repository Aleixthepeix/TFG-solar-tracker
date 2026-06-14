import numpy as np
import matplotlib.pyplot as plt

def solve_phi(A):
    rho3, rho5, rho6 = 262.5719, 120*np.sqrt(3), 120
    OB, OC, BC, OQ   = 165.0094, 240, 177.5344, 120
    OA = 422.4261 - A

    # Ángulos del triángulo OA2B
    theta3  = np.arccos((OA**2 - OB**2 + rho3**2) / (2*OA*rho3))
    thetaOB = np.pi - np.arccos((OA**2 + OB**2 - rho3**2) / (2*OA*OB))
    thetaOC = thetaOB - np.radians(47.7)

    # Coordenadas de C (origen en O)
    Cx = OC * np.cos(thetaOC)
    Cy = OC * np.sin(thetaOC)

    # Coordenadas de Q
    Qx, Qy = OQ, 0.0

    # Vector QC y distancia
    QCx   = Cx - Qx
    QCy   = Cy - Qy
    distQC = np.sqrt(QCx**2 + QCy**2)
    ang_QC = np.arctan2(QCy, QCx)   # ángulo de QC respecto horizontal

    # Ángulo en Q dentro del triángulo QCD (teorema del coseno)
    cos_alpha = np.clip(
        (rho6**2 + distQC**2 - rho5**2) / (2*rho6*distQC), -1, 1)
    alpha = np.arccos(cos_alpha)

    # phi = ángulo de QD respecto a la horizontal
    phi = ang_QC - alpha          # D queda por debajo de QC

    return np.degrees(phi)

s_vals = np.linspace(0, 300, 1000)
phi_vals = []

for s in s_vals:
    t6 = -solve_phi(s)
    phi_vals.append(t6)

plt.figure(figsize=(9, 4))
plt.plot(s_vals, phi_vals, linewidth=2)
plt.axhline(0,   color='r', linestyle='--', alpha=0.5, label='Mediodía (φ=0°)')
plt.axvline(189.6991, color='r', linestyle='--', alpha=0.5)
plt.xlabel('s — posición carro desde A₃ [mm]')
plt.ylabel('θ₆ [°]')
plt.title('Ángulo de la barra 6 vs posición del carro')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()