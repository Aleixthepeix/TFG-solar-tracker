import numpy as np
import matplotlib.pyplot as plt

def solve_theta(A):

    #Parámetros:
    rho1, rho2, rho3, rho4, rho5 = 316.57, 120, 200, 400, 221.08

    #calculo de grados A y B
    theta_A = np.arctan(rho5/(rho4-A))
    theta_B = np.arctan(rho2/rho3)

    #Cálculo de hipotenusas
    a = np.sqrt(rho5**2 + (rho4-A)**2)
    b = np.sqrt(rho2**2 + rho3**2)

    #Cálculo de theta_C y theta
    theta_C = np.arccos((a**2 + b**2 - rho1**2)/(2*a*b))

    theta = np.pi - theta_A - theta_B - theta_C
    return np.degrees(theta)

s_vals = np.linspace(0, 300, 1000)
theta_vals = []

for s in s_vals:
    theta = solve_theta(s)
    theta_vals.append(theta)

plt.figure(figsize=(9, 4))
plt.plot(s_vals, theta_vals, linewidth=2)
plt.xlabel('s — posición carro desde A₃ [mm]')
plt.ylabel('θ [°]')
plt.title('Ángulo de la barra 3 vs posición del carro')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

