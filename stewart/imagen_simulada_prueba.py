import cv2
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURACIÓN DE SIMULACIÓN ---
MODO_SIMULACION = True 

# Variables globales para la posición del sol "en el cielo" (Grados)
sol_cielo_phi = 15.0   
sol_cielo_theta = -10.0

# Variables de estado de la plataforma (Acumuladores en Radianes)
phi_actual = 0.0
theta_actual = 0.0

# Funciones de callback para los Trackbars (necesarias para OpenCV)
def on_change_phi(val):
    global sol_cielo_phi
    sol_cielo_phi = val - 30 # Mapea de 0:60 a -30:30 grados

def on_change_theta(val):
    global sol_cielo_theta
    sol_cielo_theta = val - 30 # Mapea de 0:60 a -30:30 grados

class SimuladorCamara:
    def __init__(self, w=640, h=480):
        self.w, self.h = w, h
        self.fov_h = np.radians(60)
        self.fov_v = np.radians(45)

    def generar_frame(self, p_phi, p_theta):
        frame = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        
        # El sol se mueve según la diferencia entre el sol real y la orientación de la plataforma
        # Convertimos la posición del cielo de grados a radianes para el cálculo
        s_phi_rad = np.radians(sol_cielo_phi)
        s_theta_rad = np.radians(sol_cielo_theta)
        
        error_phi = s_phi_rad - p_phi
        error_theta = s_theta_rad - p_theta
        
        sun_x = int((self.w / 2) + (error_phi / self.fov_h) * self.w)
        sun_y = int((self.h / 2) - (error_theta / self.fov_v) * self.h)
        
        if 0 < sun_x < self.w and 0 < sun_y < self.h:
            cv2.circle(frame, (sun_x, sun_y), 20, (255, 255, 255), -1)
        
        return cv2.GaussianBlur(frame, (11, 11), 0)

# --- CINEMÁTICA DE STEWART (Simplificada para el ejemplo) ---
r = 1000; f = 600; O = np.array([0, 0, 0]); C = np.array([0, 0, 0])
arg_B = np.linspace(0, 2 * np.pi, 4)
arg_P = np.linspace(np.pi, 3 * np.pi, 4)
Btri = np.array([[O[0] + r * np.cos(arg_B[i]), O[1] + r * np.sin(arg_B[i]), O[2]] for i in range(4)])
Ptri_init = np.array([[C[0] + r * np.cos(arg_P[i]), C[1] + r * np.sin(arg_P[i]), C[2] + f] for i in range(4)])
Ptri = Ptri_init.copy()

plt.ion()
fig = plt.figure(figsize=(7, 5))
ax = fig.add_subplot(111, projection='3d')

def cinematica_inversa_stewart(phi, theta, psi):
    global Ptri
    ax.cla()
    Rx = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    Ry = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0], [-np.sin(theta), 0, np.cos(theta)]])
    Rz = np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    
    Ptrirot = np.zeros((4, 3))
    for i in range(4):
        Ptrirot[i, :] = (R @ (Ptri_init[i, :] - C).T).T + C

    idx_P = [1, 2, 0, 2, 0, 1]
    ax.plot(Btri[:, 0], Btri[:, 1], Btri[:, 2], 'b-', label='Base')
    ax.plot(Ptrirot[:, 0], Ptrirot[:, 1], Ptrirot[:, 2], 'm-', label='Plataforma')
    for i in range(3):
        for j in range(2):
            idx = 2 * i + j
            ax.plot([Btri[i, 0], Ptrirot[idx_P[idx], 0]], 
                    [Btri[i, 1], Ptrirot[idx_P[idx], 1]], 
                    [Btri[i, 2], Ptrirot[idx_P[idx], 2]], 'r-')
    
    Ptri = Ptrirot
    ax.set_xlim([-1500, 1500]); ax.set_ylim([-1500, 1500]); ax.set_zlim([0, 2000])
    plt.draw(); plt.pause(0.001)

def main():
    global phi_actual, theta_actual
    
    # Crear ventana de controles
    cv2.namedWindow("Controles")
    cv2.createTrackbar("Sol Horizontal (Phi)", "Controles", 45, 60, on_change_phi)
    cv2.createTrackbar("Sol Vertical (Theta)", "Controles", 20, 60, on_change_theta)
    
    sim = SimuladorCamara()
    width, height = 640, 480

    print("Simulación iniciada. Mueve los deslizadores para cambiar la posición del sol.")

    while True:
        # Generar imagen basada en la posición actual del sol y la plataforma
        frame = sim.generar_frame(phi_actual, theta_actual)

        # 1. Detección de Visión Artificial
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(cv2.GaussianBlur(gray, (11,11), 0), 240, 255, cv2.THRESH_BINARY)
        M = cv2.moments(thresh)
        
        if M["m00"] > 0:
            sun_x, sun_y = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
            vec_x, vec_y = sun_x - (width//2), sun_y - (height//2)
            
            # 2. Ley de Control (P)
            Kp = 0.2 # Ganancia ajustable
            d_phi = (vec_x / width) * np.radians(60) * Kp
            d_theta = -(vec_y / height) * np.radians(45) * Kp
            
            # 3. Integración/Actualización de la plataforma
            phi_actual += d_phi
            theta_actual += d_theta
            
            # 4. Actualizar simulación 3D
            cinematica_inversa_stewart(phi_actual, theta_actual, 0)
            
            # Dibujar vectores en el video
            cv2.line(frame, (width//2, height//2), (sun_x, sun_y), (255, 0, 0), 2)
            cv2.circle(frame, (sun_x, sun_y), 10, (0, 255, 0), 2)

        # Info en pantalla
        cv2.putText(frame, f"Sol Cielo: {sol_cielo_phi:.1f}, {sol_cielo_theta:.1f}", (10, 20), 1, 1, (255,255,255), 1)
        cv2.putText(frame, f"Plat Orient: {np.degrees(phi_actual):.1f}, {np.degrees(theta_actual):.1f}", (10, 40), 1, 1, (0,255,0), 1)

        cv2.imshow("Vista de Camara (Simulada)", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()