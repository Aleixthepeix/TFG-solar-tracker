import cv2
import numpy as np
import matplotlib.pyplot as plt

phi = np.radians(0); theta = np.radians(0); psi= np.radians(0);  

r = 1000  # Distancia del centro a los vértices
f = 600   # Distancia focal
O = np.array([0, 0, 0])  # Origen fijo
C = np.array([300, 300, 0])  # Origen plataforma

# Generar ángulos
arg_B = np.linspace(0, 2 * np.pi, 4)
arg_P = np.linspace(np.pi, 3 * np.pi, 4)

# --- 2. Definición de Vértices (Triángulos) ---
# Base (Btri)
Btri = np.array([
    [O[0] + r * np.cos(arg_B[i]), O[1] + r * np.sin(arg_B[i]), O[2]] 
    for i in range(4)
])

# Plataforma en posición inicial (Ptri)
Ptri = np.array([
    [C[0] + r * np.cos(arg_P[i]), C[1] + r * np.sin(arg_P[i]), C[2] + f] 
    for i in range(4)
])

# ---  Visualización con Matplotlib ---
plt.ion()  # Activar modo interactivo
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1500, 1500]); ax.set_ylim([-1500, 1500]); ax.set_zlim([0, 2000])
ax.view_init(elev=30, azim=-30)

def cinematica_inversa_stewart(phi,theta,psi):
    global Ptri  # Declarar Ptri como global para poder acceder y modificar la variable global
    ax.cla() # Limpiar frame anterior

    # Matriz de rotación combinada
    Rx=np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    Ry=np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0], [-np.sin(theta), 0, np.cos(theta)]])
    Rz=np.array([[np.cos(psi),-np.sin(psi),0],[np.sin(psi),np.cos(psi),0], [0, 0, 1]])
    R=Rz @ Ry @ Rx
    
    # Rotar la plataforma respecto a su centro C
    Ptrirot = np.zeros((4, 3))
    for i in range(4):
        Ptrirot[i, :] = (R @ (Ptri[i, :] - C).T).T + C

    # --- 4. Cálculo de Actuadores ---
    # Índices de conexión manual
    idx_P = [1, 2, 0, 2, 0, 1] # Puntos de la plataforma
    idx_B = [0, 0, 1, 1, 2, 2] # Puntos de la base

    # Posición Final (act)
    act = Ptrirot[idx_P] - Btri[idx_B]
    mag = np.linalg.norm(act, axis=1, keepdims=True)

    # Posición Inicial (act0)
    act0 = Ptri[idx_P] - Btri[idx_B]
    mag0 = np.linalg.norm(act0, axis=1, keepdims=True)

    # Actuadores normalizados para dibujo (act_norm)
    act_norm = (act / mag) * mag0

    # Extensión/contracción
    var_act = mag.flatten() - mag0.flatten()
    print(f"Variación actuadores: {var_act}")

    # Dibujar Base y Puntos
    ax.scatter(O[0], O[1], O[2], color='k', s=50) # Origen
    ax.plot(Btri[:, 0], Btri[:, 1], Btri[:, 2], 'b-', label='Base')

    # Dibujar Plataforma Rotada
    ax.scatter(C[0], C[1], C[2], color='r', s=50) # Foco
    ax.plot(Ptrirot[:, 0], Ptrirot[:, 1], Ptrirot[:, 2], 'm-', label='Plataforma Rotada')
    ax.scatter(Ptrirot[:, 0], Ptrirot[:, 1], Ptrirot[:, 2], color='k', s=30)

    # Dibujar Actuadores usando quiver
    for i in range(3):
        # Actuadores Rojos (Posición Final)
        for j in range(2):
            idx = 2 * i + j
            ax.quiver(Btri[i, 0], Btri[i, 1], Btri[i, 2], 
                    act[idx, 0], act[idx, 1], act[idx, 2], 
                    arrow_length_ratio=0.05, color='r', linewidth=1.5)
            
            # Actuadores Verdes (Longitud Inicial)
            ax.quiver(Btri[i, 0], Btri[i, 1], Btri[i, 2], 
                    act_norm[idx, 0], act_norm[idx, 1], act_norm[idx, 2], 
                    arrow_length_ratio=0, color='g', linewidth=1.5, alpha=0.6)
            
    # Actualización de la plataforma inicial para el siguiente bucle
    Ptri=Ptrirot

    # Configuración de la gráfica
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Simulación de Plataforma Stewart (Simplificada)')
    ax.legend()
    ax.set_aspect('equal')# Para mantener la proporción igualada
    plt.draw()
    plt.pause(0.01) # Pausa mínima para refrescar la GUI








#cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(r"C:\Users\Aleix\Desktop\TFG\python\stewart\sunrise_to_sunset.mp4")
height, width = cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FRAME_WIDTH)
FOV__horizontal = np.radians(60)  # Campo de visión horizontal en grados
FOV__vertical = np.radians(45)  # Campo de visión vertical en grados

def variacion_angulos(vector_x,vector_y):
    Kp=0.2 # Ganancia ajustable
    delta_theta=vector_y*(FOV__vertical/height)*Kp
    delta_phi=vector_x*(FOV__horizontal/width)*Kp
    return delta_theta, delta_phi

def main():
    # 1. Inicializar cámara (usamos CAP_DSHOW para evitar retrasos en Windows)
    #cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    #cap = cv2.VideoCapture("sunrise_to_sunset.mp4")

    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara.")
        return

    print("Rastreando punto brillante... Presiona 'ESC' para salir.")

    while True:
        ret, frame = cap.read()
        #frame=
        if not ret:
            break

        # Dimensiones del frame y centro de la cámara
        height, width = frame.shape[:2]
        c_cam_x, c_cam_y = width // 2, height // 2

        # 2. Pre-procesamiento
        # Convertimos a escala de grises y aplicamos un desenfoque para reducir ruido
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        # 3. Aislar el punto más brillante
        # Con el filtro solar, el sol será casi blanco (cercano a 255)
        # Ajusta el valor 240 si necesitas más o menos sensibilidad
        _, thresh = cv2.threshold(blurred, 240, 255, cv2.THRESH_BINARY)

        # 4. Calcular el centroide del punto aislado usando momentos
        M = cv2.moments(thresh)
        
        if M["m00"] > 0:
            # Centro del sol en píxeles
            sun_x = int(M["m10"] / M["m00"])
            sun_y = int(M["m01"] / M["m00"])

            # 5. Medir el vector de separación (Error)
            vector_x = sun_x - c_cam_x
            vector_y = sun_y - c_cam_y
            delta_theta, delta_phi = variacion_angulos(vector_x, vector_y)
            cinematica_inversa_stewart(delta_theta, delta_phi, 0)

            # Dibujar elementos visuales
            # Centro de la cámara (Cruz roja)
            cv2.line(frame, (c_cam_x - 10, c_cam_y), (c_cam_x + 10, c_cam_y), (0, 0, 255), 2)
            cv2.line(frame, (c_cam_x, c_cam_y - 10), (c_cam_x, c_cam_y + 10), (0, 0, 255), 2)

            # Centro del sol (Círculo verde)
            cv2.circle(frame, (sun_x, sun_y), 15, (0, 255, 0), 2)

            # Vector de separación (Línea azul)
            cv2.line(frame, (c_cam_x, c_cam_y), (sun_x, sun_y), (255, 0, 0), 2)

            # Mostrar datos del vector en pantalla
            cv2.putText(frame, f"Vector: [{vector_x}, {vector_y}] px", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Delta: [{delta_theta:.2f}, {delta_phi:.2f}] grados", (20, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "SOL NO DETECTADO", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 6. Mostrar resultados
        cv2.imshow("Tracking Solar (Original)", frame)
        cv2.imshow("Punto Aislado (Mascara)", thresh)

        # Salir con ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()