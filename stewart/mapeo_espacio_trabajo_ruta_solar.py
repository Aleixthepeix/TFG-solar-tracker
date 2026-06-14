import win32com.client
import numpy as np
import time
import matplotlib.pyplot as plt

class SimuladorStewart:
    def __init__(self):

        # Coordenadas del punto C (foco)
        self.C_inicial = np.array([0, 0, 275.99])  # Coordenadas del punto C (foco)
        
        # Coordenadas de los cardanes en la base y en la plataforma
        self.puntos_base=np.array([
            [-60.00 , -750.00 , 27.00],
            [60.00  , -750.00 , 27.00],
            [679.52 , 323.04  , 27.00],
            [619.52 , 426.96  , 27.00],
            [-619.52 , 426.96 , 27.00],
            [-679.52 , 323.04 , 27.00]
        ])
        self.puntos_plataforma=np.array([
            [-428.99 , -316.96 , 768.99],
            [ 428.99 , -316.96 , 768.99],
            [ 488.99 , -213.04 , 768.99],
            [ 60.00  ,  530.00 , 768.99],
            [-60.00  ,  530.00 , 768.99],
            [-488.99 , -213.04 , 768.99]
        ])

        # Longitudes iniciales de los actuadores
        self.L_actuadores_inicial = np.linalg.norm(self.puntos_plataforma-self.puntos_base, axis=1)

        # Cota inicial de los actuadores en SolidWorks (en mm)
        self.cota_inicial=200.00

        # Nombres de las cotas de longitud de los actuadores en SolidWorks
        self.NOMBRES_COTAS = [
            "D1@L1", "D1@L2", "D1@L3", 
            "D1@L4", "D1@L5", "D1@L6"
        ]

        # Códigos de error CRÍTICOS de la API de SolidWorks que indican
        # que una relación geométrica (Mate) realmente no se puede resolver.
        # Se excluyen advertencias menores (ej: 1=sin error, 4=advertencia leve).
        # Referencia: swFeatureError_e en la API de SolidWorks
        self.CODIGOS_ERROR_CRITICOS = {
            2,   # swFeatureErrorGeneral         - Error general irrecuperable
            8,   # swFeatureErrorOverDefined      - Sobre-definido (imposible resolver)
            16,  # swFeatureErrorNotSolved        - No resuelto
            32,  # swFeatureErrorOutOfRange       - Fuera de rango geométrico
            64,  # swFeatureErrorNoFaceToMatch    - Sin cara para el mate
            128, # swFeatureErrorBadGeometry      - Geometría inválida
            256, # swFeatureErrorMateError        - Error específico de mate
        }

        # Conexión con SolidWorks
        self.swApp = None
        self.model = None

    def conectar_sw(self):
        try:
            self.swApp = win32com.client.GetActiveObject("SldWorks.Application")
            self.model = self.swApp.ActiveDoc
            if not self.model:
                print("Error: No hay documento abierto en SolidWorks.")
                return False
            print("Conectado al ensamblaje de SolidWorks correctamente.")
            return True
        except Exception as e:
            print(f"Error conectando: {e}")
            print("Asegúrate de abrir SolidWorks y el ensamblaje primero.")
            return False

    def tiene_errores_geometria(self):
        """
        Recorre el árbol filtrando SOLO errores críticos en Mates/Componentes.
        Ignora advertencias menores que no indican fallo real de movimiento.
        Devuelve True únicamente si hay un error que impide resolver la geometría.
        """
        try:
            features = self.model.FeatureManager.GetFeatures(False)
            if not features:
                return False

            for feat in features:
                tipo = feat.GetTypeName2()

                # Solo analizar relaciones de posición y componentes
                if "Mate" not in tipo and "Component" not in tipo:
                    continue

                codigo_error = feat.GetErrorCode2(False)

                # Código 0 o 1 = sin error, ignorar
                if codigo_error <= 1:
                    continue

                # Comprobar si el código coincide con algún error crítico
                # usando AND a nivel de bits, ya que SolidWorks combina flags
                for codigo_critico in self.CODIGOS_ERROR_CRITICOS:
                    if codigo_error & codigo_critico:
                        print(f"      [ERROR CRÍTICO] {feat.Name} (Tipo: {tipo}, Código: {codigo_error})")
                        return True

            return False

        except Exception as e:
            print(f"Advertencia al leer el árbol: {e}")
            return False  # Ante duda de lectura, no penalizar el punto

    def cinematica_inversa(self, phi, theta, psi=0):

        # Matriz de Rotación (R = Rz * Ry * Rx)
        Rx = np.array([[1, 0, 0], 
                       [0, np.cos(phi), -np.sin(phi)], 
                       [0, np.sin(phi), np.cos(phi)]])
        
        Ry = np.array([[np.cos(theta), 0, np.sin(theta)], 
                       [0, 1, 0], 
                       [-np.sin(theta), 0, np.cos(theta)]])
        
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0], 
                       [np.sin(psi), np.cos(psi), 0], 
                       [0, 0, 1]])
        
        R = Rz @ Ry @ Rx

        # Vectores que unen los cardanes de la plataforma con el punto C
        vec_plataforma_actual=self.puntos_plataforma - self.C_inicial

        # Aplicar la rotación a los vectores de la plataforma
        vec_plataforma_rotada=R @ vec_plataforma_actual.T

        # Calcular las longitudes de los actuadores
        L_actuadores=np.linalg.norm(vec_plataforma_rotada.T+self.C_inicial-self.puntos_base, axis=1)

        # Variación de las longitudes respecto a la posición inicial
        delta_L=L_actuadores-self.L_actuadores_inicial

        # Variación que hay que aplicar a las cotas en SolidWorks
        delta_cotas=(delta_L+self.cota_inicial)/1000.0  # Convertir a metros para SolidWorks

        return delta_cotas

    def ejecutar_simulacion(self):
        if not self.conectar_sw(): return

        self.model.SetAddToDB(True)

        cotas_iniciales = np.array([self.cota_inicial/1000.0] * 6)
        for i, cota in enumerate(self.NOMBRES_COTAS):
            param = self.model.Parameter(cota)
            if param:
                param.SetSystemValue3(cotas_iniciales[i], 1, None)
            else:
                print(f"Error: No se encontró la cota '{cota}' en SolidWorks durante el reseteo de cotas.")

        self.model.SetAddToDB(False)
        self.model.ForceRebuild3(False)

        phi=np.radians(14)
        theta=np.radians(14)
        frames = 10

        for paso in range(frames+1):
            t=paso/frames
            f_suave=3*(t**2)-2*(t**3)
            phi_actual = phi * f_suave
            theta_actual = theta * f_suave
            delta_cotas_sw=self.cinematica_inversa(phi_actual, theta_actual)

            self.model.SetAddToDB(True)
            for i, cota in enumerate(self.NOMBRES_COTAS):
                param = self.model.Parameter(cota)
                if param:
                    param.SetSystemValue3(delta_cotas_sw[i], 1, None)
                else:
                    print(f"Error: No se encontró la cota '{cota}' en SolidWorks.")
        
            self.model.SetAddToDB(False)
            self.model.ForceRebuild3(False)
            time.sleep(0.05)

    def mapear_espacio_trabajo(self):
        if not self.conectar_sw(): return

        # 1. Definir Límites del Actuador (en mm)
        L_MIN = 30.2
        L_MAX = 619.46

        # 2. Configurar el barrido de ángulos
        rango_grados = np.arange(-30, 31, 5)

        # Lista para guardar los resultados: [phi_grados, theta_grados, ESTADO]
        # ESTADOS: 0 = Falla por longitud, 1 = Falla en CAD, 2 = ÉXITO
        resultados = []

        # Resetear las cotas a su valor inicial
        cotas_iniciales = np.array([self.cota_inicial/1000.0] * 6)
        for i, cota in enumerate(self.NOMBRES_COTAS):
            param = self.model.Parameter(cota)
            if param:
                param.SetSystemValue3(cotas_iniciales[i], 1, None)
            else:
                print(f"Error: No se encontró la cota '{cota}' en SolidWorks durante el reseteo de cotas.")
        self.model.ForceRebuild3(False)

        total = len(rango_grados) ** 2
        procesados = 0

        for phi_g in rango_grados:
            for theta_g in rango_grados:
                procesados += 1
                phi = np.radians(phi_g)
                theta = np.radians(theta_g)

                delta_cotas_sw = self.cinematica_inversa(phi, theta)

                # El cálculo de longitud real es: cota_sw(m) * 1000 - cota_inicial + L_inicial
                # Pero para comparar con L_MIN/L_MAX usamos directamente delta_cotas en mm:
                longitudes_mm = delta_cotas_sw * 1000.0

                # Comprobar límites de carrera
                if np.any(longitudes_mm < L_MIN) or np.any(longitudes_mm > L_MAX):
                    resultados.append([phi_g, theta_g, 0])
                    print(f"[{procesados}/{total}] [{phi_g}º, {theta_g}º] -> FUERA DE CARRERA")
                    continue

                # Inyectar cotas en SolidWorks
                for i, cota in enumerate(self.NOMBRES_COTAS):
                    param = self.model.Parameter(cota)
                    if param:
                        param.SetSystemValue3(delta_cotas_sw[i], 1, None)

                # Reconstrucción doble para asentar el ensamblaje
                self.model.ForceRebuild3(False)
                self.model.ForceRebuild3(False)

                # Comprobar errores críticos de geometría
                errores_geometria = self.tiene_errores_geometria()

                if not errores_geometria:
                    resultados.append([phi_g, theta_g, 2])
                    print(f"[{procesados}/{total}] [{phi_g}º, {theta_g}º] -> OK")
                else:
                    resultados.append([phi_g, theta_g, 1])
                    print(f"[{procesados}/{total}] [{phi_g}º, {theta_g}º] -> ERROR CAD")

        # Restaurar posición inicial al terminar
        for i, cota in enumerate(self.NOMBRES_COTAS):
            param = self.model.Parameter(cota)
            if param:
                param.SetSystemValue3(cotas_iniciales[i], 1, None)
        self.model.ForceRebuild3(False)

        print("Escaneo completado. Generando gráfico...")
        self.graficar_resultados(np.array(resultados))

    def graficar_resultados(self, matriz_resultados):
        

        fallo_carrera = matriz_resultados[matriz_resultados[:, 2] == 0]
        fallo_cad     = matriz_resultados[matriz_resultados[:, 2] == 1]
        exito         = matriz_resultados[matriz_resultados[:, 2] == 2]

        plt.figure(figsize=(8, 6))

        if len(fallo_carrera): plt.scatter(fallo_carrera[:, 0], fallo_carrera[:, 1], c='red',    marker='x', s=80, label='Límite de Actuadores')
        if len(fallo_cad):     plt.scatter(fallo_cad[:, 0],     fallo_cad[:, 1],     c='orange', marker='s', s=80, label='Colisión / Error CAD')
        if len(exito):         plt.scatter(exito[:, 0],         exito[:, 1],         c='green',  marker='o', s=80, label='Posición Válida')

        # Trayectorias solares
        lat = np.radians(39.866)
        horas = np.arange(-12, 12.25, 0.25)
        angulos_horarios = np.radians(15) * horas

        for dia, nombre, color in zip(
            [172, 81, 355],                                              
            ["Solsticio verano", "Equinoccio", "Solsticio invierno"],
            ["blue", "green", "red"]
        ):
            
            dec = np.radians(23.44 * np.sin(np.radians((360/365) * (dia - 81))))

            ang_zenital = np.arccos(
                np.sin(dec) * np.sin(lat) +
                np.cos(dec) * np.cos(lat) * np.cos(angulos_horarios)
            )

            
            with np.errstate(invalid='ignore', divide='ignore'):
                cos_acimutal = np.clip(
                    (np.sin(dec) - np.cos(ang_zenital) * np.sin(lat)) /
                    (np.sin(ang_zenital) * np.cos(lat)),
                    -1, 1
                )
            ang_acimutal = np.arccos(cos_acimutal)

            zenital_deg  = np.degrees(ang_zenital)
            acimutal_deg = np.degrees(ang_acimutal)

            # CORRECCIÓN 4: usar angulos_horarios en lugar de horas para la corrección tarde/mañana
            acimutal_deg = np.where(angulos_horarios > 0, 360 - acimutal_deg, acimutal_deg)
            acimutal_deg = acimutal_deg - 180

            sobre_horizonte = zenital_deg < 90
            phi_solar   = acimutal_deg[sobre_horizonte]
            theta_solar = zenital_deg[sobre_horizonte]

            plt.plot(phi_solar, theta_solar, color=color, lw=2, label=nombre)

        plt.title("Espacio de Trabajo Angular (Plataforma Stewart)")
        plt.xlabel("Ángulo Phi / Acimutal (grados)")
        plt.ylabel("Ángulo Theta / Cenital (grados)")
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.legend()
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    sim = SimuladorStewart()
    #sim.ejecutar_simulacion()
    sim.mapear_espacio_trabajo()