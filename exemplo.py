import time
import numpy as np
import rtde_control
import rtde_receive

IP_ROBO = "192.168.2.103"

def matriz_dh(theta, d, a, alpha):
    """Calcula a matriz de transformação local usando parâmetros de Denavit-Hartenberg."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),              d],
        [0,              0,                           0,                          1]
    ])

def calcular_jacobiano_ur10(q):
    """Calcula o Jacobiano Geométrico 6x6 exato para o UR10 com base nas juntas atuais."""
    # Parâmetros físicos oficiais do UR10 (Distâncias dos braços em metros: d, a, alpha)
    dh = [
        [0.1273, 0, np.pi/2],        # Junta 1 (Base)
        [0, -0.612, 0],             # Junta 2 (Ombro)
        [0, -0.5723, 0],            # Junta 3 (Cotovelo)
        [0.1639, 0, np.pi/2],       # Junta 4 (Pulso 1)
        [0.1157, 0, -np.pi/2],      # Junta 5 (Pulso 2)
        [0.0922, 0, 0]              # Junta 6 (Pulso 3/Ferramenta)
    ]
    
    # Matrizes de transformação acumuladas a partir da base (T0)
    T = [np.eye(4)]
    T_atual = np.eye(4)
    
    for i in range(6):
        A = matriz_dh(q[i], dh[i][0], dh[i][1], dh[i][2])
        T_atual = np.dot(T_atual, A)
        T.append(T_atual)
        
    p_e = T[-1][0:3, 3] # Posição da ponta do robô (efetuador)
    J = np.zeros((6, 6))
    
    # Construção geométrica coluna por coluna
    for i in range(6):
        z_im1 = T[i][0:3, 2] # Eixo Z da junta anterior
        p_im1 = T[i][0:3, 3] # Posição da junta anterior
        
        J[0:3, i] = np.cross(z_im1, (p_e - p_im1)) # Parte Linear
        J[3:6, i] = z_im1                          # Parte Angular
        
    return J

def main():
    print("🔄 Conectando ao simulador URSim...")
    try:
        leitor = rtde_receive.RTDEReceiveInterface(IP_ROBO)
        controlador = rtde_control.RTDEControlInterface(IP_ROBO)
        print("✅ Conectado com sucesso! Iniciando monitoramento ativo...")
    except Exception as e:
        print(f"❌ Falha na conexão: {e}")
        return

    LIMITE_SINGULARIDADE = 0.015
    FREQUENCIA_HZ = 0.1

    try:
        while True:
            # 1. Lê os ângulos reais das juntas atuais (em radianos)
            q_atual = leitor.getActualQ()
            
            # 2. Calcula a matriz Jacobiana usando nossa função matemática
            J = calcular_jacobiano_ur10(q_atual)
            
            # 3. Calcula o determinante para monitorar a estabilidade
            determinante = np.abs(np.linalg.det(J))
            
            # 4. Avalia a condição de segurança
            if determinante < LIMITE_SINGULARIDADE:
                print(f"🚨 PERIGO DETECTADO! Determinante crítico: {determinante:.5f}")
                print("🛑 Acionando parada de emergência controlada...")
                controlador.speedStop(2.0)
                break
            else:
                print(f"🟢 Sistema Seguro | Métrica de Manipulabilidade: {determinante:.4f}")

            time.sleep(FREQUENCIA_HZ)

    except KeyboardInterrupt:
        print("\n🛑 Monitoramento encerrado manualmente.")
    finally:
        leitor.disconnect()
        controlador.disconnect()
        print("🔌 Conexões com o URSim encerradas.")

if __name__ == "__main__":
    main()
