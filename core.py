"""core.py — Módulo compartilhado de propriocepção do UR10.

Centraliza a matemática do Jacobiano GEOMÉTRICO e a camada de conexão RTDE
usadas pelas duas aplicações (App 1 freio escalar e App 2 teleop direcional).

Stack: ur-rtde (rtde_control / rtde_receive) + NumPy. O Jacobiano geométrico 6x6
é calculado em Python a partir dos parâmetros DH do UR10 (parte linear via produto
vetorial, parte angular = eixos z das juntas), conforme exemplo.py.

Terminologia: este é o Jacobiano GEOMÉTRICO (parte angular = velocidade angular
real ω, dada pelos eixos z das juntas no frame base), NÃO o Jacobiano analítico.

Convenção do Jacobiano (linhas):
    J[0:3, :] -> velocidade linear do efetor [m/s] (frame base)
    J[3:6, :] -> velocidade angular do efetor [rad/s] (frame base)
NOTA: a convenção/ordem do getJacobian do controlador deve ser confirmada contra
esta na validação (scripts/validate_jacobian.py) — ver comentário em validacao.

API pública (Inter-Phase Contracts do plano):
    calcular_jacobiano_ur10(q) -> np.ndarray (6, 6)
    manipulabilidade(J) -> float
    decomposicao_espectral(J) -> (eigvals (6,), V (6, 6))
    fator_escala(w, w_min) -> float
    filtra_velocidade_direcional(J, v_cmd, limiar) -> np.ndarray (6,)
    conectar(ip) -> (rtde_receive, rtde_control)
"""

import numpy as np

IP_ROBO = "192.168.2.103"  # IP fixo da VM URSim (ver exemplo.py)

# Parâmetros físicos oficiais do UR10 — [d, a, alpha] por junta (metros, rad).
DH_UR10 = [
    [0.1273, 0.0, np.pi / 2],   # Junta 1 (Base)
    [0.0, -0.612, 0.0],         # Junta 2 (Ombro)
    [0.0, -0.5723, 0.0],        # Junta 3 (Cotovelo)
    [0.1639, 0.0, np.pi / 2],   # Junta 4 (Pulso 1)
    [0.1157, 0.0, -np.pi / 2],  # Junta 5 (Pulso 2)
    [0.0922, 0.0, 0.0],         # Junta 6 (Pulso 3 / Ferramenta)
]


def matriz_dh(theta, d, a, alpha):
    """Matriz de transformação homogênea local via parâmetros de Denavit-Hartenberg."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                  np.cos(alpha),                 d],
        [0,              0,                              0,                             1],
    ])


def frames_ur10(q):
    """Retorna a lista de transformações acumuladas [T0(=I), T1, ..., T6] a partir da base."""
    T = [np.eye(4)]
    T_atual = np.eye(4)
    for i in range(6):
        A = matriz_dh(q[i], DH_UR10[i][0], DH_UR10[i][1], DH_UR10[i][2])
        T_atual = T_atual @ A
        T.append(T_atual)
    return T


def fk_pose(q):
    """Cinemática direta: matriz 4x4 do efetor no frame base para as juntas q."""
    return frames_ur10(q)[-1]


def calcular_jacobiano_ur10(q):
    """Jacobiano geométrico 6x6 do UR10 no frame base para as juntas atuais q."""
    T = frames_ur10(q)
    p_e = T[-1][0:3, 3]  # posição do efetor
    J = np.zeros((6, 6))
    for i in range(6):
        z_im1 = T[i][0:3, 2]  # eixo z da junta i-1
        p_im1 = T[i][0:3, 3]  # origem da junta i-1
        J[0:3, i] = np.cross(z_im1, (p_e - p_im1))  # parte linear
        J[3:6, i] = z_im1                            # parte angular
    return J


def manipulabilidade(J):
    """Índice de manipulabilidade de Yoshikawa: w = sqrt(det(J @ J.T))."""
    det = np.linalg.det(J @ J.T)
    return float(np.sqrt(max(det, 0.0)))


def decomposicao_espectral(J):
    """Decomposição espectral de J @ J.T.

    Retorna (autovalores, autovetores) de eigh: autovalores em ordem crescente,
    autovetores nas colunas. Autovalores negativos por ruído numérico são zerados.
    """
    eigvals, V = np.linalg.eigh(J @ J.T)
    eigvals = np.clip(eigvals, 0.0, None)
    return eigvals, V


def fator_escala(w, w_min):
    """Fator de atenuação escalar uniforme (App 1): min(1, w / w_min)."""
    if w_min <= 0:
        return 1.0
    return float(min(1.0, w / w_min))


def filtra_velocidade_direcional(J, v_cmd, limiar):
    """Filtro direcional (App 2): atenua apenas as componentes fracas do comando.

    Projeta v_cmd (6,) nos autovetores de J@J.T e escala cada componente cuja
    direção é fraca (autovalor < limiar) pelo fator autovalor/limiar, preservando
    as direções fortes. Reconstrói e retorna a velocidade filtrada (6,).
    """
    v_cmd = np.asarray(v_cmd, dtype=float)
    eigvals, V = decomposicao_espectral(J)
    proj = V.T @ v_cmd
    if limiar > 0:
        fracas = eigvals < limiar
        proj[fracas] *= eigvals[fracas] / limiar
    return V @ proj


def conectar(ip=IP_ROBO):
    """Conecta ao URSim e retorna (rtde_receive, rtde_control).

    O chamador é responsável por chamar .disconnect() em ambos (use try/finally).
    Requer URSim em modo Remote Control / Play para o controle aceitar comandos.
    """
    import rtde_control
    import rtde_receive
    rtde_r = rtde_receive.RTDEReceiveInterface(ip)
    rtde_c = rtde_control.RTDEControlInterface(ip)
    return rtde_r, rtde_c
