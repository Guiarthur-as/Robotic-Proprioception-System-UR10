"""validate_jacobian.py — Spike de validação do Jacobiano geométrico (Phase 2).

Valida a matemática de core.py em duas camadas:

  OFFLINE (não precisa do robô — roda em qualquer lugar):
    1. Bloco linear de J vs. diferenças finitas da FK (erro < 1e-4).
    2. Sanidade de singularidade: |det(J)| pequeno em pose de braço esticado.

  ONLINE (precisa de URSim em 192.168.2.103, modo Remote/Play):
    3. J manual vs. controlador: getJacobian(q) (remodelado 6x6, convenção alinhada).
    4. FK em Python vs. getForwardKinematics(q) e vs. getActualTCPPose().
       getForwardKinematics(q)/getJacobian(q) aceitam q arbitrário e NÃO movem o robô.

Uso:
    uv run python scripts/validate_jacobian.py            # offline + tenta online
    uv run python scripts/validate_jacobian.py --offline  # só offline
"""

import argparse
import sys

import numpy as np

sys.path.insert(0, ".")
import core  # noqa: E402

# Poses de teste (radianos). q_home = postura não-singular; q_esticado ~ singularidade.
Q_HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
Q_ESTICADO = [0.0, -0.1, 0.1, -1.57, -1.57, 0.0]
Q_TESTE = [
    Q_HOME,
    Q_ESTICADO,
    [0.5, -1.2, 1.0, -1.0, -1.2, 0.3],
    [-0.7, -2.0, 1.8, -0.5, 1.0, -0.4],
]


def jacobiano_geometrico_diferencas_finitas(q, eps=1e-6):
    """Jacobiano geométrico 6x6 por diferenças finitas da FK (linear + angular).

    Bloco linear: derivada da posição do efetor.
    Bloco angular: ω por coluna, extraída de ω̂ = Ṙ·Rᵀ (frame base).
    """
    q = np.asarray(q, dtype=float)
    T0 = core.fk_pose(q)
    p0, R0 = T0[0:3, 3], T0[0:3, 0:3]
    J = np.zeros((6, 6))
    for i in range(6):
        qd = q.copy()
        qd[i] += eps
        Td = core.fk_pose(qd)
        J[0:3, i] = (Td[0:3, 3] - p0) / eps          # parte linear
        dR = (Td[0:3, 0:3] - R0) / eps
        W = dR @ R0.T                                  # ω̂ (matriz antissimétrica)
        J[3:6, i] = [W[2, 1], W[0, 2], W[1, 0]]        # parte angular (ω)
    return J


def checar_offline():
    print("=== OFFLINE ===")
    ok = True

    # 1. Jacobiano completo (linear + angular) vs. diferenças finitas
    max_err = 0.0
    for q in Q_TESTE:
        J = core.calcular_jacobiano_ur10(q)
        J_fd = jacobiano_geometrico_diferencas_finitas(q)
        err = np.max(np.abs(J - J_fd))
        max_err = max(max_err, err)
    passou = max_err < 1e-4
    ok &= passou
    print(f"[{'PASS' if passou else 'FAIL'}] Jacobiano 6x6 (linear+angular) vs. "
          f"diferenças finitas: erro máx = {max_err:.2e} (alvo < 1e-4)")

    # 2. Sanidade da singularidade
    w_home = core.manipulabilidade(core.calcular_jacobiano_ur10(Q_HOME))
    w_estic = core.manipulabilidade(core.calcular_jacobiano_ur10(Q_ESTICADO))
    passou = w_estic < w_home
    ok &= passou
    print(f"[{'PASS' if passou else 'FAIL'}] Singularidade: w(home)={w_home:.4f} > "
          f"w(esticado)={w_estic:.4f}")

    # Métricas auxiliares (decomposição espectral)
    eigvals, _ = core.decomposicao_espectral(core.calcular_jacobiano_ur10(Q_HOME))
    print(f"       autovalores(home) = {np.array2string(eigvals, precision=4)}")
    return ok


def checar_online(ip=core.IP_ROBO):
    print(f"=== ONLINE (URSim @ {ip}) ===")
    try:
        rtde_r, rtde_c = core.conectar(ip)
    except Exception as e:
        print(f"[SKIP] Sem conexão com URSim ({e!r}). "
              f"Rode com o simulador em Remote/Play para validar contra o controlador.")
        return None

    ok = True
    try:
        for q in Q_TESTE:
            # FK em Python vs. getForwardKinematics(q) (não move o robô)
            pose_ctrl = np.array(rtde_c.getForwardKinematics(q))
            p_py = core.fk_pose(q)[0:3, 3]
            err_fk = np.max(np.abs(p_py - pose_ctrl[0:3]))
            passou = err_fk < 5e-3
            ok &= passou
            print(f"[{'PASS' if passou else 'FAIL'}] FK pos vs. getForwardKinematics: "
                  f"erro = {err_fk*1000:.2f} mm (alvo < 5 mm)")

            # J manual vs. getJacobian(q) — oráculo opcional.
            # NOTA: no URSim o getJacobian costuma falhar com erro de output
            # double register (recipe RTDE) — limitação da lib, não da nossa matemática.
            # O Jacobiano já é validado offline (linear+angular vs. dif. finitas) e a FK
            # bate com getForwardKinematics, então esta checagem é só um bônus.
            J_py = core.calcular_jacobiano_ur10(q)
            try:
                J_ctrl = np.array(rtde_c.getJacobian(q)).reshape(6, 6)
            except Exception as e:
                print(f"[SKIP] getJacobian indisponível no URSim ({e!r}); "
                      f"Jacobiano já validado offline.")
                continue
            err_J = np.max(np.abs(J_py - J_ctrl))
            passou = err_J < 1e-3
            ok &= passou
            print(f"[{'PASS' if passou else 'FAIL'}] J manual vs. getJacobian: "
                  f"erro máx = {err_J:.2e} (alvo < 1e-3) "
                  f"{'' if passou else '<- conferir frame/ordem linear-angular do getJacobian'}")
    finally:
        rtde_r.disconnect()
        rtde_c.disconnect()
    return ok


def main():
    parser = argparse.ArgumentParser(description="Validação do Jacobiano geométrico do UR10")
    parser.add_argument("--offline", action="store_true", help="roda apenas as checagens offline")
    parser.add_argument("--ip", default=core.IP_ROBO, help="IP do URSim (padrão: %(default)s)")
    args = parser.parse_args()

    off = checar_offline()
    on = None if args.offline else checar_online(args.ip)

    print("\n=== RESUMO ===")
    print(f"Offline: {'PASS' if off else 'FAIL'}")
    if on is None:
        print("Online:  SKIP (sem URSim) — rode com o simulador para fechar a Phase 2.")
    else:
        print(f"Online:  {'PASS' if on else 'FAIL'}")
    sys.exit(0 if off and (on is None or on) else 1)


if __name__ == "__main__":
    main()
