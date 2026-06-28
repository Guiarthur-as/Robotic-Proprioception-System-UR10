"""app2_teleop.py — App 2: Teleoperação com Consciência Direcional (proteção vetorial).

O operador comanda velocidade cartesiana pelo teclado (pynput). O comando é
projetado nos autovetores de J*J^T (decomposição espectral) e apenas as
componentes nas direções FRACAS (autovalor < LIMIAR) são atenuadas, preservando
a mobilidade nas direções fortes:

    autovalores, V = eigh(J @ J.T)
    proj = V.T @ v_cmd
    proj[i] *= autovalores[i]/LIMIAR   se autovalores[i] < LIMIAR
    v_real = V @ proj

Resultado: na mesma postura, o robô resiste numa direção fraca e obedece numa
direção forte. Loga em data/app2_run_<timestamp>.csv para os gráficos da Phase 5.

Controles:  W/S = ±X   A/D = ±Y   Q/E = ±Z   Esc = sair
(mantenha o terminal em foco para a captura de teclas)

Pré-requisito: URSim em 192.168.2.103 em modo Remote Control / Play.
Executar: uv run python app2_teleop.py
"""

import csv
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from pynput import keyboard

import core

# --- Parâmetros (semente; calibrar LIMIAR conforme a postura) ---
START_Q = [0.0, -1.0, 0.6, -1.4, -1.57, 0.0]  # postura com forte anisotropia (eig_min~0.02)
VEL = 0.05      # m/s por eixo comandado
ACCEL = 0.25    # m/s^2
DT = 0.05       # s (loop ~20 Hz)
LIMIAR = 0.30   # autovalores abaixo disto são considerados direções fracas

# Mapeamento tecla -> (índice do eixo cartesiano, sinal)
MAP = {"w": (0, +1), "s": (0, -1),   # X
       "a": (1, +1), "d": (1, -1),   # Y
       "q": (2, +1), "e": (2, -1)}   # Z

_pressed = set()
_parar = threading.Event()


def on_press(key):
    try:
        c = key.char.lower()
        if c in MAP:
            _pressed.add(c)
    except AttributeError:
        if key == keyboard.Key.esc:
            _parar.set()


def on_release(key):
    try:
        _pressed.discard(key.char.lower())
    except AttributeError:
        pass


def montar_vcmd():
    """Monta o vetor de velocidade cartesiana (6,) a partir das teclas pressionadas."""
    v = np.zeros(6)
    for c in list(_pressed):
        i, s = MAP[c]
        v[i] += s * VEL
    return v


def main(ip=core.IP_ROBO):
    print(f"Conectando ao URSim em {ip}...")
    rtde_r, rtde_c = core.conectar(ip)
    print("Conectado. Posicionando na postura inicial...")
    rtde_c.moveJ(START_Q, 0.5, 0.5)
    print("Controles: W/S=+-X  A/D=+-Y  Q/E=+-Z   Esc=sair (terminal em foco)")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    Path("data").mkdir(exist_ok=True)
    arquivo = Path("data") / f"app2_run_{datetime.now():%Y%m%d_%H%M%S}.csv"
    f = open(arquivo, "w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(["t"] + [f"q{i}" for i in range(6)] + [f"eig{i}" for i in range(6)]
                    + [f"vcmd{i}" for i in range(6)] + [f"vreal{i}" for i in range(6)])

    t0 = time.time()
    try:
        while not _parar.is_set():
            loop_ini = time.time()
            q = rtde_r.getActualQ()
            J = core.calcular_jacobiano_ur10(q)
            eig, _ = core.decomposicao_espectral(J)
            v_cmd = montar_vcmd()
            v_real = core.filtra_velocidade_direcional(J, v_cmd, LIMIAR)

            t = time.time() - t0
            writer.writerow([f"{t:.3f}"] + [f"{a:.5f}" for a in q]
                            + [f"{e:.5f}" for e in eig]
                            + [f"{a:.5f}" for a in v_cmd]
                            + [f"{a:.5f}" for a in v_real])

            ok = rtde_c.speedL(list(v_real), ACCEL)
            if not ok:
                print("Control script do RTDE parou; encerrando com seguranca.")
                break

            if np.any(v_cmd):
                atenuacao = (np.linalg.norm(v_real[:3]) / np.linalg.norm(v_cmd[:3])
                             if np.linalg.norm(v_cmd[:3]) > 0 else 1.0)
                print(f"cmd={np.round(v_cmd[:3], 3)} -> real={np.round(v_real[:3], 3)} "
                      f"| retencao={atenuacao:.2f} eig_min={eig[0]:.3f}")

            dorme = DT - (time.time() - loop_ini)
            if dorme > 0:
                time.sleep(dorme)

    except KeyboardInterrupt:
        print("\nAbortado manualmente.")
    finally:
        try:
            rtde_c.speedStop(2.0)
        except Exception as e:
            print(f"(speedStop ignorado: {e})")
        listener.stop()
        rtde_r.disconnect()
        rtde_c.disconnect()
        f.close()
        print(f"Conexoes encerradas. Log salvo em {arquivo}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="App 2 — Teleoperação com freio direcional")
    parser.add_argument("--ip", default=core.IP_ROBO, help="IP do URSim (padrão: %(default)s)")
    main(parser.parse_args().ip)
