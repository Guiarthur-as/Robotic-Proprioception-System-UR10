"""app1_freio.py — App 1: Freio Autônomo de Singularidade (proteção escalar).

O robô se move por speedL em direção à singularidade de COTOVELO (braço esticando,
junta do cotovelo -> 0) e freia uniformemente conforme o índice de manipulabilidade
de Yoshikawa w = sqrt(det(J*J^T)) cai abaixo de um limiar seguro:

    fator = min(1, w / W_MIN)
    v_real = v_comandada * fator

Longe da singularidade -> fator=1 (velocidade integral); perto -> fator<1
(velocidade proporcional à destreza); na singularidade -> fator->0 (para suave).
Abaixo de W_MIN_CRITICO aciona speedStop (parada de proteção controlada).

Loga em data/app1_run_<timestamp>.csv para os gráficos da Phase 5.

Pré-requisito: URSim em 192.168.2.103 em modo Remote Control / Play.
Executar: uv run python app1_freio.py   (Ctrl+C para abortar com parada segura)
"""

import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np

import core

# --- Parâmetros (sementes validadas offline; calibrar conforme o comportamento real) ---
START_Q = [0.0, -1.0, 1.2, -1.57, -1.57, 0.0]   # postura inicial: cotovelo dobrado, w~0.33
STRETCH_DIR = np.array([-0.92, -0.23, -0.318, 0.0, 0.0, 0.0])  # outward (ombro->TCP): estica o cotovelo
VEL_NOMINAL = 0.05      # m/s — velocidade comandada (baixa por segurança)
ACCEL = 0.25            # m/s^2
DT = 0.05               # s (loop ~20 Hz; speedL retorna após DT, pacing embutido)
# Freio começa CEDO: a banda w=0.05->0.015 é curtíssima em ângulo de cotovelo
# (o robô a cruza rápido e o controlador desarma). Com W_MIN alto há distância de frenagem.
W_MIN = 0.20            # limiar de início do freio
W_MIN_CRITICO = 0.05    # abaixo disto: parada de proteção (margem antes do desarme do controlador)


def main(ip=core.IP_ROBO):
    direcao = STRETCH_DIR / np.linalg.norm(STRETCH_DIR[0:3])  # normaliza a parte linear

    print(f"🔄 Conectando ao URSim em {ip}...")
    rtde_r, rtde_c = core.conectar(ip)
    print("✅ Conectado. Posicionando na postura inicial...")
    rtde_c.moveJ(START_Q, 0.5, 0.5)

    Path("data").mkdir(exist_ok=True)
    arquivo = Path("data") / f"app1_run_{datetime.now():%Y%m%d_%H%M%S}.csv"
    f = open(arquivo, "w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow(["t", "q0", "q1", "q2", "q3", "q4", "q5", "w", "fator", "v_cmd", "v_real"])

    t0 = time.time()
    try:
        while True:
            loop_ini = time.time()
            q = rtde_r.getActualQ()
            J = core.calcular_jacobiano_ur10(q)
            w = core.manipulabilidade(J)
            fator = core.fator_escala(w, W_MIN)

            v_cmd = direcao * VEL_NOMINAL
            v_real = v_cmd * fator
            t = time.time() - t0
            writer.writerow([f"{t:.3f}", *[f"{a:.5f}" for a in q],
                             f"{w:.5f}", f"{fator:.4f}",
                             f"{VEL_NOMINAL:.5f}", f"{VEL_NOMINAL*fator:.5f}"])

            if w < W_MIN_CRITICO:
                print(f"🚨 Singularidade crítica (w={w:.5f} < {W_MIN_CRITICO}). "
                      f"Parada de proteção controlada (margem antes do desarme).")
                rtde_c.speedStop(2.0)
                break

            estado = "🟢 livre " if fator >= 0.999 else "🟡 freando"
            print(f"{estado} | w={w:.4f} fator={fator:.3f} "
                  f"v_real={VEL_NOMINAL*fator:.4f} m/s q_cotovelo={q[2]:.3f}")
            ok = rtde_c.speedL(list(v_real), ACCEL)  # streaming contínuo (time=0)
            if not ok:
                print("⚠️  Control script do RTDE parou (provável desarme do controlador). "
                      "Encerrando com segurança.")
                break
            dorme = DT - (time.time() - loop_ini)    # pacing ~20 Hz
            if dorme > 0:
                time.sleep(dorme)

    except KeyboardInterrupt:
        print("\n🛑 Abortado manualmente.")
    finally:
        try:
            rtde_c.speedStop(2.0)
        except Exception as e:
            print(f"(speedStop ignorado: {e})")
        rtde_r.disconnect()
        rtde_c.disconnect()
        f.close()
        print(f"🔌 Conexões encerradas. Log salvo em {arquivo}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="App 1 — Freio autônomo de singularidade")
    parser.add_argument("--ip", default=core.IP_ROBO, help="IP do URSim (padrão: %(default)s)")
    main(parser.parse_args().ip)
