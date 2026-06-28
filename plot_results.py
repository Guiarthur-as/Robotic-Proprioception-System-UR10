"""plot_results.py — Geração dos gráficos a partir dos CSVs logados.

Detecta automaticamente o tipo de log pelas colunas e gera os PNGs do relatório:
  - App 1 (coluna 'w'): w(t) com limiares + v_comandada vs v_real.
  - App 2 (colunas 'eig0'..): autovalores(t) + v_comandada vs v_real por componente.

Roda offline (não conecta ao robô).

Uso:
    uv run python plot_results.py data/app1_run_<timestamp>.csv
    uv run python plot_results.py data/app2_run_<timestamp>.csv
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # backend sem display (salva PNG)
import matplotlib.pyplot as plt  # noqa: E402

# Limiares da App 1 (espelham app1_freio.py) para anotar no gráfico de w.
W_MIN = 0.20
W_MIN_CRITICO = 0.05

# Limiar da App 2 (espelha app2_teleop.py) e rótulos dos eixos translacionais.
LIMIAR_APP2 = 0.30
EIXOS = ["X", "Y", "Z"]


def ler_csv(caminho):
    with open(caminho, newline="", encoding="utf-8") as f:
        linhas = list(csv.DictReader(f))
    if not linhas:
        raise SystemExit(f"CSV vazio: {caminho}")
    colunas = linhas[0].keys()
    dados = {c: [float(r[c]) for r in linhas] for c in colunas}
    return dados


def plot_app1(dados, destino):
    t = dados["t"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    ax1.plot(t, dados["w"], color="tab:blue", label="w = √det(J·Jᵀ)")
    ax1.axhline(W_MIN, color="tab:orange", ls="--", label=f"W_MIN = {W_MIN}")
    ax1.axhline(W_MIN_CRITICO, color="tab:red", ls="--", label=f"W_MIN_CRÍTICO = {W_MIN_CRITICO}")
    ax1.set_ylabel("Manipulabilidade w")
    ax1.set_title("App 1 — Freio Autônomo: índice de manipulabilidade ao longo do tempo")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, dados["v_cmd"], color="tab:gray", ls="--", label="v comandada")
    ax2.plot(t, dados["v_real"], color="tab:green", label="v real (atenuada pelo freio)")
    ax2.set_xlabel("tempo (s)")
    ax2.set_ylabel("velocidade (m/s)")
    ax2.set_title("Velocidade comandada vs. real")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(destino, dpi=120)
    print(f"[OK] Grafico App 1 salvo em {destino}")


def plot_app2(dados, destino):
    t = dados["t"]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # Topo: só os 3 MENORES autovalores (as direções fracas, que importam) + limiar.
    # Os autovalores grandes (~4) achatariam os pequenos perto de zero se plotados juntos.
    for i in range(3):
        ax1.plot(t, dados[f"eig{i}"], label=f"λ{i}")
    ax1.axhline(LIMIAR_APP2, color="tab:red", ls="--", label=f"LIMIAR = {LIMIAR_APP2}")
    ax1.set_ylabel("autovalores de J·Jᵀ")
    ax1.set_title("App 2 — autovalores mais fracos (direções vulneráveis) ao longo do tempo")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Fundo: só os eixos translacionais X/Y/Z, comandado (tracejado) vs real (sólido),
    # com cores pareadas por eixo (componentes angulares e o ruído são omitidos).
    cores = ["tab:blue", "tab:green", "tab:orange"]
    for i, (eixo, cor) in enumerate(zip(EIXOS, cores)):
        ax2.plot(t, dados[f"vreal{i}"], color=cor, label=f"{eixo} real")
        ax2.plot(t, dados[f"vcmd{i}"], color=cor, ls="--", alpha=0.5, label=f"{eixo} cmd")
    ax2.set_xlabel("tempo (s)")
    ax2.set_ylabel("velocidade (m/s)")
    ax2.set_title("Velocidade comandada (tracejado) vs. real (sólido) por eixo")
    ax2.legend(loc="upper right", ncol=3, fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(destino, dpi=120)
    print(f"[OK] Grafico App 2 salvo em {destino}")


def main():
    if len(sys.argv) < 2:
        raise SystemExit("Uso: uv run python plot_results.py data/<arquivo>.csv")
    caminho = Path(sys.argv[1])
    dados = ler_csv(caminho)
    destino = caminho.with_suffix(".png")

    if "w" in dados:
        plot_app1(dados, destino)
    elif any(c.startswith("eig") for c in dados):
        plot_app2(dados, destino)
    else:
        raise SystemExit(f"Formato de CSV não reconhecido (colunas: {list(dados)})")


if __name__ == "__main__":
    main()
