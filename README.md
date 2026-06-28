# Robotic-Proprioception-System-UR10

Sistema de **propriocepção artificial** para o manipulador UR10: um sentido interno e
contínuo que permite ao robô perceber sua própria destreza postural e reagir **antes**
que uma singularidade provoque o desarme de proteção do controlador.

O projeto explora as propriedades do **Jacobiano Geométrico** em duas aplicações:

- **App 1 — Freio Autônomo (proteção escalar):** monitora o índice de manipulabilidade
  de Yoshikawa `w = √det(J·Jᵀ)` e reduz a velocidade *uniformemente* conforme o robô se
  aproxima de uma singularidade de cotovelo, parando suavemente antes do desarme.
- **App 2 — Teleoperação Direcional (proteção vetorial):** usa a decomposição espectral
  de `J·Jᵀ` para identificar *quais direções* estão fracas e atenuar *apenas essas*,
  preservando a mobilidade nas direções seguras durante o controle por teclado.

> **Stack de comunicação:** este projeto usa a biblioteca **`ur-rtde`** (RTDE — Real-Time
> Data Exchange) conectando direto ao URSim por IP, e calcula o **Jacobiano geométrico em
> Python** a partir dos parâmetros DH do UR10. Não usa RoboDK nem URScript via socket.

---

## Pré-requisitos

- **Python 3.12+** e **[uv](https://docs.astral.sh/uv/)** (gerenciador de ambiente/deps).
- **URSim** (simulador oficial da Universal Robots para UR10) rodando em uma VM/Docker,
  acessível pela rede a partir deste computador.
- O URSim deve estar **ligado, com um programa em execução (Play) e em modo Remote
  Control** — caso contrário a `RTDEControlInterface` não aceita comandos de movimento.

### Rede / portas

| Item | Valor |
|---|---|
| IP do robô (URSim) | `192.168.2.103` (fixo — definido em `core.py` como `IP_ROBO`) |
| Porta RTDE | `30004` (gerenciada internamente pela `ur-rtde`) |
| Porta Dashboard | `29999` (não usada pelo fluxo principal) |

> Se o IP da sua VM for diferente, passe `--ip <endereço>` para qualquer script (App 1,
> App 2 ou a validação), ou ajuste o padrão `IP_ROBO` em [`core.py`](core.py).
> Ex.: `uv run python app1_freio.py --ip 192.168.0.50`

---

## Instalação

```bash
uv sync
```

Isso instala as dependências declaradas em `pyproject.toml`: `numpy`, `ur-rtde`,
`matplotlib`, `pynput`.

---

## Como executar

### 1. Validar o Jacobiano (recomendado antes de tudo)

Compara o Jacobiano e a cinemática calculados em Python contra a cinemática calibrada do
controlador. **Não move o robô.**

```bash
uv run python scripts/validate_jacobian.py            # offline + online (se URSim disponível)
uv run python scripts/validate_jacobian.py --offline  # apenas offline
```

Esperado: Jacobiano (linear+angular) vs. diferenças finitas com erro ~1e-7; FK em Python
vs. `getForwardKinematics` < 5 mm.
(`getJacobian` do controlador costuma ficar indisponível no URSim por limitação de
registers do RTDE — o script trata isso como SKIP; a validação offline já é suficiente.)

### 2. App 1 — Freio autônomo de singularidade

```bash
uv run python app1_freio.py
```

O robô vai à postura inicial, estica o braço em direção à singularidade de cotovelo e
**freia sozinho**, parando antes do desarme. Gera `data/app1_run_<timestamp>.csv`.
`Ctrl+C` aborta com parada segura.

### 3. App 2 — Teleoperação direcional

```bash
uv run python app2_teleop.py
```

Controles (mantenha o terminal em foco): `W/S = ±X`, `A/D = ±Y`, `Q/E = ±Z`, `Esc = sair`.
Na postura inicial, mover em Y é livre, mas mover em X/Z é resistido (direções fracas).
Gera `data/app2_run_<timestamp>.csv`.

### 4. Gerar gráficos

```bash
uv run python plot_results.py data/app1_run_<timestamp>.csv
uv run python plot_results.py data/app2_run_<timestamp>.csv
```

Salva um PNG ao lado do CSV (detecta App 1 ou App 2 automaticamente pelas colunas).

---

## Estrutura do projeto

```
core.py                 # Jacobiano geométrico (DH), métricas, filtro espectral, conexão RTDE
app1_freio.py           # App 1: freio autônomo escalar (speedL + w)
app2_teleop.py          # App 2: teleoperação direcional (pynput + decomposição espectral)
plot_results.py         # Gráficos a partir dos CSVs (matplotlib)
scripts/
  validate_jacobian.py  # Validação do Jacobiano/FK contra o controlador
data/                   # CSVs e PNGs gerados nos testes
docs/                   # Descrição do projeto e do trabalho
exemplo.py              # Código de referência original (RTDE + Jacobiano DH)
plans/                  # Plano de execução e artefatos de revisão
```

---

## Fundamentação (resumo)

- **Jacobiano geométrico** `J(θ)` (6×6): `ṗ = J θ̇` (velocidades) e `τ = Jᵀ F` (dualidade).
- **Manipulabilidade de Yoshikawa** `w = √det(J·Jᵀ)`: "termômetro" escalar da destreza
  postural; `w → 0` indica proximidade de singularidade.
- **Decomposição espectral** `J·Jᵀ = V Λ Vᵀ`: os autovalores medem a destreza por direção
  principal; autovalor pequeno = direção fraca. É a base da App 2.

Detalhes completos em [`docs/project-description.md`](docs/project-description.md).

---

## Limitações conhecidas

- O freio escalar **não permite travessia** de singularidades exatas (onde `J` é singular);
  isso exigiria Damped Least Squares (DLS) — trabalho futuro.
- O UR10 tem 6 juntas para 6 DoF de tarefa, portanto **não há espaço nulo** para
  reconfiguração interna.
- O URSim não simula dinâmica de contato; a dualidade `τ = Jᵀ F` é explorada apenas
  conceitualmente.
