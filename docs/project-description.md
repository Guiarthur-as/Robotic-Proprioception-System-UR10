# Projeto: Sistema de Propriocepção Robótica para o UR10

## Propriocepção Artificial via Jacobiano Geométrico

**Disciplina:** Robótica Industrial
**Robô:** Universal Robots UR10
**Simuladores:** RoboDK + URSim
**Linguagem:** Python (NumPy, matplotlib)

---

## 1. Visão Geral do Projeto

O projeto implementa um **sistema de propriocepção artificial** para o manipulador UR10: um sentido interno e contínuo que permite ao robô perceber sua própria destreza postural e reagir antes que falhas ocorram.

Assim como um ser humano sente quando seu braço está numa posição desconfortável — esticado demais, sem força, prestes a destravar o cotovelo — antes de qualquer dor ou lesão, o robô industrial é cego para esse tipo de percepção. Ele só descobre a singularidade quando o controlador já desarmou por proteção. Este trabalho resolve esse problema com duas aplicações correlacionadas, ambas derivadas da mesma ferramenta matemática: o **Jacobiano Geométrico**.

A metáfora central é a de um **arco reflexo biológico**:

- **App 1** é o órgão sensorial (percepção escalar: "estou confortável ou não?").
- **App 2** é o reflexo direcionado (percepção vetorial: "em qual direção estou fraco?").

---

## 2. Fundamentação Matemática

### 2.1 O Jacobiano Geométrico

O Jacobiano $J(\theta)$ é uma matriz $6 \times 6$ que mapeia velocidades de junta para velocidades cartesianas do efetor:

$$\dot{p} = J(\theta)\,\dot{\theta}$$

Sua transposição mapeia forças cartesianas para torques nas juntas (dualidade):

$$\tau = J^T F$$

### 2.2 Índice de Manipulabilidade de Yoshikawa

$$w = \sqrt{\det(J\,J^T)}$$

- $w$ **alto** → o robô tem liberdade de movimento em todas as direções (postura confortável).
- $w \to 0$ → o robô está perdendo capacidade de se mover em pelo menos uma direção (aproximando-se de uma singularidade).

Este índice é o "termômetro" escalar usado na App 1. Ele resume a saúde postural do robô inteiro num único número.

### 2.3 Decomposição Espectral (Autovalores e Autovetores)

O índice $w$ é na verdade o produto de todos os autovalores de $J\,J^T$. A decomposição espectral "abre" esse número em seus componentes direcionais:

$$J\,J^T = V \Lambda V^T$$

Onde:
- $\Lambda = \text{diag}(\sigma_1^2, \sigma_2^2, \ldots, \sigma_6^2)$ são os autovalores (cada um mede a destreza numa direção).
- $V = [v_1 \; v_2 \; \cdots \; v_6]$ são os autovetores (as próprias direções).

Autovalor grande → o robô é ágil naquela direção. Autovalor pequeno → o robô é fraco naquela direção.

Esta decomposição é o mecanismo central da App 2: em vez de saber *se* o robô está fraco (escalar), sabemos *onde* ele está fraco (vetorial).

### 2.4 Relação entre as duas métricas

| Propriedade | App 1 (Escalar) | App 2 (Vetorial) |
|---|---|---|
| Métrica | $w = \sqrt{\det(JJ^T)}$ | Autovalores $\sigma_i^2$ de $JJ^T$ |
| O que diz | "Estou perto de travar?" | "Em qual direção estou fraco?" |
| Profundidade | Resumo (1 número) | Composição do resumo (6 números) |
| Analogia | Termômetro | Mapa de calor direcional |

A App 2 é literalmente "abrir o capô" do número que a App 1 calcula.

---

## 3. Arquitetura do Sistema

### 3.1 Componentes e suas funções

```
┌──────────────────────────────────────────────────────────┐
│                     Python (Controle)                    │
│                                                          │
│  ┌─────────────────────────────────────────────────────┐ │
│  │         Módulo Compartilhado (core.py)              │ │
│  │  - Lê juntas atuais                                 │ │
│  │  - Solicita J(θ) ao RoboDK                          │ │
│  │  - Calcula w, autovalores, autovetores              │ │
│  │  - Retorna fator de escala e direções fracas         │ │
│  └──────────────┬──────────────────┬───────────────────┘ │
│                 │                  │                      │
│     ┌───────────▼──────┐ ┌────────▼─────────────┐       │
│     │  App 1: Freio    │ │  App 2: Teleoperação │       │
│     │  Autônomo        │ │  Direcional          │       │
│     │  (trajetória)    │ │  (teclado)           │       │
│     └───────────┬──────┘ └────────┬─────────────┘       │
│                 │                  │                      │
│                 ▼                  ▼                      │
│          Comando de velocidade (URScript)                 │
└──────────────────────────┬───────────────────────────────┘
                           │ TCP/IP
              ┌────────────▼────────────────┐
              │    RoboDK (Cinemática)      │
              │  - FK, IK, Jacobiano        │
              │  - Visualização 3D          │
              └────────────┬────────────────┘
                           │ Post-processor / Socket
              ┌────────────▼────────────────┐
              │     URSim (Controlador)     │
              │  - Executa URScript         │
              │  - Simula UR10              │
              └─────────────────────────────┘
```

### 3.2 Fluxo do Loop Principal

```
a cada ciclo (~50ms):
  1. Ler juntas atuais θ           ← RoboDK API
  2. Obter J(θ)                    ← RoboDK API
  3. Calcular w, autovalores, etc. ← NumPy
  4. Decidir velocidade            ← Lógica da App
  5. Enviar comando                ← URScript via socket
  6. Logar variáveis no CSV        ← Para gráficos depois
```

---

## 4. Descrição das Aplicações

### 4.1 App 1 — Freio Autônomo de Singularidade (Proteção Escalar)

**Problema:** O robô segue uma trajetória pré-programada e, sem aviso, entra numa zona de singularidade. O controlador tenta compensar com velocidades de junta impossíveis e desarma por proteção.

**Solução:** Monitorar $w$ em tempo real e reduzir a velocidade **uniformemente** quando $w$ cai abaixo de um limiar seguro $w_{min}$.

**Lógica:**

```python
w = sqrt(det(J @ J.T))
fator = min(1.0, w / w_min)
vel_real = vel_comandada * fator
```

- Longe da singularidade ($w \gg w_{min}$): `fator = 1.0` → velocidade integral.
- Perto da singularidade ($w < w_{min}$): `fator < 1.0` → velocidade proporcional à destreza.
- Na singularidade ($w \to 0$): `fator → 0` → robô para suavemente.

**O que será mostrado:**
- Gráfico temporal de $w$ caindo conforme o robô se aproxima da singularidade.
- Gráfico da velocidade comandada vs. velocidade real (atenuada).
- Vídeo do URSim com o robô freando sozinho antes do desarme.

**Cenário de teste:** Singularidade de cotovelo — mover o efetor em linha reta em direção ao limite do alcance do braço (esticar completamente). Singularidade de passagem, não requer travessia exata.

### 4.2 App 2 — Teleoperação com Consciência Direcional (Proteção Vetorial)

**Problema:** Um operador humano pilota o robô pelo teclado (comandando velocidades em X, Y e Z). Frear tudo uniformemente quando $w$ cai é seguro, mas desperdiça mobilidade: talvez o robô esteja fraco numa direção mas perfeitamente ágil em outra. Frear tudo castiga o operador por algo que só afeta um eixo.

**Solução:** Usar a decomposição espectral de $JJ^T$ para identificar **quais direções** estão fracas e atenuar **apenas essas**, preservando a mobilidade nas direções seguras.

**Lógica:**

```python
autovalores, autovetores = np.linalg.eigh(J @ J.T)
direcao_cmd = np.array([vx, vy, vz, wx, wy, wz])  # input do teclado

# Projeta o comando nas direções principais
projecoes = autovetores.T @ direcao_cmd

# Atenua apenas as componentes fracas
for i in range(6):
    if autovalores[i] < limiar:
        projecoes[i] *= autovalores[i] / limiar

# Reconstrói a velocidade filtrada
vel_real = autovetores @ projecoes
```

**Resultado observável:** O operador pressiona "frente" e o robô resiste (direção fraca). Pressiona "lado" e o robô obedece normalmente (direção forte). Mesma postura, respostas diferentes por direção.

**O que será mostrado:**
- Gráfico com autovalores ao longo do tempo, mostrando qual direção enfraquece.
- Gráfico comparativo: velocidade comandada pelo teclado vs. velocidade real por componente.
- Vídeo do URSim com o operador tentando mover em diferentes direções e o robô respondendo assimetricamente.

### 4.3 Diferença matemática entre as duas aplicações

| Aspecto | App 1 | App 2 |
|---|---|---|
| Input | Trajetória automática | Teclado do operador |
| Métrica | $w$ (determinante, 1 número) | Autovalores de $JJ^T$ (6 números) |
| Ação | Freia tudo igualmente | Freia seletivamente por direção |
| Conceito central | Determinante | Decomposição espectral |
| Sofisticação | O termômetro | O que está dentro do termômetro |

---

## 5. Narrativa Unificada para o Relatório

O projeto é apresentado como **propriocepção robótica** — dar ao robô a capacidade de sentir sua própria destreza postural antes de qualquer falha.

- A **App 1** implementa o "sentido" — a percepção binária de conforto/desconforto, análoga a um tendão tenso que sinaliza perigo antes da ruptura. O determinante do Jacobiano funciona como **limiar de dor**.

- A **App 2** evolui o sentido para uma percepção direcional — não apenas "estou em perigo", mas "estou em perigo *nesta direção específica*". O reflexo é seletivo: protege onde é necessário, preserva liberdade onde é seguro. O mapa de autovalores funciona como um **campo receptivo**.

As duas aplicações compartilham o mesmo módulo computacional e demonstram uma progressão conceitual limpa: do escalar ao vetorial, do alarme ao diagnóstico, da proteção bruta à proteção inteligente.

---

## 6. Entregas

### 6.1 Relatório Técnico (PDF/Markdown no GitHub)

- Fundamentação teórica: Jacobiano, manipulabilidade, decomposição espectral.
- Arquitetura: diagrama de comunicação Python ↔ RoboDK ↔ URSim.
- Análise de dados: gráficos gerados a partir dos CSVs logados durante simulação.

### 6.2 Código-Fonte

```
projeto/
├── core.py              # Módulo compartilhado (J, w, autovalores)
├── app1_freio.py        # Freio autônomo escalar
├── app2_teleop.py       # Teleoperação direcional
├── plot_results.py      # Geração dos gráficos com matplotlib
├── data/                # CSVs logados durante os testes
└── README.md            # Pré-requisitos, IPs, portas, instruções
```

### 6.3 Vídeo (máx. 5 min)

Estrutura sugerida:
1. (30s) Contextualização: o que é singularidade e por que é perigoso.
2. (1min) App 1: explicar a conta, mostrar URSim freando, mostrar gráfico de $w$.
3. (1min30) App 2: explicar autovalores, mostrar teleoperação com freio direcional, mostrar gráfico por componente.
4. (1min) Conexão entre as duas: mesma base, profundidade diferente.
5. (1min) Limitações e trabalho futuro (DLS para travessia, redundância para espaço nulo).

---

## 7. Limitações Conhecidas (para incluir no relatório)

- O freio escalar **não permite travessia** de singularidades exatas (onde $J$ é singular). Para isso seria necessário Damped Least Squares (DLS), que fica como trabalho futuro.
- O UR10 possui exatamente 6 juntas para 6 DoF de tarefa, portanto **não há espaço nulo** para reconfiguração interna (movimentar o cotovelo sem mover o efetor). Exploração de espaço nulo exigiria relaxar um grau de liberdade da tarefa ou usar um manipulador redundante (7+ juntas).
- O URSim não simula dinâmica de contato, portanto a dualidade $\tau = J^T F$ é explorada apenas conceitualmente (análise de elipsoide de força estática), não como estimação de força externa.

---

## 8. Dependências e Configuração

- **Python 3.8+** com `numpy`, `matplotlib`
- **RoboDK** (licença educacional) com modelo UR10 carregado
- **URSim** (Docker ou VM da Universal Robots)
- Comunicação: RoboDK API (TCP/IP local) + URScript via socket (porta 30003)