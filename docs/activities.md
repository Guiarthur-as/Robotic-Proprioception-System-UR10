# Trabalho: Jacobiano Geométrico do UR10

## Objetivo do Trabalho

O objetivo deste trabalho é projetar, implementar e validar soluções práticas de engenharia voltadas para o controle, desempenho ou segurança do robô industrial UR10, utilizando o simulador oficial URSim. O núcleo do projeto deve se basear na exploração das propriedades matemáticas e físicas do Jacobiano Geométrico do manipulador, conectando os conceitos abstratos de matrizes de derivadas com o comportamento físico do robô real.

---

## 1. Descrição do Desafio

O Jacobiano Geométrico ($J$) é uma das ferramentas mais poderosas da robótica, atuando como um elemento de transformação dual: ele mapeia tanto as velocidades das juntas para o espaço cartesiano ($\dot{p} = J \dot{\theta}$) quanto as forças exercidas no efetor final para os torques internos dos motores ($\tau = J^T F$). Quando mal gerenciado pelo sistema, zonas de instabilidade matemática (singularidades) podem provocar falhas críticas, desligamentos de proteção ou danos estruturais.

Neste trabalho, os alunos (individualmente ou em duplas) têm total liberdade para propor e implementar **duas aplicações distintas** em que o Jacobiano seja o protagonista na resolução de um problema prático no robô.

### Diretrizes para a Escolha das Aplicações

Você deve identificar gargalos operacionais ou riscos físicos na operação de um manipulador e criar algoritmos baseados no Jacobiano para mitigá-los ou viabilizá-los. O desenvolvimento deve ser integrado ao URSim utilizando a biblioteca RTDE (Real-Time Data Exchange) em Python ou programação nativa em URScript (via conexões Sockets/TCP-IP).

Alguns eixos temáticos que podem guiar a sua escolha de aplicações incluem:

- **Sistemas de Proteção e Monitoramento Ativo:** Algoritmos que avaliam as condições internas ou geométricas do robô e tomam decisões em tempo real (como filtragem de trajetórias, evasão de zonas críticas ou interrupção de movimentos perigosos antes que o hardware desarme).

- **Mapeamento e Controle Baseado em Força/Torque:** Soluções que utilizam a dualidade do Jacobiano para estimar esforços, gerenciar a conformidade mecânica do braço ao interagir com o ambiente ou proteger os motores contra sobrecargas induzidas por posturas desvantajosas.

- **Manipulação e Controle Dinâmico Avançado:** Estratégias de controle em malha fechada (streaming de dados de velocidade ou posição) em que o condicionamento da matriz guie o ganho ou o comportamento do robô.

---

## 2. Requisitos da Entrega

A comprovação do funcionamento e a defesa do projeto devem ser estruturadas através dos seguintes componentes obrigatórios:

### I. Relatório Técnico / Documentação (PDF ou Markdown no GitHub)

Um documento bem estruturado contendo:

- **Fundamentação Teórica:** Demonstração conceitual do Jacobiano do UR10, explicando o significado físico de suas propriedades (como determinante, posto, número de condição ou transposição) e como elas fundamentam as soluções propostas.

- **Arquitetura da Solução:** Descrição metodológica de como a lógica foi estruturada no código e de como foi feita a comunicação com o simulador URSim.

- **Análise de Dados:** Apresentação de gráficos gerados a partir das variáveis coletadas durante os testes da simulação (ex: evolução de torques, velocidades de junta, perfis de erro cartesiano ou índices de manipulabilidade ao longo do tempo), comprovando a eficácia do algoritmo.

### II. Código-Fonte Original

Scripts organizados em Python ou arquivos `.script` comentados, acompanhados de um arquivo ReadMe detalhando os pré-requisitos, IPs e portas necessárias para reproduzir os testes.

### III. Demonstração em Vídeo (Máximo 5 minutos)

Link para um vídeo hospedado no YouTube (público ou não-listado).

O vídeo deve contextualizar brevemente o problema escolhido, detalhar como a matemática do Jacobiano foi aplicada no código e, obrigatoriamente, mostrar a tela do URSim rodando em tempo real enquanto os logs do terminal evidenciam o algoritmo agindo para controlar ou proteger o robô.

---

## 3. Critérios de Avaliação

| Critério | Descrição | Peso |
|---|---|---|
| Criatividade e Complexidade das Propostas | Originalidade na escolha das aplicações e profundidade técnica do problema resolvido pelo Jacobiano. | 25% |
| Rigor Matemático e Conceitual | Uso correto das propriedades da matriz Jacobiana, inversões, transposições e formulação física das métricas utilizadas. | 25% |
| Implementação e Integração Prática | Qualidade do código e sincronismo com o simulador URSim de forma fluida, estável e funcional. | 30% |
| Documentação e Defesa | Clareza do relatório, qualidade visual e técnica dos gráficos apresentados e poder de síntese no vídeo explicativo. | 20% |