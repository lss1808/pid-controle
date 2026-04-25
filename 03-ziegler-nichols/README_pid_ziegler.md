# Sintonização PID — Método Ziegler-Nichols

Implementação do método de sintonização automática de Ziegler-Nichols em malha aberta, com identificação de parâmetros da planta e comparação entre ganhos calculados automaticamente vs ajustados manualmente.

## Resultado

![Sintonização Ziegler-Nichols](resultado_ziegler_nichols.png)

## O que é Ziegler-Nichols?

Ziegler-Nichols é o método de sintonização de PID mais utilizado na indústria. Em vez de adivinhar os ganhos Kp, Ki e Kd, o método **analisa matematicamente o comportamento da planta** e calcula os ganhos ideais automaticamente.

## Como funciona (passo a passo)

### Etapa 1 — Resposta ao degrau em malha aberta
Aplica um degrau fixo na entrada da planta (sem controlador) e observa como a saída responde.

### Etapa 2 — Identificação dos parâmetros
Traça a **reta tangente** no ponto de inflexão da curva de resposta e extrai:
- **L** = atraso aparente do processo (dead time)
- **T** = constante de tempo do processo

### Etapa 3 — Cálculo automático dos ganhos
Aplica as regras de Ziegler-Nichols:

| Parâmetro | Fórmula Z-N |
|-----------|-------------|
| Kp | 1.2 × T / (K × L) |
| Ti | 2.0 × L |
| Td | 0.5 × L |
| Ki | Kp / Ti |
| Kd | Kp × Td |

### Resultado desta simulação

| Parâmetro identificado | Valor |
|------------------------|-------|
| Atraso L | 1.50s |
| Constante de tempo T | 8.00s |
| Ganho K | 2.00 |

| Ganhos calculados (Z-N) | Valor |
|--------------------------|-------|
| Kp | 3.198 |
| Ki | 1.065 |
| Kd | 2.400 |

## Comparação Z-N vs Manual

| Métrica | Z-N automático | Manual |
|---------|----------------|--------|
| Overshoot | ~2.9% | ~4.8% |
| Erro final | ~0 | ~0 |

O método Z-N calculou ganhos mais agressivos que resultaram em **menor overshoot** — demonstrando a eficácia da sintonização automática.

## Conceitos implementados

- Simulação de resposta ao degrau em malha aberta
- Detecção automática do ponto de inflexão via derivada máxima
- Cálculo da reta tangente e extração de L e T
- Aplicação das regras de Ziegler-Nichols (tabela malha aberta)
- Simulação em malha fechada com os ganhos calculados
- Comparação quantitativa entre métodos

## Como executar

```bash
pip install numpy matplotlib
py pid_ziegler.py
```

O terminal exibe os parâmetros identificados e os ganhos calculados. O gráfico gerado mostra 4 painéis:
1. Resposta ao degrau com reta tangente e identificação de L e T
2. Resposta em malha fechada — Z-N vs Manual
3. Sinal de controle dos dois controladores
4. Erro ao longo do tempo

---

*Projeto desenvolvido como parte do portfólio de Engenharia de Controle e Automação — UPE*
