# Dashboard Interativo — Simulação PID

Interface web interativa para simulação de controladores PID, com ajuste de ganhos em tempo real via sliders e visualização da resposta do sistema.

## Demo

![Dashboard PID](demo_dashboard.png)

> Acesse pelo navegador em `http://127.0.0.1:8050` após iniciar o servidor.

## Funcionalidades

- **3 plantas disponíveis:** Forno Industrial (temperatura), Motor DC (velocidade) e Tanque Industrial (nível)
- **Sliders interativos** para ajuste de Kp, Ki e Kd em tempo real
- **Gráficos ao vivo** com resposta do sistema, sinal de controle e erro
- **Métricas automáticas:** overshoot, erro final e tempo de subida
- **Perturbações simuladas** nas plantas de temperatura e tanque

## Como executar

```bash
# Instalar dependências
py -m pip install dash plotly numpy

# Iniciar o servidor
py pid_dashboard.py
```

Abre o navegador e acessa: **http://127.0.0.1:8050**

## Plantas disponíveis

| Planta | Variável | Setpoint | Perturbação |
|--------|----------|----------|-------------|
| 🌡️ Forno Industrial | Temperatura (°C) | 200°C | Queda em t=80s |
| ⚙️ Motor DC | Velocidade (RPM) | 120 RPM | Sem perturbação |
| 💧 Tanque Industrial | Nível (m) | 1.5m | Variação em t=60s |

## Estrutura do código

```
pid_dashboard.py
├── simular_pid()        ← simulação genérica para qualquer planta
├── app.layout           ← interface: sliders, dropdown, gráficos
└── atualizar()          ← callback: recalcula e atualiza tudo ao vivo
```

## Conceitos implementados

- Controlador PID discreto com filtro no derivativo e anti-windup
- Interface web reativa com Plotly Dash
- Callback único que recalcula toda a simulação a cada mudança de parâmetro
- Subplots sincronizados (resposta, controle, erro)
- Métricas calculadas automaticamente (overshoot, erro final, tempo de subida)

## Dependências

- Python 3.8+
- dash
- plotly
- numpy

---

*Projeto desenvolvido como parte do portfólio de Engenharia de Controle e Automação — UPE*
