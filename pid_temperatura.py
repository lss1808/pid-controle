"""
Simulação de Controle PID — Temperatura de Forno Industrial
============================================================
Planta: modelo de primeira ordem com atraso (FOPDT)
   G(s) = K / (tau*s + 1)

Autor: Lucas Schumacher Salsa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parâmetros da planta (forno industrial simplificado) ──────────────────────
K   = 2.5    # ganho estático (°C por unidade de controle)
tau = 15.0   # constante de tempo (s) — velocidade de resposta do forno
dt  = 0.1    # passo de simulação (s)
T   = 200.0  # tempo total de simulação (s)

# ── Parâmetros do controlador PID ────────────────────────────────────────────
Kp = 2.0     # ganho proporcional
Ki = 0.3     # ganho integral
Kd = 1.5     # ganho derivativo

# Limites do atuador (ex: resistência elétrica 0–100%)
u_min = 0.0
u_max = 100.0

# ── Setpoint ─────────────────────────────────────────────────────────────────
setpoint = 200.0  # temperatura desejada em °C

# ── Inicialização ─────────────────────────────────────────────────────────────
n_steps = int(T / dt)
t       = np.linspace(0, T, n_steps)

temp        = np.zeros(n_steps)   # temperatura do forno
u_hist      = np.zeros(n_steps)   # sinal de controle
erro_hist   = np.zeros(n_steps)   # erro ao longo do tempo
sp_hist     = np.full(n_steps, setpoint)

temp[0]  = 25.0  # temperatura ambiente inicial

integral  = 0.0
erro_ant  = 0.0

# ── Perturbação: porta do forno abre em t=120s ────────────────────────────────
t_perturbacao = 120.0
delta_perturbacao = -20.0  # queda brusca de 20°C

perturbacao_aplicada = False

# ── Loop de simulação ─────────────────────────────────────────────────────────
for i in range(1, n_steps):

    # Perturbação
    if t[i] >= t_perturbacao and not perturbacao_aplicada:
        temp[i-1] += delta_perturbacao
        perturbacao_aplicada = True

    # Erro
    erro = setpoint - temp[i-1]
    erro_hist[i] = erro

    # Integral com anti-windup
    integral += erro * dt
    integral  = np.clip(integral, u_min / Ki if Ki != 0 else -1e9,
                                  u_max / Ki if Ki != 0 else  1e9)

    # Derivativo
    derivativo = (erro - erro_ant) / dt

    # Sinal de controle PID
    u = Kp * erro + Ki * integral + Kd * derivativo
    u = np.clip(u, u_min, u_max)

    u_hist[i] = u
    erro_ant  = erro

    # Modelo da planta — equação de diferenças (Euler)
    dT = (K * u - temp[i-1]) / tau
    temp[i] = temp[i-1] + dT * dt

# ── Métricas de desempenho ────────────────────────────────────────────────────
# Tempo de subida (10% → 90% do setpoint)
idx_10 = next((i for i, v in enumerate(temp) if v >= 0.1 * setpoint), None)
idx_90 = next((i for i, v in enumerate(temp) if v >= 0.9 * setpoint), None)
t_subida = (t[idx_90] - t[idx_10]) if (idx_10 and idx_90) else None

# Overshoot
overshoot = max(0, (np.max(temp[:int(t_perturbacao/dt)]) - setpoint) / setpoint * 100)

# Tempo de acomodação (±2% do setpoint, antes da perturbação)
banda = 0.02 * setpoint
idx_acomodacao = None
for i in range(n_steps - 1, 0, -1):
    if t[i] >= t_perturbacao:
        continue
    if abs(temp[i] - setpoint) > banda:
        idx_acomodacao = i
        break
t_acomodacao = t[idx_acomodacao] if idx_acomodacao else None

# ── Plot ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(12, 9), facecolor='white')
fig.suptitle('Controle PID — Forno Industrial\n'
             f'Kp={Kp}  Ki={Ki}  Kd={Kd}  |  Setpoint={setpoint}°C',
             fontsize=13, fontweight='bold', y=0.98)

gs = gridspec.GridSpec(3, 1, figure=fig, hspace=0.45)

# --- Gráfico 1: Temperatura ---
ax1 = fig.add_subplot(gs[0])
ax1.plot(t, temp,     color='#D85A30', linewidth=2,   label='Temperatura (°C)')
ax1.plot(t, sp_hist,  color='#1D9E75', linewidth=1.5, linestyle='--', label=f'Setpoint ({setpoint}°C)')
ax1.axvline(t_perturbacao, color='#7F77DD', linewidth=1.2, linestyle=':', label='Perturbação (porta abre)')
ax1.fill_between(t,
                 setpoint * (1 - 0.02), setpoint * (1 + 0.02),
                 alpha=0.12, color='#1D9E75', label='Banda ±2%')
ax1.set_ylabel('Temperatura (°C)', fontsize=10)
ax1.set_title('Resposta do sistema', fontsize=10, fontweight='bold')
ax1.legend(fontsize=8, loc='lower right')
ax1.set_xlim(0, T)
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, setpoint * 1.15)

# Anotações
if t_subida:
    ax1.annotate(f'Tr ≈ {t_subida:.1f}s', xy=(t[idx_90], temp[idx_90]),
                 xytext=(t[idx_90]+5, temp[idx_90]-25),
                 arrowprops=dict(arrowstyle='->', color='gray'),
                 fontsize=8, color='gray')
if overshoot > 0.1:
    idx_max = np.argmax(temp[:int(t_perturbacao/dt)])
    ax1.annotate(f'OS ≈ {overshoot:.1f}%', xy=(t[idx_max], temp[idx_max]),
                 xytext=(t[idx_max]+5, temp[idx_max]+3),
                 arrowprops=dict(arrowstyle='->', color='gray'),
                 fontsize=8, color='gray')

# --- Gráfico 2: Sinal de controle ---
ax2 = fig.add_subplot(gs[1])
ax2.plot(t, u_hist, color='#185FA5', linewidth=1.8, label='Sinal de controle (%)')
ax2.axvline(t_perturbacao, color='#7F77DD', linewidth=1.2, linestyle=':')
ax2.axhline(u_max, color='#E24B4A', linewidth=0.8, linestyle='--', alpha=0.6, label='Saturação (100%)')
ax2.axhline(u_min, color='#E24B4A', linewidth=0.8, linestyle='--', alpha=0.6)
ax2.set_ylabel('Controle (%)', fontsize=10)
ax2.set_title('Atuador — resistência elétrica', fontsize=10, fontweight='bold')
ax2.legend(fontsize=8, loc='upper right')
ax2.set_xlim(0, T)
ax2.set_ylim(-5, 110)
ax2.grid(True, alpha=0.3)

# --- Gráfico 3: Erro ---
ax3 = fig.add_subplot(gs[2])
ax3.plot(t, erro_hist, color='#BA7517', linewidth=1.8, label='Erro (°C)')
ax3.axhline(0, color='black', linewidth=0.8, linestyle='--', alpha=0.5)
ax3.axvline(t_perturbacao, color='#7F77DD', linewidth=1.2, linestyle=':')
ax3.fill_between(t, erro_hist, 0, alpha=0.15, color='#BA7517')
ax3.set_ylabel('Erro (°C)', fontsize=10)
ax3.set_xlabel('Tempo (s)', fontsize=10)
ax3.set_title('Erro ao longo do tempo', fontsize=10, fontweight='bold')
ax3.legend(fontsize=8)
ax3.set_xlim(0, T)
ax3.grid(True, alpha=0.3)

# Caixa de métricas
metricas = (
    f"Kp = {Kp}   Ki = {Ki}   Kd = {Kd}\n"
    f"Tempo de subida ≈ {t_subida:.1f}s\n" if t_subida else ""
)
metricas += f"Overshoot ≈ {overshoot:.1f}%"

fig.text(0.98, 0.02, metricas,
         ha='right', va='bottom', fontsize=8,
         bbox=dict(boxstyle='round', facecolor='#F1EFE8', alpha=0.8))

plt.savefig('resultado_pid_temperatura.png', dpi=150, bbox_inches='tight',
            facecolor='white')
plt.close()

print("=" * 50)
print("SIMULAÇÃO PID — FORNO INDUSTRIAL")
print("=" * 50)
print(f"Setpoint:          {setpoint}°C")
print(f"Temp. final:       {temp[-1]:.2f}°C")
print(f"Overshoot:         {overshoot:.2f}%")
if t_subida:
    print(f"Tempo de subida:   {t_subida:.1f}s")
if t_acomodacao:
    print(f"Tempo de acomod.:  {t_acomodacao:.1f}s")
print(f"Erro final:        {abs(setpoint - temp[-1]):.4f}°C")
print("=" * 50)
print("Gráfico salvo: resultado_pid_temperatura.png")
