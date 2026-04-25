"""
Simulação de Controle PID — Nível de Tanque Industrial
=======================================================
Planta: tanque com válvula de entrada controlada e saída variável
  dh/dt = (Qin - Qout) / A
  Qin  = Cv * (u/100)     (controlada pelo PID — 0 a 100%)
  Qout = perturbação variável (simula abertura de válvula de saída)
  A    = área da seção transversal (m²)
  h    = nível do líquido (m)

Autor: Lucas Schumacher Salsa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parâmetros do tanque ──────────────────────────────────────────────────────
A       = 0.5    # área da seção transversal (m²)
Cv      = 0.10   # capacidade máxima da válvula de entrada (m³/s)
h_min   = 0.0
h_max   = 3.0
setpoint = 1.5   # nível desejado (m)

dt = 0.05        # passo de simulação (s)
T  = 700.0       # tempo total (s)

# ── Parâmetros do PID ─────────────────────────────────────────────────────────
Kp    = 25.0
Ki    = 2.0
Kd    = 4.0
alpha = 0.7      # filtro do derivativo (reduz ruído)
u_min, u_max = 0.0, 100.0

# ── Perturbações na vazão de saída ────────────────────────────────────────────
def qout(ti):
    """Simula abertura da válvula de saída ao longo do tempo."""
    if   ti < 150: return 0.03           # operação normal
    elif ti < 350: return 0.06           # perturbação 1 — saída dobra
    elif ti < 460: return 0.03           # retorno ao normal
    elif ti < 580: return 0.08           # perturbação 2 — saída forte
    else:          return 0.04           # novo ponto de operação

# ── Simulação ─────────────────────────────────────────────────────────────────
n = int(T / dt)
t = np.linspace(0, T, n)

h       = np.zeros(n)
u_hist  = np.zeros(n)
qin_h   = np.zeros(n)
qout_h  = np.zeros(n)
e_hist  = np.zeros(n)

h[0]     = 0.3   # nível inicial
integral = 0.0
e_ant    = 0.0
d_filt   = 0.0   # derivativo filtrado

for i in range(1, n):
    Qo        = qout(t[i])
    qout_h[i] = Qo

    e         = setpoint - h[i-1]
    e_hist[i] = e

    # Integral com anti-windup
    integral = np.clip(integral + e * dt, u_min / Ki, u_max / Ki)

    # Derivativo com filtro passa-baixa
    d_raw  = (e - e_ant) / dt
    d_filt = alpha * d_filt + (1 - alpha) * d_raw

    u         = np.clip(Kp * e + Ki * integral + Kd * d_filt, u_min, u_max)
    u_hist[i] = u
    e_ant     = e

    Qi        = (u / 100.0) * Cv
    qin_h[i]  = Qi

    dh   = (Qi - Qo) / A
    h[i] = np.clip(h[i-1] + dh * dt, h_min, h_max)

# ── Métricas ──────────────────────────────────────────────────────────────────
os_      = max(0, (np.max(h[:int(150/dt)]) - setpoint) / setpoint * 100)
ef       = abs(setpoint - h[-1])
em       = np.mean(np.abs(h[int(0.35*n):] - setpoint))

print("=" * 50)
print("CONTROLE DE NÍVEL DE TANQUE — PID")
print("=" * 50)
print(f"  Setpoint:       {setpoint:.2f} m")
print(f"  Nível final:    {h[-1]:.4f} m")
print(f"  Overshoot:      {os_:.2f}%")
print(f"  Erro final:     {ef:.4f} m")
print(f"  Erro médio:     {em:.4f} m")
print(f"  Kp={Kp}  Ki={Ki}  Kd={Kd}")
print("=" * 50)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 10), facecolor='white')
fig.suptitle('Controle PID — Nível de Tanque Industrial\n'
             'Rejeição de perturbações na vazão de saída',
             fontsize=13, fontweight='bold', y=0.99)

gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.52, wspace=0.32)
perturb = [(150, '#D85A30', 'Perturbação 1\n(Qout ↑ 2x)'),
           (460, '#BA7517', 'Perturbação 2\n(Qout ↑↑)')]

# Painel 1 — Nível
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(t, h, color='#185FA5', linewidth=2.2, label='Nível h(t)')
ax1.axhline(setpoint, color='black', linewidth=1.5, linestyle='--',
            label=f'Setpoint ({setpoint}m)', zorder=5)
ax1.fill_between(t, setpoint*0.97, setpoint*1.03,
                 alpha=0.10, color='gray', label='Banda ±3%')
ax1.fill_between(t, 0, h, alpha=0.10, color='#185FA5')
for tp, cor, lbl in perturb:
    ax1.axvline(tp, color=cor, linewidth=1.3, linestyle=':', alpha=0.85)
    ax1.text(tp + 6, h_max * 0.88, lbl, fontsize=7.5, color=cor)
ax1.set_ylabel('Nível (m)', fontsize=10)
ax1.set_title('Resposta do nível — rejeição de perturbações',
              fontsize=10, fontweight='bold')
ax1.legend(fontsize=9, loc='lower right')
ax1.set_xlim(0, T)
ax1.set_ylim(-0.05, h_max * 1.05)
ax1.grid(True, alpha=0.3)
m_txt = (f"Overshoot: {os_:.1f}%\n"
         f"Erro final: {ef:.4f}m\n"
         f"Erro médio: {em:.4f}m")
ax1.text(0.02, 0.97, m_txt, transform=ax1.transAxes, fontsize=8, va='top',
         bbox=dict(boxstyle='round', facecolor='#E6F1FB', alpha=0.9))

# Painel 2 — Válvula
ax2 = fig.add_subplot(gs[1, :])
ax2.plot(t, u_hist, color='#1D9E75', linewidth=1.8, label='Abertura da válvula (%)')
ax2.axhline(u_max, color='#E24B4A', linewidth=0.8,
            linestyle='--', alpha=0.5, label='Saturação (100%)')
for tp, cor, _ in perturb:
    ax2.axvline(tp, color=cor, linewidth=1.3, linestyle=':', alpha=0.85)
ax2.set_ylabel('Válvula (%)', fontsize=10)
ax2.set_title('Sinal de controle — abertura da válvula de entrada',
              fontsize=10, fontweight='bold')
ax2.legend(fontsize=9)
ax2.set_xlim(0, T)
ax2.set_ylim(-5, 115)
ax2.grid(True, alpha=0.3)

# Painel 3 — Vazões
ax3 = fig.add_subplot(gs[2, 0])
ax3.plot(t, qin_h,  color='#1D9E75', linewidth=1.8, label='Qin — entrada')
ax3.plot(t, qout_h, color='#D85A30', linewidth=1.8,
         linestyle='-.', label='Qout — saída')
ax3.set_ylabel('Vazão (m³/s)', fontsize=10)
ax3.set_xlabel('Tempo (s)', fontsize=10)
ax3.set_title('Vazões de entrada e saída', fontsize=10, fontweight='bold')
ax3.legend(fontsize=9)
ax3.set_xlim(0, T)
ax3.grid(True, alpha=0.3)

# Painel 4 — Erro
ax4 = fig.add_subplot(gs[2, 1])
ax4.plot(t, e_hist, color='#BA7517', linewidth=1.8, label='Erro (m)')
ax4.fill_between(t, e_hist, 0, alpha=0.15, color='#BA7517')
ax4.axhline(0, color='black', linewidth=0.8, linestyle='--', alpha=0.4)
for tp, cor, _ in perturb:
    ax4.axvline(tp, color=cor, linewidth=1.3, linestyle=':', alpha=0.85)
ax4.set_ylabel('Erro (m)', fontsize=10)
ax4.set_xlabel('Tempo (s)', fontsize=10)
ax4.set_title('Erro ao longo do tempo', fontsize=10, fontweight='bold')
ax4.legend(fontsize=9)
ax4.set_xlim(0, T)
ax4.grid(True, alpha=0.3)

fig.text(0.98, 0.01,
         f"Kp={Kp}  Ki={Ki}  Kd={Kd}  α={alpha}\nA={A}m²  Cv={Cv}m³/s  SP={setpoint}m",
         ha='right', va='bottom', fontsize=7.5,
         bbox=dict(boxstyle='round', facecolor='#F1EFE8', alpha=0.8))

plt.savefig('resultado_pid_tanque.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("\nGráfico salvo: resultado_pid_tanque.png")
