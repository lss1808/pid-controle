"""
Sintonização PID — Método Ziegler-Nichols (Malha Aberta)
=========================================================
O método analisa a resposta ao degrau da planta em malha aberta
e calcula automaticamente Kp, Ki, Kd usando as regras de Z-N.

Planta: FOPDT — G(s) = K * exp(-theta*s) / (tau*s + 1)
  K     = ganho estático
  tau   = constante de tempo
  theta = atraso de transporte (dead time)

Autor: Lucas Schumacher Salsa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch

# ── Parâmetros da planta (processo industrial genérico) ───────────────────────
K_planta = 2.0    # ganho estático
tau      = 8.0    # constante de tempo (s)
theta    = 1.5    # atraso de transporte (s)
dt       = 0.05   # passo de simulação (s)
T        = 80.0   # tempo total (s)

u_min = 0.0
u_max = 100.0
setpoint = 150.0

# ── ETAPA 1: Resposta ao degrau em malha aberta ───────────────────────────────
# Aplica degrau de 50% na entrada e observa a saída
u_degrau = 50.0
n = int(T / dt)
t = np.linspace(0, T, n)

saida_ma = np.zeros(n)   # saída malha aberta
atraso_amostras = int(theta / dt)

for i in range(1, n):
    if i <= atraso_amostras:
        u_ef = 0.0
    else:
        u_ef = u_degrau
    dY = (K_planta * u_ef - saida_ma[i-1]) / tau
    saida_ma[i] = saida_ma[i-1] + dY * dt

# ── ETAPA 2: Identificação dos parâmetros Z-N ─────────────────────────────────
# Ponto de inflexão = máxima derivada da resposta
derivada = np.diff(saida_ma) / dt
idx_inflexao = np.argmax(derivada)

# Reta tangente no ponto de inflexão
slope = derivada[idx_inflexao]
y_inf = saida_ma[idx_inflexao]
t_inf = t[idx_inflexao]

# Interseções da tangente com y=0 e y=y_final
y_final = K_planta * u_degrau
t_L = t_inf - y_inf / slope          # interseção com eixo y=0  → atraso L
t_T = t_inf + (y_final - y_inf) / slope  # interseção com y_final → L + T

L = max(t_L, 0.01)   # atraso aparente (dead time identificado)
T_id = t_T - t_L     # constante de tempo identificada

print("=" * 55)
print("IDENTIFICAÇÃO DA PLANTA (Ziegler-Nichols malha aberta)")
print("=" * 55)
print(f"  Ganho identificado (K):       {y_final/u_degrau:.3f}")
print(f"  Atraso identificado (L):      {L:.3f} s")
print(f"  Constante de tempo (T_id):    {T_id:.3f} s")
print(f"  Razão L/T:                    {L/T_id:.3f}")

# ── ETAPA 3: Cálculo automático dos ganhos Z-N ───────────────────────────────
R = slope / u_degrau   # taxa de subida normalizada

# Tabela Z-N malha aberta
Kp_zn = 1.2 / (R * L)
Ti_zn = 2.0 * L
Td_zn = 0.5 * L
Ki_zn = Kp_zn / Ti_zn
Kd_zn = Kp_zn * Td_zn

# Ganhos manuais (ajustados para comparação)
Kp_man = Kp_zn * 0.6
Ki_man = Ki_zn * 0.4
Kd_man = Kd_zn * 0.3

print()
print("GANHOS CALCULADOS AUTOMATICAMENTE (Ziegler-Nichols)")
print(f"  Kp = {Kp_zn:.4f}")
print(f"  Ki = {Ki_zn:.4f}  (Ti = {Ti_zn:.3f}s)")
print(f"  Kd = {Kd_zn:.4f}  (Td = {Td_zn:.3f}s)")
print()
print("GANHOS MANUAIS (para comparação)")
print(f"  Kp = {Kp_man:.4f}")
print(f"  Ki = {Ki_man:.4f}")
print(f"  Kd = {Kd_man:.4f}")

# ── ETAPA 4: Simulação malha fechada ─────────────────────────────────────────
def simular_mf(Kp, Ki, Kd, label):
    n2 = int(T / dt)
    t2      = np.linspace(0, T, n2)
    y       = np.zeros(n2)
    u_h     = np.zeros(n2)
    e_h     = np.zeros(n2)
    integ   = 0.0
    e_ant   = 0.0

    for i in range(1, n2):
        e = setpoint - y[i-1]
        e_h[i] = e

        integ += e * dt
        if Ki > 0:
            integ = np.clip(integ, u_min/Ki, u_max/Ki)

        deriv = (e - e_ant) / dt
        u = np.clip(Kp*e + Ki*integ + Kd*deriv, u_min, u_max)
        u_h[i] = u
        e_ant  = e

        dY = (K_planta * u - y[i-1]) / tau
        y[i] = y[i-1] + dY * dt

    return t2, y, u_h, e_h

t2, y_zn,  u_zn,  e_zn  = simular_mf(Kp_zn,  Ki_zn,  Kd_zn,  "Z-N")
t2, y_man, u_man, e_man = simular_mf(Kp_man, Ki_man, Kd_man, "Manual")

# Métricas
def metricas(y, sp):
    os_ = max(0, (np.max(y) - sp) / sp * 100)
    ef  = abs(sp - y[-1])
    return os_, ef

os_zn,  ef_zn  = metricas(y_zn,  setpoint)
os_man, ef_man = metricas(y_man, setpoint)

print()
print("MÉTRICAS MALHA FECHADA")
print(f"{'':20} {'Z-N':>12} {'Manual':>12}")
print(f"  Overshoot        {os_zn:>11.2f}%  {os_man:>11.2f}%")
print(f"  Erro final       {ef_zn:>11.4f}   {ef_man:>11.4f}")
print("=" * 55)

# ── PLOT ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 11), facecolor='white')
fig.suptitle('Sintonização PID — Método Ziegler-Nichols\n'
             'Identificação automática de parâmetros da planta',
             fontsize=13, fontweight='bold', y=0.99)

gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.52, wspace=0.32)

# --- Painel 1: Resposta malha aberta + tangente Z-N ---
ax1 = fig.add_subplot(gs[0, :])
ax1.plot(t, saida_ma, color='#185FA5', linewidth=2, label='Resposta ao degrau (malha aberta)')
ax1.axhline(y_final, color='gray', linewidth=0.8, linestyle='--', alpha=0.6, label=f'Y final = {y_final:.1f}')

# Reta tangente
t_tan = np.array([t_L - 1, t_T + 1])
y_tan = y_inf + slope * (t_tan - t_inf)
ax1.plot(t_tan, y_tan, color='#D85A30', linewidth=1.8,
         linestyle='--', label='Tangente no ponto de inflexão')

# Anotações L e T
ax1.axvline(t_L, color='#1D9E75', linewidth=1.2, linestyle=':')
ax1.axvline(t_T, color='#BA7517', linewidth=1.2, linestyle=':')
ax1.annotate('', xy=(t_T, y_final*0.15), xytext=(t_L, y_final*0.15),
             arrowprops=dict(arrowstyle='<->', color='#BA7517', lw=1.5))
ax1.text((t_L + t_T)/2, y_final*0.20, f'T={T_id:.1f}s',
         ha='center', fontsize=8, color='#BA7517')
ax1.annotate('', xy=(t_L, y_final*0.35), xytext=(0, y_final*0.35),
             arrowprops=dict(arrowstyle='<->', color='#1D9E75', lw=1.5))
ax1.text(t_L/2, y_final*0.40, f'L={L:.1f}s',
         ha='center', fontsize=8, color='#1D9E75')

ax1.scatter([t_inf], [y_inf], color='#D85A30', zorder=5, s=50,
            label=f'Ponto de inflexão (t={t_inf:.1f}s)')
ax1.set_ylabel('Saída', fontsize=10)
ax1.set_title('Etapa 1 — Resposta ao degrau em malha aberta (identificação da planta)',
              fontsize=10, fontweight='bold')
ax1.legend(fontsize=8, loc='lower right')
ax1.set_xlim(0, T*0.6)
ax1.set_ylim(-5, y_final * 1.15)
ax1.grid(True, alpha=0.3)

# Caixa de resultados identificação
id_txt = (f"Identificado:\n"
          f"  L = {L:.2f}s\n"
          f"  T = {T_id:.2f}s\n"
          f"  K = {y_final/u_degrau:.2f}")
ax1.text(0.02, 0.95, id_txt, transform=ax1.transAxes,
         fontsize=8, va='top',
         bbox=dict(boxstyle='round', facecolor='#E6F1FB', alpha=0.9))

# --- Painel 2: Resposta malha fechada ---
ax2 = fig.add_subplot(gs[1, :])
ax2.plot(t2, np.full_like(t2, setpoint), color='black',
         linewidth=1.5, linestyle='--', label=f'Setpoint ({setpoint})', zorder=5)
ax2.plot(t2, y_zn,  color='#D85A30', linewidth=2,
         label=f'PID Ziegler-Nichols  (OS={os_zn:.1f}%)')
ax2.plot(t2, y_man, color='#1D9E75', linewidth=2,
         label=f'PID Manual           (OS={os_man:.1f}%)', linestyle='-.')
ax2.fill_between(t2, setpoint*0.98, setpoint*1.02,
                 alpha=0.08, color='gray', label='Banda ±2%')
ax2.set_ylabel('Saída', fontsize=10)
ax2.set_title('Etapa 2 — Comparação: ganhos Z-N automáticos vs ganhos manuais',
              fontsize=10, fontweight='bold')
ax2.legend(fontsize=9, loc='lower right')
ax2.set_xlim(0, T)
ax2.grid(True, alpha=0.3)

# Caixa de ganhos
ganhos_txt = (f"Z-N:    Kp={Kp_zn:.2f}  Ki={Ki_zn:.3f}  Kd={Kd_zn:.2f}\n"
              f"Manual: Kp={Kp_man:.2f}  Ki={Ki_man:.3f}  Kd={Kd_man:.2f}")
ax2.text(0.98, 0.05, ganhos_txt, transform=ax2.transAxes,
         fontsize=7.5, ha='right', va='bottom',
         bbox=dict(boxstyle='round', facecolor='#F1EFE8', alpha=0.9))

# --- Painel 3: Sinal de controle Z-N ---
ax3 = fig.add_subplot(gs[2, 0])
ax3.plot(t2, u_zn,  color='#D85A30', linewidth=1.8, label='Z-N')
ax3.plot(t2, u_man, color='#1D9E75', linewidth=1.8,
         linestyle='-.', label='Manual')
ax3.axhline(u_max, color='gray', linewidth=0.8, linestyle='--', alpha=0.5)
ax3.set_ylabel('Controle (%)', fontsize=10)
ax3.set_xlabel('Tempo (s)', fontsize=10)
ax3.set_title('Sinal de controle', fontsize=10, fontweight='bold')
ax3.legend(fontsize=9)
ax3.set_xlim(0, T)
ax3.set_ylim(-5, 115)
ax3.grid(True, alpha=0.3)

# --- Painel 4: Erro ---
ax4 = fig.add_subplot(gs[2, 1])
ax4.plot(t2, e_zn,  color='#D85A30', linewidth=1.8, label='Z-N')
ax4.plot(t2, e_man, color='#1D9E75', linewidth=1.8,
         linestyle='-.', label='Manual')
ax4.axhline(0, color='black', linewidth=0.8, linestyle='--', alpha=0.4)
ax4.set_ylabel('Erro', fontsize=10)
ax4.set_xlabel('Tempo (s)', fontsize=10)
ax4.set_title('Erro ao longo do tempo', fontsize=10, fontweight='bold')
ax4.legend(fontsize=9)
ax4.set_xlim(0, T)
ax4.grid(True, alpha=0.3)

plt.savefig('resultado_ziegler_nichols.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("\nGráfico salvo: resultado_ziegler_nichols.png")
