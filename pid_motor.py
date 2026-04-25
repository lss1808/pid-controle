"""
Simulação de Controle PID — Velocidade de Motor DC
===================================================
Planta: modelo de primeira ordem (motor DC com inércia e atrito)
   G(s) = K / (tau*s + 1)

Compara P vs PI vs PID no mesmo gráfico.

Autor: Lucas Schumacher Salsa
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── Parâmetros da planta (motor DC) ──────────────────────────────────────────
K   = 1.8    # ganho do motor (RPM por % de tensão aplicada)
tau = 0.8    # constante de tempo (s) — inércia do motor
dt  = 0.01   # passo de simulação (s)
T   = 10.0   # tempo total (s)

# Limites do atuador (0 a 100% de tensão)
u_min =  0.0
u_max = 100.0

# ── Setpoints ─────────────────────────────────────────────────────────────────
# Degrau para 120 RPM em t=0, muda para 80 RPM em t=5s
setpoint_1 = 120.0
setpoint_2 =  80.0
t_mudanca  =   5.0

# ── Configurações dos 3 controladores ────────────────────────────────────────
controladores = {
    "P":   {"Kp": 1.5,  "Ki": 0.0,  "Kd": 0.0},
    "PI":  {"Kp": 1.5,  "Ki": 2.0,  "Kd": 0.0},
    "PID": {"Kp": 1.5,  "Ki": 2.0,  "Kd": 0.15},
}

cores = {
    "P":   "#E24B4A",
    "PI":  "#BA7517",
    "PID": "#1D9E75",
}

# ── Função de simulação ───────────────────────────────────────────────────────
def simular(Kp, Ki, Kd):
    n       = int(T / dt)
    t       = np.linspace(0, T, n)
    rpm     = np.zeros(n)
    u_hist  = np.zeros(n)
    sp_hist = np.zeros(n)
    erro_hist = np.zeros(n)

    rpm[0]   = 0.0
    integral = 0.0
    erro_ant = 0.0

    for i in range(1, n):
        sp = setpoint_1 if t[i] < t_mudanca else setpoint_2
        sp_hist[i] = sp

        erro = sp - rpm[i-1]
        erro_hist[i] = erro

        # Integral com anti-windup
        integral += erro * dt
        if Ki > 0:
            integral = np.clip(integral, u_min / Ki, u_max / Ki)

        derivativo = (erro - erro_ant) / dt

        u = Kp * erro + Ki * integral + Kd * derivativo
        u = np.clip(u, u_min, u_max)

        u_hist[i] = u
        erro_ant  = erro

        # Modelo do motor (Euler)
        dRPM   = (K * u - rpm[i-1]) / tau
        rpm[i] = rpm[i-1] + dRPM * dt

    sp_hist[0] = setpoint_1
    return t, rpm, u_hist, sp_hist, erro_hist

# ── Executa simulações ────────────────────────────────────────────────────────
resultados = {}
for nome, params in controladores.items():
    t, rpm, u, sp, erro = simular(**params)
    resultados[nome] = {"t": t, "rpm": rpm, "u": u, "sp": sp, "erro": erro}

# ── Métricas (fase 1) ─────────────────────────────────────────────────────────
def calcular_metricas(rpm, sp, t):
    idx_fim = int(t_mudanca / dt) - 1
    rpm_f   = rpm[:idx_fim]
    t_f     = t[:idx_fim]

    overshoot  = max(0, (np.max(rpm_f) - sp) / sp * 100)
    erro_final = abs(sp - rpm_f[-1])

    idx_10 = next((i for i, v in enumerate(rpm_f) if v >= 0.1 * sp), None)
    idx_90 = next((i for i, v in enumerate(rpm_f) if v >= 0.9 * sp), None)
    t_subida = round(t_f[idx_90] - t_f[idx_10], 3) if (idx_10 and idx_90) else None

    return overshoot, erro_final, t_subida

print("=" * 62)
print("MÉTRICAS — fase 1 (0 → 120 RPM)")
print(f"{'Controlador':<12} {'Overshoot':>12} {'Erro final':>12} {'T. subida':>10}")
print("-" * 62)
for nome in controladores:
    r = resultados[nome]
    os_, ef, ts = calcular_metricas(r["rpm"], setpoint_1, r["t"])
    ts_str = f"{ts}s" if ts else "N/A"
    print(f"{nome:<12} {os_:>11.2f}%  {ef:>10.4f} RPM  {ts_str:>10}")
print("=" * 62)

# ── Plot ──────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 9), facecolor='white')
fig.suptitle('Controle de Velocidade — Motor DC\nComparação P vs PI vs PID',
             fontsize=13, fontweight='bold', y=0.98)

gs = gridspec.GridSpec(3, 1, figure=fig, hspace=0.48)

# Gráfico 1: RPM
ax1 = fig.add_subplot(gs[0])
sp_ref = resultados["PID"]["sp"]
t_ref  = resultados["PID"]["t"]

ax1.plot(t_ref, sp_ref, color='black', linewidth=1.5,
         linestyle='--', label='Setpoint', zorder=5)
for nome, r in resultados.items():
    ax1.plot(r["t"], r["rpm"], color=cores[nome],
             linewidth=2, label=nome, alpha=0.9)

ax1.axvline(t_mudanca, color='gray', linewidth=1,
            linestyle=':', label='Mudança de setpoint')
ax1.fill_between(t_ref, sp_ref * 0.98, sp_ref * 1.02,
                 alpha=0.08, color='gray')

# Anotação erro residual P
idx_p_final = int((t_mudanca - 0.2) / dt)
rpm_p = resultados["P"]["rpm"][idx_p_final]
ax1.annotate(f'Erro residual P ≈ {abs(setpoint_1 - rpm_p):.1f} RPM',
             xy=(t_ref[idx_p_final], rpm_p),
             xytext=(2.0, rpm_p - 18),
             arrowprops=dict(arrowstyle='->', color='#E24B4A'),
             fontsize=8, color='#E24B4A')

ax1.set_ylabel('Velocidade (RPM)', fontsize=10)
ax1.set_title('Resposta de velocidade', fontsize=10, fontweight='bold')
ax1.legend(fontsize=9, loc='lower right')
ax1.set_xlim(0, T)
ax1.set_ylim(-5, 160)
ax1.grid(True, alpha=0.3)

# Gráfico 2: Sinal de controle
ax2 = fig.add_subplot(gs[1])
for nome, r in resultados.items():
    ax2.plot(r["t"], r["u"], color=cores[nome],
             linewidth=1.8, label=nome, alpha=0.9)
ax2.axvline(t_mudanca, color='gray', linewidth=1, linestyle=':')
ax2.axhline(u_max, color='#E24B4A', linewidth=0.8,
            linestyle='--', alpha=0.5, label='Saturação (100%)')
ax2.set_ylabel('Tensão (%)', fontsize=10)
ax2.set_title('Sinal de controle — tensão aplicada ao motor', fontsize=10, fontweight='bold')
ax2.legend(fontsize=9, loc='upper right')
ax2.set_xlim(0, T)
ax2.set_ylim(-5, 115)
ax2.grid(True, alpha=0.3)

# Gráfico 3: Erro
ax3 = fig.add_subplot(gs[2])
for nome, r in resultados.items():
    ax3.plot(r["t"], r["erro"], color=cores[nome],
             linewidth=1.8, label=nome, alpha=0.9)
ax3.axhline(0, color='black', linewidth=0.8, linestyle='--', alpha=0.4)
ax3.axvline(t_mudanca, color='gray', linewidth=1, linestyle=':')
ax3.set_ylabel('Erro (RPM)', fontsize=10)
ax3.set_xlabel('Tempo (s)', fontsize=10)
ax3.set_title('Erro ao longo do tempo', fontsize=10, fontweight='bold')
ax3.legend(fontsize=9)
ax3.set_xlim(0, T)
ax3.grid(True, alpha=0.3)

# Caixa de parâmetros
params_txt = "Parâmetros:\n"
for nome, p in controladores.items():
    params_txt += f"{nome}: Kp={p['Kp']}  Ki={p['Ki']}  Kd={p['Kd']}\n"
fig.text(0.98, 0.01, params_txt.strip(),
         ha='right', va='bottom', fontsize=7.5,
         bbox=dict(boxstyle='round', facecolor='#F1EFE8', alpha=0.8))

plt.savefig('resultado_pid_motor.png', dpi=150,
            bbox_inches='tight', facecolor='white')
plt.close()
print("\nGráfico salvo: resultado_pid_motor.png")
