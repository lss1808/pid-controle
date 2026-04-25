"""
Dashboard Interativo — Simulação PID
=====================================
Interface web com sliders para ajuste de Kp, Ki, Kd em tempo real.
Escolha entre 3 plantas: Temperatura, Motor DC e Nível de Tanque.

Como usar:
  py -m pip install dash plotly
  py pid_dashboard.py
  Abre o navegador em: http://127.0.0.1:8050

Autor: Lucas Schumacher Salsa
"""

import numpy as np
from dash import Dash, dcc, html, Input, Output, callback
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ── Simulação PID genérica ────────────────────────────────────────────────────
def simular_pid(Kp, Ki, Kd, planta, T=150.0, dt=0.05):
    """
    Simula o sistema em malha fechada com o controlador PID.
    Retorna vetores de tempo, saída, sinal de controle e erro.
    """
    n        = int(T / dt)
    t        = np.linspace(0, T, n)
    y        = np.zeros(n)
    u_hist   = np.zeros(n)
    e_hist   = np.zeros(n)
    sp_hist  = np.zeros(n)

    # ── Parâmetros de cada planta ─────────────────────────────────────────────
    if planta == "temperatura":
        K, tau      = 2.5, 15.0
        setpoint    = 200.0
        y[0]        = 25.0
        u_min, u_max = 0, 100
        # Perturbação: queda em t=80s
        def perturbacao(i): return -15.0 if abs(t[i] - 80) < dt else 0.0
        unidade = "°C"
        titulo  = "Forno Industrial"

    elif planta == "motor":
        K, tau      = 1.8, 0.8
        setpoint    = 120.0
        y[0]        = 0.0
        u_min, u_max = 0, 100
        def perturbacao(i): return 0.0
        unidade = "RPM"
        titulo  = "Motor DC"

    else:  # tanque
        K, tau      = 0.13, 5.0   # modelo linearizado
        setpoint    = 1.5
        y[0]        = 0.3
        u_min, u_max = 0, 100
        # Perturbação em t=60s
        def perturbacao(i): return -0.3 if abs(t[i] - 60) < dt else 0.0
        unidade = "m"
        titulo  = "Tanque Industrial"

    # Setpoint fixo
    sp_hist[:] = setpoint

    integral = 0.0
    e_ant    = 0.0
    d_filt   = 0.0
    alpha    = 0.7

    for i in range(1, n):
        y[i-1] += perturbacao(i)

        e = setpoint - y[i-1]
        e_hist[i] = e

        integral = np.clip(
            integral + e * dt,
            u_min / Ki if Ki > 0 else -1e9,
            u_max / Ki if Ki > 0 else  1e9
        )

        d_raw  = (e - e_ant) / dt
        d_filt = alpha * d_filt + (1 - alpha) * d_raw

        u = np.clip(Kp * e + Ki * integral + Kd * d_filt, u_min, u_max)
        u_hist[i] = u
        e_ant = e

        dY   = (K * u - y[i-1]) / tau
        y[i] = y[i-1] + dY * dt

    # Métricas
    os_   = max(0, (np.max(y) - setpoint) / setpoint * 100)
    ef    = abs(setpoint - y[-1])
    idx_r = next((i for i, v in enumerate(y) if v >= 0.9 * setpoint), None)
    tr    = round(t[idx_r], 2) if idx_r else None

    return t, y, u_hist, e_hist, sp_hist, {
        "overshoot": os_,
        "erro_final": ef,
        "t_subida": tr,
        "unidade": unidade,
        "titulo": titulo,
        "setpoint": setpoint,
    }


# ── Layout do Dashboard ───────────────────────────────────────────────────────
app = Dash(__name__)

CORES = {
    "bg":        "#F8F8F6",
    "card":      "#FFFFFF",
    "borda":     "#E0DDD5",
    "azul":      "#185FA5",
    "verde":     "#1D9E75",
    "vermelho":  "#D85A30",
    "texto":     "#1A1A1A",
    "subtexto":  "#666666",
}

app.layout = html.Div(style={"backgroundColor": CORES["bg"],
                              "minHeight": "100vh",
                              "fontFamily": "Inter, sans-serif",
                              "padding": "24px"}, children=[

    # Título
    html.Div([
        html.H1("Dashboard PID — Simulação Interativa",
                style={"color": CORES["texto"], "margin": "0 0 4px",
                       "fontSize": "24px", "fontWeight": "600"}),
        html.P("Ajuste os ganhos e veja a resposta do sistema em tempo real.",
               style={"color": CORES["subtexto"], "margin": "0 0 24px",
                      "fontSize": "14px"}),
    ]),

    html.Div(style={"display": "flex", "gap": "20px", "flexWrap": "wrap"}, children=[

        # ── Painel de controles ───────────────────────────────────────────────
        html.Div(style={
            "background": CORES["card"], "border": f"1px solid {CORES['borda']}",
            "borderRadius": "12px", "padding": "20px",
            "width": "280px", "flexShrink": "0"
        }, children=[

            html.H3("Configurações", style={"margin": "0 0 16px",
                                             "fontSize": "15px",
                                             "color": CORES["texto"]}),

            # Seleção de planta
            html.Label("Planta", style={"fontSize": "12px", "fontWeight": "500",
                                         "color": CORES["subtexto"]}),
            dcc.Dropdown(
                id="planta",
                options=[
                    {"label": "🌡️  Forno Industrial (Temperatura)", "value": "temperatura"},
                    {"label": "⚙️  Motor DC (Velocidade)",          "value": "motor"},
                    {"label": "💧  Tanque Industrial (Nível)",       "value": "tanque"},
                ],
                value="temperatura",
                clearable=False,
                style={"marginBottom": "20px", "fontSize": "13px"}
            ),

            # Slider Kp
            html.Label("Kp — Ganho Proporcional",
                       style={"fontSize": "12px", "fontWeight": "500",
                              "color": CORES["subtexto"]}),
            html.Div(id="kp-label", style={"fontSize": "20px", "fontWeight": "600",
                                            "color": CORES["azul"],
                                            "marginBottom": "4px"}),
            dcc.Slider(id="kp", min=0.1, max=10.0, step=0.1, value=2.0,
                       marks={i: str(i) for i in range(0, 11, 2)},
                       tooltip={"always_visible": False}),
            html.Div(style={"height": "16px"}),

            # Slider Ki
            html.Label("Ki — Ganho Integral",
                       style={"fontSize": "12px", "fontWeight": "500",
                              "color": CORES["subtexto"]}),
            html.Div(id="ki-label", style={"fontSize": "20px", "fontWeight": "600",
                                            "color": CORES["verde"],
                                            "marginBottom": "4px"}),
            dcc.Slider(id="ki", min=0.0, max=5.0, step=0.05, value=0.3,
                       marks={i: str(i) for i in range(0, 6)},
                       tooltip={"always_visible": False}),
            html.Div(style={"height": "16px"}),

            # Slider Kd
            html.Label("Kd — Ganho Derivativo",
                       style={"fontSize": "12px", "fontWeight": "500",
                              "color": CORES["subtexto"]}),
            html.Div(id="kd-label", style={"fontSize": "20px", "fontWeight": "600",
                                            "color": CORES["vermelho"],
                                            "marginBottom": "4px"}),
            dcc.Slider(id="kd", min=0.0, max=10.0, step=0.1, value=1.5,
                       marks={i: str(i) for i in range(0, 11, 2)},
                       tooltip={"always_visible": False}),

            html.Hr(style={"margin": "20px 0", "borderColor": CORES["borda"]}),

            # Métricas
            html.H3("Métricas", style={"margin": "0 0 12px",
                                        "fontSize": "15px",
                                        "color": CORES["texto"]}),
            html.Div(id="metricas", style={"fontSize": "13px",
                                            "color": CORES["subtexto"],
                                            "lineHeight": "2.0"}),
        ]),

        # ── Gráficos ──────────────────────────────────────────────────────────
        html.Div(style={"flex": "1", "minWidth": "500px"}, children=[
            dcc.Graph(id="grafico", style={"height": "700px"},
                      config={"displayModeBar": False}),
        ]),
    ]),

    # Rodapé
    html.Div(style={"marginTop": "24px", "fontSize": "12px",
                    "color": CORES["subtexto"], "textAlign": "center"},
             children="Lucas Schumacher Salsa — Portfólio Engenharia de Controle e Automação | UPE"),
])


# ── Callbacks ─────────────────────────────────────────────────────────────────
@callback(
    Output("grafico",  "figure"),
    Output("metricas", "children"),
    Output("kp-label", "children"),
    Output("ki-label", "children"),
    Output("kd-label", "children"),
    Input("kp",     "value"),
    Input("ki",     "value"),
    Input("kd",     "value"),
    Input("planta", "value"),
)
def atualizar(Kp, Ki, Kd, planta):
    t, y, u, e, sp, meta = simular_pid(Kp, Ki, Kd, planta)

    un  = meta["unidade"]
    sp_ = meta["setpoint"]

    # ── Figura com subplots ───────────────────────────────────────────────────
    fig = make_subplots(
        rows=3, cols=1,
        subplot_titles=(
            f"Resposta do sistema — {meta['titulo']}",
            "Sinal de controle — atuador (%)",
            "Erro ao longo do tempo",
        ),
        vertical_spacing=0.10,
        row_heights=[0.5, 0.25, 0.25],
    )

    # Setpoint
    fig.add_trace(go.Scatter(
        x=t, y=sp, name="Setpoint",
        line=dict(color="black", width=1.5, dash="dash"),
        showlegend=True,
    ), row=1, col=1)

    # Resposta
    fig.add_trace(go.Scatter(
        x=t, y=y, name="Saída do sistema",
        line=dict(color=CORES["azul"], width=2.5),
        fill="tozeroy", fillcolor="rgba(24,95,165,0.07)",
    ), row=1, col=1)

    # Banda ±3%
    fig.add_hrect(
        y0=sp_ * 0.97, y1=sp_ * 1.03,
        fillcolor="rgba(100,100,100,0.07)",
        line_width=0, row=1, col=1,
        annotation_text="±3%", annotation_position="right",
    )

    # Controle
    fig.add_trace(go.Scatter(
        x=t, y=u, name="Controle",
        line=dict(color=CORES["verde"], width=2),
        showlegend=False,
    ), row=2, col=1)

    # Erro
    fig.add_trace(go.Scatter(
        x=t, y=e, name="Erro",
        line=dict(color=CORES["vermelho"], width=2),
        fill="tozeroy", fillcolor="rgba(216,90,48,0.10)",
        showlegend=False,
    ), row=3, col=1)

    fig.add_hline(y=0, line_dash="dot", line_color="gray",
                  line_width=1, row=3, col=1)

    # Layout
    fig.update_layout(
        template="plotly_white",
        margin=dict(l=50, r=30, t=60, b=40),
        legend=dict(orientation="h", y=1.04, x=0),
        font=dict(family="Inter, sans-serif", size=12),
        paper_bgcolor=CORES["card"],
        plot_bgcolor=CORES["card"],
    )
    fig.update_yaxes(title_text=f"Saída ({un})", row=1, col=1)
    fig.update_yaxes(title_text="Controle (%)", row=2, col=1)
    fig.update_yaxes(title_text=f"Erro ({un})",  row=3, col=1)
    fig.update_xaxes(title_text="Tempo (s)", row=3, col=1)

    # ── Métricas ──────────────────────────────────────────────────────────────
    os_str = f"{meta['overshoot']:.2f}%"
    ef_str = f"{meta['erro_final']:.4f} {un}"
    tr_str = f"{meta['t_subida']}s" if meta['t_subida'] else "—"

    metricas = [
        html.Div([html.Strong("Overshoot: "),    os_str]),
        html.Div([html.Strong("Erro final: "),   ef_str]),
        html.Div([html.Strong("T. subida: "),    tr_str]),
        html.Div([html.Strong("Setpoint: "),     f"{sp_} {un}"]),
    ]

    return fig, metricas, f"Kp = {Kp}", f"Ki = {Ki}", f"Kd = {Kd}"


# ── Execução ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 50)
    print("Dashboard PID — Iniciando servidor...")
    print("Acesse: http://127.0.0.1:8050")
    print("Pressione Ctrl+C para encerrar")
    print("=" * 50)
    app.run(debug=False)
