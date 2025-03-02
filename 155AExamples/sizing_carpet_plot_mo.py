import marimo

__generated_with = "0.10.4"
app = marimo.App(width="medium")


@app.cell
def _(mo):
    mo.md("""# **Aircraft wing and engine sizing**""")
    return


@app.cell(hide_code=True)
def _():
    import marimo as mo
    import plotly.graph_objects as go
    import plotly.colors as pc
    import numpy as np
    import pandas as pd
    import os
    from collections import namedtuple

    return go, mo, namedtuple, np, os, pc, pd


@app.cell(hide_code=True)
def _(mo):
    _md = mo.md(r"""
    Assuming gross weight has been estimated, we aim to perform preliminary sizing of the wing and engine,
    i.e., to estimate the wing loading ($W/S$) and thrust-to-weight ratio ($T/W$), respectively.
    These two quantities are coupled; therefore, they must be selected simultaneously 
    while considering a number of performance requirements.
    We will perform constraint analysis by generating a carpet plot, which will facilitate selection of $W/S$ and $T/W$.
    Below, we list the the critical operational conditions.
    Note: For non-jet-propelled aircraft, the quantity of interest for engine sizing is $P/W$.

    The analysis assumes the following drag polar:

    $$
    C_D = C_{D_0} + \frac{C_L^2}{\pi e AR}
    $$

    1. Stall constraint:

    $$
    \boxed{
        \frac{W}{S} \le C_{L_{\text{max}}} \frac{1}{2} \rho_{sl} V_{\text{stall}}^2
    }
    $$

    2. Climb constraint:

    $$
    \begin{array}{l}
        T - D - W \sin \gamma = 0 \\
        L - W = 0 \\ 
    \end{array}
    $$

    $$
    \boxed{
        \frac{T}{W} \ge \frac{C_{D_0} q}{W/S} + \frac{W/S}{\pi e AR q} + \sin \gamma
    }
    $$

    3. Maneuver constraint:

    $$
    \begin{array}{l}
        T - D = 0 \\
        L - nW = 0 \\ 
    \end{array}
    $$

    $$
    \boxed{
        \frac{T}{W} \ge \frac{C_{D_0} q}{W/S} + n^2 \frac{W/S}{\pi e AR q}
    }
    $$

    4. Takeoff constraint:

    $$
    d_{\text{takeoff}} = k_{\text{takeoff}} TOP
    $$

    $$
    \boxed{
        \frac{T}{W} \ge \frac{W/S}{TOP \sigma C_{L_{\text{takeoff}}}}
    }
    $$

    5. Landing constraint:

    $$
    d_{\text{landing}} - d_{\text{airborne}} = k_{\text{landing}} LP
    $$

    $$
    \boxed{
        \frac{W}{S} \le \sigma C_{L_{\text{max}}} LP
    }
    $$

    6. Ceiling constraint:

    $$
    (L/D)_{\text{max}} = \sqrt{\frac{\pi e AR}{4 C_{D_0}}}
    $$

    $$
    \boxed{
        \frac{T}{W} \ge \frac{1}{(L/D)_{\text{max}}}
    }
    $$
    """)

    mo.accordion({r"**Background**": _md})
    return


@app.cell(hide_code=True)
def _(mo):
    V_cruise_text = mo.ui.text(value="250.", label=r"Cruise speed, $V_{\text{cruise}}$ [m/s]")
    V_climb_text = mo.ui.text(value="150.", label=r"Climb speed, $V_{\text{climb}}$ [m/s]")
    V_stall_text = mo.ui.text(value="70.", label=r"Stall speed, $V_{\text{stall}}$ [m/s]")
    V_takeoff_text = mo.ui.text(value="80.", label=r"Takeoff speed, $V_{\text{takeoff}}$ [m/s]")
    V_approach_text = mo.ui.text(value="91.", label=r"Approach speed, $V_{\text{approach}}$ [m/s]")
    h_cruise_text = mo.ui.text(value="10.e3", label=r"Cruise altitude, $h_{\text{cruise}}$ [m]")
    CD_0_text = mo.ui.text(value="0.015", label=r"Parasitic drag coeff., $C_{D_0}$")
    CL_max_text = mo.ui.text(value="2.11", label=r"Max. lift coeff., $C_{L_{\text{max}}}$")
    CL_takeoff_text = mo.ui.text(value="1.6", label=r"Takeoff lift coeff., $C_{L_{\text{takeoff}}}$")
    n_text = mo.ui.text(value="2.5", label=r"Maneuver load factor, $n$")
    gamma_deg_text = mo.ui.text(value="4", label=r"Climb angle, $\gamma$ [deg]")
    e_text = mo.ui.text(value="0.85", label=r"Oswald efficiency factor, $e$")
    AR_text = mo.ui.text(value="9.8", label=r"Aspect ratio, $AR$")
    k_engine_text = mo.ui.text(value="0.4", label=r"Thrust Mach lapse, $k$ ($T/T_{sl} = 1 - k \cdot M$)")
    m_engine_text = mo.ui.text(value="0.7", label=r"Thrust density lapse, $m$ ($T/T_{sl} = (\rho/\rho_{sl})^m$))")
    h_takeoff_text = mo.ui.text(value="1609.", label=r"Takeoff density altitude, $h_{\text{takeoff}}$ [m]")
    h_landing_text = mo.ui.text(value="1609.", label=r"Landing density altitude, $h_{\text{landing}}$ [m]")
    k_takeoff_text = mo.ui.text(value="0.255", label=r"Takeoff parameter coefficient, $k_{\text{takeoff}}$ [m^3/N]")
    k_landing_text = mo.ui.text(value="0.51", label=r"Landing parameter coefficient, $k_{\text{landing}}$ [m^3/N]")
    d_landing_text = mo.ui.text(value="1500.", label=r"Landing dist., $d_{\text{landing}}$ [m]")
    d_airborne_text = mo.ui.text(value="300.", label=r"Airborne dist., $d_{\text{airborne}}$ [m]")
    d_takeoff_text = mo.ui.text(value="2900.", label=r"Takeoff dist., $d_{\text{takeoff}}$ [m]")

    mo.accordion({r"**Sizing parameters**": mo.vstack([
        mo.hstack([
            V_cruise_text,
            V_climb_text,
            V_stall_text,
        ], justify="start"),
        mo.hstack([
            V_takeoff_text,
            V_approach_text,
        ], justify="start"),
        mo.hstack([
            h_cruise_text,
            CD_0_text,
        ], justify="start"),
        mo.hstack([
            CL_max_text,
            CL_takeoff_text,
        ], justify="start"),
        mo.hstack([
            n_text,
            gamma_deg_text,
        ], justify="start"),
        mo.hstack([
            e_text,
            AR_text,
        ], justify="start"),
        mo.hstack([
            k_engine_text,
            m_engine_text,
        ], justify="start"),
        mo.hstack([
            h_takeoff_text,
            h_landing_text,
        ], justify="start"),
        mo.hstack([
            k_takeoff_text,
            k_landing_text,
        ], justify="start"),
        mo.hstack([
            d_landing_text,
            d_airborne_text,
            d_takeoff_text,
        ], justify="start"),
    ])})
    return (
        AR_text,
        CD_0_text,
        CL_max_text,
        CL_takeoff_text,
        V_approach_text,
        V_climb_text,
        V_cruise_text,
        V_stall_text,
        V_takeoff_text,
        d_airborne_text,
        d_landing_text,
        d_takeoff_text,
        e_text,
        gamma_deg_text,
        h_cruise_text,
        h_landing_text,
        h_takeoff_text,
        k_engine_text,
        k_landing_text,
        k_takeoff_text,
        m_engine_text,
        n_text,
    )


@app.cell(hide_code=True)
def _(np):
    # Implementation of standard atmosphere models

    _T0 = 288.16
    _T1 = 216.65
    _L = 6.5e-3
    _ht = 11e3

    _p0 = 101325.
    _p1 = 22632.
    _g = 9.81
    _R = 287

    _gamma = 1.4

    def compute_temperature(h):
        _h = np.atleast_1d(h)

        _T = np.empty(len(_h))

        mask = _h <= _ht
        _T[mask] = _T0 - _L * _h[mask]

        mask = _h > _ht
        _T[mask] = _T1

        if np.isscalar(h):
            _T = _T[0]

        return _T

    def compute_pressure(h):
        _h = np.atleast_1d(h)

        _T = compute_temperature(_h)

        _p = np.empty(len(_h))

        mask = _h <= _ht
        _p[mask] = _p0 * (_T[mask] / _T0) ** (_g / _L / _R)

        mask = _h > _ht
        _p[mask] = _p1 * np.exp(_g * (_ht - _h[mask]) / _R / _T1)

        if np.isscalar(h):
            _p = _p[0]

        return _p

    def compute_density(h):
        _T = compute_temperature(h)
        _p = compute_pressure(h)
        _rho = _p / _R / _T

        return _rho

    def compute_sonic_speed(h):
        _T = compute_temperature(h)
        _a = np.sqrt(_gamma * _R * _T)

        return _a
    return (
        compute_density,
        compute_pressure,
        compute_sonic_speed,
        compute_temperature,
    )


@app.cell
def _(mo):
    max_W_S_text = mo.ui.text(value="8000.", label=r"Maximum wing loading, $W/S$ [N/m^2]")

    max_W_S_text
    return (max_W_S_text,)


@app.cell(hide_code=True)
def _(max_W_S_text, np):
    # Compute wing loading linspace

    _num = 1000
    _max_W_S = float(max_W_S_text.value)

    W_S = np.linspace(1e1, _max_W_S, _num)
    return (W_S,)


@app.cell
def _(compute_density, compute_sonic_speed, k_engine_text, m_engine_text):
    _k_engine = float(k_engine_text.value)
    _m_engine = float(m_engine_text.value)

    def compute_thrust_lapse_factor(_h, _V):
        _rho = compute_density(_h)
        _rho_sl = compute_density(0.)
        _a = compute_sonic_speed(_h)
        _M = _V / _a

        return (_rho / _rho_sl) ** _m_engine * (1 - _k_engine * _M)   
    return (compute_thrust_lapse_factor,)


@app.cell
def _(CL_max_text, V_stall_text, compute_density):
    # 1. Stall constraint

    _CL_max = float(CL_max_text.value)
    _V_stall = float(V_stall_text.value)

    _rho_sl = compute_density(0.)

    W_S_stall = _CL_max * 0.5 * _rho_sl * _V_stall ** 2
    return (W_S_stall,)


@app.cell
def _(
    AR_text,
    CD_0_text,
    V_climb_text,
    V_cruise_text,
    W_S,
    compute_density,
    compute_thrust_lapse_factor,
    e_text,
    gamma_deg_text,
    h_cruise_text,
    np,
):
    # 2. Climb constraint

    _rad_deg = np.pi / 180.

    _CD_0 = float(CD_0_text.value)
    _e = float(e_text.value)
    _AR = float(AR_text.value)
    _h_cruise = float(h_cruise_text.value)
    _V_climb = float(V_climb_text.value)
    _V_cruise = float(V_cruise_text.value)
    _gamma_deg = float(gamma_deg_text.value)

    _gamma = _gamma_deg * _rad_deg

    _rho_cruise = compute_density(_h_cruise)
    _rho_sl = compute_density(0.)

    _q_top = 0.5 * _rho_cruise * _V_cruise ** 2
    _q_bot = 0.5 * _rho_sl * _V_climb ** 2

    T_W_climb_top = _CD_0 * _q_top / W_S + W_S / np.pi / _e / _AR / _q_top + np.sin(_gamma)
    T_W_climb_bot = _CD_0 * _q_bot / W_S + W_S / np.pi / _e / _AR / _q_bot + np.sin(_gamma)

    T_W_climb_top /= compute_thrust_lapse_factor(_h_cruise, _V_cruise)
    T_W_climb_bot /= compute_thrust_lapse_factor(0., _V_climb)
    return T_W_climb_bot, T_W_climb_top


@app.cell
def _(
    AR_text,
    CD_0_text,
    V_approach_text,
    V_cruise_text,
    W_S,
    compute_density,
    compute_thrust_lapse_factor,
    e_text,
    h_cruise_text,
    n_text,
    np,
):
    # 3. Maneuver constraint

    _CD_0 = float(CD_0_text.value)
    _e = float(e_text.value)
    _AR = float(AR_text.value)
    _V_approach = float(V_approach_text.value)
    _V_cruise = float(V_cruise_text.value)
    _h_cruise = float(h_cruise_text.value)
    _n = float(n_text.value)

    _rho_cruise = compute_density(_h_cruise)
    _rho_sl = compute_density(0.)

    _q_cruise = 0.5 * _rho_cruise * _V_cruise ** 2
    _q_approach = 0.5 * _rho_sl * _V_approach ** 2

    T_W_maneuver_cruise = _CD_0 * _q_cruise / W_S + _n ** 2 * W_S / np.pi / _e / _AR / _q_cruise
    T_W_maneuver_approach = _CD_0 * _q_approach / W_S + _n ** 2 * W_S / np.pi / _e / _AR / _q_approach

    T_W_maneuver_cruise /= compute_thrust_lapse_factor(_h_cruise, _V_cruise)
    T_W_maneuver_approach /= compute_thrust_lapse_factor(0., _V_approach)
    return T_W_maneuver_approach, T_W_maneuver_cruise


@app.cell
def _(
    CL_takeoff_text,
    V_takeoff_text,
    W_S,
    compute_density,
    compute_thrust_lapse_factor,
    d_takeoff_text,
    h_takeoff_text,
    k_takeoff_text,
):
    # 4. Takeoff constraint

    _h_takeoff = float(h_takeoff_text.value)
    _d_takeoff = float(d_takeoff_text.value)
    _k_takeoff = float(k_takeoff_text.value)
    _V_takeoff = float(V_takeoff_text.value)
    _CL_takeoff = float(CL_takeoff_text.value)

    _TOP = _d_takeoff / _k_takeoff
    _rho_takeoff = compute_density(_h_takeoff)
    _rho_sl = compute_density(0.)
    _sigma = _rho_takeoff / _rho_sl

    T_W_takeoff = W_S / _CL_takeoff / _sigma / _TOP
    T_W_takeoff /= compute_thrust_lapse_factor(0., _V_takeoff)
    return (T_W_takeoff,)


@app.cell
def _(
    CL_max_text,
    compute_density,
    d_airborne_text,
    d_landing_text,
    h_landing_text,
    k_landing_text,
):
    # 5. Landing constraint

    _h_landing = float(h_landing_text.value)
    _d_landing = float(d_landing_text.value)
    _d_airborne = float(d_airborne_text.value)
    _k_landing = float(k_landing_text.value)
    _CL_max = float(CL_max_text.value)

    _LP = (_d_landing - _d_airborne) / _k_landing
    _rho_landing = compute_density(_h_landing)
    _rho_sl = compute_density(0.)
    _sigma = _rho_landing / _rho_sl

    W_S_landing = _CL_max * _sigma * _LP
    return (W_S_landing,)


@app.cell
def _(
    AR_text,
    CD_0_text,
    V_cruise_text,
    W_S,
    compute_thrust_lapse_factor,
    e_text,
    h_cruise_text,
    np,
):
    # 6. Ceiling

    _CD_0 = float(CD_0_text.value)
    _e = float(e_text.value)
    _AR = float(AR_text.value)
    _V_cruise = float(V_cruise_text.value)
    _h_cruise = float(h_cruise_text.value)

    T_W_ceiling = np.sqrt(4. * _CD_0 / np.pi / _e / _AR) * np.ones(len(W_S))
    T_W_ceiling /= compute_thrust_lapse_factor(_h_cruise, _V_cruise)
    return (T_W_ceiling,)


@app.cell
def _(max_W_S_text, mo):
    # Define selected W/S and T/W sliders

    _max_W_S = float(max_W_S_text.value)

    selected_T_W_slider = mo.ui.slider(
        start=0., stop=1., step=0.01, value=0.25,
        label=r"Selected $T/W$",
        show_value=True,
    )

    selected_W_S_slider = mo.ui.slider(
        start=0., stop=_max_W_S, step=0.01, value=0.25,
        label=r"Selected $T/W$",
        show_value=True,
    )
    return selected_T_W_slider, selected_W_S_slider


@app.cell
def _():
    return


@app.cell(hide_code=True)
def _(
    T_W_ceiling,
    T_W_climb_bot,
    T_W_climb_top,
    T_W_maneuver_approach,
    T_W_maneuver_cruise,
    T_W_takeoff,
    W_S,
    W_S_landing,
    W_S_stall,
    go,
    np,
):
    # Carpet plot figure

    _num = 500

    _fig = go.Figure()

    for _W_S, _name in [
        (W_S_stall, "Stall"),
        (W_S_landing, "Landing"),
    ]:
        _fig.add_trace(go.Scatter(
            x=_W_S * np.ones(_num),
            y=np.linspace(0., 1., _num),
            mode="lines",
            # line=dict(color="blue", width=2),
            name=_name,
        ))

    for _T_W, _name in [
        (T_W_climb_top, "Top of climb"),
        (T_W_climb_bot, "Bottom of climb"),
        (T_W_maneuver_approach, "Maneuver (approach)"),
        (T_W_maneuver_cruise, "Maneuver (cruise)"),
        (T_W_takeoff, "Takeoff"),
        (T_W_ceiling, "Ceiling"),
    ]:
        _fig.add_trace(go.Scatter(
            x=W_S,
            y=_T_W,
            mode="lines",
            # line=dict(color="blue", width=2),
            name=_name,
        ))

    _fig.update_layout(
        xaxis=dict(range=[0, np.max(W_S)]),
        yaxis=dict(range=[0, 1]),
        xaxis_title=r"Wing loading, $W/S$ [N/m^2]",
        yaxis_title=r"Thrust-to-weight ratio, $T/W$",
        legend=dict(
            x=0.5,  # Center the legend horizontally
            y=-0.12,  # Place the legend below the plot
            xanchor="center",  # Align legend to its center
            orientation="h",  # Horizontal legend layout
            bordercolor="Black",
            borderwidth=1
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        height=550, width=800,
    )

    fig = _fig
    return (fig,)


@app.cell
def _(fig, mo):
    mo.hstack([fig], justify="center")
    return


if __name__ == "__main__":
    app.run()
