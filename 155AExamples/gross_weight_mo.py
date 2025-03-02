import marimo

__generated_with = "0.10.12"
app = marimo.App(width="medium")


@app.cell(hide_code=True)
def _(mo):
    mo.md("""# **Aircraft initial sizing**""")
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
    One of the first steps in aircraft conceptual design is initial sizing, which centers around estimating the gross weight of the aircraft. 
    The gross weight can be decomposed as

    $$
    W_g = W_p + W_e + W_s,
    $$

    based on the nomenclature below:

    |||
    | :-- | :-- |
    | $W_g$ | Gross weight |
    | $W_p$ | Payload weight |
    | $W_e$ | Empty weight | 
    | $W_s$ | Energy-storage weight (fuel, batteries, or both) |
    | $W_f$ | Fuel weight |
    | $W_b$ | Battery weight |

    It is easier at this point to estimate weight fractions, rather than absolute weight values, based on historical data and requirements.
    Therefore, we rewrite as

    $$
    W_g = W_p + \left( \frac{W_e}{W_g} \right) W_g + \left( \frac{W_p}{W_s} \right) W_g,
    $$

    and rearrange to obtain

    $$
    W_g = \frac{W_p}{1 - \frac{W_e}{W_g} - \frac{W_s}{W_g}}.
    $$

    Historically, empty weight fraction $W_e/W_g$ follows a linear trend versus gross weight on a log-log scale.
    Therefore, we can write

    $$
    \log \left( \frac{W_e}{W_g} \right) = b \log W_g + \log a
    \quad \text{and} \quad
    \frac{W_e}{W_g} = a W_g^b,
    $$

    where $a$ and $b$ are constants that must be determined from historical data.

    Energy-storage weight fraction $W_s/W_g$ can be related to the aircraft range via what are known as range equations.
    The precise form of the equation differs depending on the source of energy.

    - Jet-propelled aircraft:

    $$
    R = \frac{V}{c_T \cdot g} \frac{L}{D} \ln \left( \frac{1}{1 - \frac{W_f}{W_g}} \right)
    $$

    - Turbo-electric aircraft:

    $$
    R = \frac{\eta_s}{c_P \cdot g} \frac{L}{D} \ln \left( \frac{1}{1 - \frac{W_f}{W_g}} \right)
    $$

    - Turbo-electric aircraft (alternate, equivalent form):

    $$
    R = \frac{\eta_s \cdot e_f}{g} \frac{L}{D} \ln \left( \frac{1}{1 - \frac{W_f}{W_g}} \right)
    $$

    - Pure-electric aircraft:

    $$
    R = \frac{\eta_b \cdot e_b}{g} \frac{L}{D} \frac{W_b}{W_g}
    $$

    |||
    | :-- | :-- |
    | $R$ | Range |
    | $V$ | Cruise speed |
    | $g$ | Acceleration due to gravity | 
    | $L/D$ | Lift-to-drag ratio |
    | $c_T$ | Thrust-specific fuel consumption (TSFC): fuel burn rate by mass per unit thrust |
    | $c_P$ | Brake-specific fuel consumption (BSFC): fuel burn rate by mass per unit power |
    | $\eta_s$ | Conversion efficiency from shaft power to thrust power |
    | $\eta_b$ | Conversion efficiency from battery power to thrust power |
    | $e_b$ | Battery specific energy (energy per unit mass) |
    | $e_f$ | Fuel specific energy (energy per unit mass) |

    Turbo-electric aircraft use energy from fuel that is in part or in full converted to electrical energy to power motors.
    Hybrid electric aircraft use energy from a combination of fuel and batteries. 
    The range equation for this class of aircraft is much more complicated, so it is excluded here.

    Below are the rearranged forms of the above range equations that isolate $W_s/W_g$:

    - Jet-propelled aircraft:

    $$
    \frac{W_f}{W_g} = 1 - \exp \left( -\frac{R \cdot c_T \cdot g}{\frac{L}{D} \cdot V} \right)
    $$

    - Turbo-electric aircraft:

    $$
    \frac{W_f}{W_g} = 1 - \exp \left( -\frac{R \cdot c_P \cdot g}{\frac{L}{D} \cdot \eta_s} \right)
    $$

    - Turbo-electric aircraft (alternate, equivalent form):

    $$
    \frac{W_f}{W_g} = 1 - \exp \left( -\frac{R \cdot g}{\frac{L}{D} \cdot \eta_s \cdot e_f} \right)
    $$

    - Pure-electric aircraft:

    $$
    \frac{W_b}{W_g} = \frac{R \cdot g}{\frac{L}{D} \cdot \eta_b \cdot e_b}
    $$

    Using these methods for estimating $W_e/W_g$ and $W_s/W_g$,
    we can compute $W_g$ by driving the following residual function to zero:

    $$
    R(W_g) = W_g - \frac{W_p}{1 - \frac{W_e}{W_g} - \frac{W_s}{W_g}}.
    $$
    """)

    mo.accordion({r"**Background**": _md})
    return


@app.cell(hide_code=True)
def _(mo):
    aircraft_type_dropdown = mo.ui.dropdown(
        ["Jet-propelled", "Turbo-electric", "Turbo-electric (alternate)", "Pure-electric"],
        label="Aircraft type", value="Jet-propelled",
    )
    return (aircraft_type_dropdown,)


@app.cell(hide_code=True)
def _(aircraft_type_dropdown, mo):
    # Sizing parameter inputs

    Wp_text = mo.ui.text(value="562000", label=r"Payload weight, $W_p$ [N]")
    R_text = mo.ui.text(value="11.7e6", label=r"Range, $R$ [m]")
    V_text = mo.ui.text(value="250", label=r"Cruise speed, $V$ [m/s]")
    ewf_a_text = mo.ui.text(value="0.97", label=r"Coefficient, $a$")
    ewf_b_text = mo.ui.text(value="-0.06", label=r"Exponent, $b$")
    LD_text = mo.ui.text(value="20", label=r"Lift-to-drag ratio, $L/D$")
    cT_text = mo.ui.text(value="1.45e-5", label=r"TSFC, $c_T$ [kg/N/s]")
    cP_text = mo.ui.text(value="5e-8", label=r"BSFC, $c_P$ [kg*s/W]")
    ns_text = mo.ui.text(value="0.9", label=r"Efficiency, shaft to thrust power, $\eta_s$")
    nb_text = mo.ui.text(value="0.9", label=r"Efficiency, battery to thrust power, $\eta_b$")
    eb_text = mo.ui.text(value="7e5", label=r"Battery specific energy, $e_b$ [J/kg]")
    ef_text = mo.ui.text(value="4.4e7", label=r"Battery specific energy, $e_b$ [J/kg]")

    mo.accordion({r"**Sizing parameters**": mo.vstack([
        aircraft_type_dropdown,
        mo.hstack([
            mo.md(r"Empty weight fraction regression parameters:"), ewf_a_text, ewf_b_text,
        ]),
        mo.hstack([
            Wp_text,
            R_text,
            V_text,
        ], justify="start"),
        mo.hstack([
            LD_text,
            cT_text,
            cP_text,
        ], justify="start"),
        mo.hstack([
            ns_text,
            nb_text,
        ], justify="start"),
        mo.hstack([
            eb_text,
            ef_text,
        ], justify="start"),
    ])})
    return (
        LD_text,
        R_text,
        V_text,
        Wp_text,
        cP_text,
        cT_text,
        eb_text,
        ef_text,
        ewf_a_text,
        ewf_b_text,
        nb_text,
        ns_text,
    )


@app.cell(hide_code=True)
def _(
    LD_text,
    R,
    R_text,
    V_text,
    Wp_text,
    aircraft_type_dropdown,
    cP_text,
    cT_text,
    eb_text,
    ef_text,
    ewf_a_text,
    ewf_b_text,
    nb_text,
    np,
    ns_text,
):
    # Define root finding problem

    _Wp = float(Wp_text.value)
    _R = float(R_text.value)
    _V = float(V_text.value)
    _ewf_a = float(ewf_a_text.value)
    _ewf_b = float(ewf_b_text.value)
    _LD = float(LD_text.value)
    _cT = float(cT_text.value)
    _cP = float(cP_text.value)
    _ns = float(ns_text.value)
    _nb = float(nb_text.value)
    _eb = float(eb_text.value)
    _ef = float(ef_text.value)

    _aircraft_type = aircraft_type_dropdown.value
    _g = 9.81        
    if _aircraft_type == "Jet-propelled":
        _Ws_Wg = 1 - np.exp(-_R * _cT * _g / _LD / _V)
    elif _aircraft_type == "Turbo-electric":
        _Ws_Wg = 1 - np.exp(-_R * _cP * _g / _LD / _ns)
    elif _aircraft_type == "Turbo-electric (alternate)":
        _Ws_Wg = 1 - np.exp(-_R * _g / _LD / _ns / _ef)
    elif _aircraft_type == "Pure-electric":
        _Ws_Wg = _R * _g / _LD / _nb / _eb

    Ws_Wg = _Ws_Wg

    def compute_We_Wg(_Wg):
        return _ewf_a * _Wg ** _ewf_b

    def compute_rhs(_Wg):
        _We_Wg = compute_We_Wg(_Wg)
        return _Wp / (1.0 - _We_Wg - _Ws_Wg)
    return Ws_Wg, compute_We_Wg, compute_rhs


@app.cell(hide_code=True)
def _(mo):
    # Parameter sweep inputs

    Wg_bounds_md = mo.md(r"Bounds for parameter sweep:")
    Wg_lower_text = mo.ui.text(value="600000", label=r"Lower bound [N]")
    Wg_upper_text = mo.ui.text(value="3000000", label=r"Upper bound [N]")
    return Wg_bounds_md, Wg_lower_text, Wg_upper_text


@app.cell(hide_code=True)
def _(Wg_lower_text, Wg_upper_text, mo):
    # Parameters for root finding algorithm

    _Wg_lower = float(Wg_lower_text.value)
    _Wg_upper = float(Wg_upper_text.value)

    Wg_guess_slider = mo.ui.slider(
        start=_Wg_lower, stop=_Wg_upper, 
        step=(_Wg_upper - _Wg_lower) / 100,
        value=(_Wg_upper + _Wg_lower) / 2,
        label=r"Initial guess for $W_g$",
        show_value=True,
    )

    maxiter_slider = mo.ui.slider(
        start=1, stop=1000, step=1, value=100,
        label=r"Maximum number of iterations",
        show_value=True,
    )

    tol_input = mo.ui.slider(
        label="Convergence tolerance (10^x)",
        start=-16, stop=3, step=1, value=-3,
        show_value=True,
    )
    return Wg_guess_slider, maxiter_slider, tol_input


@app.cell(hide_code=True)
def _(Wg_guess_slider, compute_rhs, maxiter_slider, mo, tol_input):
    # Root finding algorithm

    _Wg = float(Wg_guess_slider.value)
    _maxiter = int(maxiter_slider.value)
    _tol = 10 ** float(tol_input.value)

    with mo.capture_stdout() as buffer:
        print("Running root-finding algorithm")
        print(f"Iteration: {0}. Gross weight: {_Wg}. Residual: {abs(_Wg - compute_rhs(_Wg))}")
        for _index in range(_maxiter):
            _residual = _Wg - compute_rhs(_Wg)
            _error = abs(_residual)
            if _error < _tol:
                break
            _Wg -= _residual
            print(f"Iteration: {_index + 1}. Gross weight: {_Wg}. Residual: {_error}")

    Wg = _Wg
    return Wg, buffer


@app.cell(hide_code=True)
def _(np):
    def numpy_round_sigfig(arr, _sig=3):
        """
        Round a NumPy array to the specified number of significant figures.
        Parameters:
            arr (array-like): Input array
        Returns:
            numpy.ndarray: Array rounded to the specified significant figures
        """
        arr = np.asarray(arr, dtype=float)
        _nonzero = arr != 0
        _scale = 10 ** (_sig - 1 - np.floor(np.log10(np.abs(arr[_nonzero]))))
        arr[_nonzero] = np.round(arr[_nonzero] * _scale) / _scale
        return arr
    return (numpy_round_sigfig,)


@app.cell(hide_code=True)
def _(Wg, Wp_text, Ws_Wg, compute_We_Wg, mo):
    # Solution table

    _We_Wg = compute_We_Wg(Wg)
    _We = _We_Wg * Wg
    _Ws = Ws_Wg * Wg
    _Wp = float(Wp_text.value)

    _desc = "Description"
    _units = "Units"
    _value = "Value"

    solution_table = mo.md(f"""
    | | | | |
    | -- | -- | -- | -- |
    | Gross weight | $W_g$ | [N] | {Wg} |
    | Empty weight | $W_e$ | [N] | {_We} |
    | Payload weight | $W_p$ | [N] | {_Wp} |
    | Energy-storage weight | $W_s$ | [N] | {_Ws} |
    | Empty weight fraction | $W_e/W_g$ |  | {_We_Wg} |
    | Energy-storage weight fraction | $W_s/W_g$ | | {Ws_Wg} |
    """)
    return (solution_table,)


@app.cell(hide_code=True)
def _(
    Wg_guess_slider,
    buffer,
    maxiter_slider,
    mo,
    solution_table,
    tol_input,
):
    mo.accordion({r"**Computation of gross weight**": mo.vstack([
        mo.hstack([
            Wg_guess_slider, maxiter_slider, tol_input,
        ]),  
        mo.callout(mo.vstack([
            mo.md(f"`{line}`") for line in buffer.getvalue().splitlines() 
        ]), kind="info"),
        solution_table,
    ])})
    return


@app.cell(hide_code=True)
def _(compute_rhs, go, np):
    # Gross weight sweep figure

    _num = 500

    _fig = go.Figure()

    _Wg = np.linspace(1e3, 3e6, _num)

    _fig.add_trace(go.Scatter(
        x=_Wg,
        y=_Wg - compute_rhs(_Wg),
        mode="lines",
        line=dict(color="blue", width=2),
        name="",
    ))

    _fig.update_layout(
        xaxis_title="Gross weight [N]",
        yaxis_title="Residual [N]",
        margin=dict(l=0, r=0, t=0, b=0),
        height=500, width=1080,
    )

    gross_weight_sweep_fig = _fig
    return (gross_weight_sweep_fig,)


@app.cell(hide_code=True)
def _(
    Wg_bounds_md,
    Wg_lower_text,
    Wg_upper_text,
    gross_weight_sweep_fig,
    mo,
):
    mo.accordion({r"**Gross weight sweep**": mo.vstack([
        mo.hstack([Wg_bounds_md, Wg_lower_text, Wg_upper_text]),
        gross_weight_sweep_fig,
    ])}, lazy=True)
    return


@app.cell(hide_code=True)
def _(mo):
    # Parameter sweep variable dropdown

    parameter_sweep_variable_dropdown = mo.ui.dropdown(
        value="Payload weight",
        options=[
            "Payload weight", "Range", "Cruise speed", 
            r"Coefficient, $a$", r"Coefficient, $b$",
            "Lift-to-drag ratio", "TSFC", "BSFC",
            "Efficiency, shaft to thrust power",
            "Efficiency, battery to thrust power",
            "Battery specific energy", "Fuel specific energy",
        ], 
        label="Sweep variable",
    )
    return (parameter_sweep_variable_dropdown,)


@app.cell(hide_code=True)
def _(
    LD_text,
    R_text,
    V_text,
    Wp_text,
    cP_text,
    cT_text,
    eb_text,
    ef_text,
    ewf_a_text,
    ewf_b_text,
    mo,
    nb_text,
    ns_text,
    parameter_sweep_variable_dropdown,
):
    # Parameter sweep inputs

    _sweep_variable = parameter_sweep_variable_dropdown.value

    if _sweep_variable == "Payload weight":
        _value = float(Wp_text.value)
    elif _sweep_variable == "Range":
        _value = float(R_text.value)
    elif _sweep_variable == "Cruise speed":
        _value = float(V_text.value)
    elif _sweep_variable == r"Coefficient, $a$":
        _value = float(ewf_a_text.value)
    elif _sweep_variable == r"Coefficient, $b$":
        _value = float(ewf_b_text.value)
    elif _sweep_variable == "Lift-to-drag ratio":
        _value = float(LD_text.value)
    elif _sweep_variable == "TSFC":
        _value = float(cT_text.value)
    elif _sweep_variable == "BSFC":
        _value = float(cP_text.value)
    elif _sweep_variable == "Efficiency, shaft to thrust power":
        _value = float(ns_text.value)
    elif _sweep_variable == "Efficiency, battery to thrust power":
        _value = float(nb_text.value)
    elif _sweep_variable == "Battery specific energy":
        _value = float(eb_text.value)
    elif _sweep_variable == "Fuel specific energy":
        _value = float(ef_text.value)
    else:
        raise Exception()

    parameter_sweep_lower_text = mo.ui.text(value=str(_value - 0.5 * _value), label="Lower bound")
    parameter_sweep_upper_text = mo.ui.text(value=str(_value + _value), label="Upper bound")
    parameter_sweep_num = mo.ui.slider(
        start=1, stop=1000, step=1, value=100,
        show_value=True, label="Number of sweep points", 
    )
    return (
        parameter_sweep_lower_text,
        parameter_sweep_num,
        parameter_sweep_upper_text,
    )


@app.cell(hide_code=True)
def _(
    LD_text,
    R,
    R_text,
    V_text,
    Wp_text,
    aircraft_type_dropdown,
    cP_text,
    cT_text,
    eb_text,
    ef_text,
    ewf_a_text,
    ewf_b_text,
    nb_text,
    np,
    ns_text,
    parameter_sweep_lower_text,
    parameter_sweep_num,
    parameter_sweep_upper_text,
    parameter_sweep_variable_dropdown,
):
    # Define root finding problem for parameter sweep

    _Wp = float(Wp_text.value)
    _R = float(R_text.value)
    _V = float(V_text.value)
    _ewf_a = float(ewf_a_text.value)
    _ewf_b = float(ewf_b_text.value)
    _LD = float(LD_text.value)
    _cT = float(cT_text.value)
    _cP = float(cP_text.value)
    _ns = float(ns_text.value)
    _nb = float(nb_text.value)
    _eb = float(eb_text.value)
    _ef = float(ef_text.value)

    _lower = float(parameter_sweep_lower_text.value)
    _upper = float(parameter_sweep_upper_text.value)
    _num = int(parameter_sweep_num.value)
    _linspace = np.linspace(_lower, _upper, _num)

    sweep_linspace = _linspace

    _sweep_variable = parameter_sweep_variable_dropdown.value
    if _sweep_variable == "Payload weight":
        _Wp = _linspace
    elif _sweep_variable == "Range":
        _R = _linspace
    elif _sweep_variable == "Cruise speed":
        _V = _linspace
    elif _sweep_variable == r"Coefficient, $a$":
        _ewf_a = _linspace
    elif _sweep_variable == r"Coefficient, $b$":
        _ewf_b = _linspace
    elif _sweep_variable == "Lift-to-drag ratio":
        _LD = _linspace
    elif _sweep_variable == "TSFC":
        _cT = _linspace
    elif _sweep_variable == "BSFC":
        _cP = _linspace
    elif _sweep_variable == "Efficiency, shaft to thrust power":
        _ns = _linspace
    elif _sweep_variable == "Efficiency, battery to thrust power":
        _nb = _linspace
    elif _sweep_variable == "Battery specific energy":
        _eb = _linspace
    elif _sweep_variable == "Fuel specific energy":
        _ef = _linspace
    else:
        raise Exception()

    _aircraft_type = aircraft_type_dropdown.value
    _g = 9.81        
    if _aircraft_type == "Jet-propelled":
        _Ws_Wg = 1 - np.exp(-_R * _cT * _g / _LD / _V)
    elif _aircraft_type == "Turbo-electric":
        _Ws_Wg = 1 - np.exp(-_R * _cP * _g / _LD / _ns)
    elif _aircraft_type == "Turbo-electric (alternate)":
        _Ws_Wg = 1 - np.exp(-_R * _g / _LD / _ns / _ef)
    elif _aircraft_type == "Pure-electric":
        _Ws_Wg =_R * _g / _LD / _nb / _eb

    def compute_We_Wg_sweep(_Wg):
        return _ewf_a * _Wg ** _ewf_b

    def compute_rhs_sweep(_Wg):
        _We_Wg = compute_We_Wg_sweep(_Wg)
        return _Wp / (1.0 - _We_Wg - _Ws_Wg)
    return compute_We_Wg_sweep, compute_rhs_sweep, sweep_linspace


@app.cell(hide_code=True)
def _(
    Wg_guess_slider,
    compute_rhs_sweep,
    go,
    maxiter_slider,
    np,
    parameter_sweep_variable_dropdown,
    sweep_linspace,
    tol_input,
):
    # General parameter sweep figure

    _sweep_variable = parameter_sweep_variable_dropdown.value

    _Wg = float(Wg_guess_slider.value)
    _maxiter = int(maxiter_slider.value)
    _tol = 10 ** float(tol_input.value)

    for _index in range(_maxiter):
        _residual = _Wg - compute_rhs_sweep(_Wg)
        _error = np.max(abs(_residual))
        if _error < _tol:
            break
        _Wg -= _residual

    _fig = go.Figure()

    try:
        _fig.add_trace(go.Scatter(
            x=sweep_linspace,
            y=_Wg,
            mode="lines",
            line=dict(color="blue", width=2),
            name="",
        ))
    except:
        pass

    _fig.update_layout(
        xaxis_title=_sweep_variable,
        yaxis_title="Gross weight [N]",
        margin=dict(l=0, r=0, t=0, b=0),
        height=500, width=1080,
    )

    parameter_sweep_fig = _fig
    return (parameter_sweep_fig,)


@app.cell(hide_code=True)
def _(
    mo,
    parameter_sweep_fig,
    parameter_sweep_lower_text,
    parameter_sweep_num,
    parameter_sweep_upper_text,
    parameter_sweep_variable_dropdown,
):
    mo.accordion({r"**Parameter sweep**": mo.vstack([
        parameter_sweep_variable_dropdown,
        mo.hstack([parameter_sweep_lower_text, parameter_sweep_upper_text, parameter_sweep_num]),
        parameter_sweep_fig,
    ])}, lazy=True)
    return


if __name__ == "__main__":
    app.run()
