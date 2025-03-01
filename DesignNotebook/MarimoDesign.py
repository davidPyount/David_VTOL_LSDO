import marimo

__generated_with = "0.11.12"
app = marimo.App()


@app.cell
def _():
    import marimo as mo

    x = mo.ui.slider(1,5)

    x

    return mo, x


@app.cell
def _(mo):
    mo.md(r"I am figuring out marimo notebooks!!!!")
    return


@app.cell
def _(mo, x):
    mo.md(f"value of x is {x.value}")
    return


@app.cell
def _(mo):
    mo.md(r"Ok sick we can do this")
    return


if __name__ == "__main__":
    app.run()
