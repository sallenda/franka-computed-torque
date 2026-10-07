#!/usr/bin/env python3
"""Graficos estilo Figs. 5-8 del paper a partir de los log.csv de `experimento`.

Por cada joint genera una figura con tres paneles:
  a) posicion q_i y referencia q_di
  b) error e_i = q_di - q_i con la banda +-e_max (zoom, como en el paper)
  c) torque tau_i con el limite +-tau_max

Uso:
  python3 scripts/graficar.py resultados/vsc resultados/fl resultados/ff
  python3 scripts/graficar.py resultados/fl --joints 1,2,4 --salida figuras/

Cada carpeta es una corrida (su nombre se usa como etiqueta de la serie).
"""

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

# Paleta categorica en orden fijo (slots 1-3 validados en todos los pares,
# incluido daltonismo). La referencia y los limites van en grises neutros.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4", "#008300"]
REFERENCIA = "#52514e"
LIMITE = "#9a9994"
TINTA = "#0b0b0b"
TINTA_2 = "#52514e"
GRILLA = "#e6e5e1"


def estilo_ejes(ax):
    ax.grid(True, color=GRILLA, linewidth=0.8)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    for lado in ("left", "bottom"):
        ax.spines[lado].set_color(LIMITE)
    ax.tick_params(colors=TINTA_2, labelsize=9)
    ax.yaxis.label.set_color(TINTA)
    ax.xaxis.label.set_color(TINTA)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("corridas", nargs="+", help="carpetas con log.csv y parametros.csv")
    p.add_argument("--joints", default="1,2,3,4,5,6,7", help="p.ej. 1,2")
    p.add_argument("--salida", default="figuras", help="carpeta de salida")
    p.add_argument("--zoom", type=float, default=2.0,
                   help="rango del panel de error en multiplos de e_max")
    args = p.parse_args()

    corridas = []
    for carpeta in args.corridas:
        log = pd.read_csv(os.path.join(carpeta, "log.csv"))
        par = pd.read_csv(os.path.join(carpeta, "parametros.csv")).set_index("joint")
        corridas.append((os.path.basename(os.path.normpath(carpeta)), log, par))
    if len(corridas) > len(SERIES):
        raise SystemExit(f"maximo {len(SERIES)} corridas por figura")

    os.makedirs(args.salida, exist_ok=True)
    for j in (int(x) for x in args.joints.split(",")):
        fig, (ax_q, ax_e, ax_t) = plt.subplots(3, 1, figsize=(8, 8.5), sharex=True)
        _, log0, par0 = corridas[0]

        ax_q.plot(log0["t"], log0[f"qd{j}"], color=REFERENCIA, linewidth=2,
                  linestyle=(0, (4, 3)), label=f"$q_{{d{j}}}$")
        for k, (nombre, log, _) in enumerate(corridas):
            c = SERIES[k]
            e = log[f"qd{j}"] - log[f"q{j}"]
            ax_q.plot(log["t"], log[f"q{j}"], color=c, linewidth=2, label=f"$q_{j}$ {nombre}")
            ax_e.plot(log["t"], e, color=c, linewidth=1.5, label=f"$e_{j}$ {nombre}")
            ax_t.plot(log["t"], log[f"tau{j}"], color=c, linewidth=1.5,
                      label=fr"$\tau_{j}$ {nombre}")

        e_max = par0.loc[j, "e_max"]
        tau_max = par0.loc[j, "tau_max"]
        for signo in (1, -1):
            ax_e.axhline(signo * e_max, color=LIMITE, linestyle=(0, (4, 3)), linewidth=1.2)
            ax_t.axhline(signo * tau_max, color=LIMITE, linestyle=(0, (4, 3)), linewidth=1.2)
        ax_e.set_ylim(-args.zoom * e_max, args.zoom * e_max)
        ax_e.text(log0["t"].iloc[-1], e_max, r"  $\pm e_{max}$", color=TINTA_2,
                  va="center", fontsize=9, clip_on=False)
        ax_t.text(log0["t"].iloc[-1], tau_max, r"  $\pm \tau_{max}$", color=TINTA_2,
                  va="center", fontsize=9, clip_on=False)

        ax_q.set_ylabel(f"$q_{j}$ [rad]")
        ax_e.set_ylabel(f"$e_{j}$ [rad]")
        ax_t.set_ylabel(fr"$\tau_{j}$ [Nm]")
        ax_t.set_xlabel("t [s]")
        for ax, letra in zip((ax_q, ax_e, ax_t), "abc"):
            estilo_ejes(ax)
            ax.set_title(f"{letra})", loc="left", fontsize=10, color=TINTA)
        # Una sola leyenda arriba: referencia + una entrada por corrida
        handles, _ = ax_q.get_legend_handles_labels()
        etiquetas = [f"$q_{{d{j}}}$ referencia"] + [nombre for nombre, _, _ in corridas]
        fig.legend(handles, etiquetas, loc="upper right", ncol=len(etiquetas),
                   frameon=False, fontsize=9, labelcolor=TINTA_2,
                   bbox_to_anchor=(0.9, 0.995))
        fig.suptitle(f"Joint {j}", color=TINTA, fontsize=12, x=0.08, y=0.985,
                     ha="left")
        fig.tight_layout(rect=(0, 0, 0.9, 0.97))
        ruta = os.path.join(args.salida, f"joint{j}.png")
        fig.savefig(ruta, dpi=150, facecolor="white")
        plt.close(fig)
        print("escrito", ruta)


if __name__ == "__main__":
    main()
