#!/usr/bin/env python3
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/home/ivan/ros2_ws/src/shrimp_modu")

from shrimp_modu.pure_pursuit_node import build_controller_reference_path


def build_reference_path(trajectory_type="figure8"):
    return build_controller_reference_path(trajectory_type)


def extract_trajectory_type(log_path):
    try:
        with log_path.open("r", encoding="utf-8") as f:
            first_line = f.readline().strip()
    except OSError:
        return "figure8"

    for marker in ("trajectory_type=", "trayectoria="):
        if marker in first_line:
            value = first_line.split(marker, 1)[1].split()[0].strip()
            return value or "figure8"

    return "figure8"


def load_log():
    candidates = [
        Path.cwd() / "log_pp_corregido.txt",
        Path("/home/ivan/ros2_ws/log_pp_corregido.txt"),
        Path.cwd() / "log_pp.txt",
        Path("/home/ivan/ros2_ws/log_pp.txt"),
        Path.cwd() / "log_pp_ackermann.txt",
        Path("/home/ivan/ros2_ws/log_pp_ackermann.txt"),
        Path.cwd() / "log_pp_figure8.txt",
        Path("/home/ivan/ros2_ws/log_pp_figure8.txt"),
        Path.cwd() / "log_pp_circle.txt",
        Path("/home/ivan/ros2_ws/log_pp_circle.txt"),
        Path.cwd() / "log_pp_s_curve.txt",
        Path("/home/ivan/ros2_ws/log_pp_s_curve.txt"),
        Path.cwd() / "log_pp_spline.txt",
        Path("/home/ivan/ros2_ws/log_pp_spline.txt"),
        Path.cwd() / "log_pp_waypoints.txt",
        Path("/home/ivan/ros2_ws/log_pp_waypoints.txt"),
    ]

    existing_candidates = []
    for path in candidates:
        if path.exists() and path.stat().st_size > 0:
            existing_candidates.append(path)

    for path in sorted(existing_candidates, key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            data = np.loadtxt(path)
        except ValueError:
            # Compatibilidad con logs viejos cuyo header no estaba comentado.
            data = np.loadtxt(path, skiprows=1)
        data = np.atleast_2d(data)
        if data.shape[1] >= 3:
            return path, data

    raise FileNotFoundError(
        "No encontre un log valido. Busque en log_pp_corregido.txt, log_pp.txt y log_pp_ackermann.txt"
    )


def main():
    log_path, data = load_log()
    trajectory_type = extract_trajectory_type(log_path)

    x = data[:, 0]
    y = data[:, 1]
    error = data[:, 2]

    waypoints, px, py = build_reference_path(trajectory_type)

    print(f"Usando log: {log_path}")
    print(f"Trayectoria de referencia: {trajectory_type}")
    print(f"Muestras: {len(data)}")
    print(
        f"Error min/mean/max: {error.min():.3f} / {error.mean():.3f} / {error.max():.3f}"
    )
    if np.allclose(x, x[0]) and np.allclose(y, y[0]):
        print(
            "Aviso: la trayectoria real del log esta congelada en un solo punto. "
            "Eso normalmente significa que pure_pursuit no recibio /odom."
        )

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.plot(px, py, "b--", linewidth=2, label="trayectoria ideal")
    if waypoints is not None:
        ax1.scatter(
            waypoints[:, 0], waypoints[:, 1],
            c="royalblue", s=30, alpha=0.8, zorder=2, label="waypoints"
        )
    ax1.plot(x, y, "r", linewidth=2, zorder=3, label="trayectoria real")
    ax1.scatter([x[0]], [y[0]], c="green", s=80, zorder=4, label="inicio")
    ax1.scatter([x[-1]], [y[-1]], c="black", s=80, zorder=5, label="fin")
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_title(f"Seguimiento de trayectoria ({trajectory_type})")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.plot(error, color="darkorange", linewidth=2)
    ax2.set_title("Error en el tiempo")
    ax2.set_xlabel("iteraciones")
    ax2.set_ylabel("error [m]")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = Path("/home/ivan/ros2_ws/plot_pp.png")
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Grafica guardada en: {output_path}")
    if "agg" in plt.get_backend().lower():
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
