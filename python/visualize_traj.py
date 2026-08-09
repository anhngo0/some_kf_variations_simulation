import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter


def load_trajectory(json_path):
    with open(json_path, "r") as f:
        traj = json.load(f)["trajectory"]

    x = np.array([p["position"][0] for p in traj])
    y = np.array([p["position"][1] for p in traj])

    return x, y


def main():
    x, y = load_trajectory("../json/observer_traj.json")
    target_x, target_y = load_trajectory("../json/target_traj.json")

    fig, ax = plt.subplots(figsize=(8, 8))

    # Draw trajectory
    ax.plot(
        x,
        y,
        color="blue",
        linewidth=2,
        label="observer trajectory"
    )

    ax.plot(
        target_x,
        target_y,
        color="red",
        linewidth=2,
        label="tatget trajectory"
    )

    # Start point
    ax.scatter(
        x[0],
        y[0],
        color="green",
        s=80,
        label="Start",
        zorder=3
    )

    # End point
    ax.scatter(
        x[-1],
        y[-1],
        color="red",
        s=80,
        label="End",
        zorder=3
    )

   # =====================================================
    # Display both trajectories with margin
    # =====================================================
    all_x = np.concatenate((x, target_x))
    all_y = np.concatenate((y, target_y))

    xmin = all_x.min()
    xmax = all_x.max()
    ymin = all_y.min()
    ymax = all_y.max()

    x_margin = 0.1 * (xmax - xmin)
    y_margin = 0.1 * (ymax - ymin)

    # Trường hợp span = 0
    if x_margin == 0:
        x_margin = 1.0
    if y_margin == 0:
        y_margin = 1.0

    ax.set_xlim(xmin - x_margin, xmax + x_margin)
    ax.set_ylim(ymin - y_margin, ymax + y_margin)

    # Giữ đúng tỉ lệ mét trên hai trục
    ax.set_aspect("equal")

    # =====================================================
    # Scientific notation: ×10^n
    # =====================================================
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-2, 2))

    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)

    ax.ticklabel_format(style="scientific",
                        axis="both",
                        scilimits=(-2, 2))

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("2D Trajectory")

    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()