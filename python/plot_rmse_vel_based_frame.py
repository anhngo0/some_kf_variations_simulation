import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter


def load_rmse(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)["rmse_vel"]

    frame = np.array([item["frame"] for item in data])
    rmse = np.array([item["rmse"] for item in data])

    return frame, rmse


def main():
    frame, rmse_ukf = load_rmse("../json/rmse_pos_ukf_015.json")

    frame2, rmse_ekf = load_rmse("../json/rmse_pos_ekf_015.json")

    frame3, rmse_pcrlb = load_rmse("../json/rmse_pos_pcrlb_015.json")

    fig, ax = plt.subplots(figsize=(10, 5))

    # RMSE curve
    ax.plot(
        frame,
        rmse_ukf,
        color="blue",
        linewidth=1,
        label="UKF"
    )

    ax.plot(
        frame,
        rmse_ekf,
        color="red",
        linewidth=1,
        label="EKF"
    )   

    ax.plot(
        frame,
        rmse_pcrlb,
        color="black",
        linewidth=1,
        label="PCRLB"
    )   

    # Scientific notation (×10^n)
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-2, 2))

    ax.yaxis.set_major_formatter(formatter)

    ax.set_xlabel("Frame")
    ax.set_ylabel("RMSE (m/s)")
    ax.set_title("Velocity RMSE vs Frame")

    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()