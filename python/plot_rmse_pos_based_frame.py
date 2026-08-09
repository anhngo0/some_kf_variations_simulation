import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, MultipleLocator


def load_rmse(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)["rmse_pos"]

    frame = np.array([item["frame"] for item in data])
    rmse = np.array([item["rmse"] for item in data])

    return frame, rmse


def main():
    frame, rmse_ukf = load_rmse("../json/rmse_pos_ukf_015.json")
    _, rmse_ekf = load_rmse("../json/rmse_pos_ekf_015.json")
    _, rmse_pcrlb = load_rmse("../json/rmse_pos_pcrlb_015.json")
    _, rmse_upf_500 = load_rmse("../json/rmse_pos_upf_500_015.json")
    _, rmse_pf_500 = load_rmse("../json/rmse_pos_pf_500_015.json")
    _, rmse_pf_5000 = load_rmse("../json/rmse_pos_pf_015.json")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16,12))

    # ---------------- Tổng quan ----------------
    ax1.plot(frame, rmse_ukf, label="UKF", color="blue")
    ax1.plot(frame, rmse_ekf, label="EKF", color="red")
    ax1.plot(frame, rmse_pcrlb, label="PCRLB", color="black")
    ax1.plot(frame, rmse_upf_500, label="UPF 5000", color="green")
    ax1.plot(frame, rmse_pf_500, linestyle="-.", label="Weighted PF 500", color="gray")
    ax1.plot(frame, rmse_pf_5000, label="Weighted PF 5000", color="gray")

    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-2, 2))
    ax1.yaxis.set_major_formatter(formatter)

    ax1.set_title("Position RMSE vs Frame (Overview)")
    ax1.set_xlabel("Frame")
    ax1.set_ylabel("RMSE (m)")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend()

    # ---------------- Zoom ----------------
    ax2.plot(frame, rmse_ukf, label="UKF", color="blue")
    ax2.plot(frame, rmse_ekf, label="EKF", color="red")
    ax2.plot(frame, rmse_pcrlb, label="PCRLB", color="black")
    ax2.plot(frame, rmse_upf_500, label="UPF 5000", color="green")
    ax2.plot(frame, rmse_pf_500, linestyle="-.", label="weighted PF 500", color="gray")
    ax2.plot(frame, rmse_pf_5000, label="Weighted PF 5000", color="gray")

    ax2.set_xlim(150, 210)      # zoom frame
    ax2.set_ylim(500, 25000)        # zoom RMSE

    ax2.yaxis.set_major_locator(MultipleLocator(1000))

    ax2.set_title("Zoomed View")
    ax2.set_xlabel("Frame")
    ax2.set_ylabel("RMSE (m)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()