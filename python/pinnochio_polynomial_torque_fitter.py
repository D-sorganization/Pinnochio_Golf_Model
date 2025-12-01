#!/usr/bin/env python3
"""
Torque polynomial fitting tool.
- Fits tau(t) with an n-th degree polynomial via least squares.
- Plots original vs fitted data.
- Optionally saves polynomial coefficients to a .npy file.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np  # noqa: TID253
import numpy.typing as npt  # noqa: TID253


def fit_torque_poly(
    t: npt.ArrayLike, tau: npt.ArrayLike, degree: int = 6
) -> npt.NDArray[np.float64]:
    """
    Fit polynomial: tau(t) ≈ p(t) = c0 + c1 t + ... + cN t^N
    Returns coefficients (highest degree first, like np.polyval).
    """
    t = np.asarray(t).flatten()
    tau = np.asarray(tau).flatten()
    if t.shape != tau.shape:
        raise ValueError("t and tau must have same shape")
    coeffs = np.polyfit(t, tau, degree)
    return coeffs


def evaluate_torque_poly(
    coeffs: npt.NDArray[np.float64], t: npt.ArrayLike
) -> npt.NDArray[np.float64]:
    return np.polyval(coeffs, t)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fit polynomial to torque vs time data."
    )
    parser.add_argument("csv", help="CSV file with columns t, tau")
    parser.add_argument(
        "-d", "--degree", type=int, default=6, help="Polynomial degree (default: 6)"
    )
    parser.add_argument(
        "-o", "--out", type=str, default="", help="Output .npy file for coefficients"
    )
    args = parser.parse_args()

    data = np.loadtxt(args.csv, delimiter=",", skiprows=1)
    t = data[:, 0]
    tau = data[:, 1]

    coeffs = fit_torque_poly(t, tau, degree=args.degree)
    print(f"Fitted polynomial degree {args.degree}")
    print("Coefficients (highest-degree first):")
    print(coeffs)

    # Plot
    t_dense = np.linspace(t.min(), t.max(), 1000)
    tau_fit = evaluate_torque_poly(coeffs, t_dense)

    plt.figure()
    plt.plot(t, tau, ".", label="data")
    plt.plot(t_dense, tau_fit, "-", label=f"poly deg {args.degree}")
    plt.xlabel("time")
    plt.ylabel("torque")
    plt.legend()
    plt.title("Torque polynomial fit")
    plt.grid(True)
    plt.show()

    if args.out:
        np.save(args.out, coeffs)
        print(f"Saved coefficients to {args.out}")


if __name__ == "__main__":
    main()
