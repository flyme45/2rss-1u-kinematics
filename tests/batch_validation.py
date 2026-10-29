#!/usr/bin/env python3
"""Deterministic 1000-pose validation for the three public 2-RSS-1U geometries."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import reference_validation as reference


DEFAULT_CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "configs"


@dataclass
class Metrics:
    name: str
    samples: int
    closure_max: float
    closure_rms: float
    roundtrip_max: float
    roundtrip_rms: float
    jacobian_max: float
    jacobian_relative_max: float
    jacobian_relative_rms: float


def configurations(
    config_directory: Path = DEFAULT_CONFIG_DIRECTORY,
) -> list[tuple[str, list[reference.Branch]]]:
    return [
        (
            "I: fixed x axes",
            reference.load_geometry_config(config_directory / "validation-I.json"),
        ),
        (
            "II: fixed y axes",
            reference.load_geometry_config(config_directory / "validation-II.json"),
        ),
        (
            "III: nonparallel spatial axes",
            reference.load_geometry_config(config_directory / "validation-III.json"),
        ),
    ]


def pose_grid() -> np.ndarray:
    """Exactly 1000 poses over a nonsingular common validation range."""
    pitch = np.linspace(-0.20, 0.20, 40)
    roll = np.linspace(-0.20, 0.20, 25)
    return np.array([(p, r) for p in pitch for r in roll])


def finite_difference_jacobian(
    branches: list[reference.Branch],
    q: np.ndarray,
    theta_reference: np.ndarray,
    step: float,
) -> np.ndarray:
    result = np.zeros((2, 2))
    for column in range(2):
        plus = q.copy()
        minus = q.copy()
        plus[column] += step
        minus[column] -= step
        result[:, column] = (
            reference.inverse(branches, plus, theta_reference)
            - reference.inverse(branches, minus, theta_reference)
        ) / (2.0 * step)
    return result


def validate_configuration(
    name: str,
    branches: list[reference.Branch],
    poses: np.ndarray,
    difference_step: float = 1e-6,
) -> Metrics:
    closure_values: list[float] = []
    roundtrip_values: list[float] = []
    jacobian_values: list[float] = []
    jacobian_relative_values: list[float] = []

    for q in poses:
        theta = reference.inverse(branches, q, np.zeros(2))
        closure_values.extend(np.abs(reference.closure_error(branches, q, theta)))

        reconstructed, _ = reference.forward(branches, theta, 0.8 * q)
        roundtrip_values.append(float(np.linalg.norm(reconstructed - q)))
        reconstructed_closure = reference.closure_error(branches, reconstructed, theta)
        if np.linalg.norm(reconstructed_closure) > reference.CLOSURE_TOLERANCE:
            raise AssertionError(
                f"{name}: FK closure residual exceeds tolerance at q={q.tolist()}: "
                f"{reconstructed_closure.tolist()}"
            )

        analytical = reference.jacobian(branches, q, theta)
        numerical = finite_difference_jacobian(
            branches, q, theta, difference_step
        )
        error = analytical - numerical
        jacobian_values.extend(np.abs(error).ravel())
        jacobian_relative_values.append(
            float(
                np.linalg.norm(error, ord="fro")
                / max(np.linalg.norm(numerical, ord="fro"), np.finfo(float).eps)
            )
        )

    closure = np.asarray(closure_values)
    roundtrip = np.asarray(roundtrip_values)
    jacobian_error = np.asarray(jacobian_values)
    jacobian_relative = np.asarray(jacobian_relative_values)
    return Metrics(
        name=name,
        samples=len(poses),
        closure_max=float(np.max(closure)),
        closure_rms=float(math.sqrt(np.mean(closure**2))),
        roundtrip_max=float(np.max(roundtrip)),
        roundtrip_rms=float(math.sqrt(np.mean(roundtrip**2))),
        jacobian_max=float(np.max(jacobian_error)),
        jacobian_relative_max=float(np.max(jacobian_relative)),
        jacobian_relative_rms=float(math.sqrt(np.mean(jacobian_relative**2))),
    )


def print_metrics(metrics: list[Metrics]) -> None:
    print("configuration,samples,closure_max_mm,closure_rms_mm,"
          "roundtrip_max_rad,roundtrip_rms_rad,jacobian_max,"
          "jacobian_relative_max,jacobian_relative_rms")
    for item in metrics:
        print(
            f"{item.name},{item.samples},"
            f"{item.closure_max:.12e},{item.closure_rms:.12e},"
            f"{item.roundtrip_max:.12e},{item.roundtrip_rms:.12e},"
            f"{item.jacobian_max:.12e},{item.jacobian_relative_max:.12e},"
            f"{item.jacobian_relative_rms:.12e}"
        )


def configure_plot_style() -> None:
    import matplotlib as mpl

    mpl.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "font.size": 10.5,
            "axes.labelsize": 10.5,
            "xtick.labelsize": 8.0,
            "ytick.labelsize": 8.0,
            "legend.fontsize": 8.0,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.1,
            "figure.facecolor": "white",
            "axes.unicode_minus": False,
        }
    )


def save_roundtrip_figure(output_directory: Path, config_directory: Path) -> None:
    import matplotlib.pyplot as plt

    branches = reference.load_geometry_config(config_directory / "validation-III.json")
    phase = np.linspace(0.0, 2.0 * math.pi, 240)
    requested = np.column_stack(
        (0.18 * np.sin(phase), 0.14 * np.sin(2.0 * phase + 0.35))
    )
    recovered = np.zeros_like(requested)
    previous_theta = np.zeros(2)
    previous_q = requested[0]
    for index, q in enumerate(requested):
        theta = reference.inverse(branches, q, previous_theta)
        recovered[index], _ = reference.forward(branches, theta, previous_q)
        previous_theta = theta
        previous_q = recovered[index]

    centimetre = 1.0 / 2.54
    figure, axis = plt.subplots(figsize=(9.00 * centimetre, 6.75 * centimetre), dpi=300)
    degrees = 180.0 / math.pi
    axis.plot(requested[:, 0] * degrees, color="#0068a8", label="输入俯仰角")
    axis.plot(
        recovered[:, 0] * degrees,
        color="#e4572e",
        linestyle="--",
        label="FK 恢复俯仰角",
    )
    axis.plot(requested[:, 1] * degrees, color="#2a9d55", label="输入横滚角")
    axis.plot(
        recovered[:, 1] * degrees,
        color="#7b2cbf",
        linestyle="--",
        label="FK 恢复横滚角",
    )
    axis.set_xlabel("采样序号")
    axis.set_ylabel("角度/(°)")
    axis.grid(True, color="#d9d9d9", linewidth=0.5)
    axis.legend(ncol=2, frameon=False, loc="upper center")
    figure.subplots_adjust(left=0.16, right=0.98, bottom=0.16, top=0.97)
    figure.savefig(output_directory / "validation_roundtrip.png", dpi=300)
    plt.close(figure)


def save_jacobian_step_figure(output_directory: Path, config_directory: Path) -> None:
    import matplotlib.pyplot as plt

    steps = np.logspace(-2, -8, 7)
    poses = pose_grid()[::10]
    centimetre = 1.0 / 2.54
    figure, axis = plt.subplots(figsize=(9.00 * centimetre, 6.75 * centimetre), dpi=300)
    colours = ["#0068a8", "#e4572e", "#2a9d55"]
    short_names = ["构型 I：x 轴", "构型 II：y 轴", "构型 III：空间轴"]

    for (_, branches), colour, short_name in zip(
        configurations(config_directory), colours, short_names
    ):
        maxima = []
        for step in steps:
            largest = 0.0
            for q in poses:
                theta = reference.inverse(branches, q, np.zeros(2))
                analytical = reference.jacobian(branches, q, theta)
                numerical = finite_difference_jacobian(branches, q, theta, step)
                largest = max(largest, float(np.max(np.abs(analytical - numerical))))
            maxima.append(largest)
        axis.loglog(
            steps, maxima, marker="o", markersize=3.0, color=colour, label=short_name
        )

    axis.set_xlabel("中心差分步长 $h$/rad")
    axis.set_ylabel("最大绝对误差")
    axis.grid(True, which="major", color="#d9d9d9", linewidth=0.5)
    axis.legend(frameon=False)
    axis.invert_xaxis()
    figure.subplots_adjust(left=0.17, right=0.98, bottom=0.17, top=0.97)
    figure.savefig(output_directory / "validation_jacobian_steps.png", dpi=300)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--figures",
        type=Path,
        help="optional directory for the two paper-ready validation figures",
    )
    parser.add_argument(
        "--config-dir",
        type=Path,
        default=DEFAULT_CONFIG_DIRECTORY,
        help="directory containing validation-I/II/III.json",
    )
    args = parser.parse_args()

    poses = pose_grid()
    metrics = [
        validate_configuration(name, branches, poses)
        for name, branches in configurations(args.config_dir)
    ]
    print_metrics(metrics)

    if args.figures is not None:
        args.figures.mkdir(parents=True, exist_ok=True)
        configure_plot_style()
        save_roundtrip_figure(args.figures, args.config_dir)
        save_jacobian_step_figure(args.figures, args.config_dir)


if __name__ == "__main__":
    main()
