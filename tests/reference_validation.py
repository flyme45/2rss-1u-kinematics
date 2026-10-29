#!/usr/bin/env python3
"""Independent NumPy validation of the public 2-RSS-1U equations."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


CONFIG_DIRECTORY = Path(__file__).resolve().parents[1] / "configs"


def unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = unit(axis)
    cross = skew(axis)
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * cross @ cross


def platform_rotation(q: np.ndarray) -> np.ndarray:
    pitch, roll = q
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array(
        [[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]]
    )


def near(angle: float, reference: float) -> float:
    return reference + math.remainder(angle - reference, 2.0 * math.pi)


@dataclass
class Branch:
    axis_point: np.ndarray
    axis: np.ndarray
    crank_zero: np.ndarray
    platform_offset: np.ndarray
    platform_point: np.ndarray
    rod_length: float
    motor_offset: float = 0.0
    motor_lower: float = -2.6
    motor_upper: float = 2.6


def load_geometry_config(path: Path | str) -> list[Branch]:
    """Load the public visualizer-compatible version-2 geometry format."""
    config_path = Path(path)
    document = json.loads(config_path.read_text(encoding="utf-8"))
    if document.get("version") != 2:
        raise ValueError(f"{config_path}: expected configuration version 2")
    sources = document.get("branches")
    if not isinstance(sources, list) or len(sources) != 2:
        raise ValueError(f"{config_path}: expected exactly two RSS branches")

    result: list[Branch] = []
    for index, source in enumerate(sources, start=1):
        try:
            branch = Branch(
                axis_point=np.asarray(source["axisPoint"], dtype=float),
                axis=np.asarray(source["axis"], dtype=float),
                crank_zero=np.asarray(source["crankZero"], dtype=float),
                platform_offset=np.asarray(source["platformOffset"], dtype=float),
                platform_point=np.asarray(source["platformPoint"], dtype=float),
                rod_length=float(source["rodLength"]),
                motor_offset=float(source.get("motorOffset", 0.0)),
                motor_lower=float(source.get("motorLower", -2.6)),
                motor_upper=float(source.get("motorUpper", 2.6)),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"{config_path}: invalid branch {index}") from error

        vectors = (
            branch.axis_point,
            branch.axis,
            branch.crank_zero,
            branch.platform_offset,
            branch.platform_point,
        )
        if any(vector.shape != (3,) or not np.all(np.isfinite(vector)) for vector in vectors):
            raise ValueError(f"{config_path}: branch {index} vectors must be finite 3-vectors")
        if not math.isclose(float(np.linalg.norm(branch.axis)), 1.0, abs_tol=1e-12):
            raise ValueError(f"{config_path}: branch {index} axis must be a unit vector")
        if not math.isfinite(branch.rod_length) or branch.rod_length <= 0.0:
            raise ValueError(f"{config_path}: branch {index} rod length must be positive")
        if not branch.motor_lower <= branch.motor_upper:
            raise ValueError(f"{config_path}: branch {index} motor limits are reversed")
        result.append(branch)
    return result


def current_geometry() -> list[Branch]:
    return load_geometry_config(CONFIG_DIRECTORY / "validation-II.json")


def arbitrary_axis_geometry() -> list[Branch]:
    axes = [unit(np.array([0.90, 0.35, 0.20])), unit(np.array([-0.60, 0.75, -0.15]))]
    neutral = [np.array([-60.0, -40.0, 0.0]), np.array([55.0, -35.0, 0.0])]
    offsets = [np.array([10.0, -8.0, 4.0]), np.array([-6.0, 9.0, -3.0])]
    axial = [12.0, -9.0]
    result: list[Branch] = []
    for axis, crank_point, offset, shift in zip(axes, neutral, offsets, axial):
        reference = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        transverse = 50.0 * unit(np.cross(reference, axis))
        result.append(
            Branch(
                crank_point - transverse + shift * axis,
                axis,
                transverse - shift * axis,
                offset,
                crank_point - offset + np.array([0.0, 0.0, -240.0]),
                240.0,
            )
        )
    return result


def fixed_x_axis_geometry() -> list[Branch]:
    """Configuration I exported from the visualizer/CAD parameter set."""
    return load_geometry_config(CONFIG_DIRECTORY / "validation-I.json")


def saved_spatial_axis_geometry() -> list[Branch]:
    """Arbitrary spatial-axis geometry exported from the visualizer."""
    return load_geometry_config(CONFIG_DIRECTORY / "validation-III.json")


def moving_point(branch: Branch, q: np.ndarray) -> np.ndarray:
    return branch.platform_offset + platform_rotation(q) @ branch.platform_point


def crank_point(branch: Branch, theta: float) -> np.ndarray:
    return branch.axis_point + axis_rotation(branch.axis, theta + branch.motor_offset) @ branch.crank_zero


def branch_roots(branch: Branch, q: np.ndarray, reference: float) -> np.ndarray:
    displacement = moving_point(branch, q) - branch.axis_point
    parallel = branch.axis * np.dot(branch.axis, branch.crank_zero)
    perpendicular = branch.crank_zero - parallel
    sine_vector = np.cross(branch.axis, branch.crank_zero)
    k = -2.0 * np.dot(displacement, perpendicular)
    m = -2.0 * np.dot(displacement, sine_vector)
    n = (
        np.dot(displacement, displacement)
        + np.dot(branch.crank_zero, branch.crank_zero)
        - branch.rod_length**2
        - 2.0 * np.dot(displacement, parallel)
    )
    amplitude = math.hypot(k, m)
    normalized = -n / amplitude
    if normalized < -1.0 - 1e-10 or normalized > 1.0 + 1e-10:
        raise ValueError("unreachable")
    phase = math.atan2(m, k)
    opening = math.acos(np.clip(normalized, -1.0, 1.0))
    return np.array(
        [near(phase + opening - branch.motor_offset, reference),
         near(phase - opening - branch.motor_offset, reference)]
    )


def inverse(branches: list[Branch], q: np.ndarray, reference: np.ndarray) -> np.ndarray:
    result = np.zeros(2)
    for i, branch in enumerate(branches):
        roots = branch_roots(branch, q, reference[i])
        admissible = roots[
            (roots >= branch.motor_lower - 1e-10)
            & (roots <= branch.motor_upper + 1e-10)
        ]
        if admissible.size == 0:
            raise ValueError("no inverse-kinematics root lies within the motor limits")
        result[i] = admissible[np.argmin(np.abs(admissible - reference[i]))]
    return result


def closure_error(branches: list[Branch], q: np.ndarray, theta: np.ndarray) -> np.ndarray:
    return np.array(
        [np.linalg.norm(moving_point(branch, q) - crank_point(branch, theta[i])) - branch.rod_length
         for i, branch in enumerate(branches)]
    )


def jacobian(branches: list[Branch], q: np.ndarray, theta: np.ndarray) -> np.ndarray:
    pitch = q[0]
    rotation = platform_rotation(q)
    pitch_axis = np.array([0.0, 1.0, 0.0])
    roll_axis = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])
    result = np.zeros((2, 2))
    for i, branch in enumerate(branches):
        platform_vector = rotation @ branch.platform_point
        moving = branch.platform_offset + platform_vector
        crank = crank_point(branch, theta[i])
        rod = moving - crank
        crank_velocity = np.cross(branch.axis, crank - branch.axis_point)
        denominator = np.dot(rod, crank_velocity)
        result[i, 0] = np.dot(rod, np.cross(pitch_axis, platform_vector)) / denominator
        result[i, 1] = np.dot(rod, np.cross(roll_axis, platform_vector)) / denominator
    return result


def forward(branches: list[Branch], theta: np.ndarray, seed: np.ndarray) -> tuple[np.ndarray, int]:
    q = seed.copy()
    for iteration in range(12):
        predicted = inverse(branches, q, theta)
        residual = predicted - theta
        j = jacobian(branches, q, predicted)
        if np.max(np.abs(residual)) < 1e-9:
            return q, iteration + 1
        step = np.linalg.solve(j, residual)
        scale = min(1.0, 0.20 / np.max(np.abs(step)))
        q -= scale * step
    raise RuntimeError("FK iteration limit")


def validate_geometry(name: str, branches: list[Branch]) -> None:
    zero = inverse(branches, np.zeros(2), np.zeros(2))
    assert np.linalg.norm(zero) < 1e-9, (name, zero)
    target = np.array([0.06, -0.04])
    theta = inverse(branches, target, np.zeros(2))
    assert np.linalg.norm(closure_error(branches, target, theta)) < 1e-9

    for branch_index, branch in enumerate(branches):
        roots = branch_roots(branch, target, theta[branch_index])
        for root in roots:
            candidate = theta.copy()
            candidate[branch_index] = root
            assert abs(closure_error(branches, target, candidate)[branch_index]) < 1e-9

    reconstructed, iterations = forward(branches, theta, target * 0.8)
    assert np.linalg.norm(reconstructed - target) < 1e-8
    assert iterations <= 12

    analytical = jacobian(branches, target, theta)
    numerical = np.zeros((2, 2))
    epsilon = 1e-7
    for column in range(2):
        plus, minus = target.copy(), target.copy()
        plus[column] += epsilon
        minus[column] -= epsilon
        numerical[:, column] = (
            inverse(branches, plus, theta) - inverse(branches, minus, theta)
        ) / (2.0 * epsilon)
    assert np.allclose(analytical, numerical, atol=2e-6), (analytical, numerical)

    q_dot = np.array([0.24, -0.17])
    tau_q = np.array([12.0, -7.0])
    theta_dot = analytical @ q_dot
    tau_theta = np.linalg.solve(analytical.T, tau_q)
    assert abs(tau_q @ q_dot - tau_theta @ theta_dot) < 1e-10


def main() -> None:
    validate_geometry("paper-compatible", current_geometry())
    validate_geometry("synthetic arbitrary spatial axes", arbitrary_axis_geometry())
    validate_geometry("configuration I fixed x axes", fixed_x_axis_geometry())
    validate_geometry("saved arbitrary spatial axes", saved_spatial_axis_geometry())
    print("reference validation passed: IK/FK, dual roots, analytic Jacobian, virtual work")


if __name__ == "__main__":
    main()

