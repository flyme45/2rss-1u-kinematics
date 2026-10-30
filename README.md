# 2-RSS-1U Kinematics Solver

English | [Chinese](README_zh-CN.md)

A standalone, reusable 2-RSS-1U parallel-mechanism kinematics library designed for real-time control.

The library provides analytic inverse kinematics, numerical forward kinematics, an analytic Jacobian, velocity relations, virtual-work-consistent torque relations, continuous assembly-mode selection, and singularity diagnostics for 2-RSS-1U mechanisms. Hardware communication and robot-level control are outside its scope.

For the full derivation, see:

- [Technical derivation (PDF, Chinese)](output/pdf/2RSS1U_%E8%BF%90%E5%8A%A8%E5%AD%A6%E5%AE%8C%E6%95%B4%E6%8E%A8%E5%AF%BC.pdf)

The technical derivation is distributed as a PDF; its editable source files are not included.

## Mathematical Background

The paper *On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle Mechanism* presents inverse kinematics, a Jacobian, and Newton-Raphson forward kinematics for a fixed actuator-axis layout. This library builds on that solution chain and generalizes each branch to a parameterized spatial-axis model, with assembly-mode continuity and explicit failure diagnostics.

The branch geometry is expressed as:

```text
moving point = platform_offset + R_y(pitch) R_x(roll) platform_point
crank point  = axis_point + R(axis, theta + offset) crank_zero
```

Here, `platform_offset` is the fixed coordinate of the universal-joint center (O) in the branch calculation frame. `platform_point` is the platform-fixed mounting vector from O to the spherical joint (B_i) at the zero pose. Therefore, the former is not a moving-platform-fixed offset; only the latter rotates with the platform. If a common base frame centered at O is used directly, `platform_offset` should be zero.

A branch calculation frame may differ from the common base frame only by a translation while keeping the axes parallel. `axis_point`, `platform_offset`, endpoints, and all branch vectors must be expressed in the same frame. If a local frame is also rotated relative to the common base frame, apply that rotation explicitly before calling the solver.

When `axis = UnitY()` and `axis_point = 0`, the crank rotates about the y-axis through the branch-frame origin.

## Main Contributions

### 1. Generalization from Fixed y-Axes to Arbitrary Spatial Actuator Axes

Each RSS branch independently specifies a point on the actuator axis, the axis direction, the zero-pose crank vector, the fixed universal-joint-center coordinate, and the zero-pose platform spherical-joint mounting vector. The two motor axes may be parallel, orthogonal, skew, or left-right mirrored. A different mechanical layout requires only a different parameter set.

### 2. Analytic IK Returns Both Assembly Roots and Selects Continuously

The loop-closure equation is reduced to

\[
K_i\cos\phi_i+M_i\sin\phi_i+N_i=0.
\]

The solver computes both analytic roots and selects the one nearest to the current or previous crank angle, preventing assembly-mode jumps along a continuous trajectory that can occur with `atan`-based or single-root formulas.

### 3. Control-Oriented Failure States

Unreachable poses, degenerate trigonometric equations, actuator-limit violations, singular branch denominators, singular constraint Jacobians, and forward-kinematics iteration limits all return explicit status codes. The real-time functions neither substitute a zero vector for a valid solution nor throw exceptions.

### 4. Analytic 2 x 2 Jacobian and Warm-Started Forward Kinematics

The solver directly constructs

\[
\dot{\boldsymbol\theta}=\mathbf J\dot{\mathbf q},
\]

without temporary `2x6` or `6x2` matrices. Forward kinematics follows the paper's iteration,

\[
\mathbf q_{k+1}=\mathbf q_k-
\mathbf J^{-1}\left(f_{IK}(\mathbf q_k)-\boldsymbol\theta_m\right),
\]

and adds previous-frame warm starts, a maximum iteration count, a maximum angular step, and explicit singularity detection.

### 5. One Coordinate Definition for Velocity and Torque

Velocity follows `theta_dot = J * q_dot`. Torque follows directly from virtual work:

\[
\boldsymbol\tau_q=\mathbf J^T\boldsymbol\tau_\theta.
\]

The tests compare the analytic Jacobian with central finite differences and verify instantaneous-power consistency on both sides.

## Scope of Novelty

The following are established mathematics or previously published methods and are not claimed as original contributions:

- Rodrigues' rotation formula;
- solving sine-cosine trigonometric equations;
- Newton-Raphson iteration;
- Jacobians, singularity analysis, and the principle of virtual work;
- the fixed-layout 2-RSS-1U solution chain presented in the cited paper.

This library combines **spatial actuator-axis parameterization, continuous selection between two analytic roots, a complete IK/FK/Jacobian implementation, explicit failure diagnostics, and reproducible tests**.

Being able to describe arbitrary spatial actuator axes does not imply that an arbitrary mechanism can be controlled without validation. Users must still satisfy the 2-RSS-1U topology and rigid-body assumptions and correctly calibrate axis directions, zero positions, spherical-joint centers, link lengths, rotation order, and assembly mode.

## Repository Layout

```text
include/rss1u/solver.hpp       Public API and data structures
src/solver.cpp                 IK, FK, Jacobian, velocity, and torque relations
examples/example_geometry.hpp  Paper-compatible parameters and a spatial-axis example
examples/basic_demo.cpp        Minimal usage example
configs/validation-I.json      Configuration I, fixed x-axis reproduction (version 2)
configs/validation-II.json     Configuration II, fixed y-axis reproduction (version 2)
configs/validation-III.json    Configuration III, nonparallel spatial axes (version 2)
apps/visualizer/               Offline interactive 3D kinematics visualizer
apps/studio/                   Qt 6/QML/Quick 3D desktop workbench
tests/test_solver.cpp          C++ regression tests
tests/test_solver_failures.cpp Failure and degeneracy regression tests
tests/test_interactive_state.cpp  Four-variable automatic IK/FK state regression tests
tests/test_visualizer.js       Web model and expression regression tests
tests/reference_validation.py  Independent NumPy formula validation
tests/batch_validation.py      1,000-pose batch validation for each configuration
tests/README.md                Test matrix and consolidated test instructions
output/pdf/                    Final typeset technical PDF only
```

## Requirements and Build

- C++17
- CMake 3.16+
- Eigen 3.3+

On Ubuntu or Debian:

```bash
sudo apt-get install cmake ninja-build libeigen3-dev
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/rss1u_demo
```

If a C++ toolchain is unavailable, run the independent reference validations first:

```bash
python tests/reference_validation.py
python tests/batch_validation.py --config-dir configs
```

The three public JSON files use the same import format as the visualizer's configuration manager. Regenerate the validation plots with:

```bash
python tests/batch_validation.py --config-dir configs --figures build/validation
```

## Interactive Visualizer

Open [apps/visualizer/index.html](apps/visualizer/index.html) directly. It requires no installation, network connection, or local server. The page provides:

- mouse-drag 3D rotation, wheel zoom, and double-click reset;
- continuous pitch/roll adjustment and trajectory animation;
- independent azimuth and elevation control for both motor output axes;
- presets for Configuration I with fixed x-axes, Configuration II with fixed y-axes, and arbitrary spatial axes;
- a configuration manager that documents the parameters, stores custom configurations in the current browser, and imports, exports, or deletes JSON files;
- formula input supporting `sin`, `cos`, `tan`, `sqrt`, `pi`, and degree notation while preserving the original expressions in saved configurations and JSON;
- analytic IK with continuous selection between two roots, loop-closure length error, K/M/N coefficients, Jacobian determinant, and condition number displayed in real time;
- explicit warnings for unreachable poses, actuator-limit violations, and singularities.

The browser formulas correspond term by term to `src/solver.cpp`. If Node.js is installed, run the following regression check for IK/FK round trips and the analytic Jacobian across both geometry parameter sets:

```bash
node tests/test_visualizer.js
```

## Qt 6 Desktop Workbench

`apps/studio` provides **RSS1U Kinematics Studio**. All four variables, Pitch, Roll, theta_1, and theta_2, have both sliders and numeric input fields; no manual IK/FK mode selection is required:

- editing Pitch or Roll calls `rss1u::Solver::inverse()` automatically;
- editing theta_1 or theta_2 calls `rss1u::Solver::forward()` automatically;
- every successful result synchronizes the other two variables, 3D endpoints, loop-closure residuals, and the Jacobian;
- an unreachable, limit-violating, or nonconvergent request preserves the last valid state;
- QML handles presentation only, while C++ manages branch continuity, request merging, and state commits.

See the [Studio README](apps/studio/README.md) for complete build instructions. With Qt 6.5 or newer installed:

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=/path/to/Qt/6.x/gcc_64
cmake --build build --target rss1u_studio
./build/rss1u_studio
```

GitHub Actions compiles the library and examples and runs the C++ tests, independent NumPy validation, 1,000-pose batch validation for all three configurations, and web-model tests.

## Minimal Example

```cpp
#include <rss1u/solver.hpp>

rss1u::Geometry geometry;
geometry.branches[0] = /* CAD/calibration data */;
geometry.branches[1] = /* CAD/calibration data */;

rss1u::Solver solver(geometry);
rss1u::Vec2 previous_theta = rss1u::Vec2::Zero();
rss1u::Vec2 desired_q(0.08, -0.05);  // [pitch, roll], rad

auto ik = solver.inverse(desired_q, previous_theta);
if (!ik.ok()) {
    // Stop command generation and handle ik.status.
}

auto J = solver.jacobian(desired_q, ik.actuator_angle);
auto fk = solver.forward(ik.actuator_angle, desired_q); // Use the previous q in practice.
```

## Units and Solver Tolerances

Internally, generalized coordinates are ordered as `[pitch, roll]` and measured in radians. Geometry may use millimeters or meters, but one mechanism must use a single consistent length unit.

`SolverOptions::residual_tolerance` is the forward-kinematics actuator-angle residual tolerance in radians. `SolverOptions::closure_tolerance` is the Euclidean norm tolerance of the two branch length residuals in the selected geometry unit. Forward kinematics reports success only when both conditions are satisfied, preventing a small angular residual from hiding insufficient loop-closure accuracy.

## Validation Coverage

The automated tests cover:

- zero-pose and nonzero-pose IK/FK round trips using the fixed y-axis example in `examples/example_geometry.hpp`;
- IK/FK round trips for Configuration I with fixed x-axes and for arbitrary spatial actuator axes;
- loop-closure length error for both analytic assembly roots;
- the analytic Jacobian against central finite differences;
- forward and inverse velocity and torque mappings, including joint-side and actuator-side virtual-work consistency;
- invalid geometry, non-finite inputs, unreachable poses, coefficient degeneracy, actuator limits, singularities, and iteration limits;
- preservation of the last valid state after a four-variable interactive update fails;
- default web configurations, formula input, JSON round trips, and rejection of invalid parameters.

All automated tests are under [tests](tests/). See the [test matrix and instructions](tests/README.md) for details.

The example parameters demonstrate only that the formulas and implementation are self-consistent. A physical machine also requires validation of the workspace, spherical-joint articulation limits, collisions, encoder signs, structural deformation, backlash, and worst-case execution time at the target control rate.

## References

1. C. Zhou and N. G. Tsagarakis, "On the Comprehensive Kinematics Analysis of a Humanoid Parallel Ankle Mechanism," *Journal of Mechanisms and Robotics*, 2018, 10(5): 051015, DOI: [10.1115/1.4040886](https://doi.org/10.1115/1.4040886).
2. R. M. Murray, Z. Li, and S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994.

The references identify the sources of the foundational methods; they do not imply endorsement of this repository by the cited authors.

## License

[MIT](LICENSE)
