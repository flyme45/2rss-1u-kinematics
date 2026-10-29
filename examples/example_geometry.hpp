#pragma once

#include "rss1u/solver.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace rss1u::examples {

inline Geometry currentAnkleExample() {
    constexpr double pi = 3.14159265358979323846;
    const double a1 = 15.0 * pi / 180.0;
    const double a2 = 15.0034005 * pi / 180.0;
    const double a3 = 4.17330971 * pi / 180.0;
    const double cx = -50.0 * std::cos(a1);
    const double cz = -50.0 * std::sin(a1);
    const double bx = -50.0 * std::cos(a2);
    const double bz = -50.0 * std::sin(a2);
    const double ax = 315.22 * std::sin(a3);
    const double az = -315.22 * std::cos(a3);

    Geometry geometry;
    geometry.branches[0].axis = Vec3::UnitY();
    geometry.branches[0].axis_point = Vec3::Zero();
    geometry.branches[0].platform_offset = Vec3(0.0, -35.5, -400.0);
    geometry.branches[0].platform_point = Vec3(cx, 35.5, cz);
    geometry.branches[0].crank_zero = Vec3(cx, 0.0, cz);
    geometry.branches[0].rod_length = 400.0;
    geometry.branches[0].motor_limit = AngleLimit{-2.6, 2.6};

    geometry.branches[1].axis = Vec3::UnitY();
    geometry.branches[1].axis_point = Vec3::Zero();
    geometry.branches[1].platform_offset = Vec3(ax, 35.5, az);
    geometry.branches[1].platform_point = Vec3(bx, -35.5, bz);
    geometry.branches[1].crank_zero = Vec3(bx, 0.0, bz);
    geometry.branches[1].rod_length = 315.22;
    geometry.branches[1].motor_limit = AngleLimit{-2.6, 2.6};
    return geometry;
}

inline Vec3 perpendicularVector(const Vec3& raw_axis, double radius) {
    const Vec3 axis = raw_axis.normalized();
    const Vec3 reference = std::abs(axis.z()) < 0.9
        ? Vec3::UnitZ() : Vec3::UnitX();
    return radius * reference.cross(axis).normalized();
}

inline Geometry arbitraryAxisExample() {
    const Vec3 axes[2] = {
        Vec3(0.90, 0.35, 0.20).normalized(),
        Vec3(-0.60, 0.75, -0.15).normalized()};
    const Vec3 neutral_crank_points[2] = {
        Vec3(-60.0, -40.0, 0.0),
        Vec3(55.0, -35.0, 0.0)};
    const Vec3 offsets[2] = {
        Vec3(10.0, -8.0, 4.0),
        Vec3(-6.0, 9.0, -3.0)};
    const double axial_shift[2] = {12.0, -9.0};

    Geometry geometry;
    for (int i = 0; i < 2; ++i) {
        const Vec3 transverse = perpendicularVector(axes[i], 50.0);
        BranchGeometry& branch = geometry.branches[static_cast<std::size_t>(i)];
        branch.axis = axes[i];
        branch.axis_point = neutral_crank_points[i] - transverse
            + axial_shift[i] * axes[i];
        branch.crank_zero = transverse - axial_shift[i] * axes[i];
        branch.platform_offset = offsets[i];
        branch.platform_point = neutral_crank_points[i] - offsets[i]
            + Vec3(0.0, 0.0, -240.0);
        branch.rod_length = 240.0;
        branch.motor_limit = AngleLimit{-2.6, 2.6};
    }
    return geometry;
}

}  // namespace rss1u::examples

