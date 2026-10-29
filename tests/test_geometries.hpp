#pragma once

#include "rss1u/solver.hpp"

namespace rss1u::tests {

inline Geometry fixedXAxisGeometry() {
    Geometry geometry;

    geometry.branches[0].axis_point = Vec3(-41.0, 0.0, 195.0);
    geometry.branches[0].axis = Vec3::UnitX();
    geometry.branches[0].crank_zero =
        Vec3(0.0, 23.326823374855284, 8.992180560815337);
    geometry.branches[0].platform_offset = Vec3::Zero();
    geometry.branches[0].platform_point = Vec3(-41.0, 25.0, 8.9);
    geometry.branches[0].rod_length = 195.09935529363676;
    geometry.branches[0].motor_limit = AngleLimit{-2.6, 2.6};

    geometry.branches[1].axis_point = Vec3(-41.0, 0.0, 140.0);
    geometry.branches[1].axis = Vec3::UnitX();
    geometry.branches[1].crank_zero =
        Vec3(0.0, -23.328001980749068, 8.98912251480463);
    geometry.branches[1].platform_offset = Vec3::Zero();
    geometry.branches[1].platform_point = Vec3(-41.0, -25.0, 8.9);
    geometry.branches[1].rod_length = 140.0991000126136;
    geometry.branches[1].motor_limit = AngleLimit{-2.6, 2.6};

    return geometry;
}

inline Geometry spatialAxisGeometry() {
    Geometry geometry;

    geometry.branches[0].axis_point = Vec3(-10.0, 30.0, 380.0);
    geometry.branches[0].axis =
        Vec3(-0.49999999999999994, 0.8660254037844387, 0.0);
    geometry.branches[0].crank_zero =
        Vec3(-42.60289043074065, -24.999999999999996, 7.745561757955797);
    geometry.branches[0].platform_offset = Vec3::Zero();
    geometry.branches[0].platform_point = Vec3(-90.0, 30.0, -10.0);
    geometry.branches[0].rod_length = 400.2812457538911;
    geometry.branches[0].motor_limit = AngleLimit{-2.6, 2.6};

    geometry.branches[1].axis_point = Vec3(-10.0, -30.0, 330.0);
    geometry.branches[1].axis =
        Vec3(-0.49999999999999994, -0.8660254037844387, 0.0);
    geometry.branches[1].crank_zero =
        Vec3(-42.65861100064014, 24.999999999999996, 7.432557264903163);
    geometry.branches[1].platform_offset = Vec3::Zero();
    geometry.branches[1].platform_point = Vec3(-90.0, -30.0, -10.0);
    geometry.branches[1].rod_length = 350.3266492575632;
    geometry.branches[1].motor_limit = AngleLimit{-2.6, 2.6};

    return geometry;
}

}  // namespace rss1u::tests
