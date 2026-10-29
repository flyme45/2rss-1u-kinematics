#include "example_geometry.hpp"
#include "rss1u/solver.hpp"

#include <iomanip>
#include <iostream>

int main() {
    using namespace rss1u;

    Solver solver(examples::currentAnkleExample());
    const Vec2 desired_joint(0.08, -0.05);  // [pitch, roll], rad
    const InverseResult inverse = solver.inverse(desired_joint, Vec2::Zero());
    if (!inverse.ok()) {
        std::cerr << "IK failed: " << statusMessage(inverse.status) << '\n';
        return 1;
    }

    const JacobianResult jacobian = solver.jacobian(
        desired_joint, inverse.actuator_angle);
    if (!jacobian.ok()) {
        std::cerr << "Jacobian failed: " << statusMessage(jacobian.status) << '\n';
        return 1;
    }

    // A real controller should use the previous successful state as this seed.
    const ForwardResult forward = solver.forward(
        inverse.actuator_angle, Vec2(0.07, -0.04));
    if (!forward.ok()) {
        std::cerr << "FK failed: " << statusMessage(forward.status) << '\n';
        return 1;
    }

    std::cout << std::fixed << std::setprecision(10)
              << "joint target [pitch roll] : " << desired_joint.transpose() << '\n'
              << "actuator angle            : " << inverse.actuator_angle.transpose() << '\n'
              << "closure length error      : "
              << solver.closureLengthError(desired_joint, inverse.actuator_angle).transpose()
              << '\n'
              << "reconstructed joint       : " << forward.joint_angle.transpose() << '\n'
              << "FK iterations             : " << forward.iterations << '\n'
              << "J = d(theta)/d(q):\n" << jacobian.matrix << '\n'
              << "J Frobenius condition     : " << jacobian.condition_frobenius << '\n';
    return 0;
}

