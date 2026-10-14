#include "hand/kinematics.hpp"
#include <cmath>
#include <iostream>

bool check_position(hand::Position2D actual_position, double expected_x,
                    double expected_y) {
    const double tolerance = 1e-9;

    bool x_matches = std::abs(actual_position.x - expected_x) < tolerance;
    bool y_matches = std::abs(actual_position.y - expected_y) < tolerance;

    return x_matches && y_matches;
}

int main() {
    const double tolerance = 1e-9;
    bool all_passed = true;
    hand::JointAngles2D elbow_down_angles{};
    bool elbow_down_solved =
        hand::inverse_kinematics(1, 1, 1, 1, elbow_down_angles, false);
    if (elbow_down_solved) {
        auto actual_position = hand::forward_kinematics(
            1, 1, elbow_down_angles.theta1, elbow_down_angles.theta2);
        bool position_matches = check_position(actual_position, 1, 1);
        if (position_matches) {
            std::cout << "Elbow-down inverse kinematics: PASS\n";
        } else {
            std::cout
                << "Elbow-down inverse kinematics: FAIL (position mismatch)\n";
            all_passed = false;
        }
    } else {
        std::cout << "Elbow-down inverse kinematics: FAIL (no solution)\n";
        all_passed = false;
    }

    hand::JointAngles2D elbow_up_angles{};
    bool elbow_up_solved =
        hand::inverse_kinematics(1, 1, 1, 1, elbow_up_angles, true);
    if (elbow_up_solved) {
        auto actual_position = hand::forward_kinematics(
            1, 1, elbow_up_angles.theta1, elbow_up_angles.theta2);
        bool position_matches = check_position(actual_position, 1, 1);
        if (position_matches) {
            std::cout << "Elbow-up inverse kinematics: PASS\n";
        } else {
            std::cout
                << "Elbow-up inverse kinematics: FAIL (position mismatch)\n";
            all_passed = false;
        }
    } else {
        std::cout << "Elbow-up inverse kinematics: FAIL (no solution)\n";
        all_passed = false;
    }
    bool up_check =
        std::abs(elbow_up_angles.theta2 - 1.57079632679) < tolerance;
    bool down_check =
        std::abs(elbow_down_angles.theta2 + 1.57079632679) < tolerance;
    if (up_check && down_check && elbow_down_solved && elbow_up_solved) {
        std::cout << "PASS\n";
    } else {
        std::cout << "FAIL\n";
        all_passed = false;
    }

    hand::JointAngles2D joint_angles{};
    bool outside_maximum_reach_solved =
        hand::inverse_kinematics(1, 1, 3, 0, joint_angles, true);
    if (!outside_maximum_reach_solved) {
        std::cout << "Target beyond maximum reach: PASS (rejected)\n";
    } else {
        all_passed = false;
        std::cout << "Target beyond maximum reach: FAIL (accepted)\n";
    }

    bool inside_minimum_reach_solved =
        hand::inverse_kinematics(2, 1, 0.5, 0, joint_angles, true);
    if (!inside_minimum_reach_solved) {
        std::cout << "Target inside minimum reach: PASS (rejected)\n";
    } else {
        all_passed = false;
        std::cout << "Target inside minimum reach: FAIL (accepted)\n";
    }

    bool maximum_reach_solved =
        hand::inverse_kinematics(1, 1, 2, 0, joint_angles, true);
    if (maximum_reach_solved) {
        auto actual_position = hand::forward_kinematics(
            1, 1, joint_angles.theta1, joint_angles.theta2);
        bool position_matches = check_position(actual_position, 2, 0);
        if (position_matches) {
            std::cout << "Target at maximum reach: PASS\n";
        } else {
            all_passed = false;
            std::cout << "Target at maximum reach: FAIL (position mismatch)\n";
        }
    } else {
        all_passed = false;
        std::cout << "Target at maximum reach: FAIL (no solution)\n";
    }

    return all_passed ? 0 : 1;
}
