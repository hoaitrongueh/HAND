#include <iostream>
#include <cmath>
#include "hand/kinematics.hpp"

bool check_position(hand::Position2D actual, double expected_x,
                    double expected_y) {
    const double tolerance = 1e-9;

    bool x_ok = std::abs(actual.x - expected_x) < tolerance;
    bool y_ok = std::abs(actual.y - expected_y) < tolerance;

    return x_ok && y_ok;
}

int main() {
    const double pi = 3.141592653589793;

    auto straight = hand::forward_kinematics(1, 0.5, 0, 0);
    auto upright = hand::forward_kinematics(1, 0.5, pi / 2, 0);
    auto bent = hand::forward_kinematics(1, 0.5, 0, pi / 2);
    auto folded = hand::forward_kinematics(1, 0.5, 0, pi);

    bool straight_ok = check_position(straight, 1.5, 0);
    bool upright_ok = check_position(upright, 0, 1.5);
    bool bent_ok = check_position(bent, 1, 0.5);
    bool folded_ok = check_position(folded, 0.5, 0);

    bool all_ok = straight_ok && upright_ok && bent_ok && folded_ok;

    std::cout << "Forward kinematics: " << (all_ok ? "PASS" : "FAIL") << '\n';

    bool reachable = hand::is_reachable(1, 0.5, 0.5, 0);

    std::cout << "Reachability test: "
              << (reachable ? "REACHABLE" : "UNREACHABLE") << '\n';

    return all_ok ? 0 : 1;
}
