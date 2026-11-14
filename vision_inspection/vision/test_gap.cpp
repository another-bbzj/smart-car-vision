#include "gap_state.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}

task::GapInput sample(bool visible, double odometer, double dt = 0.1) {
    task::GapInput in;
    in.enabled = true;
    in.odometerM = odometer;
    in.speedMps = 0.18;
    in.dtS = dt;
    in.trackVisible = visible;
    in.headingValid = true;
    in.headingErrorDeg = 0;
    return in;
}

int main() {
    try {
        task::GapStateMachine machine;
        check(machine.validConfig(), "default V1.1 gap configuration is valid");
        auto out = machine.update(sample(true, 0.0));
        check(out.phase == task::GapPhase::Approach, "route enable arms approach after visible track");
        out = machine.update(sample(true, 0.2));
        check(out.phase == task::GapPhase::Crossing && out.useHeadingHold && out.suggestedSpeedMps < 0.25,
              "0.20 m approach enters slow heading-held crossing");
        for (int i = 0; i < 3; ++i) out = machine.update(sample(true, 1.8 + i * 0.01));
        check(out.phase == task::GapPhase::Recovering, "track visible after 1.60 m enters recovery");
        for (int i = 0; i < 3; ++i) out = machine.update(sample(true, 2.11 + i * 0.01));
        check(out.phase == task::GapPhase::Complete && out.routeComplete && !out.requestStop,
              "0.30 m confirmed recovery completes route");

        machine.reset();
        machine.update(sample(true, 0.0));
        machine.update(sample(true, 0.2));
        for (int i = 0; i < 9; ++i) out = machine.update(sample(false, 0.3 + i * 0.01));
        check(out.phase == task::GapPhase::Fault && out.fault == task::GapFault::Timeout && out.requestStop,
              "prolonged loss after entry faults instead of pretending success");

        machine.reset();
        out = machine.update(sample(true, 0.0));
        auto invalid = sample(true, 0.2); invalid.headingValid = false;
        out = machine.update(invalid);
        check(out.phase == task::GapPhase::Fault && out.fault == task::GapFault::InvalidInput,
              "heading validity is required before entering the gap");

        machine.reset();
        machine.update(sample(true, 0.0));
        machine.update(sample(true, 0.2));
        auto overshoot = sample(true, 2.7);
        out = machine.update(overshoot);
        check(out.phase == task::GapPhase::Fault && out.fault == task::GapFault::Overshoot,
              "excessive travel faults instead of silently continuing");

        machine.reset();
        auto bad = sample(true, 0.0, 2.0);
        out = machine.update(bad);
        check(out.phase == task::GapPhase::Fault && out.fault == task::GapFault::InvalidInput,
              "invalid frame interval is rejected");
        machine.reset();
        auto disabled = task::GapInput{};
        out = machine.update(disabled);
        check(out.phase == task::GapPhase::Disabled && out.routeEnableRequired,
              "disabled route remains inactive");
        std::cout << "All gap state tests passed. Sensor and actuator integration remain unverified.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
