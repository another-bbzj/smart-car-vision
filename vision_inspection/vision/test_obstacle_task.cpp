#include "obstacle_task.hpp"
#include <iostream>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
    std::cout << "PASS: " << message << '\n';
}

obstacle::Detection target(obstacle::Color color, float x = 320, float y = 350) {
    obstacle::Detection d;
    d.color = color; d.center = {x, y}; d.confidence = 0.9; d.stable = true;
    return d;
}

obstacle_task::Input input(int lap, std::vector<obstacle::Detection> targets = {}) {
    obstacle_task::Input in;
    in.lap = lap;
    in.targets = std::move(targets);
    return in;
}

void clearFrames(obstacle_task::Machine& machine, int lap) {
    for (int i = 0; i < 3; ++i) machine.update(input(lap));
}

int main() {
    try {
        obstacle_task::Machine machine;
        auto out = machine.update(input(1, {target(obstacle::Color::Green)}));
        check(out.commandValid && out.action == obstacle_task::Action::PassGreen && out.eventId != 0,
              "green requests pass-through with a unique event id");
        const auto greenId = out.eventId;
        out = machine.update(input(1, {target(obstacle::Color::Green)}));
        check(out.commandValid && out.eventId == greenId, "unacknowledged event is retransmitted without duplicating id");
        auto accepted = input(1, {target(obstacle::Color::Green)});
        accepted.ackEventId = greenId; accepted.ack = obstacle_task::Ack::Accepted;
        out = machine.update(accepted);
        check(!out.commandValid && out.actionAccepted && !out.actionCompleted,
              "accepted event is no longer a pending command but action is not complete");
        out = machine.update(input(1));
        check(out.phase == obstacle_task::Phase::AwaitingClear && !out.actionCompleted,
              "target disappearance alone cannot complete an accepted action");
        auto done = input(1);
        done.ackEventId = greenId; done.ack = obstacle_task::Ack::Completed;
        out = machine.update(done);
        check(out.actionCompleted && out.phase == obstacle_task::Phase::AwaitingClear,
              "completion acknowledgment is latched while waiting for visual clear");
        out = machine.update(input(1, {target(obstacle::Color::Green)}));
        check(out.phase == obstacle_task::Phase::AwaitingClear, "same visible obstacle does not trigger a second event");
        clearFrames(machine, 1);
        check(machine.current().phase == obstacle_task::Phase::Idle,
              "completed action and clear target rearm the machine");

        out = machine.update(input(1, {target(obstacle::Color::Red)}));
        check(out.action == obstacle_task::Action::IgnoreRed, "red obstacle on first lap is ignored");
        auto redDone = input(1); redDone.ackEventId = out.eventId; redDone.ack = obstacle_task::Ack::Completed;
        machine.update(redDone); clearFrames(machine, 1);
        out = machine.update(input(2, {target(obstacle::Color::Red, 320, 300)}));
        check(out.action == obstacle_task::Action::StopRedFiveSeconds,
              "red obstacle on second lap requests a five-second stop");

        machine.reset();
        out = machine.update(input(2, {target(obstacle::Color::Yellow)}));
        check(out.action == obstacle_task::Action::BypassYellow, "yellow obstacle requests bypass independent of lap");
        auto rejected = input(2); rejected.ackEventId = out.eventId; rejected.ack = obstacle_task::Ack::Rejected;
        out = machine.update(rejected);
        check(out.phase == obstacle_task::Phase::Fault && out.fault,
              "control rejection enters a visible fault state");

        machine.reset();
        out = machine.update(input(0, {target(obstacle::Color::Red)}));
        check(out.phase == obstacle_task::Phase::Fault,
              "unknown or invalid lap count cannot trigger red-lap policy");

        machine.reset();
        out = machine.update(input(1, {target(obstacle::Color::Unknown)}));
        check(out.phase == obstacle_task::Phase::Idle && !out.commandValid,
              "unknown-color candidate is ignored");
        std::cout << "All obstacle task tests passed. Controller actions are not executed here.\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
