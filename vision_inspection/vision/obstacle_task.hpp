#pragma once

#include "obstacle_detector.hpp"
#include <cstdint>
#include <vector>

namespace obstacle_task {

// 控制端对已发布事件的处理结果。必须和 eventId 一起回传，防止确认错事件。
enum class Ack { None, Accepted, Completed, Rejected };

// 需要上层控制执行的动作类别；这里描述意图，不代表动作已经完成。
enum class Action { None, PassGreen, IgnoreRed, BypassYellow, StopRedFiveSeconds };

// 任务机所处阶段。每次只处理一个当前目标，避免多个目标命令互相覆盖。
enum class Phase { Idle, AwaitingAck, AwaitingClear, Fault };

// 一帧策略输入：稳定识别结果、当前圈数，以及控制端对上一事件的反馈。
struct Input {
    int lap = 0;                              // 当前圈数，比赛从 1 开始计圈。
    std::vector<obstacle::Detection> targets; // 本帧可见且已多帧稳定的候选。
    std::uint32_t ackEventId = 0;             // 被确认的事件编号；0 表示没有确认。
    Ack ack = Ack::None;                      // Accepted=接收请求，Completed=动作完成。
};

// 发布给控制端的当前事件。AwaitingAck 阶段会重复发布同一 eventId，便于可靠传输。
struct Event {
    std::uint32_t eventId = 0;                // 单调递增编号；同一任务重发时编号不变。
    Phase phase = Phase::Idle;                // 当前任务机阶段。
    Action action = Action::None;             // 控制端应执行的动作类型。
    obstacle::Color color = obstacle::Color::Unknown;
    cv::Point2f centerPx;                     // 触发时的位置快照，像素坐标，不是物理距离。
    double confidence = 0;                    // 识别器给出的启发式分数，不是概率。
    bool commandValid = false;                // true 时事件包含待处理的控制请求。
    bool fault = false;                       // 圈数或协议输入非法时置 true。
    bool actionAccepted = false;              // 控制端已回 ACCEPTED，但动作未完成。
    bool actionCompleted = false;             // 控制端已回 COMPLETED，动作已执行完毕。
};

// 策略参数：控制端接收确认后，目标需连续消失多少帧才重新布防。
struct Config {
    int clearFrames = 3;
    bool valid() const { return clearFrames > 0 && clearFrames <= 30; }
};

// 将稳定视觉目标翻译成比赛规则事件，并通过 eventId/ack 与控制端握手。
// 绿色请求直行通过；黄色请求绕行；红色第一圈忽略，后续圈停车 5 秒。
// 本类不驱动电机，也不判断车辆是否真的完成了绕行或停车。
class Machine {
public:
    explicit Machine(Config config = {}) : config_(config) {}

    // 清除当前事件、消失计数和故障状态；下一次目标会获得新的事件编号。
    void reset() {
        phase_ = Phase::Idle;
        action_ = Action::None;
        color_ = obstacle::Color::Unknown;
        eventId_ = 0;
        clearCount_ = 0;
        fault_ = false;
        actionAccepted_ = actionCompleted_ = false;
    }

    // 输入本帧稳定目标和控制确认，返回本帧应发布给控制端的事件。
    Event update(const Input& input) {
        if (!config_.valid() || input.lap < 1 || input.targets.size() > 64 ||
            (input.ack != Ack::None && input.ackEventId == 0)) {
            phase_ = Phase::Fault;
            fault_ = true;
            return output();
        }
        const obstacle::Detection* target = nearestStable(input.targets);
        switch (phase_) {
        case Phase::Idle:
            if (target) beginEvent(*target, input.lap);
            break;
        case Phase::AwaitingAck:
            if (input.ackEventId == eventId_) {
                if (input.ack == Ack::Rejected) {
                    phase_ = Phase::Fault;
                    fault_ = true;
                } else if (input.ack == Ack::Accepted) {
                    actionAccepted_ = true;
                    phase_ = Phase::AwaitingClear;
                } else if (input.ack == Ack::Completed) {
                    actionAccepted_ = actionCompleted_ = true;
                    phase_ = Phase::AwaitingClear;
                }
            }
            break;
        case Phase::AwaitingClear:
            if (input.ackEventId == eventId_ && input.ack == Ack::Rejected) {
                phase_ = Phase::Fault;
                fault_ = true;
                break;
            }
            if (input.ackEventId == eventId_ && input.ack == Ack::Completed) actionCompleted_ = true;
            if (target) clearCount_ = 0;
            else if (actionCompleted_ && ++clearCount_ >= config_.clearFrames) resetCurrent();
            break;
        case Phase::Fault:
            break;
        }
        return output();
    }

    // 读取当前事件状态，不更新消失计数；适合诊断界面查询。
    Event current() const { return output(); }

private:
    static const obstacle::Detection* nearestStable(const std::vector<obstacle::Detection>& targets) {
        const obstacle::Detection* best = nullptr;
        for (const auto& target : targets) {
            if (!target.stable || target.color == obstacle::Color::Unknown) continue;
            if (!best || target.center.y > best->center.y ||
                (target.center.y == best->center.y && target.confidence > best->confidence)) best = &target;
        }
        return best;
    }

    void beginEvent(const obstacle::Detection& target, int lap) {
        if (++eventId_ == 0) ++eventId_; // Reserve zero to mean "no event".
        color_ = target.color;
        centerPx_ = target.center;
        confidence_ = target.confidence;
        switch (target.color) {
        case obstacle::Color::Green: action_ = Action::PassGreen; break;
        case obstacle::Color::Yellow: action_ = Action::BypassYellow; break;
        case obstacle::Color::Red:
            action_ = lap == 1 ? Action::IgnoreRed : Action::StopRedFiveSeconds;
            break;
        default:
            action_ = Action::None;
            phase_ = Phase::Fault;
            fault_ = true;
            return;
        }
        phase_ = Phase::AwaitingAck;
        clearCount_ = 0;
        actionAccepted_ = actionCompleted_ = false;
    }

    void resetCurrent() {
        phase_ = Phase::Idle;
        action_ = Action::None;
        color_ = obstacle::Color::Unknown;
        centerPx_ = {};
        confidence_ = 0;
        clearCount_ = 0;
        fault_ = false;
        actionAccepted_ = actionCompleted_ = false;
    }

    Event output() const {
        Event event;
        event.eventId = eventId_;
        event.phase = phase_;
        event.action = action_;
        event.color = color_;
        event.centerPx = centerPx_;
        event.confidence = confidence_;
        event.commandValid = phase_ == Phase::AwaitingAck;
        event.fault = fault_;
        event.actionAccepted = actionAccepted_;
        event.actionCompleted = actionCompleted_;
        return event;
    }

    Config config_;
    Phase phase_ = Phase::Idle;
    Action action_ = Action::None;
    obstacle::Color color_ = obstacle::Color::Unknown;
    std::uint32_t eventId_ = 0;
    cv::Point2f centerPx_;
    double confidence_ = 0;
    int clearCount_ = 0;
    bool fault_ = false;
    bool actionAccepted_ = false;
    bool actionCompleted_ = false;
};

inline const char* phaseName(Phase phase) {
    switch (phase) {
    case Phase::Idle: return "IDLE";
    case Phase::AwaitingAck: return "AWAITING_ACK";
    case Phase::AwaitingClear: return "AWAITING_CLEAR";
    case Phase::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
inline const char* actionName(Action action) {
    switch (action) {
    case Action::None: return "NONE";
    case Action::PassGreen: return "PASS_GREEN";
    case Action::IgnoreRed: return "IGNORE_RED";
    case Action::BypassYellow: return "BYPASS_YELLOW";
    case Action::StopRedFiveSeconds: return "STOP_RED_5S";
    }
    return "UNKNOWN";
}
inline const char* ackName(Ack ack) {
    switch (ack) {
    case Ack::None: return "NONE";
    case Ack::Accepted: return "ACCEPTED";
    case Ack::Completed: return "COMPLETED";
    case Ack::Rejected: return "REJECTED";
    }
    return "UNKNOWN";
}

} // namespace obstacle_task
