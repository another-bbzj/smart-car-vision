#pragma once

#include <cmath>
#include <cstdint>

namespace task {

// V1.1: the mandatory gap is 1.60 m total (L100 + L60) on the upper-right straight.
// 断路任务的大阶段：从未启用、接近缺口、驶过缺口，到重新看到并确认赛道。
enum class GapPhase {
    Disabled,   // 未使能，按普通路线行驶。
    Armed,      // 路线层已使能，等待看到断路前的有效赛道。
    Approach,   // 沿可见赛道接近断路入口。
    Crossing,   // 已进入缺口，控制侧应使用航向保持策略。
    Recovering, // 已看到后续赛道，继续行驶并确认赛道持续有效。
    Complete,   // 断路任务完成。
    Fault       // 输入异常、行程超限或长时间未恢复赛道。
};

// 断路失败原因；用于日志和上层保护策略定位问题。
enum class GapFault {
    None,          // 当前没有故障。
    InvalidInput,  // 传感器输入无效或关键输入缺失。
    Overshoot,     // 通过距离或时间超过允许上限。
    Timeout,       // 缺口中长时间没有看到后续赛道。
    LostAfterGap   // 还在接近阶段就连续丢失赛道。
};

// 断路行程、时间、速度和确认帧数参数。默认总长 1.60 m 对应 V1.1 的 L100+L60。
struct GapConfig {
    double totalLengthM = 1.60;
    double approachDistanceM = 0.20;
    double maxCrossingDistanceM = 2.40;
    double maxCrossingTimeS = 8.0;
    double recoveryDistanceM = 0.30;
    double cruiseSpeedMps = 0.25;
    double gapSpeedMps = 0.18;
    double maxSteeringDeg = 8.0;
    int entryConfirmFrames = 3;
    int recoveryConfirmFrames = 3;
    int lostFrameLimit = 8;

    // 检查距离、速度和帧数阈值是否在合理范围，拒绝明显错误的配置。
    bool valid() const {
        return std::isfinite(totalLengthM) && totalLengthM > 0.5 && totalLengthM < 5.0 &&
            std::isfinite(approachDistanceM) && approachDistanceM >= 0 && approachDistanceM < totalLengthM &&
            std::isfinite(maxCrossingDistanceM) && maxCrossingDistanceM >= totalLengthM && maxCrossingDistanceM < 10.0 &&
            std::isfinite(maxCrossingTimeS) && maxCrossingTimeS > 0 && maxCrossingTimeS < 60 &&
            std::isfinite(recoveryDistanceM) && recoveryDistanceM > 0 && recoveryDistanceM < 2.0 &&
            std::isfinite(cruiseSpeedMps) && cruiseSpeedMps > 0 && cruiseSpeedMps < 3.0 &&
            std::isfinite(gapSpeedMps) && gapSpeedMps > 0 && gapSpeedMps <= cruiseSpeedMps &&
            std::isfinite(maxSteeringDeg) && maxSteeringDeg > 0 && maxSteeringDeg <= 45 &&
            entryConfirmFrames > 0 && entryConfirmFrames <= 30 &&
            recoveryConfirmFrames > 0 && recoveryConfirmFrames <= 30 &&
            lostFrameLimit >= recoveryConfirmFrames && lostFrameLimit <= 120;
    }
};

// 每次更新状态机需要的一组输入，由路线层、编码器、IMU 和视觉模块提供。
struct GapInput {
    bool enabled = false;             // 路线层确认下一段是 V1.1 右上角断路时设为 true。
    double odometerM = 0;             // 编码器累计里程，单位米，任务期间应单调增加。
    double speedMps = 0;              // 当前前进速度，单位米/秒。
    double dtS = 0;                   // 距上一次 update 的时间，单位秒。
    bool trackVisible = false;        // 本帧视觉是否检测到有效赛道。
    bool headingValid = false;        // IMU/控制层当前航向估计是否可信。
    double headingErrorDeg = 0;       // 当前车头相对断路前直行方向的偏差，单位度。
};

// 状态机输出给控制层的建议和状态，不是直接执行的底盘命令。
struct GapOutput {
    GapPhase phase = GapPhase::Disabled; // 当前任务阶段。
    GapFault fault = GapFault::None;     // 若失败，给出失败原因。
    bool routeEnableRequired = true;     // true 表示路线层需要负责使能/重新准备任务。
    bool holdLastSteering = false;       // 丢失赛道时提示控制层暂时保持上一转向。
    bool useHeadingHold = false;         // true 表示断路中应使用 IMU 航向保持。
    bool requestStop = false;            // 故障保护建议；最终停车动作由控制层执行。
    bool routeComplete = false;          // 断路任务已完成，可继续后续路线。
    double suggestedSpeedMps = 0;        // 当前阶段建议速度，单位米/秒。
    double distanceFromEntryM = 0;       // 从进入断路计程原点累计的距离，单位米。
    double headingErrorDeg = 0;          // 最近一次输入的航向误差，单位度。
    double maxSteeringDeg = 0;           // 供控制侧限制转向幅度的参考值，单位度。
};

// 断路状态机：路线层显式使能后，按里程、航向和视觉恢复情况逐帧推进。
// 它只给出状态/建议，不直接发送转向、油门或刹车命令。
class GapStateMachine {
public:
    // 创建状态机并保存本次任务使用的距离、速度和确认阈值。
    explicit GapStateMachine(GapConfig config = {}) : config_(config) {}

    // 查询构造时提供的参数是否有效。
    bool validConfig() const { return config_.valid(); }

    // 清空阶段、计程原点、累计时间和连续帧计数，准备一轮新的断路任务。
    void reset() {
        phase_ = GapPhase::Disabled;
        fault_ = GapFault::None;
        enabledLast_ = false;
        entryArmed_ = false;
        originOdometerM_ = 0;
        crossingStartTimeS_ = 0;
        elapsedS_ = 0;
        entryFrames_ = recoveryFrames_ = lostFrames_ = 0;
        lastOdometerM_ = 0;
        lastTrackVisible_ = false;
        lastHeadingErrorDeg_ = 0;
    }

    // 每帧调用一次：先检查输入，再按当前阶段更新状态，并返回给控制层的建议。
    // enabled 应由路线定位模块在右上角断路前置地标处打开。
    GapOutput update(const GapInput& in) {
        if (!finiteInput(in)) return fail(GapFault::InvalidInput);
        if (!config_.valid()) return fail(GapFault::InvalidInput);

        if (!in.enabled) {
            // A completed/faulted route stays latched until the route planner resets it.
            if (phase_ == GapPhase::Complete || phase_ == GapPhase::Fault) return output();
            reset();
            return output();
        }
        if (phase_ == GapPhase::Disabled) phase_ = GapPhase::Armed;

        if (enabledLast_ && in.dtS > 0) elapsedS_ += in.dtS;
        enabledLast_ = true;
        lastOdometerM_ = in.odometerM;
        lastTrackVisible_ = in.trackVisible;
        lastHeadingErrorDeg_ = in.headingErrorDeg;

        switch (phase_) {
        case GapPhase::Disabled:
            break;
        case GapPhase::Armed:
            // Route localization must enable this state before the physical gap.
            if (in.trackVisible) phase_ = GapPhase::Approach;
            break;
        case GapPhase::Approach:
            if (!in.trackVisible) {
                if (++entryFrames_ >= config_.entryConfirmFrames) return fail(GapFault::LostAfterGap);
                break;
            }
            entryFrames_ = 0;
            if (in.odometerM - originOdometerM_ >= config_.approachDistanceM) {
                if (!in.headingValid) return fail(GapFault::InvalidInput);
                originOdometerM_ = in.odometerM;
                crossingStartTimeS_ = elapsedS_;
                phase_ = GapPhase::Crossing;
                lostFrames_ = 0;
            }
            break;
        case GapPhase::Crossing: {
            const double distance = in.odometerM - originOdometerM_;
            if (distance < -0.02) return fail(GapFault::InvalidInput);
            if (distance > config_.maxCrossingDistanceM ||
                elapsedS_ - crossingStartTimeS_ > config_.maxCrossingTimeS) return fail(GapFault::Overshoot);
            if (in.trackVisible) {
                if (++recoveryFrames_ >= config_.recoveryConfirmFrames && distance >= config_.totalLengthM) {
                    phase_ = GapPhase::Recovering;
                    recoveryFrames_ = 0;
                }
            } else {
                recoveryFrames_ = 0;
                if (++lostFrames_ > config_.lostFrameLimit) return fail(GapFault::Timeout);
            }
            break;
        }
        case GapPhase::Recovering: {
            const double distance = in.odometerM - originOdometerM_;
            if (!in.trackVisible) {
                phase_ = GapPhase::Crossing;
                recoveryFrames_ = 0;
                if (++lostFrames_ > config_.lostFrameLimit) return fail(GapFault::Timeout);
                break;
            }
            lostFrames_ = 0;
            if (distance > config_.maxCrossingDistanceM) return fail(GapFault::Overshoot);
            if (distance >= config_.totalLengthM + config_.recoveryDistanceM) {
                if (++recoveryFrames_ >= config_.recoveryConfirmFrames) {
                    phase_ = GapPhase::Complete;
                    fault_ = GapFault::None;
                }
            } else {
                recoveryFrames_ = 0;
            }
            break;
        }
        case GapPhase::Complete:
        case GapPhase::Fault:
            break;
        }
        return output();
    }

    // 读取最近一次状态输出，不推进状态；适合只需查询当前阶段的调用方。
    GapOutput current() const { return output(); }

private:
    // 检查传感器数值是否有限且范围合理，避免 NaN/错误时间步污染状态机。
    bool finiteInput(const GapInput& in) const {
        return std::isfinite(in.odometerM) && std::isfinite(in.speedMps) && in.speedMps >= 0 &&
            std::isfinite(in.dtS) && in.dtS >= 0 && in.dtS <= 1.0 &&
            std::isfinite(in.headingErrorDeg) && std::abs(in.headingErrorDeg) <= 180;
    }

    // 统一进入故障状态并记录原因；调用方应根据输出执行自己的安全策略。
    GapOutput fail(GapFault reason) {
        phase_ = GapPhase::Fault;
        fault_ = reason;
        return output();
    }

    // 根据内部阶段拼出完整输出结构，包括建议速度、距离和保护标志。
    GapOutput output() const {
        GapOutput out;
        out.phase = phase_;
        out.fault = fault_;
        out.distanceFromEntryM = phase_ == GapPhase::Crossing || phase_ == GapPhase::Recovering || phase_ == GapPhase::Complete
            ? lastOdometerM_ - originOdometerM_ : 0;
        out.routeEnableRequired = phase_ == GapPhase::Disabled || phase_ == GapPhase::Complete || phase_ == GapPhase::Fault;
        out.routeComplete = phase_ == GapPhase::Complete;
        out.requestStop = phase_ == GapPhase::Fault;
        out.suggestedSpeedMps = (phase_ == GapPhase::Approach || phase_ == GapPhase::Crossing || phase_ == GapPhase::Recovering)
            ? config_.gapSpeedMps : config_.cruiseSpeedMps;
        out.useHeadingHold = phase_ == GapPhase::Crossing;
        out.holdLastSteering = phase_ == GapPhase::Crossing && !lastTrackVisible_;
        out.headingErrorDeg = lastHeadingErrorDeg_;
        out.maxSteeringDeg = config_.maxSteeringDeg;
        return out;
    }

    GapConfig config_;                       // 本状态机使用的阈值配置。
    GapPhase phase_ = GapPhase::Disabled;    // 当前处理阶段。
    GapFault fault_ = GapFault::None;        // 当前锁存的故障原因。
    bool enabledLast_ = false;               // 上一帧是否使能，用于累计任务耗时。
    bool entryArmed_ = false;                // 保留的入口标记字段。
    double originOdometerM_ = 0;             // 进入缺口时记录的编码器里程原点。
    double crossingStartTimeS_ = 0;          // 开始跨越缺口时的任务累计时间。
    double elapsedS_ = 0;                    // 本次使能后的累计时间。
    double lastOdometerM_ = 0;               // 最近一帧编码器累计里程，用于构造输出。
    bool lastTrackVisible_ = false;          // 最近一帧是否检测到赛道。
    double lastHeadingErrorDeg_ = 0;          // 最近一帧的航向误差。
    int entryFrames_ = 0, recoveryFrames_ = 0, lostFrames_ = 0; // 入口、恢复和丢线连续帧计数。
};

// 把断路阶段转成 CSV/调试日志中可读的固定名称。
inline const char* gapPhaseName(GapPhase phase) {
    switch (phase) {
    case GapPhase::Disabled: return "DISABLED";
    case GapPhase::Armed: return "ARMED";
    case GapPhase::Approach: return "APPROACH";
    case GapPhase::Crossing: return "CROSSING";
    case GapPhase::Recovering: return "RECOVERING";
    case GapPhase::Complete: return "COMPLETE";
    case GapPhase::Fault: return "FAULT";
    }
    return "UNKNOWN";
}
// 把故障枚举转成日志文本，便于查看断路状态为何停止推进。
inline const char* gapFaultName(GapFault fault) {
    switch (fault) {
    case GapFault::None: return "NONE";
    case GapFault::InvalidInput: return "INVALID_INPUT";
    case GapFault::Overshoot: return "OVERSHOOT";
    case GapFault::Timeout: return "TIMEOUT";
    case GapFault::LostAfterGap: return "LOST_AFTER_GAP";
    }
    return "UNKNOWN";
}

} // namespace task
