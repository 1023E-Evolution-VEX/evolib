#include "evolib/telemetry.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <utility>

#include "evolib/robot.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

namespace {
std::string label(const std::string& input) {
    std::string result;
    for (unsigned char c : input) {
        if (result.size() == 40) break;
        result += ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '_')
                      ? char(c)
                      : '_';
    }
    return result.empty() ? "unnamed" : result;
}
void number(std::string& row, double value) {
    row += ',';
    if (!std::isfinite(value) || value == INT32_MAX) return;
    char text[48];
    std::snprintf(text, sizeof(text), "%.4f", value);
    row += text;
}
const char* mode(uint8_t state) {
    return state & COMPETITION_DISABLED     ? "disabled"
           : state & COMPETITION_AUTONOMOUS ? "autonomous"
                                            : "driver";
}
}  // namespace

namespace evolib {
Telemetry::Telemetry(Robot& robot, std::initializer_list<TelemetryMotor> motors,
                     TelemetryConfig config)
    : Telemetry([&robot] { return robot.getPose(); }, motors,
                std::move(config)) {}

Telemetry::Telemetry(std::function<Pose()> readPose,
                     std::initializer_list<TelemetryMotor> motors,
                     TelemetryConfig config)
    : readPose_(std::move(readPose)), config_(std::move(config)) {
    valid_ = config_.periodMs >= 20 && config_.flushMs >= config_.periodMs &&
             config_.maxFileBytes >= 4096 && !config_.directory.empty() &&
             motors.size() <= 21 && bool(readPose_);
    bool used[22]{};
    for (const auto& motor : motors) {
        const int port = std::abs(int(motor.port));
        if (port < 1 || port > 21 || used[port]) {
            valid_ = false;
            continue;
        }
        used[port] = true;
        motors_.push_back(
            {motor.port, label(motor.name ? motor.name : "motor")});
    }
    if (!valid_) status_ = TelemetryStatus::invalidConfig;
}

Telemetry::~Telemetry() {
    shutdown_ = true;
    while (!exited_) pros::delay(5);
}
void Telemetry::initialize() {
    std::lock_guard<pros::Mutex> guard(control_);
    if (!valid_ || worker_) return;
    exited_ = false;
    worker_ = std::make_unique<pros::Task>(
        [this] { work(); }, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT,
        "evo telemetry");
}
void Telemetry::start(const std::string& runName) {
    initialize();
    std::lock_guard<pros::Mutex> guard(control_);
    if (!valid_) return;
    requestedName_ = label(runName);
    ++requested_;
    wanted_ = true;
    status_ = TelemetryStatus::opening;
}
void Telemetry::stop() {
    std::lock_guard<pros::Mutex> guard(control_);
    wanted_ = false;
    ++requested_;
}
std::string Telemetry::filename() {
    std::lock_guard<pros::Mutex> guard(control_);
    return filename_;
}

void Telemetry::work() {
    FILE* file = nullptr;
    uint32_t observed = 0, nextFile = 1, lastClock = pros::millis(),
             flushed = lastClock;
    uint64_t uptime = lastClock, runStart = uptime, due = uptime;
    uint32_t samples = 0, missed = 0, bytes = 0;
    Pose previous(0, 0, 0);
    uint64_t previousTime = 0;
    bool havePrevious = false;
    std::string run, row,
        header =
            "sample,uptime_ms,elapsed_ms,elapsed_s,run,mode,x_in,y_in,heading_"
            "deg,"
            "linear_speed_ips,angular_speed_dps,battery_mv,battery_capacity_"
            "pct,missed_samples";
    row.reserve(8192);
    for (const auto& motor : motors_) {
        const std::string prefix =
            ",m" + std::to_string(std::abs(int(motor.port))) + "_" + motor.name;
        for (const char* field :
             {"_rpm", "_target_rpm", "_temp_c", "_voltage_mv", "_current_ma"})
            header += prefix + field;
    }
    header += '\n';
    const auto close = [&] {
        if (file) {
            const bool failed = std::fclose(file) != 0;
            file = nullptr;
            if (failed) status_ = TelemetryStatus::ioError;
        }
    };
    while (!shutdown_) {
        uint32_t now = pros::millis();
        uptime += uint32_t(now - lastClock);
        lastClock = now;
        bool changed = false, wanted = false;
        {
            std::lock_guard<pros::Mutex> guard(control_);
            if (requested_ != observed) {
                observed = requested_;
                run = requestedName_;
                wanted = wanted_;
                changed = true;
            }
        }
        if (changed) {
            close();
            if (!wanted) {
                if (status_ == TelemetryStatus::recording ||
                    status_ == TelemetryStatus::opening)
                    status_ = TelemetryStatus::idle;
            } else if (pros::c::usd_is_installed() != 1)
                status_ = TelemetryStatus::noCard;
            else {
                std::string path;
                for (; nextFile <= 999999; ++nextFile) {
                    char suffix[32];
                    std::snprintf(suffix, sizeof(suffix), "/evo_%06lu.csv",
                                  static_cast<unsigned long>(nextFile));
                    path = config_.directory + suffix;
                    errno = 0;
                    FILE* existing = std::fopen(path.c_str(), "rb");
                    if (existing) {
                        std::fclose(existing);
                        continue;
                    }
                    if (errno != ENOENT) {
                        path.clear();
                        break;
                    }
                    file = std::fopen(path.c_str(), "wb");
                    ++nextFile;
                    break;
                }
                if (!file)
                    status_ = TelemetryStatus::ioError;
                else {
                    if (std::fwrite(header.data(), 1, header.size(), file) !=
                            header.size() ||
                        std::fflush(file)) {
                        status_ = TelemetryStatus::ioError;
                        close();
                    } else {
                        {
                            std::lock_guard<pros::Mutex> guard(control_);
                            filename_ = path;
                        }
                        samples = missed = 0;
                        bytes = header.size();
                        now = pros::millis();
                        uptime += uint32_t(now - lastClock);
                        lastClock = now;
                        runStart = due = uptime;
                        flushed = now;
                        havePrevious = false;
                        status_ = TelemetryStatus::recording;
                    }
                }
            }
        }
        if (file && uptime >= due) {
            if (pros::c::usd_is_installed() != 1) {
                status_ = TelemetryStatus::noCard;
                close();
            } else {
                const uint64_t skipped = (uptime - due) / config_.periodMs;
                missed += skipped;
                due += (skipped + 1) * config_.periodMs;
                const Pose pose = readPose_();
                char first[128];
                std::snprintf(
                    first, sizeof(first), "%lu,%llu,%llu,%.3f,",
                    static_cast<unsigned long>(samples),
                    static_cast<unsigned long long>(uptime),
                    static_cast<unsigned long long>(uptime - runStart),
                    (uptime - runStart) * 0.001);
                row = first;
                row += run;
                row += ',';
                row += mode(pros::c::competition_get_status());
                number(row, pose.x);
                number(row, pose.y);
                number(row, pose.theta);
                const double seconds = (uptime - previousTime) * 0.001;
                const double unknown = std::numeric_limits<double>::quiet_NaN();
                number(row, havePrevious && seconds > 0
                                ? std::hypot(pose.x - previous.x,
                                             pose.y - previous.y) /
                                      seconds
                                : unknown);
                number(row, havePrevious && seconds > 0
                                ? std::remainder(pose.theta - previous.theta,
                                                 360.0) /
                                      seconds
                                : unknown);
                number(row, pros::c::battery_get_voltage());
                number(row, pros::c::battery_get_capacity());
                number(row, missed);
                for (const auto& motor : motors_) {
                    number(row, pros::c::motor_get_actual_velocity(motor.port));
                    number(row, pros::c::motor_get_target_velocity(motor.port));
                    number(row, pros::c::motor_get_temperature(motor.port));
                    number(row, pros::c::motor_get_voltage(motor.port));
                    number(row, pros::c::motor_get_current_draw(motor.port));
                }
                row += '\n';
                if (bytes + row.size() > config_.maxFileBytes) {
                    status_ = TelemetryStatus::fileLimit;
                    close();
                } else if (std::fwrite(row.data(), 1, row.size(), file) !=
                           row.size()) {
                    status_ = TelemetryStatus::ioError;
                    close();
                } else {
                    bytes += row.size();
                    ++samples;
                    previous = pose;
                    previousTime = uptime;
                    havePrevious = std::isfinite(pose.x) &&
                                   std::isfinite(pose.y) &&
                                   std::isfinite(pose.theta);
                    if (uint32_t(now - flushed) >= config_.flushMs) {
                        if (std::fflush(file)) {
                            status_ = TelemetryStatus::ioError;
                            close();
                        }
                        flushed = now;
                    }
                }
            }
        }
        pros::delay(5);
    }
    close();
    exited_ = true;
}
}  // namespace evolib
