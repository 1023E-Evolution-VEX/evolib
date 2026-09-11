#pragma once
#include "evolib/pose.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace evolib {
class Robot;
struct TelemetryMotor {
    int8_t port;
    const char *name;
};

struct TelemetryConfig {
    uint32_t periodMs = 50;
    uint32_t flushMs = 1000;
    uint32_t maxFileBytes = 32 * 1024 * 1024;
    std::string directory = "/usd";
};

enum class TelemetryStatus { idle, opening, recording, noCard, ioError, fileLimit, invalidConfig };

class Telemetry {
    struct Motor {
        int8_t port;
        std::string name;
    };
    std::function<Pose()> readPose_;
    TelemetryConfig config_;
    std::vector<Motor> motors_;
    pros::Mutex control_;
    std::string requestedName_, filename_;
    uint32_t requested_ = 0;
    bool wanted_ = false, valid_ = true;
    std::atomic<bool> shutdown_{false}, exited_{true};
    std::atomic<TelemetryStatus> status_{TelemetryStatus::idle};
    std::unique_ptr<pros::Task> worker_;
    void work();

  public:
    Telemetry(Robot &robot, std::initializer_list<TelemetryMotor> motors, TelemetryConfig config = {});
    Telemetry(std::function<Pose()> readPose, std::initializer_list<TelemetryMotor> motors,
              TelemetryConfig config = {});
    ~Telemetry();
    Telemetry(const Telemetry &) = delete;
    Telemetry &operator=(const Telemetry &) = delete;
    void initialize();
    void start(const std::string &runName = "autonomous");
    void stop();
    TelemetryStatus status() const { return status_.load(); }
    std::string filename();
};
}                    
