#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <utility>
#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 8192
namespace pros {
inline std::atomic<uint32_t> clockOffset{0};
inline const auto epoch = std::chrono::steady_clock::now();
inline uint32_t millis() {
    return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - epoch).count()) + clockOffset;
}
inline void delay(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
using Mutex = std::mutex;
class Task {
    std::thread thread_;
public:
    template<class F> explicit Task(F&& fn, uint32_t, uint16_t, const char*) : thread_(std::forward<F>(fn)) {}
    ~Task() { if (thread_.joinable()) thread_.join(); }
};
}
