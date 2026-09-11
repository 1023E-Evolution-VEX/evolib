#pragma once
#include <atomic>
#include <cstdint>
inline constexpr int COMPETITION_DISABLED = 1, COMPETITION_AUTONOMOUS = 2;
namespace pros::c {
inline std::atomic<bool> card{true};
inline int usd_is_installed() { return card; }
inline uint8_t competition_get_status() { return COMPETITION_AUTONOMOUS; }
inline int battery_get_voltage() { return 12345; }
inline double battery_get_capacity() { return 87.5; }
}
