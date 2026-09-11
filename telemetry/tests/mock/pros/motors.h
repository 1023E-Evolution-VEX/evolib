#pragma once
#include <cmath>
#include <cstdint>
namespace pros::c {
inline double motor_get_actual_velocity(int8_t port) { return port < 0 ? -321 : 321; }
inline int motor_get_target_velocity(int8_t) { return 400; }
inline double motor_get_temperature(int8_t port) { return port == 9 ? INFINITY : 42.5; }
inline int motor_get_voltage(int8_t) { return 6000; }
inline int motor_get_current_draw(int8_t port) { return port == 9 ? INT32_MAX : 1500; }
}
