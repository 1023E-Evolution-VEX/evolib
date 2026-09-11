#include "evolib/rcl.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pros/rtos.hpp"

namespace evolib {
namespace {
constexpr float rad = 0.017453292519943295f;
struct Obstacle {
    float x, y, a, b;
    bool circle;
    std::uint32_t created, lifetime;
};
pros::Mutex mutex;
struct Lock {
    Lock() { mutex.take(); }
    ~Lock() { mutex.give(); }
};
std::vector<RclSensor> sensors;
std::vector<Obstacle> obstacles;
RclConfig config;
RclStatus status;
std::uint32_t last = 0, countX = 0, countY = 0;
float sumX = 0, sumY = 0;
bool scheduled = false;

void reset() {
    sumX = sumY = 0;
    countX = countY = 0;
    scheduled = false;
    status.correctionX = status.correctionY = 0;
}

bool blocked(float x, float y, float dx, float dy, float limit) {
    for (const auto& o : obstacles) {
        float qx = o.x - x, qy = o.y - y;
        if (o.circle) {
            float projection = qx * dx + qy * dy;
            float discriminant =
                o.a * o.a - (qx * qx + qy * qy - projection * projection);
            if (discriminant < 0) continue;
            float extent = std::sqrt(discriminant);
            if (projection + extent >= 0 && projection - extent <= limit)
                return true;
        } else {
            float vx = o.a - o.x, vy = o.b - o.y;
            float det = dx * vy - dy * vx;
            if (std::abs(det) < 1e-6f) {
                if (std::abs(qx * dy - qy * dx) < 1e-5f) {
                    float a = qx * dx + qy * dy;
                    float b = (o.a - x) * dx + (o.b - y) * dy;
                    if (std::max(a, b) >= 0 && std::min(a, b) <= limit)
                        return true;
                }
                continue;
            }
            float t = (qx * vy - qy * vx) / det;
            float u = (qx * dy - qy * dx) / det;
            if (t >= 0 && t <= limit && u >= 0 && u <= 1) return true;
        }
    }
    return false;
}
}  // namespace

bool configureRcl(std::vector<RclSensor> inputs, RclConfig c) {
    const float values[] = {
        c.fieldHalfSize, c.angleTolerance, c.minDistanceMm,   c.maxDistanceMm,
        c.minCorrection, c.maxCorrection,  c.maxSyncPerSecond};
    for (float v : values)
        if (!std::isfinite(v)) return false;
    if (inputs.empty() || c.fieldHalfSize <= 0 || c.angleTolerance < 0 ||
        c.angleTolerance >= 45 || c.minDistanceMm <= 0 ||
        c.maxDistanceMm < c.minDistanceMm || c.maxDistanceMm > 2000 ||
        c.minConfidence < 0 || c.minConfidence > 63 || c.minCorrection < 0 ||
        c.maxCorrection < c.minCorrection || c.maxSyncPerSecond <= 0 ||
        c.periodMs < 10 || c.periodMs > 1000 || c.samples == 0 ||
        c.samples > 100)
        return false;
    for (const auto& s : inputs)
        if (!s.sensor || !std::isfinite(s.right) || !std::isfinite(s.forward) ||
            !std::isfinite(s.heading))
            return false;
    Lock lock;
    sensors = std::move(inputs);
    config = c;
    status = {};
    status.enabled = true;
    reset();
    return true;
}

void disableRcl() {
    Lock lock;
    status.enabled = false;
    reset();
}
RclStatus getRclStatus() {
    Lock lock;
    return status;
}
void detail::resetRcl() {
    Lock lock;
    reset();
}
void clearRclObstacles() {
    Lock lock;
    obstacles.clear();
}

bool addRclLine(float x1, float y1, float x2, float y2,
                std::uint32_t lifetime) {
    if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) ||
        !std::isfinite(y2) || (x1 == x2 && y1 == y2) || lifetime > 0x7fffffffU)
        return false;
    Lock lock;
    obstacles.push_back({x1, y1, x2, y2, false, pros::millis(), lifetime});
    return true;
}

bool addRclCircle(float x, float y, float radius, std::uint32_t lifetime) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) ||
        radius <= 0 || lifetime > 0x7fffffffU)
        return false;
    Lock lock;
    obstacles.push_back({x, y, radius, 0, true, pros::millis(), lifetime});
    return true;
}

Pose detail::correctRcl(Pose pose, std::uint32_t now) {
    Lock lock;
    if (!status.enabled || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta))
        return pose;
    if (scheduled && now - last < config.periodMs) return pose;
    float dt = scheduled ? std::min(now - last, config.periodMs * 2) / 1000.f
                         : config.periodMs / 1000.f;
    last = now;
    scheduled = true;
    status.correctionX = status.correctionY = 0;
    obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(),
                                   [now](const auto& o) {
                                       return o.lifetime &&
                                              now - o.created >= o.lifetime;
                                   }),
                    obstacles.end());
    float xTotal = 0, yTotal = 0;
    unsigned nx = 0, ny = 0;
    const float h = pose.theta * rad, half = config.fieldHalfSize;
    for (const auto& s : sensors) {
        int mm = s.sensor->get();
        float heading = pose.theta + s.heading;
        float angle = std::abs(std::remainder(heading, 90.f));
        if (mm < config.minDistanceMm || mm > config.maxDistanceMm ||
            angle > config.angleTolerance ||
            (mm > 200 && (s.sensor->get_confidence() < config.minConfidence ||
                          s.sensor->get_confidence() > 63))) {
            ++status.rejected;
            continue;
        }
        float ox = s.right * std::cos(h) + s.forward * std::sin(h);
        float oy = -s.right * std::sin(h) + s.forward * std::cos(h);
        float x = pose.x + ox, y = pose.y + oy;
        float dx = std::sin(heading * rad), dy = std::cos(heading * rad);
        if (std::abs(x) >= half || std::abs(y) >= half) {
            ++status.rejected;
            continue;
        }
        float tx = std::abs(dx) < 1e-6f ? INFINITY
                                        : ((dx > 0 ? half : -half) - x) / dx;
        float ty = std::abs(dy) < 1e-6f ? INFINITY
                                        : ((dy > 0 ? half : -half) - y) / dy;
        bool axisX = tx < ty;
        float limit = std::min(tx, ty);
        float component = axisX ? dx : dy;
        float coordinate = (component > 0 ? half : -half) -
                           mm / 25.4f * component - (axisX ? ox : oy);
        float error = coordinate - (axisX ? pose.x : pose.y);
        if (blocked(x, y, dx, dy, limit) || std::abs(coordinate) > half ||
            std::abs(error) > config.maxCorrection) {
            ++status.rejected;
            continue;
        }
        ++status.accepted;
        if (axisX) {
            xTotal += error;
            ++nx;
        } else {
            yTotal += error;
            ++ny;
        }
    }
    if (nx) {
        sumX += xTotal / nx;
        ++countX;
    } else {
        sumX = 0;
        countX = 0;
    }
    if (ny) {
        sumY += yTotal / ny;
        ++countY;
    } else {
        sumY = 0;
        countY = 0;
    }
    float cx = countX >= config.samples ? sumX / countX : 0;
    float cy = countY >= config.samples ? sumY / countY : 0;
    if (countX >= config.samples) {
        sumX = 0;
        countX = 0;
    }
    if (countY >= config.samples) {
        sumY = 0;
        countY = 0;
    }
    if (std::abs(cx) < config.minCorrection) cx = 0;
    if (std::abs(cy) < config.minCorrection) cy = 0;
    float magnitude = std::hypot(cx, cy), bound = config.maxSyncPerSecond * dt;
    if (magnitude > bound) {
        cx *= bound / magnitude;
        cy *= bound / magnitude;
    }
    status.correctionX = cx;
    status.correctionY = cy;
    return {pose.x + cx, pose.y + cy, pose.theta};
}
}  // namespace evolib
