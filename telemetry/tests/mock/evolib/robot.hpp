#pragma once
#include "evolib/pose.hpp"
#include <mutex>
namespace evolib {
class Robot {
    Pose pose_{1, 2, 359};
    std::mutex mutex_;
public:
    Pose getPose() { std::lock_guard<std::mutex> guard(mutex_); return pose_; }
    void setPose(Pose pose) { std::lock_guard<std::mutex> guard(mutex_); pose_ = pose; }
};
}
