#include "evolib/telemetry.hpp"
#include "evolib/robot.hpp"
#include "pros/misc.h"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace evolib;
template<class F> void until(F test, uint32_t timeout = 2000) {
    const auto start = std::chrono::steady_clock::now();
    while (!test() && std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout)) pros::delay(2);
    assert(test());
}
std::string read(const std::string& path) {
    std::ifstream file(path);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
std::vector<std::string> fields(const std::string& line) {
    std::stringstream stream(line);
    std::vector<std::string> result;
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    if (line.back() == ',') result.push_back("");
    return result;
}
int main() {
    const auto root = std::filesystem::absolute("bin/telemetry-tests") /
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    std::filesystem::create_directories(root);
    TelemetryConfig config{20, 20, 32768, root.string()};
    Robot robot;
    std::string firstFile, firstContent;
    {
        Telemetry log(robot, {{-5, "drive,\nname"}, {9, "missing"}}, config);
        log.initialize();
        pros::c::card = false;
        log.start("no_card"); until([&] { return log.status() == TelemetryStatus::noCard; });
        assert(log.filename().empty());
        pros::c::card = true;
        pros::clockOffset = UINT32_MAX - pros::millis() - 30;
        log.start("route,\nA"); until([&] { return log.status() == TelemetryStatus::recording; });
        firstFile = log.filename();
        pros::delay(80); robot.setPose({4, 6, 1}); pros::delay(60);
        log.stop(); until([&] { return log.status() == TelemetryStatus::idle; });
        firstContent = read(firstFile);
        std::stringstream csv(firstContent);
        std::string line; std::getline(csv, line);
        const auto columns = fields(line);
        assert(columns.size() == 24);
        assert(columns[14] == "m5_drive__name_rpm");
        uint64_t previous = 0;
        int count = 0;
        while (std::getline(csv, line)) {
            const auto row = fields(line);
            assert(row.size() == columns.size());
            assert(row[4] == "route__A" && row[5] == "autonomous");
            assert(row[14] == "-321.0000" && row[16] == "42.5000");
            assert(row[21].empty() && row[23].empty());
            assert(std::stoull(row[1]) >= previous);
            previous = std::stoull(row[1]);
            if (count == 0) assert(row[9].empty() && row[10].empty());
            ++count;
        }
        assert(count >= 4 && previous > UINT32_MAX);
        log.start("second"); until([&] { return log.status() == TelemetryStatus::recording; });
        assert(log.filename() != firstFile);
        pros::c::card = false; until([&] { return log.status() == TelemetryStatus::noCard; });
        pros::c::card = true;
    }
    assert(read(firstFile) == firstContent);
    {
        Telemetry restart(robot, {{5, "drive"}}, config);
        restart.start(); until([&] { return restart.status() == TelemetryStatus::recording; });
        assert(restart.filename() != firstFile);
        pros::delay(25);
    }
    assert(read(firstFile) == firstContent);
    {
        auto small = config; small.maxFileBytes = 4096;
        Telemetry limited(robot, {{5, "drive"}}, small);
        limited.start(); until([&] { return limited.status() == TelemetryStatus::fileLimit; });
        assert(std::filesystem::file_size(limited.filename()) <= 4096);
    }
    {
        auto missing = config; missing.directory += "/does-not-exist";
        Telemetry error(robot, {}, missing);
        error.start(); until([&] { return error.status() == TelemetryStatus::ioError; });
        Telemetry duplicate(robot, {{5, "one"}, {-5, "two"}}, config);
        duplicate.start(); assert(duplicate.status() == TelemetryStatus::invalidConfig);
        Telemetry invalid(robot, {{0, "zero"}}, config);
        assert(invalid.status() == TelemetryStatus::invalidConfig);
    }
    {
        Telemetry custom([] { return Pose{12, 34, 90}; }, {{2, "custom"}}, config);
        custom.start("custom_pose");
        until([&] { return custom.status() == TelemetryStatus::recording; });
        pros::delay(30);
        custom.stop(); until([&] { return custom.status() == TelemetryStatus::idle; });
        assert(read(custom.filename()).find(",12.0000,34.0000,90.0000,") != std::string::npos);
        Telemetry empty(std::function<Pose()>{}, {}, config);
        assert(empty.status() == TelemetryStatus::invalidConfig);
    }
    std::cout << "PASS: CSV schema/units, rollover, error blanks, card removal/recovery, unique files, flush/close, size limit, configuration, custom pose callback\n";
}
