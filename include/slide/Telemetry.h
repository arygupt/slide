#pragma once

#include <cstdio>
#include <string>
#include <vector>

namespace slide {

class Vehicle;

// CSV data logger, like a test-cell DAQ: one row per recorded step. Channels
// cover the chassis, powertrain, aids and every corner's tire and suspension.
class TelemetryLog {
public:
    TelemetryLog() = default;
    TelemetryLog(const TelemetryLog&) = delete;
    TelemetryLog& operator=(const TelemetryLog&) = delete;
    ~TelemetryLog() { close(); }

    // `decimation` = record every Nth call (e.g. 10 for 100 Hz from 1 kHz).
    bool open(const std::string& path, int decimation = 1);
    void record(const Vehicle& vehicle);
    void close();

    bool isOpen() const { return file_ != nullptr; }
    long rows() const { return rows_; }
    const std::string& path() const { return path_; }

private:
    std::FILE* file_ = nullptr;
    std::string path_;
    std::vector<double> values_;
    int decimation_ = 1;
    long calls_ = 0;
    long rows_ = 0;
};

}  // namespace slide
