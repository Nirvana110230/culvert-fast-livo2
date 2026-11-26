#pragma once

#include "shared_timestamp.hpp"
#include "../../../include/rolling_csv.hpp"

class ScanTimestampWriter {
 public:
  ScanTimestampWriter()
      : shared_(timestamp_environment("LIVO_TIMESHARE_PATH"), true),
        trace_(timestamp_environment("LIVO_TIMESHARE_TRACE"),
               "scan_sequence,shared_stamp_ns,scan_first_point_ns,scan_last_point_ns,"
               "points,write_before_mono_ns,write_after_mono_ns") {}

  void publish(uint64_t stamp, uint64_t first, uint64_t last, uint32_t points) {
    if (stamp == 0 || first > last || points == 0)
      throw std::runtime_error("Invalid scan for timeshare publication");
    const auto before = timestamp_monotonic_ns();
    shared_.store(stamp);
    const auto after = timestamp_monotonic_ns();
    trace_.row() << ++sequence_ << ',' << stamp << ',' << first << ',' << last << ','
           << points << ',' << before << ',' << after << '\n';
    trace_.flush();
  }

 private:
  SharedTimestamp shared_;
  RollingCsv trace_;
  uint64_t sequence_ = 0;
};
