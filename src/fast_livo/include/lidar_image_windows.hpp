#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

// The callback sorts each cloud by curvature (milliseconds from its header).
// Keep every unconsumed suffix in that same source cloud, with its original
// header. The existing coverage guard can therefore use it even at EOF.
namespace lidar_image_windows {

inline bool strictEnabled()
{
  static const bool enabled = [] {
    const char *v = std::getenv("FAST_LIVO_STRICT_WINDOWS");
    return v && std::strcmp(v,"1") == 0;
  }();
  return enabled;
}

template<class Points>
void validateOffsets(const Points &points)
{
  // A new header is a lower bound on its scan's point times only under
  // this contract. Validate before sorting so NaNs never enter the comparator.
  for (const auto &point : points) {
    if (!std::isfinite(point.curvature) || point.curvature < 0)
      throw std::runtime_error("strict image windows require finite nonnegative scan offsets");
  }
}

template<class Clouds, class Headers>
bool covered(const Clouds &clouds, const Headers &headers, double end, bool input_eof)
{
  if (clouds.size() != headers.size())
    throw std::logic_error("LiDAR cloud and header queues are out of step");
  if (headers.empty()) return false;
  // Later monotonic headers cannot introduce a return before end. A scan's
  // last point alone does not rule out an overlapping scan still in transit.
  if (headers.back() >= end) return true;
  if (!input_eof) return false;
  for (std::size_t i = 0; i < clouds.size(); ++i) {
    if (!clouds[i]->empty() && headers[i] + clouds[i]->back().curvature * 0.001 >= end)
      return true;
  }
  return false;
}

template<class Clouds, class Headers, class Output, class Observer>
void consume(Clouds &clouds, Headers &headers, Output &output, double start, double end, Observer observe)
{
  output.clear();
  auto scan = clouds.begin();
  auto header = headers.begin();
  while (scan != clouds.end() && *header <= end) {
    auto &cloud = **scan;
    const double scan_start = *header;
    // The comparison uses the same absolute double sensor clock as the cutoff.
    // Exact-cut returns stay available to the following half-open window.
    auto cut = std::lower_bound(cloud.begin(), cloud.end(), end,
        [scan_start](const auto &p, double bound) { return scan_start + double(p.curvature)*0.001 < bound; });
    for (auto pt = cloud.begin(); pt != cut; ++pt) {
      const double relative = (scan_start-start)*1000.0 + pt->curvature;
      if (relative < -0.001)
        throw std::runtime_error("strict image cut received a return older than the closed interval");
      auto accepted = *pt;
      // Equality after epoch rounding can differ by a fraction of a microsecond.
      accepted.curvature = static_cast<float>(std::max(0.0, relative));
      observe(scan->get(), scan_start, pt->curvature, accepted.curvature);
      output.push_back(accepted);
    }
    if (cut == cloud.end()) {
      scan = clouds.erase(scan);
      header = headers.erase(header);
    } else {
      cloud.erase(cloud.begin(), cut);
      ++scan; ++header;
    }
  }
  // Scans may overlap. Reverse-time deskew requires a globally ordered slice.
  auto earlier = [](const auto &a, const auto &b) { return a.curvature < b.curvature; };
  if (!std::is_sorted(output.begin(), output.end(), earlier))
    std::stable_sort(output.begin(), output.end(), earlier);
}
}
