#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <vector>

namespace replay_timing {

inline const char *directory()
{
  static const char *path = std::getenv("FAST_LIVO_KEYFRAMES");
  return path && *path ? path : nullptr;
}

inline std::ofstream csv(const char *name, const char *columns)
{
  std::filesystem::create_directories(directory());
  std::ofstream file(std::filesystem::path(directory()) / name);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << std::setprecision(17) << columns << std::endl;
  return file;
}

// All offsets below are milliseconds. One microsecond exceeds both the double
// timestamp ULP at 2026 epochs and the float ULP for a typical 100 ms scan.
constexpr double tolerance_ms = 0.001;
inline double &windowEnd() { static double end = -1; return end; }

struct SourceTime {
  double header_s;
  float offset_ms;
  double relativeTo(double epoch) const { return (header_s - epoch) * 1000.0 + offset_ms; }
};

struct Window {
  double start = 0, end = 0, early_sum = 0, early_max = 0, rebase_max = 0;
  double min_offset = std::numeric_limits<double>::infinity(), max_offset = -min_offset;
  unsigned long long popped = 0, carry_in = 0, used = 0, late = 0, early = 0, early_carry = 0, exact_end = 0, near_end = 0;
  unsigned long long assigned_exact_end = 0, assigned_near_end = 0, new_scans = 0;
  std::vector<SourceTime> carry;

  void begin(double a, double b)
  {
    start = a; end = b; windowEnd() = b;
    carry_in = carry.size();
    popped = used = late = early = early_carry = exact_end = near_end = new_scans = 0;
    assigned_exact_end = assigned_near_end = 0;
    early_sum = early_max = rebase_max = 0;
    min_offset = std::numeric_limits<double>::infinity(); max_offset = -min_offset;
  }

  void assigned(SourceTime source, float actual_ms, bool carried)
  {
    const double offset = source.relativeTo(start), ahead = source.relativeTo(end);
    min_offset = std::min(min_offset, offset); max_offset = std::max(max_offset, offset);
    rebase_max = std::max(rebase_max, std::abs(offset - actual_ms));
    ++used;
    late += offset < -tolerance_ms;
    assigned_exact_end += ahead == 0; assigned_near_end += std::abs(ahead) <= tolerance_ms;
    if (ahead > tolerance_ms) {
      ++early; early_carry += carried; early_sum += ahead;
      early_max = std::max(early_max, ahead);
    }
  }

  void finish(std::size_t current, std::size_t next, std::size_t queue_points, std::size_t queued_scans)
  {
    static auto file = csv("timing_windows.csv",
      "start_s,end_s,new_scans,new_points,carry_in,current_points,carry_out,queued_scans,queued_points,identity_count_ok,early_gt_1us,early_carried_gt_1us,before_start_gt_1us,early_sum_ms,early_max_ms,offset_min_ms,offset_max_ms,max_rebase_error_ms,new_exact_cut,new_within_1us_cut,assigned_exact_cut,assigned_within_1us_cut");
    const bool balanced = used == current && carry.size() == next && popped + carry_in == current + next;
    file << start << ',' << end << ',' << new_scans << ',' << popped << ',' << carry_in << ',' << current
         << ',' << next << ',' << queued_scans << ',' << queue_points << ',' << balanced << ',' << early
         << ',' << early_carry << ',' << late << ',' << early_sum << ',' << early_max << ','
         << (used ? min_offset : 0) << ',' << (used ? max_offset : 0) << ',' << rebase_max << ','
         << exact_end << ',' << near_end << ',' << assigned_exact_end << ',' << assigned_near_end << std::endl;
  }
};
inline Window &windows() { static Window instance; return instance; }

struct Preprocess {
  unsigned long long valid = 0, changed = 0, retained_changed = 0;
  double sum_abs = 0, max_abs = 0, retained_max = 0, min_ms = 0, max_ms = 0;
  void point(float raw_ms, float effective_ms, bool retained) {
    ++valid;
    const double delta = std::abs(double(raw_ms) - effective_ms);
    if (delta > 0) {
      ++changed; sum_abs += delta; max_abs = std::max(max_abs, delta);
      if (retained) { ++retained_changed; retained_max = std::max(retained_max, delta); }
    }
  }
  template<class Points> void finish(double stamp, std::size_t input, bool features, const Points &output) {
    if (!output.empty()) {
      min_ms = max_ms = output.front().curvature;
      for (const auto &p : output) { min_ms = std::min(min_ms, double(p.curvature)); max_ms = std::max(max_ms, double(p.curvature)); }
    }
    static auto file = csv("preprocess_timing.csv","scan_s,input_points,feature_enabled,smoothing_inputs,smoothing_changed,smoothing_abs_sum_ms,smoothing_abs_max_ms,output_points,retained_changed,retained_abs_max_ms,min_output_ms,max_output_ms");
    file << stamp << ',' << input << ',' << features << ',' << valid << ',' << changed << ',' << sum_abs << ',' << max_abs
         << ',' << output.size() << ',' << retained_changed << ',' << retained_max << ',' << min_ms << ',' << max_ms << std::endl;
  }
};

template<class Groups, class Time> void imuInit(const Groups &samples, Time stamp, double cut, int counter, int limit) {
  static auto file = csv("imu_init_samples.csv","cut_s,stamp_s,counter_after_group,limit,ax,ay,az,gx,gy,gz");
  for (const auto &p : samples) {
    const auto &a = p->linear_acceleration; const auto &g = p->angular_velocity;
    file << cut << ',' << stamp(p->header.stamp) << ',' << counter << ',' << limit << ','
         << a.x << ',' << a.y << ',' << a.z << ',' << g.x << ',' << g.y << ',' << g.z << '\n';
  }
  file.flush();
}

template<class Points> void deskew(double a, double b, const Points &points, std::size_t skipped, std::size_t poses)
{
  unsigned long long descending = 0, outside = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    descending += i && points[i].curvature < points[i-1].curvature;
    outside += points[i].curvature > (b-a)*1000.0 + tolerance_ms || points[i].curvature < -tolerance_ms;
  }
  static auto file = csv("deskew_timing.csv","prop_start_s,prop_end_s,input_points,transformed_points,untransformed_points,pose_count,time_inversions,outside_interval_gt_1us");
  file << a << ',' << b << ',' << points.size() << ',' << points.size()-skipped << ',' << skipped << ','
       << poses << ',' << descending << ',' << outside << std::endl;
}

template<class Matches, class Vec> void planes(const Matches &matches, const Vec &gravity, const Vec &sensor, int iter)
{
  static auto file = csv("plane_constraints.csv","cut_s,iterations,class,count,abs_sum_m,sq_sum_m,p50_abs_m,p95_abs_m,max_abs_m,nxx_sum,nxy_sum,nxz_sum,nyy_sum,nyz_sum,nzz_sum");
  struct Bin { std::vector<double> residual; double sum=0, sq=0; std::array<double,6> tensor{}; };
  std::array<Bin, 5> bins;
  const double g = gravity.norm();
  for (const auto &m : matches) {
    const double n = m.normal_.norm();
    int which = 4;
    if (g > 1e-6 && n > 1e-6) {
      const double cosine = std::abs(m.normal_.dot(gravity)/(g*n));
      const double height = -(m.point_w_ - sensor).dot(gravity)/g;
      if (cosine >= 0.9396926207859084 && height < -0.20) which=1;
      else if (cosine >= 0.9396926207859084 && height > 0.20) which=2;
      else if (cosine <= 0.3420201433256687) which=3;
    }
    for (const auto idx : {0,which}) {
      auto &bin = bins[idx];
      const double r = std::abs(m.dis_to_plane_);
      bin.residual.push_back(r); bin.sum += r; bin.sq += r*r;
      if (n > 1e-6) {
        std::size_t k = 0;
        for (int i=0; i<3; ++i) for (int j=i; j<3; ++j)
          bin.tensor[k++] += m.normal_(i)*m.normal_(j)/(n*n);
      }
    }
  }
  const char *names[] = {"all","horizontal_below","horizontal_above","vertical","other"};
  for (std::size_t i=0; i<bins.size(); ++i) {
    auto &bin=bins[i]; auto &r=bin.residual;
    std::sort(r.begin(), r.end());
    auto q=[&](double f) { return r.empty() ? 0.0 : r[std::size_t(std::ceil(f*r.size()))-1]; };
    file << windowEnd() << ',' << iter << ',' << names[i] << ',' << r.size() << ',' << bin.sum << ',' << bin.sq
         << ',' << q(0.5) << ',' << q(0.95) << ',' << q(1.0);
    for (const auto x : bin.tensor) file << ',' << x;
    file << '\n';
  }
  file.flush();
}
}
