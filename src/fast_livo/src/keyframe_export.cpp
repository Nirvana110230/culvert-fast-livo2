#include "keyframe_export.h"

#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <opencv2/core.hpp>

namespace fs = std::filesystem;
namespace {
double threshold(const char* name, double fallback) {
  const char* value = std::getenv(name);
  if (!value) return fallback;
  char* end = nullptr;
  const double number = std::strtod(value, &end);
  if (*value == '\0' || *end != '\0' || !std::isfinite(number) || number <= 0)
    throw std::invalid_argument(std::string("Invalid positive threshold ") + name);
  return number;
}

std::ofstream output(const fs::path& path) {
  std::ofstream file;
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file.open(path, std::ios::out | std::ios::binary);
  file.imbue(std::locale::classic());
  return file;
}

std::string pose(const StatesGroup& state) {
  const Eigen::Quaterniond q = Eigen::Quaterniond(state.rot_end).normalized();
  std::ostringstream row;
  row.imbue(std::locale::classic());
  row << std::setprecision(17) << state.pos_end.x() << ',' << state.pos_end.y() << ','
      << state.pos_end.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
  return row.str();
}

// PCD v0.7 binary records on the supported little-endian RK3588/x86 platforms.
struct IntensityPoint { float x, y, z, intensity; };
struct RgbPoint { float x, y, z; uint32_t rgb; };
static_assert(sizeof(IntensityPoint) == 16 && sizeof(RgbPoint) == 16, "Packed PCD records");

template<class Record>
void writeCloud(const fs::path& path, const char* field, const std::vector<Record>& points) {
  const fs::path temp = path.string() + ".part";
  auto file = output(temp);
  file << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z " << field
       << "\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH " << points.size()
       << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA binary\n";
  file.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(Record));
  file.close();
  fs::rename(temp, path);
}

struct Frame {
  size_t id = 0, input_points = 0, downsampled = 0, matches = 0;
  int64_t stamp = 0, image_stamp = 0;
  double residual = 0;
  std::string pose, selection, color_status;
  std::vector<IntensityPoint> cloud;
  std::vector<RgbPoint> colors;
};

struct Job {
  std::string state_row;
  std::unique_ptr<Frame> frame;
};
}

struct KeyframeExport::Impl {
  fs::path dir;
  const double distance = threshold("FAST_LIVO_KF_DISTANCE_M", 1.0);
  const double angle_deg = threshold("FAST_LIVO_KF_ANGLE_DEG", 15.0);
  const double interval = threshold("FAST_LIVO_KF_INTERVAL_S", 5.0);
  size_t selected = 0, written = 0, states_written = 0;
  int64_t last_stamp = 0;
  V3D last_position = V3D::Zero();
  Eigen::Quaterniond last_rotation = Eigen::Quaterniond::Identity();
  std::unique_ptr<Frame> pending;
  std::ofstream index, states;
  std::deque<Job> jobs;
  std::mutex mutex;
  std::condition_variable ready;
  std::exception_ptr failure;
  bool stopping = false, closed = false;
  std::thread worker;

  Impl(const std::string& directory, const std::string& calibration) : dir(directory) {
    fs::create_directories(dir);
    // Atomic creation also prevents two writers racing for the same directory.
    if (!fs::is_empty(dir) || !fs::create_directory(dir / "clouds"))
      throw std::invalid_argument("Keyframe export requires its own empty directory");
    index = output(dir / "index.csv");
    states = output(dir / "states.csv");
    index << "id,stamp_ns,x,y,z,qx,qy,qz,qw,cloud,color_cloud,input_points,points,downsampled,plane_matches,"
             "mean_abs_plane_residual_m,color_points,image_stamp_ns,color_status,selection\n";
    states << "stamp_ns,stage,x,y,z,qx,qy,qz,qw,vx,vy,vz,gx,gy,gz,bgx,bgy,bgz,bax,bay,baz,"
              "var_rx,var_ry,var_rz,var_x,var_y,var_z\n";
    auto metadata = output(dir / "metadata.txt");
    metadata << std::setprecision(17)
             << "format=1\npose=T_odom_imu\nodom_frame=camera_init\ncloud_frame=imu\nunits=metres,radians,seconds\n"
                "stamp=rounded_ns_from_estimator_measurement_seconds\nreference=post_LIO_at_deskew_end\n"
                "color=nearest_visible_pixel_of_same_stamp_with_T_camera_imu\n"
                "states=LIO_post_update,VIO_processed,IMU_propagated_when_enabled\n"
             << "keyframe_distance_m=" << distance << "\nkeyframe_angle_deg=" << angle_deg
             << "\nkeyframe_interval_s=" << interval << '\n' << calibration;
    for (const char* key : {"FAST_LIVO2_SESSION_ID", "FAST_LIVO2_CONFIG_SHA256", "FAST_LIVO2_ALGORITHM_SHA256"}) {
      const char* value = std::getenv(key);
      metadata << key << '=' << (value ? value : "unsupplied") << '\n';
    }
    metadata.close();
    worker = std::thread([this] { consume(); });
  }

  void add(Job job) {
    std::lock_guard<std::mutex> lock(mutex);
    if (failure) std::rethrow_exception(failure);
    if (stopping) throw std::logic_error("Keyframe exporter is closing");
    if (jobs.size() >= 1024) throw std::runtime_error("Keyframe writer queue exhausted");
    jobs.push_back(std::move(job));
    ready.notify_one();
  }

  void sendPending() {
    if (pending) add({{}, std::move(pending)});
  }

  void save(const Frame& frame) {
    const auto prefix = "clouds/" + std::to_string(frame.id);
    const auto cloud_path = prefix + ".pcd";
    writeCloud(dir / cloud_path, "intensity", frame.cloud);
    std::string color_path;
    if (!frame.colors.empty()) {
      color_path = prefix + "_rgb.pcd";
      writeCloud(dir / color_path, "rgb", frame.colors);
    }
    index << std::setprecision(17) << frame.id << ',' << frame.stamp << ',' << frame.pose << ','
          << cloud_path << ',' << color_path << ',' << frame.input_points << ',' << frame.cloud.size()
          << ',' << frame.downsampled << ',' << frame.matches << ',' << frame.residual << ','
          << frame.colors.size() << ',';
    if (frame.image_stamp) index << frame.image_stamp;
    index << ',' << frame.color_status << ',' << frame.selection << '\n';
    index.flush();
    ++written;
  }

  void consume() noexcept {
    try {
      while (true) {
        Job job;
        {
          std::unique_lock<std::mutex> lock(mutex);
          ready.wait(lock, [&] { return stopping || !jobs.empty(); });
          if (jobs.empty()) break;
          job = std::move(jobs.front());
          jobs.pop_front();
        }
        if (job.frame) save(*job.frame);
        else { states << job.state_row; ++states_written; }
      }
      index.close();
      states.close();
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      failure = std::current_exception();
    }
  }

  void close(bool complete) {
    if (closed) return;
    closed = true;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (pending) jobs.push_back({{}, std::move(pending)});
      stopping = true;
    }
    ready.notify_one();
    worker.join();
    if (failure) std::rethrow_exception(failure);
    if (complete) {
      auto marker = output(dir / "complete.json.part");
      marker << "{\"frames\":" << written << ",\"state_samples\":" << states_written << "}\n";
      marker.close();
      fs::rename(dir / "complete.json.part", dir / "complete.json");
    }
  }
};

KeyframeExport::KeyframeExport(const std::string& directory, const std::string& calibration)
    : impl_(std::make_unique<Impl>(directory, calibration)) {}

KeyframeExport::~KeyframeExport() {
  try { impl_->close(std::uncaught_exceptions() == 0); }
  catch (const std::exception& ex) { std::cerr << "Keyframe export failed: " << ex.what() << '\n'; }
}

void KeyframeExport::finish() { impl_->close(true); }

void KeyframeExport::recordState(int64_t stamp, const char* stage, const StatesGroup& state) {
  std::ostringstream row;
  row.imbue(std::locale::classic());
  row << std::setprecision(17) << stamp << ',' << stage << ',' << pose(state);
  for (const auto* v : {&state.vel_end, &state.gravity, &state.bias_g, &state.bias_a})
    row << ',' << v->x() << ',' << v->y() << ',' << v->z();
  for (int i = 0; i < 6; ++i) row << ',' << state.cov(i,i);
  row << '\n';
  impl_->add({row.str(), {}});
}

void KeyframeExport::lidar(int64_t stamp, const StatesGroup& state, const PointCloudXYZI& deskewed,
                           const M3D& imu_from_lidar, const V3D& imu_from_lidar_t,
                           size_t downsampled, size_t matches, double residual, bool expect_image) {
  impl_->sendPending();
  recordState(stamp, "LIO", state);
  const Eigen::Quaterniond rotation(state.rot_end);
  std::string selection;
  if (!impl_->selected) selection = "first";
  else if (stamp <= impl_->last_stamp) throw std::runtime_error("Non-increasing LIO keyframe clock");
  else if ((state.pos_end - impl_->last_position).norm() >= impl_->distance) selection = "distance";
  else if (rotation.angularDistance(impl_->last_rotation) * 180.0 / std::acos(-1.0) >= impl_->angle_deg)
    selection = "angle";
  else if ((stamp - impl_->last_stamp) * 1e-9 >= impl_->interval) selection = "time";
  else return;

  auto frame = std::make_unique<Frame>();
  frame->stamp = stamp;
  frame->pose = pose(state);
  frame->input_points = deskewed.size();
  frame->downsampled = downsampled;
  frame->matches = matches;
  frame->residual = residual;
  frame->selection = selection;
  frame->color_status = expect_image ? "no_paired_image" : "camera_disabled";
  frame->cloud.reserve(deskewed.size());
  for (const auto& p : deskewed) {
    const V3D local = imu_from_lidar * V3D(p.x,p.y,p.z) + imu_from_lidar_t;
    if (local.allFinite()) frame->cloud.push_back({float(local.x()), float(local.y()), float(local.z()), p.intensity});
  }
  if (frame->cloud.empty()) return;
  frame->id = impl_->selected++;
  impl_->last_stamp = stamp;
  impl_->last_position = state.pos_end;
  impl_->last_rotation = rotation;
  impl_->pending = std::move(frame);
  if (!expect_image) impl_->sendPending();
}

void KeyframeExport::image(int64_t stamp, const cv::Mat& bgr, const vk::AbstractCamera& camera,
                           const M3D& camera_from_imu, const V3D& camera_from_imu_t, double blind) {
  if (!impl_->pending || impl_->pending->stamp != stamp) return;
  auto& frame = *impl_->pending;
  frame.image_stamp = stamp;
  if (bgr.type() != CV_8UC3 || bgr.cols != camera.width() || bgr.rows != camera.height()) {
    frame.color_status = "invalid_image";
    impl_->sendPending();
    return;
  }
  // Keep the nearest measured surface at each sampled pixel, in the input IMU frame.
  struct Hit { double z; size_t index; int x, y; };
  std::unordered_map<size_t, Hit> visible;
  visible.reserve(frame.cloud.size());
  for (size_t i = 0; i < frame.cloud.size(); ++i) {
    const auto& p = frame.cloud[i];
    const V3D cam = camera_from_imu * V3D(p.x,p.y,p.z) + camera_from_imu_t;
    if (!cam.allFinite() || cam.z() <= 0 || cam.squaredNorm() <= blind * blind) continue;
    const V2D uv = camera.world2cam(cam);
    if (!uv.allFinite() || uv.x() < 0 || uv.y() < 0 || uv.x() >= bgr.cols-0.5 || uv.y() >= bgr.rows-0.5) continue;
    const int x = int(std::lround(uv.x())), y = int(std::lround(uv.y()));
    const size_t key = size_t(y) * size_t(bgr.cols) + size_t(x);
    auto hit = visible.emplace(key, Hit{cam.z(), i, x, y});
    if (!hit.second && cam.z() < hit.first->second.z) hit.first->second = Hit{cam.z(), i, x, y};
  }
  frame.colors.reserve(visible.size());
  for (const auto& entry : visible) {
    const auto& hit = entry.second;
    const auto& p = frame.cloud[hit.index];
    const auto& pixel = bgr.at<cv::Vec3b>(hit.y, hit.x);
    const uint32_t rgb = (uint32_t(pixel[2]) << 16) | (uint32_t(pixel[1]) << 8) | pixel[0];
    frame.colors.push_back({p.x, p.y, p.z, rgb});
  }
  frame.color_status = frame.colors.empty() ? "no_visible_points" : "paired";
  impl_->sendPending();
}
