#include "IMU_Processing.h"
#include "vio.h"
#include "visual_voxel_key.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace {
void require(bool ok, const char *reason)
{
  if (!ok) throw std::runtime_error(reason);
}

VoxelMapConfig config(double size)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("lio.voxel_size", size),
                               rclcpp::Parameter("local_map.half_map_size", 1),
                               rclcpp::Parameter("local_map.sliding_thresh", 0.0)});
  auto node = std::make_shared<rclcpp::Node>("geometry_regression", options);
  VoxelMapConfig cfg{};
  loadVoxelConfig(node, cfg);
  return cfg;
}

struct Scene
{
  VoxelMapConfig cfg = config(0.75);
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> empty;
  VoxelMapManager map{cfg, empty};
  const V3D boundary{-0.75, 0.375, 1.875};

  Scene()
  {
    map.extR_.setIdentity();
    map.extT_.setZero();
    // A planar, non-collinear patch, including the negative cell boundary.
    for (float x : {-0.75f, -0.58f, -0.29f, -0.025f})
      for (float y : {0.22f, 0.375f, 0.52f})
      {
        PointType p{};
        p.x = x;
        p.y = y;
        p.z = boundary.z();
        map.feats_down_world_->push_back(p);
        map.feats_down_body_->push_back(p);
      }
    map.feats_down_size_ = map.feats_down_body_->size();
    map.BuildVoxelMap();
  }

  ~Scene()
  {
    for (const auto &cell : map.voxel_map_) delete cell.second;
  }
};

pointWithVar measurement(const V3D &world)
{
  pointWithVar p;
  p.point_w = p.point_b = world;
  p.var = p.body_var = M3D::Identity() * 0.01;
  return p;
}

void mapChecks(Scene &s)
{
  auto &map = s.map;
  const VOXEL_LOCATION cell(-1, 0, 2);
  require(map.voxel_map_.size() == 1 && map.voxel_map_.count(cell) == 1, "BuildVoxelMap negative boundary");
  const auto *root = map.voxel_map_.at(cell);
  require(root->plane_ptr_->is_plane_ && root->voxel_center_[0] == -0.375 && root->quater_length_ == 0.1875,
          "root dimensions or plane fit");
  require(lidarRootKey(V3D(-0.75, 0, 0.75), 0.75) == VOXEL_LOCATION(-1, 0, 1), "half-open endpoints");
  require(lidarRootKey(V3D(std::nextafter(-0.75, -1.0), std::nextafter(-0.75, 0.0), -0.0), 0.75) ==
          VOXEL_LOCATION(-2, -1, 0), "negative boundary neighbours");
  require(visualVoxelKey(s.boundary) == VOXEL_LOCATION(-2, 0, 3), "independent visual grid");

  map.UpdateVoxelMap({measurement(s.boundary), measurement(V3D(0.03, 0.375, 1.875))});
  require(map.voxel_map_.size() == 2 && map.voxel_map_.count(VOXEL_LOCATION(0, 0, 2)) == 1,
          "UpdateVoxelMap roots");
  // Hits in the current root, hits requiring a metre-based neighbour lookup, and missing roots.
  std::vector<pointWithVar> queries;
  for (int i = 0; i < 257; ++i)
  {
    const double x = i % 3 == 0 ? -0.3 : i % 3 == 1 ? 0.03 : 7.0;
    auto p = measurement(V3D(x, 0.375, 1.875));
    p.point_b.y() = i;
    queries.push_back(p);
  }
  std::vector<PointToPlane> results;
  for (int repeat = 0; repeat < 12; ++repeat)
  {
    map.BuildResidualListOMP(queries, results);
    size_t used = 0;
    for (size_t i = 0; i < queries.size(); ++i)
    {
      if (i % 3 == 2) continue;
      require(used < results.size(), "missing residual");
      const auto &r = results[used++];
      require(r.point_b_.y() == double(i) && r.is_valid_ && std::isfinite(r.eigen_value_) &&
              std::abs(r.dis_to_plane_) < 1e-5 && r.normal_.allFinite(), "OMP ordering or plane data");
    }
    require(results.size() == used, "unexpected residual on missing root");
  }
  queries.clear();
  map.BuildResidualListOMP(queries, results);
  require(results.empty(), "empty batch retained previous residuals");

  for (int run = 0; run < 2; ++run)
  {
    StatesGroup predicted = map.state_;
    const double exposure_variance = predicted.cov(6, 6);
    map.StateEstimation(predicted);
    require(map.effct_feat_num_ > 0 && map.state_.cov.allFinite() && map.state_.pos_end.allFinite() &&
            map.state_.cov(6, 6) == exposure_variance, "LIO gain or covariance outside measured block");
  }
}

void visualPlaneChecks(Scene &s)
{
  cv::Mat image(96, 96, CV_8UC1, cv::Scalar(31));
  vk::PinholeCamera camera(96, 96, 1.0, 50, 50, 48, 48, 0, 0, 0, 0, 0);
  VIOManager vio;
  vio.cam = &camera;
  vio.Rli.setIdentity(); vio.Rcl.setIdentity(); vio.Pli.setZero(); vio.Pcl.setZero();
  vio.lidar_root_size = s.map.config_setting_.max_voxel_size_;
  vio.grid_size = 16;
  vio.patch_size = 4;
  vio.patch_pyrimid_level = 1;
  vio.normal_en = vio.raycast_en = true;
  vio.initializeVIO();
  vio.resetGrid();
  vio.new_frame_.reset(new Frame(&camera, image));
  vio.new_frame_->T_f_w_ = SE3<double>();
  vio.insertPointIntoVoxelMap(new VisualPoint(V3D(10, 10, 10)));
  for (auto &ray : vio.rays_with_sample_points) ray.clear();
  vio.rays_with_sample_points.at(vio.grid_n_width + 1).push_back(s.boundary);
  std::vector<pointWithVar> empty_scan;
  vio.retrieveFromVisualSparseMap(image, empty_scan, s.map.voxel_map_);
  require(vio.visual_submap->add_from_voxel_map.size() == 1, "VIO ray did not retrieve the configured LiDAR root");
  const auto &plane = vio.visual_submap->add_from_voxel_map.front();
  require(std::abs(plane.normal.z()) > 0.999, "VIO ray returned wrong plane");

  auto *p = new VisualPoint(s.boundary);
  p->is_normal_initialized_ = true;
  p->previous_normal_ = p->normal_ = V3D::UnitX();
  p->covariance_ = M3D::Identity() * 0.01;
  vio.insertPointIntoVoxelMap(p);
  for (int i = 0; i < 6; ++i)
  {
    auto *feature = new Feature(p, new float[vio.patch_size_total], V2D(24, 48), s.boundary.normalized(), SE3<double>(), 0);
    std::iota(feature->patch_, feature->patch_ + vio.patch_size_total, 0.0f);
    feature->id_ = i;
    feature->img_ = image;
    memory_diagnostics::retain(feature->img_);
    ++memory_diagnostics::patches_allocated;
    p->addFrameRef(feature);
  }
  vio.visual_submap->voxel_points.push_back(p);
  vio.total_points = 1;
  vio.update_flag.at(0) = 1;
  vio.updateReferencePatch(s.map.voxel_map_);
  require(std::abs(p->normal_.z()) > 0.999 && p->has_ref_patch_, "VIO normal update missed the configured LiDAR root");
}

void slidingCheck(Scene &s)
{
  // Center -1, half_map_size 1: keep cells -2, -1 and 0; drop -3 and +1.
  for (double x : {-2.25, -1.5, 0.75}) s.map.UpdateVoxelMap({measurement(V3D(x, 0.375, 1.875))});
  s.map.position_last_ = s.boundary;
  s.map.mapSliding();
  require(s.map.voxel_map_.size() == 3, "sliding window has wrong width");
  for (int x = -2; x <= 0; ++x)
    require(s.map.voxel_map_.count(VOXEL_LOCATION(x, 0, 2)) == 1, "sliding boundary cell");
}

sensor_msgs::msg::Imu::SharedPtr imuAt(int second, uint32_t nanoseconds)
{
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->header.stamp.sec = second;
  imu->header.stamp.nanosec = nanoseconds;
  imu->linear_acceleration.z = G_m_s2;
  return imu;
}

void deskewCheck(const std::vector<float> &offsets_ms)
{
  ImuProcess imu;
  imu.set_imu_init_frame_num(1);
  StatesGroup state;
  state.vel_end = V3D::UnitX();
  LidarMeasureGroup group;
  group.lio_vio_flg = LIO;
  group.measures.emplace_back();
  auto &slice = group.measures.back();
  slice.lio_time = 100.0;
  slice.imu.push_back(imuAt(100, 0));
  auto output = std::make_shared<PointCloudXYZI>();
  imu.Process2(group, state, output);
  require(!imu.imu_need_init && group.last_lio_update_time == 100.0, "initial state time anchor");
  slice.lio_time = 100.03;
  slice.imu.clear();
  for (uint32_t ns : {10000000u, 20000000u, 30000000u}) slice.imu.push_back(imuAt(100, ns));
  for (const float t : offsets_ms)
  {
    PointType p{};
    p.x = 2;
    p.z = 1;
    p.curvature = t;
    group.pcl_proc_cur->push_back(p);
  }
  imu.Process2(group, state, output);
  require(std::abs(state.pos_end.x() - 0.03) < 1e-9 && state.cov.allFinite(), "first propagation time span");
  require(output->size() == offsets_ms.size(), "deskew changed point count");
  // At 1 m/s each point shifts by its remaining time until the common end pose.
  for (size_t i = 0; i < output->size(); ++i)
    require(std::abs(output->points[i].x - (2.0 + offsets_ms[i] * 0.001 - 0.03)) < 1e-5,
            "deskew endpoint was skipped or applied more than once");
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    require(!std::getenv("FAST_LIVO2_GRAVITY_DIR") && !std::getenv("FAST_LIVO2_SESSION_ID") &&
            !std::getenv("FAST_LIVO2_CONFIG_SHA256") && !std::getenv("FAST_LIVO2_ALGORITHM_SHA256"),
            "run the standalone geometry test outside a mapping session");
    for (double size : {0.0, -0.5, std::numeric_limits<double>::infinity()})
    {
      bool rejected = false;
      try { config(size); }
      catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "invalid root size accepted");
    }
    Scene s;
    mapChecks(s);
    visualPlaneChecks(s);
    slidingCheck(s);
    deskewCheck({});
    deskewCheck({25});
    deskewCheck({22, 25, 28});
    deskewCheck({0, 10, 15, 30});
    std::cout << "PASS geometry: configured 0.75m roots, two VIO plane consumers, stable residuals, LIO covariance, sliding and deskew\n";
  }
  catch (const std::exception &ex)
  {
    std::cerr << ex.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
