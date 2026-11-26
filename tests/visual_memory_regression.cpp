#include "visual_voxel_key.h"
#include "vio.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr std::uint64_t frame_bytes = 1280 * 1024;
constexpr std::uint64_t previous_boundary = 512ULL * 1024 * 1024;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message); // Release builds define NDEBUG.
}

struct CountingAllocator : cv::MatAllocator {
  cv::MatAllocator* base = cv::Mat::getDefaultAllocator();
  mutable std::uint64_t allocations = 0, frees = 0, bytes = 0, freed_bytes = 0;
  cv::UMatData* allocate(int dims, const int* sizes, int type, void* data, std::size_t* step,
                         cv::AccessFlag flags, cv::UMatUsageFlags usage) const override {
    auto* u = base->allocate(dims, sizes, type, data, step, flags, usage);
    u->currAllocator = this;
    ++allocations;
    bytes += u->size;
    return u;
  }
  bool allocate(cv::UMatData* u, cv::AccessFlag a, cv::UMatUsageFlags b) const override {
    return base->allocate(u, a, b);
  }
  void deallocate(cv::UMatData* u) const override {
    ++frees;
    bytes -= u->size;
    freed_bytes += u->size;
    u->currAllocator = base;
    base->deallocate(u);
  }
};

// The regression must never truncate the diagnostics of a mapping session,
// even when the tester invokes it inside that session's directory.
struct TestDirectory {
  std::filesystem::path previous = std::filesystem::current_path(), path;
  TestDirectory() {
    char pattern[] = "/tmp/fast-livo2-visual-XXXXXX";
    char* directory = ::mkdtemp(pattern);
    require(directory != nullptr, "could not create temporary test directory");
    path = directory;
    std::filesystem::create_directory(path / "Log");
    std::filesystem::current_path(path);
  }
  ~TestDirectory() {
    std::error_code ec;
    std::filesystem::current_path(previous,ec);
    std::filesystem::remove_all(path,ec);
  }
};

Feature* observe(VisualPoint& point, const cv::Mat& image, std::uint64_t used) {
  auto* f = new Feature(&point, new float[64]{}, V2D(10,10), V3D(0,0,1), SE3<double>(), 0);
  ++memory_diagnostics::patches_allocated;
  f->img_ = image;
  memory_diagnostics::retain(f->img_);
  f->last_used_frame_ = used;
  f->id_ = static_cast<int>(f->lru_id_);
  point.addFrameRef(f);
  if (!point.ref_patch) {
    point.ref_patch = f;
    point.has_ref_patch_ = true;
  }
  return f;
}

void requireEmpty(const CountingAllocator& allocator) {
  using namespace memory_diagnostics;
  require(images.empty() && image_bytes == 0 && image_refs == 0 && external_images == 0, "retained image at teardown");
  require(points_created == points_deleted && features_created == features_deleted, "retained visual object at teardown");
  require(allocator.bytes == 0 && allocator.allocations == allocator.frees, "OpenCV backing allocation at teardown");
}

void samplingChecks() {
  VIOManager manager;
  manager.patch_size = 4;
  manager.patch_size_total = 16;
  manager.patch_pyrimid_level = 3;
  cv::Mat storage(38, 41, CV_8UC1, cv::Scalar(0));
  cv::Mat roi = storage(cv::Rect(3, 2, 32, 32));
  for (int y = 0; y < roi.rows; ++y)
    for (int x = 0; x < roi.cols; ++x) roi.at<unsigned char>(y, x) = x + 2*y;
  float patch[16]{};
  const V2D center(16.5, 16.25);
  for (int level = 0; level < 3; ++level) {
    require(manager.getImagePatch(roi, center, patch, level), "valid strided pyramid patch");
    const int scale = 1 << level;
    for (int y = 0; y < 4; ++y) for (int x = 0; x < 4; ++x) {
      const double expected = center.x() + (x-2)*scale + 2*(center.y() + (y-2)*scale);
      require(std::abs(patch[4*y+x] - expected) < 1e-5, "pyramid sample or row stride mismatch");
    }
  }
  require(!manager.getImagePatch(roi, V2D(4,4), patch, 2), "insufficient pyramid footprint");
  cv::Mat stripe(32, 32, CV_8UC1, cv::Scalar(0));
  stripe.col(17).setTo(240);
  require(manager.getImagePatch(stripe, V2D(16.0+1e-7, 16), patch, 0) && patch[10] == 0,
          "subpixel sampling must preserve the camera's float coordinate quantization");
  float warped[32]{};
  require(manager.warpAffine(Matrix2d::Identity(), roi, V2D(2,8), 0, 0, 1, 2, warped),
          "finite partially covered reference warp");
  for (int y = 0; y < 4; ++y) for (int x = 0; x < 4; ++x) {
    const int col = 2 + 2*(x-2), row = 8 + 2*(y-2);
    const float expected = col < 0 ? 0 : col + 2*row;
    require(warped[16+4*y+x] == expected, "reference warp must retain valid pixels and pad uncovered pixels");
  }
  const float a[]{1, 3, 5, 7}, b[]{11, 13, 15, 17}, c[]{4, 4, 4, 4};
  require(std::abs(manager.calculateNCC(a, b, 4) - 1) < 1e-12 &&
          manager.calculateNCC(a, c, 4) == 0., "NCC intensity offset and constant patch");
  cv::Mat bgr;
  cv::cvtColor(storage, bgr, cv::COLOR_GRAY2BGR);
  cv::Mat color_roi = bgr(cv::Rect(3, 2, 32, 32));
  require((manager.getInterpolatedPixel(color_roi, center) - V3F::Constant(49.)).norm() < 1e-5 &&
          (manager.getInterpolatedPixel(roi, center) - V3F::Constant(49.)).norm() < 1e-5, "strided RGB and gray");
  std::cout << "VISUAL_SAMPLING_PASS\n";
}

void referenceChecks() {
  VIOManager manager;
  manager.visual_submap = new SubSparseMap;
  manager.total_points = 1;
  manager.patch_size_total = 64;
  manager.update_flag = {1};
  VisualPoint point(V3D(0, 0, 2));
  Feature* view = nullptr;
  point.findMinScoreFeature(V3D::Zero(), view);
  require(!view && !point.getCloseViewObs(V3D::Zero(), view, V2D::Zero()), "empty observation selection");
  point.normal_ = V3D(0, 0, 1);
  point.is_normal_initialized_ = true;
  manager.visual_submap->voxel_points.push_back(&point);
  cv::Mat image(8, 8, CV_8UC1, cv::Scalar(0));
  for (int i = 0; i < 6; ++i) {
    Feature* f = observe(point, image, 1);
    for (int k = 0; k < 64; ++k) f->patch_[k] = i == 0 ? 0 : float(i + k);
  }
  manager.updateReferencePatch({});
  Feature* selected = point.ref_patch;
  require(selected && std::abs(selected->score_ - 1.8) < 1e-5, "per-pair reference NCC");
  manager.updateReferencePatch({});
  require(point.ref_patch == selected && std::abs(selected->score_ - 1.8) < 1e-5,
          "reused mean changed reference score");
  std::cout << "VISUAL_REFERENCE_PASS\n";
}

void ownershipChecks(CountingAllocator& allocator) {
  using namespace memory_diagnostics;
  const V3D center = V3D::Zero();
  visual_lru::Options options;
  VIOManager manager;
  auto* point = new VisualPoint(V3D(90,0,0));
  manager.insertPointIntoVoxelMap(point);
  for (int i=0; i<15; ++i) observe(*point, cv::Mat(8,8,CV_8UC1,cv::Scalar(i)), 1);
  std::unordered_set<VisualPoint*> borrowed{point};
  const auto live = image_bytes;
  const auto reserve = point->ref_patch;
  const auto newest = point->obs_.front();
  auto* recently_selected = *std::next(point->obs_.begin(), 4);
  recently_selected->last_used_frame_ = 200;
  Resources low;
  low.total = 8192;
  low.available = 64;
  low.memory_valid = true;
  require(low.shortfall() == 960, "host shortfall units");
  auto result = visual_lru::maintain(manager.feat_map, center, 200, borrowed, options, low.shortfall(), true);
  require(result.bytes_after == live && result.borrowed_points == 1 && result.protected_images == 15, "live borrower");
  require(reserve->img_.at<unsigned char>(0,0) == 0 && newest->img_.at<unsigned char>(0,0) == 14, "pinned pixels");

  point->last_used_frame_ = 200;
  const auto frees0 = allocator.frees;
  result = visual_lru::maintain(manager.feat_map, center, 200, {}, options, 0, true);
  require(result.observations_pruned == 4 && point->obs_.size() == 11, "active point stale observations");
  result = visual_lru::maintain(manager.feat_map, center, 200, {}, options, low.shortfall(), true);
  require(result.observations_pruned == 5 && point->obs_.size() == 6 && result.freed_bytes == 5*64, "early pressure pruning");
  require(result.protected_images == 6 && allocator.frees == frees0 + 9, "allocator accounting");
  require(point->ref_patch == reserve && point->obs_.front() == newest &&
          std::find(point->obs_.begin(),point->obs_.end(),recently_selected) != point->obs_.end(), "needed history");

  // A single physical image shared between features/ROIs. One outside Mat
  // survives the owning point. Last Feature detachment cannot claim its bytes.
  cv::Mat external(8,8,CV_8UC1,cv::Scalar(9));
  observe(*point, external, 200);
  observe(*point, external(cv::Rect(1,1,4,4)), 200);
  const auto external0 = image_detached_external_ref;
  const auto freed0 = allocator.freed_bytes;
  result = visual_lru::maintain(manager.feat_map, center, 320, {}, options, 0, true);
  require(result.far_evicted == 1 && result.freed_bytes == 6*64 && allocator.freed_bytes-freed0 == 6*64, "physical release");
  require(image_detached_external_ref == external0+1 && image_bytes == 0 &&
          external.at<unsigned char>(1,1) == 9 && allocator.bytes == 64, "shared external owner");
  external.release();

  auto inactive = [&](double x, std::uint64_t used) {
    auto* p = new VisualPoint(V3D(x,0,0));
    manager.insertPointIntoVoxelMap(p);
    p->last_used_frame_ = used;
    observe(*p,cv::Mat(8,8,CV_8UC1,cv::Scalar(4)),used);
    return p;
  };
  auto* near_old = inactive(1,50);
  auto* far_old = inactive(91,60);
  auto* far_newer = inactive(92,61);
  auto* recent = inactive(93,99);
  options.idle_frames = options.far_idle_frames = 300;
  result = visual_lru::maintain(manager.feat_map,center,100,{},options,64,true);
  require(result.pressure_points_evicted == 1 && result.freed_bytes == 64 && result.bytes_after == 192, "pressure relief target");
  // Stable map keys allow verification without touching the deleted point.
  require(!manager.feat_map.count(visualVoxelKey(V3D(91,0,0))) &&
          near_old->ref_patch && far_newer->ref_patch && recent->ref_patch, "spatial then effective-use LRU");
  (void)far_old;
}

std::vector<pointWithVar> scene() {
  std::vector<pointWithVar> out;
  for (int row=0; row<3; ++row) for (int col=0; col<4; ++col) {
    pointWithVar p;
    const double z = 5 + 0.2*row;
    p.point_w = V3D((72+50*col-160)*z/250, (70+44*row-120)*z/250, z);
    p.normal = V3D(0,0,1);
    out.push_back(p);
  }
  return out;
}

void setupCamera(VIOManager& manager, vk::PinholeCamera& camera, StatesGroup& state, StatesGroup& propagated) {
  manager.cam = &camera;
  manager.state = &state;
  manager.state_propagat = &propagated;
  manager.Rli.setIdentity(); manager.Pli.setZero();
  manager.Rcl.setIdentity(); manager.Pcl.setZero();
  manager.grid_size = 20;
  manager.patch_size = 8;
  manager.patch_pyrimid_level = 2;
  manager.max_iterations = 2;
  manager.img_point_cov = 100.;
  manager.outlier_threshold = 1000.;
  manager.normal_en = true;
  manager.inverse_composition_en = manager.raycast_en = manager.plot_flag = manager.exposure_estimate_en = false;
  manager.initializeVIO();
}

void inverseChecks() {
  vk::PinholeCamera camera(320,240,1.0,250,250,160,120,0,0,0,0,0);
  StatesGroup state, propagated;
  VIOManager manager;
  setupCamera(manager, camera, state, propagated);
  manager.inverse_composition_en = true;
  manager.normal_en = false; // The reference is selected from the closest usable observation.
  auto landmarks = scene();
  const std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> planes;
  cv::Mat empty;
  require(!manager.processFrame(empty, landmarks, planes, 0.) && !manager.new_frame_, "empty frame rejection");
  cv::Mat parent(246, 337, CV_8UC1);
  cv::Mat image = parent(cv::Rect(3, 1, 320, 240));
  for (int y = 0; y < image.rows; ++y) for (int x = 0; x < image.cols; ++x)
    image.at<unsigned char>(y, x) = static_cast<unsigned char>(125+35*std::sin(.31*x)+28*std::cos(.27*y)+18*std::sin(.13*x+.19*y));
  require(manager.processFrame(image, landmarks, planes, .1) && !manager.feat_map.empty(), "inverse seed");
  require(manager.processFrame(image, landmarks, planes, .2) && manager.visual_update_steps > 0 &&
          state.cov.allFinite(), "inverse strided frame update");
  const auto covariance = state.cov;
  const auto steps = manager.visual_update_steps;
  manager.total_points = 0;
  manager.computeJacobianAndUpdateEKF(image);
  require(manager.G.isZero() && manager.H_T_H.isZero() && manager.visual_update_steps == steps &&
          state.cov == covariance, "unused frame changed covariance");
  std::cout << "VISUAL_INVERSE_PASS\n";
}

void photometricGateCheck() {
  vk::PinholeCamera camera(320,240,1.0,250,250,160,120,0,0,0,0,0);
  StatesGroup state, propagated;
  VIOManager manager;
  setupCamera(manager, camera, state, propagated);
  manager.outlier_threshold = 1.;
  cv::Mat image(240, 320, CV_8UC1, cv::Scalar(32));
  for (int x = 1; x < image.cols; x += 2) image.col(x).setTo(224);
  manager.new_frame_.reset(new Frame(&camera, image));
  state.pos_end.z() = 2.;
  manager.updateFrameState(state);
  manager.resetGrid();
  const V2D center(160, 120);
  auto* point = new VisualPoint(V3D(0, 0, 4));
  manager.insertPointIntoVoxelMap(point);
  point->normal_ = point->previous_normal_ = V3D::UnitZ();
  point->is_normal_initialized_ = true;
  Feature* reference = observe(*point, image, 0);
  reference->px_ = center;
  require(manager.getImagePatch(image, center, reference->patch_, 0), "reference patch extraction");
  // Moving from depth 4 to depth 2 gives a 2x reference warp (search level 1).
  // Fine stripes distinguish the level-zero gate from the optimizer's coarser stride.
  std::vector<pointWithVar> scan(1);
  scan[0].point_w = point->pos_;
  manager.retrieveFromVisualSparseMap(image, scan, {});
  require(manager.total_points == 1 && manager.visual_submap->search_levels.at(0) == 1 &&
          manager.visual_submap->propa_errors.at(0) < 1e-5, "level-zero photometric gate for a magnified patch");
  std::cout << "VISUAL_GATE_PASS\n";
}

void boundaryCase(std::uint64_t target, bool reclaimable, CountingAllocator& allocator) {
  using namespace memory_diagnostics;
  vk::PinholeCamera camera(320,240,1.0,250,250,160,120,0,0,0,0,0);
  StatesGroup state, propagated;
  VIOManager manager;
  setupCamera(manager, camera, state, propagated);
  auto landmarks = scene();
  const std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> planes;

  // The visible camera view uses an ROI of the first complete allocation. It
  // exercises precisely the original u->size accounting even on the small scene.
  std::vector<cv::Mat> buffers;
  for (auto left=target; left>0;) {
    const auto size = std::min(frame_bytes,left);
    buffers.emplace_back(1, static_cast<int>(size), CV_8UC1, cv::Scalar(17));
    left -= size;
  }
  cv::Mat camera_image = buffers.front().colRange(0,320*240).reshape(1,240);
  for (int y=0; y<240; ++y) for (int x=0; x<320; ++x)
    camera_image.at<unsigned char>(y,x) = static_cast<unsigned char>(
        125+35*std::sin(.31*x)+28*std::cos(.27*y)+18*std::sin(.13*x+.19*y));
  manager.processFrame(camera_image,landmarks,planes,0.0);
  require(!manager.feat_map.empty() && image_bytes == frame_bytes, "production frame failed to seed real image features");
  // Restore the archived failure frame number. Off-screen points model the
  // exact 570 point/2324 Feature totals. The observed image sizes are real.
  manager.frame_count = 3546;
  for (auto& cell : manager.feat_map) for (auto* p : cell.second->voxel_points) {
    p->created_frame_ = p->last_used_frame_ = 3545;
    for (auto* f : p->obs_) f->last_used_frame_ = 3545;
  }
  const auto original_points = points_created - points_deleted;
  const auto original_features = features_created - features_deleted;
  const auto added_points = 570-original_points;
  const auto added_refs = 2324-original_features;
  std::uint64_t expected_freed = 0;
  for (std::size_t i=1; i<buffers.size(); ++i)
    if (reclaimable && i%11 == 0) expected_freed += buffers[i].u->size;
  for (std::size_t i=0; i<added_points; ++i) {
    const auto image_index = 1 + i%(buffers.size()-1);
    auto* p = new VisualPoint(V3D(100+2*i,0,3));
    manager.insertPointIntoVoxelMap(p);
    const auto used = reclaimable && image_index%11 == 0 ? 1 : 3545;
    p->created_frame_ = p->last_used_frame_ = used;
    const auto count = added_refs/added_points + (i<added_refs%added_points);
    for (std::size_t j=0; j<count; ++j) observe(*p,buffers[image_index],used);
  }
  const auto n_images = buffers.size();
  buffers.clear();
  require(image_bytes == target && images.size() == n_images && image_refs == 2324, "wrong old decision input");
  const auto frees0 = allocator.freed_bytes;
  const auto covariance0 = state.cov;
  const auto initial = manager.visual_update_steps;
  manager.last_memory_sample_s = -1.0; // request a real sampled production frame
  manager.processFrame(camera_image,landmarks,planes,0.1);
  const auto boundary = manager.last_maintenance;
  require(boundary.bytes_before == target && boundary.bytes_after == target-expected_freed, "decision boundary bytes");
  require(boundary.freed_bytes == expected_freed && allocator.freed_bytes-frees0 >= expected_freed, "real backing deallocation");
  require(boundary.protected_bytes == boundary.bytes_after, "retained active set not accounted");
  require(manager.visual_update_frames == 1 && manager.visual_update_steps > initial &&
          (state.cov-covariance0).norm() > 0, "boundary frame did not update the visual EKF");
  for (int i=1; i<=2; ++i) {
    propagated = state;
    manager.processFrame(camera_image,landmarks,planes,0.1+i*0.1);
    require(manager.frame_count == 3547+i && manager.visual_update_frames == static_cast<std::uint64_t>(1+i) &&
            manager.total_points > 0 && state.cov.allFinite() && state.pos_end.allFinite(), "subsequent visual state missing");
  }
  std::cout << "MEMORY_CASE {\"input_bytes\":" << target << ",\"images\":" << n_images
            << ",\"input_points\":570,\"input_feature_refs\":2324,\"reclaimable\":" << (reclaimable?"true":"false")
            << ",\"retained_bytes\":" << boundary.bytes_after << ",\"freed_bytes\":" << boundary.freed_bytes
            << ",\"protected_bytes\":" << boundary.protected_bytes << ",\"used_points\":" << manager.total_points
            << ",\"visual_update_frames\":" << manager.visual_update_frames << ",\"visual_update_steps\":" << manager.visual_update_steps
            << ",\"final_frame\":" << manager.frame_count << "}\n";
}
} // namespace

int main() {
  cv::setNumThreads(0); // Serial OpenCV without creating a TBB worker arena.
  CountingAllocator allocator;
  cv::Mat::setDefaultAllocator(&allocator);
  try {
    TestDirectory test_directory;
    samplingChecks();
    referenceChecks();
    inverseChecks();
    photometricGateCheck();
    requireEmpty(allocator);
    ownershipChecks(allocator);
    requireEmpty(allocator);
    for (const auto bytes : {409*frame_bytes,previous_boundary,410*frame_bytes}) {
      for (const bool eligible : {false,true}) {
        boundaryCase(bytes,eligible,allocator);
        requireEmpty(allocator);
      }
    }
    cv::Mat::setDefaultAllocator(allocator.base);
    std::cout << "MEMORY_PASS allocations=" << allocator.allocations << " deallocations=" << allocator.frees << "\n";
    return 0;
  } catch (const std::exception& ex) {
    cv::Mat::setDefaultAllocator(allocator.base);
    std::cerr << "MEMORY_FAIL " << ex.what() << '\n';
    return 1;
  }
}
