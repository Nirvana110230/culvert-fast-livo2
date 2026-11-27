#include "keyframe_export.h"
#include <vikit/pinhole_camera.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;
namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
std::vector<std::string> split(const std::string& row) {
  std::vector<std::string> fields;
  std::istringstream in(row);
  std::string field;
  while (std::getline(in, field, ',')) fields.push_back(field);
  return fields;
}
template<class T> T firstPoint(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::string line;
  while (std::getline(file, line) && line != "DATA binary") {}
  T point{};
  require(bool(file.read(reinterpret_cast<char*>(&point), sizeof(point))), "Missing binary PCD record");
  return point;
}
}

int main() {
  char pattern[] = "/tmp/livo-keyframe-regression-XXXXXX";
  const char* made = mkdtemp(pattern);
  require(made, "Temporary directory");
  const fs::path dir(made);
  StatesGroup state;
  state.pos_end = V3D(10, 20, 30);
  state.gravity = V3D(0, 0, -9.81);
  const M3D r_il = Eigen::AngleAxisd(std::acos(-1.0)/2, V3D::UnitZ()).toRotationMatrix();
  const V3D t_il(0.2, 0.5, 0), t_ci(-1.5, 0.2, 0);
  PointCloudXYZI cloud;
  for (float z : {2.f, 4.f, -1.f}) {
    PointType p{};
    p.x = 1; p.y = 0; p.z = z; p.intensity = 9;
    cloud.push_back(p);
  }
  vk::PinholeCamera camera(20, 20, 1, 10, 10, 10, 10, 0, 0, 0, 0, 0);
  cv::Mat image(20, 20, CV_8UC3, cv::Scalar(0,0,0));
  image.at<cv::Vec3b>(10,10) = cv::Vec3b(3,2,1);
  {
    KeyframeExport writer(dir.string(), "fixture=nonidentity_extrinsics\n");
    auto lidar = [&](int64_t stamp, bool take_image) {
      writer.lidar(stamp, state, cloud, r_il, t_il, 2, 2, 0.01, take_image);
    };
    lidar(1000000000LL, true);
    writer.image(1000000001LL, image, camera, r_il.transpose(), t_ci, 0);
    writer.image(1000000000LL, image, camera, r_il.transpose(), t_ci, 0);
    lidar(2000000000LL, true);  // subthreshold
    state.pos_end.x() += 1.1;
    lidar(3000000000LL, true);  // unpaired image must not spill into another frame
    state.rot_end = Eigen::AngleAxisd(0.3, V3D::UnitZ()).toRotationMatrix();
    lidar(4000000000LL, false);
    lidar(9000000000LL, true);  // final pending cloud must survive drain
    writer.finish();
    writer.finish();
  }
  std::ifstream index(dir/"index.csv");
  std::string line;
  std::getline(index,line);
  for (size_t id = 0; id < 4; ++id) {
    require(bool(std::getline(index,line)), "Missing keyframe");
    const auto row = split(line);
    require(row.size() == 20 && std::stoul(row[0]) == id, "Keyframe row schema");
    require(row[19] == std::vector<std::string>{"first","distance","angle","time"}.at(id), "Keyframe selection");
    require(row[18] == std::vector<std::string>{"paired","no_paired_image","camera_disabled","no_paired_image"}.at(id), "Pair status");
    if (id) continue;
    require(row[1] == "1000000000" && row[17] == row[1] && std::stod(row[2]) == 10, "Pose clock");
    require(row[12] == "3" && row[16] == "1", "Visibility and occlusion");
    struct Intensity { float x,y,z,i; };
    struct Color { float x,y,z; uint32_t rgb; };
    auto p = firstPoint<Intensity>(dir/row[9]);
    auto c = firstPoint<Color>(dir/row[10]);
    require(std::abs(p.x-0.2f)<1e-5 && p.y==1.5f && p.z==2 && p.i==9, "LiDAR to IMU transform");
    require(c.x==p.x && c.y==p.y && c.z==p.z && c.rgb==0x010203, "Same-frame camera transform and BGR order");
  }
  require(!std::getline(index,line), "Unexpected keyframe");
  std::ifstream states(dir/"states.csv");
  size_t state_lines=0;
  while (std::getline(states,line)) ++state_lines;
  require(state_lines==6 && fs::exists(dir/"complete.json"), "Full state rows and clean drain");
  bool refused=false;
  try { KeyframeExport duplicate(dir.string(),""); }
  catch (const std::invalid_argument&) { refused=true; }
  require(refused, "Run directory reuse protection");
  fs::remove_all(dir);
}
