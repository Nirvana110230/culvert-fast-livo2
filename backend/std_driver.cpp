// Isolated adapter to the pinned GPLv2 upstream descriptor library.
#include "descriptor.h"
#include <chrono>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::ifstream inputs(argv[1]);
  std::ofstream results(argv[2]);
  if (!inputs || !results) return 3;
  ConfigSetting cfg{};
  #include "params.inc"
  STDescManager manager(cfg);
  results << "group,points,corners,descriptors,target,score,matched_triangles,elapsed_s,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22\n";
  int group = 0;
  std::string path;
  while (std::getline(inputs, path)) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::ifstream file(path, std::ios::binary);
    if (!file) return 4;
    float xyz[3];
    while (file.read(reinterpret_cast<char*>(xyz), sizeof(xyz))) {
      pcl::PointXYZI p;
      p.x = xyz[0]; p.y = xyz[1]; p.z = xyz[2]; p.intensity = 0;
      cloud->push_back(p);
    }
    auto start = std::chrono::steady_clock::now();
    std::vector<STDesc> desc;
    manager.GenerateSTDescs(cloud,desc);
    std::pair<int,double> hit(-1,0.);
    std::pair<Eigen::Vector3d,Eigen::Matrix3d> T{Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity()};
    std::vector<std::pair<STDesc,STDesc>> matches;
    if (group > cfg.skip_near_num_ && !desc.empty()) manager.SearchLoop(desc,hit,T,matches);
    manager.AddSTDescs(desc);
    results << std::setprecision(12) << group << ',' << cloud->size() << ',' << manager.corner_cloud_vec_.back()->size()
            << ',' << desc.size() << ',' << hit.first << ',' << hit.second << ',' << matches.size() << ','
            << std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    for (int r=0;r<3;++r) results << ',' << T.first[r];
    for (int r=0;r<3;++r) for (int c=0;c<3;++c) results << ',' << T.second(r,c);
    results << std::endl;
    ++group;
  }
  return 0;
}
