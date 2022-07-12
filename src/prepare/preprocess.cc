/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 09:34:07
 * @LastEditors: Tianyun Xuan
 */

#include "preprocess.h"

#include <pcl/common/transforms.h>
using namespace std;
using namespace std::filesystem;
using json = nlohmann::json;

namespace preprocess {

void PreOpreator::Proc() {
  to_format_xyzi(m_source_folder_, m_xyzi_folder_);
  match(m_xyzi_folder_, m_transformed_folder_);
}

bool PreOpreator::parse_yaml(const std::string& yaml_addr) {
  YAML::Node tf = YAML::LoadFile(yaml_addr);
  for (YAML::const_iterator iter = tf.begin(); iter != tf.end(); ++iter) {
    std::string param = iter->first.as<std::string>();
    std::string value = iter->second.as<std::string>();
    if (param == "source_folder") m_source_folder_ = value;
    if (param == "xyzi_folder") m_xyzi_folder_ = value;
    if (param == "pose_folder") m_pose_folder_ = value;
    if (param == "transformed_folder") m_transformed_folder_ = value;
  }
  return true;
}

void PreOpreator::to_format_xyzi(const std::string& target,
                                 const std::string& destination) {
  std::vector<std::string> source{};
  std::vector<std::string> result{};
  if (!exists(destination)) {
    create_directories(destination);
  }

  for (auto& item : directory_iterator(target)) {
    path temp = item;
    if (temp.extension() == ".pcd") {
      source.emplace_back(temp);
      std::string tt = temp.filename();
      std::string dest = destination + "/" + tt;
      result.emplace_back(dest);
    }
  }
  std::sort(source.begin(), source.end());
  std::sort(result.begin(), result.end());
  if (source.size() == result.size()) {
    pcl::PointCloud<PointXYZTI>::Ptr pc_f(new pcl::PointCloud<PointXYZTI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_r(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < source.size(); ++i) {
      pc_r->clear();
      pcl::io::loadPCDFile<PointXYZTI>(source[i], *pc_f);
      for (auto point : pc_f->points) {
        pcl::PointXYZI temp({point.x, point.y, point.z});
        temp.intensity = float(point.intensity);
        pc_r->emplace_back(temp);
      }
      pcl::io::savePCDFileBinary<pcl::PointXYZI>(result[i], *pc_r);
    }
  }
}

void PreOpreator::match(const std::string& from, const std::string& to) {
  if (!exists(to)) {
    create_directories(to);
  }
  std::vector<std::string> source{};
  for (auto& item : directory_iterator(from)) {
    path temp = item;
    if (temp.extension() == ".pcd") {
      std::string tt = temp.stem();
      tt = tt.substr(10, tt.size());
      source.emplace_back(tt);
    }
  }
  std::sort(source.begin(), source.end());
  // for (auto& item : source) {
  //   auto point = item.find('.');
  //   std::string second = item.substr(0, point);
  //   std::string nano = item.substr(point + 1, item.size());
  //   int prec = std::stoi(nano);
  //   // std::cout << "Need to load pose : " << second << std::endl;
  //   update_pose(second);
  //   Eigen::Matrix4d rtm;
  //   // std::cout << "Target : " << prec << std::endl;
  //   find_nearest(prec, rtm);
  //   std::string source_cloud = from + "/iv_points_" + item + ".pcd";
  //   std::string save_cloud = to + "/iv_points_" + item + ".pcd";

  //   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
  //       new pcl::PointCloud<pcl::PointXYZI>);
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr output(
  //       new pcl::PointCloud<pcl::PointXYZI>);
  //   pcl::io::loadPCDFile<pcl::PointXYZI>(source_cloud, *cloud);
  //   pcl::transformPointCloud<pcl::PointXYZI>(*cloud, *output, rtm);
  //   pcl::io::savePCDFileBinary<pcl::PointXYZI>(save_cloud, *output);
  // }
  Eigen::Matrix4d rtm = Eigen::Matrix4d::Identity();
  rtm(2, 3) = -450.0;
  for (auto& item : source) {
    auto point = item.find('.');
    std::string second = item.substr(0, point);
    std::string nano = item.substr(point + 1, item.size());
    int prec = std::stoi(nano);
    update_speed(second);
    double deplace;
    find_nearest(prec, deplace);
    std::string source_cloud = from + "/iv_points_" + item + ".pcd";
    std::string save_cloud = to + "/iv_points_" + item + ".pcd";
    rtm(2, 3) += deplace / 10.f;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(source_cloud, *cloud);
    pcl::transformPointCloud<pcl::PointXYZI>(*cloud, *output, rtm);
    pcl::io::savePCDFileBinary<pcl::PointXYZI>(save_cloud, *output);
  }
}

void PreOpreator::find_nearest(const int& ns, Eigen::Matrix4d& rtm) {
  std::vector<std::pair<int, Eigen::Matrix4d>> poses = m_poses_;
  for (auto& pair : poses) {
    pair.first = std::abs(pair.first - ns);
  }
  std::sort(
      poses.begin(), poses.end(),
      [](std::pair<int, Eigen::Matrix4d>& a,
         std::pair<int, Eigen::Matrix4d>& b) { return a.first < b.first; });
  // for (auto& pair : poses) {
  //   std::cout << pair.first << std::endl;
  // }
  rtm = poses[0].second;
}

void PreOpreator::find_nearest(const int& ns, double& dep_z) {
  std::vector<std::pair<int, double>> speed = m_speed_;
  for (auto& pair : speed) {
    pair.first = std::abs(pair.first - ns);
  }
  std::sort(speed.begin(), speed.end(),
            [](std::pair<int, double>& a, std::pair<int, double>& b) {
              return a.first < b.first;
            });

  dep_z = speed[0].second;
}

void PreOpreator::update_pose(const std::string& target) {
  if (target != m_current_pose_) {
    m_current_pose_ = target;
    m_poses_.clear();
    std::string pose_file =
        m_pose_folder_ + "/current_pose_" + m_current_pose_ + ".pose";
    std::ifstream i(pose_file);
    json j;
    i >> j;
    for (auto& item : j) {
      std::string ts = item.at("timestamp");
      auto point = ts.find('.');
      // std::string second = ts.substr(0, point);
      // std::cout << "Load pose: " << second << std::endl;
      std::string nano = ts.substr(point + 1, ts.size());
      int prec = std::stoi(nano);

      std::vector<double> orien = item.at("orientation");
      Eigen::Quaterniond quat(orien[0], orien[1], orien[2], orien[3]);

      std::vector<double> translation = item.at("translation");
      Eigen::Vector3d trans(translation[0], translation[1], translation[2]);

      Eigen::Matrix4d rtm;
      Eigen::Matrix3d rotat = quat.toRotationMatrix();
      rtm.block(0, 0, 3, 3) = rotat;
      rtm.block(0, 3, 3, 1) = trans;

      m_poses_.emplace_back(std::make_pair(prec, rtm));
    }
  }
}

void PreOpreator::update_speed(const std::string& target) {
  if (target != m_current_pose_) {
    m_current_pose_ = target;
    m_speed_.clear();
    std::string pose_file =
        m_pose_folder_ + "/current_pose_" + m_current_pose_ + ".pose";
    std::ifstream i(pose_file);
    json j;
    i >> j;
    for (auto& item : j) {
      std::string ts = item.at("timestamp");
      auto point = ts.find('.');
      // std::string second = ts.substr(0, point);
      // std::cout << "Load pose: " << second << std::endl;
      std::string nano = ts.substr(point + 1, ts.size());
      int prec = std::stoi(nano);

      std::vector<double> speed = item.at("speed");

      m_speed_.emplace_back(std::make_pair(prec, speed[1]));
    }
  }
}
}  // namespace preprocess