/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 09:29:58
 * @LastEditors: Tianyun Xuan
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../common/point_type.h"
#include "Eigen/Core"
#include "Eigen/LU"

using namespace std;
using namespace std::filesystem;
using json = nlohmann::json;

namespace preprocess {

/*
0. Parse config yaml
1. Load pcds filelist and sort
2. Load certain pcd file, get timestamp, assume pose filename
3. If not current loaded pose, load pose in container
4. Search nearest pose, calculate RTmatrix
5. Apply transform and save pcd
*/

class PreOpreator {
 private:
  std::string m_current_pose_;
  std::vector<std::pair<int, Eigen::Matrix4d>> m_poses_;
  std::string m_source_folder_;
  std::string m_xyzi_folder_;
  std::string m_pose_folder_;
  std::string m_transformed_folder_;

 public:
  PreOpreator() : m_current_pose_(""), m_poses_({}) {}
  ~PreOpreator() = default;
  bool parse_yaml(const std::string& yaml_addr);
  void to_format_xyzi(const std::string& target,
                      const std::string& destination);
  void update_pose(const std::string& target);
  void find_nearest(const int& ns, Eigen::Matrix4d& rtm);
  void match(const std::string& from, const std::string& to);

  void Proc();
};

}  // namespace preprocess