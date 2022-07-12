/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 15:08:18
 * @LastEditors: Tianyun Xuan
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "../common/point_type.h"

namespace visualization {

class VisualCenter {
  typedef std::shared_ptr<pcl::visualization::PCLVisualizer> ViewPtr;

 private:
  std::string m_folder_;
  std::string m_mode_;
  std::string m_depz_;
  std::vector<double> m_deplacement_;
  std::vector<std::string> m_pcds_;
  int m_index_;
  ViewPtr m_viewer_;
  double m_distance_;
  pcl::PointXYZ m_txt_position_;

 public:
  VisualCenter() {
    m_pcds_ = {};
    m_mode_ = "";
    m_depz_ ="";
    m_index_ = 0;
    m_distance_ = 458.4;
    m_txt_position_ = {5, 5, 5};
    m_viewer_ =
        std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    m_viewer_->setBackgroundColor(0, 0, 0);
    // m_viewer_->addCoordinateSystem(1.0);
    m_viewer_->initCameraParameters();
    m_viewer_->setCameraPosition(1.31497, -0.394183, -13.3555, 0.995829,
                                 -0.000587688, 0.0912344, 1.0, 0.0, 0.0);
    m_viewer_->addText3D(std::to_string(m_distance_), m_txt_position_, 1, 255,
                         255, 255, "dis");
    // m_viewer_->setCameraPosition(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  }
  ~VisualCenter() = default;
  void pointPickingEventOccurred(
      const pcl::visualization::PointPickingEvent& event, void* viewer_void);
  void pointPickingEventOccurred(
      const pcl::visualization::PointPickingEvent& event);
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
                             void* viewer_void);
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);
  void update_cloud();
  void spin();
  void play();
  void parse_yaml(const std::string& yaml_addr);
};

}  // namespace visualization