#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <point_type.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "point_type.h"

namespace visualization {

class VisualCenter {
  typedef std::shared_ptr<pcl::visualization::PCLVisualizer> ViewPtr;

 private:
  std::string m_folder_;
  std::vector<std::string> m_pcds_;
  int m_index_;
  ViewPtr m_viewer_;

 public:
  VisualCenter() {
    m_pcds_ = {};
    m_index_ = 0;
    m_viewer_ =
        std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    m_viewer_->setBackgroundColor(0, 0, 0);
    m_viewer_->addCoordinateSystem(1.0);
    m_viewer_->initCameraParameters();
    m_viewer_->setCameraPosition(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
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
  void update_source(std::string target);
  void spin();
};

}  // namespace visualization