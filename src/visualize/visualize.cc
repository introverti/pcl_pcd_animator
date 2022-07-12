/*
 * Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 13:55:47
 * @LastEditors: Tianyun Xuan
 */
#include "visualize.h"

namespace visualization {

void VisualCenter::pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
  std::cout << "[INFO] Point picking event occurred." << std::endl;

  float x, y, z;
  if (event.getPointIndex() == -1) {
    return;
  }
  event.getPoint(x, y, z);
  std::cout << "[INFO] Point coordinate ( " << x << ", " << y << ", " << z
            << ")" << std::endl;
}

void VisualCenter::pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent& event) {
  std::cout << "[INFO] Point picking event occurred." << std::endl;

  float x, y, z;
  if (event.getPointIndex() == -1) {
    return;
  }
  event.getPoint(x, y, z);
  std::cout << "[INFO] Point coordinate ( " << x << ", " << y << ", " << z
            << ")" << std::endl;
}

void VisualCenter::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event, void* viewer_void) {
  if (event.getKeySym() == "a" && event.keyDown()) {
    if (m_index_ == 0) {
      std::cout << "Already the first frame" << std::endl;
    } else {
      --m_index_;
      std::cout << "Prev frame : " << m_index_ << "/" << m_pcds_.size() - 1
                << std::endl;
      update_cloud();
    }
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    if ((size_t)m_index_ == m_pcds_.size() - 1) {
      std::cout << "Already the last frame" << std::endl;
    } else {
      ++m_index_;
      std::cout << "Next frame : " << m_index_ << "/" << m_pcds_.size() - 1
                << std::endl;
      update_cloud();
    }
  }
}

void VisualCenter::keyboardEventOccurred(
    const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeySym() == "a" && event.keyDown()) {
    if (m_index_ == 0) {
      std::cout << "Already the first frame" << std::endl;
    } else {
      std::cout << "Prev" << std::endl;
      --m_index_;
      update_cloud();
    }
  } else if (event.getKeySym() == "d" && event.keyDown()) {
    if ((size_t)m_index_ == m_pcds_.size() - 1) {
      std::cout << "Already the last frame" << std::endl;
    } else {
      std::cout << "Next" << std::endl;
      ++m_index_;
      update_cloud();
    }
  }
}

void VisualCenter::update_source(const std::string& target,
                                 const std::string& mode) {
  m_folder_ = target;
  m_mode_ = mode;
  m_pcds_.clear();
  for (auto& item : std::filesystem::directory_iterator(m_folder_)) {
    std::filesystem::path temp = item;
    if (temp.extension() == ".pcd") {
      m_pcds_.emplace_back(temp);
    }
  }
  std::sort(m_pcds_.begin(), m_pcds_.end());
  std::cout << m_pcds_.size() << std::endl;
}

void VisualCenter::update_cloud() {
  m_viewer_->removeAllPointClouds();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (m_mode_ == "XYZTI") {
    pcl::PointCloud<PointXYZTI>::Ptr source(new pcl::PointCloud<PointXYZTI>);
    if (m_pcds_.size() && m_index_ >= 0 && (size_t)m_index_ < m_pcds_.size()) {
      pcl::io::loadPCDFile<PointXYZTI>(m_pcds_[m_index_], *source);
      cloud->clear();
      for (auto point : source->points) {
        pcl::PointXYZI temp({point.x, point.y, point.z});
        temp.intensity = 256.f - float(point.intensity);
        cloud->emplace_back(temp);
      }
    }
  } else {
    pcl::io::loadPCDFile<pcl::PointXYZI>(m_pcds_[m_index_], *cloud);
    for (auto& point : cloud->points) {
      point.intensity = 256.f - point.intensity;
    }
  }
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
      fildColor(cloud, "intensity");
  m_viewer_->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "current");
  m_viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "current");
}

void VisualCenter::spin() {
  update_cloud();
  m_viewer_->registerPointPickingCallback(
      [this](const pcl::visualization::PointPickingEvent& e) {
        this->pointPickingEventOccurred(e);
      });
  m_viewer_->registerKeyboardCallback(
      [this](const pcl::visualization::KeyboardEvent& e) {
        this->keyboardEventOccurred(e);
      });
  m_viewer_->spin();
}

// boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(
//     pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
//       new pcl::visualization::PCLVisualizer("3D Viewer"));
//   viewer->setBackgroundColor(0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
//       fildColor(cloud, "intensity");
//   viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");
//   viewer->setPointCloudRenderingProperties(
//       pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//   viewer->addCoordinateSystem(1.0);
//   viewer->initCameraParameters();
//   return (viewer);
// }
}  // namespace visualization