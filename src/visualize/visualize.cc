/*
 * Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 17:47:18
 * @LastEditors: Tianyun Xuan
 */
#include "visualize.h"

#include <fstream>
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
  } else if (event.getKeySym() == "y" && event.keyDown()) {
    play();
  }
}

void VisualCenter::update_cloud() {
  m_viewer_->removeAllPointClouds();
  m_viewer_->removeText3D("dis");
  double curr = m_distance_ - m_deplacement_[m_index_];
  m_viewer_->addText3D(std::to_string(curr), m_txt_position_, 1, 255, 255, 255,
                       "dis");
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

void VisualCenter::play() {
  m_index_ = m_pcds_.size() - 1;
  while (m_index_) {
    update_cloud();
    m_viewer_->spinOnce(200);
    // std::this_thread::sleep_for(std::chrono::milliseconds(90));
    --m_index_;
  }
}

void VisualCenter::parse_yaml(const std::string& yaml_addr) {
  YAML::Node tf = YAML::LoadFile(yaml_addr);
  for (YAML::const_iterator iter = tf.begin(); iter != tf.end(); ++iter) {
    std::string param = iter->first.as<std::string>();
    std::string value = iter->second.as<std::string>();
    if (param == "pcd_folder") m_folder_ = value;
    if (param == "point_type") m_mode_ = value;
    if (param == "deplacement_txt") m_depz_ = value;
    if (param == "origin_distance") {
      double dvalue = iter->second.as<double>();
      m_distance_ = dvalue;
    }
  }
  m_pcds_.clear();
  for (auto& item : std::filesystem::directory_iterator(m_folder_)) {
    std::filesystem::path temp = item;
    if (temp.extension() == ".pcd") {
      m_pcds_.emplace_back(temp);
    }
  }
  std::sort(m_pcds_.begin(), m_pcds_.end());

  m_deplacement_.clear();
  std::ifstream infile(m_depz_);
  std::string item;
  while (getline(infile, item)) {
    m_deplacement_.emplace_back(std::stod(item));
  }

  std::cout << m_pcds_.size() << "," << m_deplacement_.size() << std::endl;
}
}  // namespace visualization