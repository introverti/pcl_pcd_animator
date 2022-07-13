/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 16:57:14
 * @LastEditors: Tianyun Xuan
 */
#include <iostream>

#include "visualize.h"

int main(int argc, char** argv) {
  visualization::VisualCenter opera;
  if (argc > 1) {
    opera.parse_yaml(argv[1]);
  } else {
    opera.parse_yaml("/home/xavier/repos/pcl_pcd_animator/config/visualize.yaml");
  }
  opera.spin();
  return 1;
}