/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-11 17:02:25
 * @LastEditors: Tianyun Xuan
 */
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "preprocess.h"


int main(int argc, char** argv) {
  preprocess::PreOpreator dodo;
  dodo.parse_yaml("/home/xavier/repos/pcl_pcd_animator/src/config/prepare_data.yaml");
  dodo.Proc();
  return 1;
}
