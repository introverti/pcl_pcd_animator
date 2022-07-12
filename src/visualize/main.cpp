/*** 
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 13:44:20
 * @LastEditors: Tianyun Xuan
 */
#include <iostream>

#include "visualize.h"

int main(int argc, char** argv) {
  if (argc == 3) {
    std::cout << "Received folder : " << argv[1] << std::endl;
    visualization::VisualCenter opera;
    opera.update_source(argv[1], argv[2]);
    opera.spin();
  }
  return 1;
}