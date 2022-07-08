#include <iostream>

#include "visualize.h"

int main(int argc, char** argv) {
  if (argc == 2) {
    std::cout << "Received folder : " << argv[1] << std::endl;
    visualization::VisualCenter opera;
    opera.update_source(argv[1]);
    opera.spin();
  }
  return 1;
}

/*
Clipping plane [near,far] 0.72546, 725.46
Focal point [x,y,z] 3.85858, -0.686107, 1.90509
Position [x,y,z] 4.19226, -0.690048, 0.479531
View up [x,y,z] 0.973681, -0.000232473, 0.227915
Camera view angle [degrees] 49.1311
Window size [x,y] 1280, 1260
Window position [x,y] 10, 82
*/