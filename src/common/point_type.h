/*** 
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 09:28:31
 * @LastEditors: Tianyun Xuan
 */
#ifndef POINT_TYPE_H_
#define POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointXYZTI  // enforce SSE padding for correct memory alignment
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  double timestamp;
  std::uint16_t intensity;
  PCL_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZTI,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (double, timestamp, timestamp)
                                   (std::uint16_t, intensity, intensity))
#endif