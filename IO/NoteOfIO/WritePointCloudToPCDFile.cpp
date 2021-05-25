//
// Created by cs18 on 5/25/21.
//

/*将PointCloud数据写入到PCD文件中
在本教程中，我们将学习如何将点云数据写入PCD文件中
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
  mainFun2 (int argc, char** argv)
{
//创建一个空的点云对象
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data，填充点云对象，确定其参数
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
//随机赋值给点云对象内的点
  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
//将对象保存为ascii类型的点云文件
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  //输出保存的点云数据信息
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}