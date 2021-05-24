//
// Created by cs18 on 5/24/21.
//

/*
在本教程中，我们将学习如何沿着指定的维度进行简单的过滤，截取给定范围内或范围外的值

*/
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 mainFun1 (int argc, char** argv)
{
    //实例化对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // 创建点云对象
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create the filtering object，创建PassFiltering对象，并设置其参数。FilterFieldName设置为z坐标，可接受的间隔值FilterLimits设置为(0.0,1.0);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 1.0);
  pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

//最后，我们显示过滤后的点云数据
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  return (0);
}


