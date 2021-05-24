//
// Created by cs18 on 5/24/21.
//
/*使用参数模型投影点
在本教程中，我们将学习如何将点投影到参数模型（例如，平面，球体等等）上。参数模型通过一组系数和其方程式ax+by+cz+d=0给出在平面的情况下。
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int
 mainFun4 (int argc, char** argv)
{
    //实例化
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data，创建一个点云数据集
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before projection: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create a set of planar coefficients with X=Y=0,Z=1，我们填写模型系数值，在这种情况下，我们使用一个平面模型ax+by+cz+d=0,其中a=b=d=0,c=1,也就是XY平面
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // Create the filtering object我们创建ProjectInliers对象，并将上面定义的ModelCoefficients用作要投影到的模型。
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (const auto& point: *cloud_projected)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}
