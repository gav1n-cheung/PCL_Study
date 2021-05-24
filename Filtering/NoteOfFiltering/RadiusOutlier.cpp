//
// Created by cs18 on 5/24/21.
//
/*使用条件或者半径异常值删除来删除离群值
本文档眼视力如何使用过滤器模块中的几种不同方法来从点云中删除异常值。首先，我们将研究如何使用ConditionalRemoval过滤器，该过滤器将删除给定输入点云中不满足一个或者多个给定条件
的所有索引。然后，我们将学习如何使用RadiusOutliterRemoval过滤器，该过滤器将删除其输入点云中在一定范围内没有至少一定数量的邻居的所有索引。









*/
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
 mainFun6 (int argc, char** argv)
{

  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  //实例化对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data，填充创建点云数据
  cloud->width  = 5;
  cloud->height = 1;
  cloud->resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
//如果输入-r，则使用RadiusOutlierRemoval过滤器
  if (strcmp(argv[1], "-r") == 0){
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter，创建过滤器并设置参数，搜索半径设置为0.8，并且在搜索点半径内至少有两个相邻点才能保留该点
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    outrem.setKeepOrganized(true);
    // apply filter，进行过滤操作
    outrem.filter (*cloud_filtered);
  }
  //如果输入-c，则使用ConditionalRemoval过滤器
  else if (strcmp(argv[1], "-c") == 0){
    // build the condition，创建给定点必须满足的条件，设定z轴的限制，必须大于GT，小于LT
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));//大于GT
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));//小于LT
    // build the filter，创建过滤器并且设定参数
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter，进行过滤操作
    condrem.filter (*cloud_filtered);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;
  return (0);
}