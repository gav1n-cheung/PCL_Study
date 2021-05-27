//
// Created by cs18 on 5/27/21.
//
/*如何使用Kd进行搜索

在本教程中，我们将介绍如何使用KdTree查找特定点或位置的K个最近临近点，然后还将探讨如何找到用户指定半径内的所有邻居（在这种情况下是随机的）。

理论基础
Kd树是CS中用于组织k维空间中一些点的数据结构，它是一个二叉搜索树，上面有其他的约束。
Kd树对于范围搜索和最临近点搜索非常有用。为了我们的目的，我们通常只处理三维空间的点云，因此我们所有的Kd树都是三维空间的。Kd树的每个级别都使用垂直于相应轴的超平面沿特定维度拆分所有子级。
在树的根部，所有子级都会根据第一维进行拆分（即，如果第一维坐标小于根，则将其分在左子树中，如果大于根，则将其分在右子树中）。树中向下的每个级别都在下一个维度划分，一旦所有其他级别都用尽后，
将返回第一维度。生成Kd树的有效方法是使用一种分区方法，例如快速排序所使用的一种方法，将中点放置在根上，所有具有较小一维值的东西都放置在左子树中，而将较大的放置在右子树中，我们不断重复这样
的过程，知道分区的最后一棵树只由一个元素组成。
*/
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

int
mainFun1 (int argc, char** argv)
{
  srand (time (NULL));

//创建点云数据集
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data，填充点云数据
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
//创建kdtree对象
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//将创建的点云作为输入
  kdtree.setInputCloud (cloud);
//创建一个新的点云数据集searchPoint
  pcl::PointXYZ searchPoint;
//填充数据
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K nearest neighbor search，搜索K个临近点

  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;
//如果我们的KdTree返回了0个以上的最近邻居，那么它会打印出我们随机存储的“searchPoint”的所有10个最临近点的位置，这些位置已经存储在了前面的向量中
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

  // Neighbors within radius search，在搜索半径内的临近点

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}
