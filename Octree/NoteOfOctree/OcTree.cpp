//
// Created by cs18 on 5/27/21.
//
/*使用八叉树进行空间分区和搜索操作
八叉树是用于管理稀疏3D数据的基于树的数据结构。每个内部节点都有八个子节点。
我们要说明如何执行体素搜索内的临近点，k最近邻搜索和半径搜索内的临近点。
*/
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

int
mainFun1 (int argc, char** argv)
{
  srand ((unsigned int) time (NULL));
//创建点云数据并填充
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }

  float resolution = 128.0f;//分辨率
//创建OcTree实例，使用分辨率对其进行初始化。此八叉树在其叶节点内保留一个点索引向量。分辨率参数描述了最低八叉树级别上最小体素的长度，因此
//八叉树的深度是分辨率以及点云的空间尺寸的函数。如果知道点云数据集的边界框，则应该使用define Bounding Box（创建包围盒）方法将其分配给八叉树、
//然后我们为PointCloud分配一个指针，并将输入点云的所有点都添加进去
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();
//输出结果
  pcl::PointXYZ searchPoint;
//填充点云
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // Neighbors within voxel search，使用体素搜索，将搜索点分配给相应的叶节点体素，并返回点索引的向量。这些索引与属于同一体素的点有关。
  //搜索点和搜索结果之间的距离取决于八叉树的分辨率参数。

  std::vector<int> pointIdxVec;

  if (octree.voxelSearch (searchPoint, pointIdxVec))
  {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x 
     << " " << searchPoint.y 
     << " " << searchPoint.z << ")" 
     << std::endl;
              
    for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
   std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
       << " " << (*cloud)[pointIdxVec[i]].y 
       << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
  }

  // K nearest neighbor search。K最临近点搜索方法，得到搜索点的最近的k个点的索引

  int K = 10;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }

  // Neighbors within radius search，半径搜索

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with radius=" << radius << std::endl;


  if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }

}
/*额外细节
PCL八叉树组件提供了几种八叉树类型。他们的基本区别在于各自的叶子节点特征。
OctreePointCloudPointVector(与OctreePointCloud相同) 此八叉树可以在每个叶节点处保存点索引列表
OctreePointCloudSinglePoint,此八叉树类在每个叶节点上仅保留一个点索引，仅存储分配给叶节点的最新点索引
OctreePointCloudOccupancy,此八叉树在其叶节点上不存储任何点信息，他可以用于空间占用检查
OctreePointCloudDensity，此八叉树计算每个叶节点体素内的点数，它允许空间密度差查询
如果需要高速率创建八叉树，请查看八叉树双缓冲实现（Octree2BufBase类）。此类同时在内存中保留两个并行的八叉树结构。除了搜索操作之外，这还可以进行空间变化检测。此外，高级内存管理可在八叉树构建过程中减少内存分配和释放操作。可以通过模板参数“ OctreeT”将双缓冲八叉树实现分配给所有OctreePointCloud类。

所有八叉树都支持八叉树结构和八叉树数据内容的序列化和反序列化。
*/

