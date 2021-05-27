//
// Created by cs18 on 5/27/21.
//
/*无组织点云数据的空间变化检测
通过递归比较八叉树的树结构，可以识别由体素配置差异表示的空间变化，此外，我们解释了如何使用pcl八叉树双缓冲计数使我们能够随着时间流逝有效的处理多个点云。
*/
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>

int
mainFun2 (int argc, char** argv)
{
  srand ((unsigned int) time (NULL));

  // Octree resolution - side length of octree voxels，八叉树体素的分辨率，即八叉树体素的边长
  float resolution = 32.0f;

  // Instantiate octree-based point cloud change detection class，示例化八叉树对象
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );

  // Generate pointcloud data for cloudA，创建一个点云数据集并且填充它
  cloudA->width = 128;
  cloudA->height = 1;
  cloudA->points.resize (cloudA->width * cloudA->height);

  for (std::size_t i = 0; i < cloudA->size (); ++i)
  {
    (*cloudA)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  // Add points from cloudA to octree，设定八叉树的输入，OctreePointCloudChangeDetector类继承自Octree2BufBase类，该类可以使我们同时在内存中管理两个八叉树。
  //另外，他实现了一个内存池，该内存池可重用已分配的节点对象，因此在生成多点云的八叉树时减少了内存分配和释放操作。通
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.通过调用octree.switchBuffers ()，我们重置了octree类，同时将先前的octree结构保留在内存中。
  octree.switchBuffers ();

//在生成一个点云对象并且填充他，用于构建新的八叉树结构
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );
   
  // Generate pointcloud data for cloudB 
  cloudB->width = 128;
  cloudB->height = 1;
  cloudB->points.resize (cloudB->width * cloudB->height);

  for (std::size_t i = 0; i < cloudB->size (); ++i)
  {
    (*cloudB)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

//为检索存储在基于cloudB的octree结构在基于cloudA的octree结构中不存在的点，我们调用getPointdicesFromNewVexols，该方法返回结果的向量点索引。
  std::vector<int> newPointIdxVector;
  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points，输出结果
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
              << (*cloudB)[newPointIdxVector[i]].y << " "
              << (*cloudB)[newPointIdxVector[i]].z << std::endl;

}
