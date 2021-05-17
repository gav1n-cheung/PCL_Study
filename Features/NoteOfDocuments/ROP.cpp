//
// Created by cs18 on 5/17/21.
//


/*ROP（旋转投影统计）功能
在本教程中，我们将学习如何使用pcl::ROPSEstimation类来提取点要素。

理论基础
提取特征的思想如下。我们有一个网格和一组必须为其计算特征的点，我们执行一些简单的步骤。首先，对于给定的兴趣点，裁剪局部表面。局部曲面由位于给定支撑半径内的点和三角形组成。对于给定的兴趣点，
LRF（局部参考系）。LRF只是向量的三元组，我们可以在本文中找到有关如何计算这些向量的全面信息。真正重要的是，使用这些向量，我们可以为点云的旋转提供不变性。为此，我们只需以使关注点称为原点的
方式平移局部表面的点，然后旋转局部表面，以使LRF矢量与x，y，z轴对齐，完成这些操作后，我们便开始特征提取。对于每个轴x,y,z，执行以下步骤，我们将这些轴称为当前轴：
（1）局部表面绕当前轴旋转给定角度
（2）旋转的局部表面的点投影到三个平面xy,xz和yz上
（3）对于构建的每个投影分布矩阵，此矩阵仅显示有多少点落在每个仓上。箱数代表矩阵尺寸，是算法的参数以及支撑半径
（4）对于每个分布矩阵，计算中心矩 
（5）然后将计算得出的值连接起来以形成子功能
我们重复上面的步骤数次，迭代的次数取决于给定的转数。连接不同轴的子功能以形成最终的PoRs描述符。

代码
为了使代码正常工作，选择的数据集要使用三角剖分算法来获取多边形。
*/

#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

int mainFun6 (int argc, char** argv)
{
  if (argc != 4)
    return (-1);

//加载点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
    return (-1);

// 此处加载了必须为其计算RoPS功能的点的索引
  pcl::PointIndicesPtr indices (new pcl::PointIndices);
  std::ifstream indices_file;
  indices_file.open (argv[2], std::ifstream::in);
  for (std::string line; std::getline (indices_file, line);)
  {
    std::istringstream in (line);
    unsigned int index = 0;
    in >> index;
    indices->indices.push_back (index - 1);
  }
  indices_file.close ();

//这里加载有关多边形的信息，如果只有点云而不是网格，则可以用三角测量代码来替换他们
  std::vector <pcl::Vertices> triangles;
  std::ifstream triangles_file;
  triangles_file.open (argv[3], std::ifstream::in);
  for (std::string line; std::getline (triangles_file, line);)
  {
    pcl::Vertices triangle;
    std::istringstream in (line);
    unsigned int vertex = 0;
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    in >> vertex;
    triangle.vertices.push_back (vertex - 1);
    triangles.push_back (triangle);
  }
//这里定义了重要的算法参数：局部表面裁剪的支持半径，用于形成分布矩阵的分区箱的数量，转数。最后一个参数影响描述符的长度。
  float support_radius = 0.0285f;
  unsigned int number_of_partition_bins = 5;
  unsigned int number_of_rotations = 3;
//设置了算法的搜索方法
  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
  search_method->setInputCloud (cloud);
//这是将pcl::ROPSEstimination类实例化的部分，他一共有两个参数
//PointInT--输入点的类型
//PointOutT--输出点的类型
//此后，我们将特征计算需要的所有必要数据设置为输入
  pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
  feature_estimator.setSearchMethod (search_method);
  feature_estimator.setSearchSurface (cloud);
  feature_estimator.setInputCloud (cloud);
  feature_estimator.setIndices (indices);
  feature_estimator.setTriangles (triangles);
  feature_estimator.setRadiusSearch (support_radius);
  feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
  feature_estimator.setNumberOfRotations (number_of_rotations);
  feature_estimator.setSupportRadius (support_radius);
// 启动计算过程
  pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
  feature_estimator.compute (*histograms);

  return (0);
}

