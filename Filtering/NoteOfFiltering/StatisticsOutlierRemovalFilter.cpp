//
// Created by cs18 on 5/24/21.
//

/*使用统计离群值删除(Statistics Outlier Removal)过滤器删除离群值
在本教程中，我们将学习如何使用统计分析计数从点云数据集中删除嘈杂的测量值，例如离群值

背景：激光扫描通常会生成变化的点密度的点云数据集。此外，测量误差会导致稀疏的异常值，从而进一步破坏结果。这会使得局部点云特征（比如表面法线或曲率变化）的估计复杂化，
            从而导致错误的估计值，进而使得点云配准失败。通过对每个点的邻域进行统计分析，并修整不符合特定标准的那些不规则现象，可以解决其中的一些不规则现象。我们稀疏的离群值
            消除基于输入数据集中点到临近距离的分布的计算。对于每个点，我们计算从它到所有相邻点的平均距离。假设结果的分布是高斯分布，具有均值和标准差。
            见（https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html#statistical-outlier-removal 图）
            展示了稀疏离群值分析和删除的结果：原始数据在左侧，结果数据集在右侧。该图显示了滤波前后一个点邻域中的平均k最近距离值。
*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
mainFun3 (int argc, char** argv)
{
    //实例化对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data，输入点云数据
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object，创建过滤器对象
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);//设定要进行分析的邻居数为50，标准差乘数设置为1。这意味着所有距离查询点的平均距离大于1标准差的所有点都将标记为异常值并且将被删除。
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
//将结果写入文件中
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
//然后使用相同的参数调用过滤器，但将其输出取反，从而获取离群值（被删除的点），将其写入文件中
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}
