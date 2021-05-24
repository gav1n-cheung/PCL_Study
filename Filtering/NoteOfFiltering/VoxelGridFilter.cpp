//
// Created by cs18 on 5/24/21.
//
/*
使用VoxelGrid过滤器对PointCloud进行下采样

本教程中，我们将学习如何使用体素化网格方法来对点云数据集进行降采样（即减少点的数量）。
VoxelGrid：我们在输入点云数据上创建一个3D体素网格（将体素网格视为一组空间中的微小3D框）。然后，在每个体素（即3D框）中，所有存在的点都将以其质心进行近似（即降采样）。
                        这种方法比用体素的中心来逼近它们要慢一点，但是可以更准确的表示物体表面。

 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
mainFun2 (int argc, char** argv)
{
    //实例化变量
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data，输入点云数据
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object，创建过滤对象，设置过滤尺寸为1cm的VoxelGrid过滤器。
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);
//输出体素化后的对象
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
//将体素化后的点云数据写出
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
