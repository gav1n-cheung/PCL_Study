//
// Created by cs18 on 5/13/21.
//该程序应放置在main函数下， 请注释该部分，再
//
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int
mainFun ()
{
    // load point cloud,加载数据集
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals，估计法线 
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//创建法线点云数据类型

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;//创建点云估计变量
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);//设定点云估计方法
    ne.setMaxDepthChangeFactor(0.02f);//设定最大深度变化系数
    ne.setNormalSmoothingSize(10.0f);//设定法线平滑尺寸
    ne.setInputCloud(cloud);//设定输入参数
    ne.compute(*normals);//计算结果存储至法线变量中
//    我们可以使用以下常规估算方法
//    enum NormalEstimationMethod
//    {
//        COVARIANCE_MATRIX,                    //该模式创建9个积分图像，以根据其局部邻域的协方差矩阵为特定点计算法线
//        AVERAGE_3D_GRADIENT,               //创建6个积分图像，以计算水平和垂直3D渐变的平滑版本，并使用这两个渐变之间的叉积计算法线
//        AVERAGE_DEPTH_CHANGE          //仅创建单个积分图像，并根据平均深度变化来计算法线
//    };

    // visualize normals，可视化法线
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");//创建新可视化窗口命名为PCL Viewer
    viewer.setBackgroundColor (0.0, 0.0, 0.5);//设定背景
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);//添加要可视化的点云数据

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}
