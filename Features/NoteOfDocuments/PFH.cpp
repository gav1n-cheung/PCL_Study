//
// Created by cs18 on 5/13/21.
//

#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>


int mainFun1(){

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
pcl::io::loadPCDFile("test.pcd", *cloud);
pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
ne.setMaxDepthChangeFactor(0.02f);
ne.setNormalSmoothingSize(10.0f);
ne.setInputCloud(cloud);
ne.compute(*normals);

// Create the PFH estimation class, and pass the input dataset+normals to it,创建PFH估计类，并且传递输入数据集和法线
pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;//创建变量
pfh.setInputCloud (cloud);//设定输入数据集
pfh.setInputNormals (normals);//设定输入法线
// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud); //或者，如果点云是点法类型，可以直接设定输入法线为点云

// Create an empty kdtree representation, and pass it to the PFH estimation object.创建一个空的kd树表示，并且将他作为PFH估计的对象
// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).根据给定的输入数据集（因为无其他搜索面），其内容将填充在对象内部
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
pfh.setSearchMethod (tree);//设定PFH的搜索方法

// Output datasets，输出数据集
pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

// Use all neighbors in a sphere of radius 5cm，使用半径在5cm内的所有相邻点
// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!这里设定的半径必须比估计表面法线时设定的半径大
pfh.setRadiusSearch (0.05);

// Compute the features。计算特征
pfh.compute (*pfhs);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfhs,300);
    plotter.plot();

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;

// pfhs->size () should have the same size as the input cloud->size ()*
}