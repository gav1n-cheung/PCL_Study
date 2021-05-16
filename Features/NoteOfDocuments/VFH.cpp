//
// Created by cs18 on 5/15/21.
//


/*估计一组点的VFH签名
本文档描述了视点特征直方图（VFH）描述符，他是针对于点簇的新的表现形式，用于点簇识别和6DOF姿势估计的问题。
下图显示了VFH识别和姿势估计的示例。给定一组训练数据，然后使用点云查询/测试模型，见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation）图1

理论基础：
视点特征直方图（VFH）的根源是FPFH，由于其速度和判别能力，我们决定利用FPFH强大的识别结果，但在保留视点不变性的同时增加视点方差。
我们对物体识别和姿势识别的问题做出了改进：扩展了整个物体簇的FPFH估计值，并计算了视点方向在每个点处估计的法线之间的附加统计量。为此，我们使用了将视点方向直接混合到FPFH中的相对
法线角度计算中的关键思想。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图2）
通过收集视点方向与每个法线所成角度的直方图来计算视点分量。需要注意的是，我们并不是指每个法线的视角，因为不会是比例不变的，而是指中心视点方向之间平移到每个法线的角度，这是VFH的第一部分，
而第二部分则是按照FPFH测量 relative pan, tilt and yaw angles，但现在在中心点的视点方向与曲面上的每个法线之间进行测量。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图3）
因此，新的部分特征称为VFH，下图通过包含两个部分的新功能展示了此设想：
（1）视点方向分量
（2）由拓展的FPFH组成的表面形状分量 
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图4）
*/

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

int mainFun4(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPLYFile("bunny_D01_L01.ply", *cloud);
pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
ne.setInputCloud(cloud);//设定点云法线估计输入
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
//它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
ne.setSearchMethod(tree1);
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
ne.compute(*normals);//计算发现的结果存储于输出中

  // Create the VFH estimation class, and pass the input dataset+normals to it,创建VFH估计类，并将数据集和法线输入
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);        //设定输入点云
  vfh.setInputNormals (normals);  //设定输入的法线
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);如果输入类型为点法型点云数据，则可以将点云数据集作为输入法线参数

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.//创建一个kd树，并且将他作为FPFH估计参数
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);//设定数据集搜寻方法

  // Output datasets,输出数据集
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());//创建一个VFHS对象作为输出
  //对点云的法线进行检查，防止法线中有无限值或空值
    for (int i = 0; i < normals->size(); i++) {
        if (!pcl::isFinite<pcl::Normal>((*normals)[i])) {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }
  // Compute the features,计算VGF值
  std::cout<<"start compute"<<endl;
  vfh.compute (*vfhs);
  std::cout<<"end compute"<<endl;

  // vfhs->size () should be of size 1*,可视化VFHS
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*vfhs,300);
    plotter.plot();
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}














































