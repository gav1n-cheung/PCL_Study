//
// Created by cs18 on 5/13/21.
//
/*点特征直方图（PFH）描述符
随着点特征表示的发展，围绕特定点的几何图形表示的表面法线和曲率估计有一些基础。虽然他们非常快速并且容易计算，但是无法捕获太多细节，因为他们仅用几个值来近似点的k邻域的几何形状。
这就导致了大多数场景将包含许多具有相同或非常相似的特征值的点，从而减少了它们的信息特征。
为了简单起见，本教程介绍了一系列PFH创造的3D特征描述符，介绍了它们的理论优势，并从PCL的角度讨论了实现细节。
学习这部分内容的前提是先阅读PointCloud叫承重的estimate surface normal （https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation），因为PFH签名
依赖于XYZ 3D数据以及表面法线。

基础理论
PFH公式的目的是通过值的多维直方图来概括点周围的平均曲率，从而对点的k邻域几何特性进行编码。此高维超空间为特征表示提供了有意义的签名，对与下层表面的6D姿势不变，并且可以很好地应对附近
存在的不同采样密度或噪声水平。
点特征直方图表示基于k邻域中的点与其估计的表面法线之间的关系。简而言之，他试图通过考虑估计法线方向之间的所有相互作用来尽可能地捕获采样部分的表面变化。因此，最终的超空间取决于每个点的
表面法线估计的质量（精确度？就是估计的好与坏）。
下图展示了查询点（Pq）的PFH计算的影响区域图，该查询点用红色标记并放置在半径为r的圆和所有k个邻居（距离小于3D球体半径r的点）的中间完全互联成网格。最终的PFH描述符被计算为邻域中所有对点
之间的关系直方图，因此计算复杂度为O(K^2).（见 https://pcl.readthedocs.io/projects/tutorials/en/latest/pfh_estimation.html#pfh-estimation 图1）
为了计算两个点Pi和Pj及其关联的法线Ni和之间的相对差Nj，我们在其中一个点处定义了一个固定的坐标系。
（见 https://pcl.readthedocs.io/projects/tutorials/en/latest/pfh_estimation.html#pfh-estimation 公式1 图2）
解释一下，u轴即其中一个点的法线方向，这里取Ps点的法向量Ns；v轴（两点确定一条线，Ns确定一条线，这两条线确定一个平面）垂直于该平面，从而v轴必然就垂直于u轴
        w轴垂直于uv确定的平面。
使用上面所述的uvw轴，两条法线Ns和Nt之间的差别，可以表示如下的一组角度：
（见https://pcl.readthedocs.io/projects/tutorials/en/latest/pfh_estimation.html#pfh-estimation 公式2）
    其中d为两点Ps和Pt之间的欧氏距离，此时我们共有12个值（xyz信息*2，NxNyNz法线向量值*2）。

要估计一对点的PFH四联体，使用
computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1,
                     const Eigen::Vector4f &p2, const Eigen::Vector4f &n2,
                     float &f1, float &f2, float &f3, float &f4);
为表示查询点的最终PFH，将所有四元组的集合合并到直方图中，合并过程将每个要素的值范围划分为b个细分，并对每个子区间中的出现次数进行计数。由于上面呈现的四个特征中的三个是法线之间角度的度量，
因此可以将其值归一化为三角圆上的相同间隔。一个装箱示例是将每个特征区间划分为相同数量的相等部分，并因此创建具有以下特征的直方图：b^4箱完全相同的空间。在此空间中，直方图箱增量对应于所有4个
特征都具有特定值的点。
在某种情况下，第四个特征d对于机器人获取的2.5D数据集意义并不大，因为从视点开始，相邻点之间的距离会增加。
PFHEstimation 类的实际计算调用在内部不执行任何操作，但
for each point p in cloud P                           //遍历点云P内的点p

  1. get the nearest neighbors of p         //获取p的最邻近点

  2. for each pair of neighbors, compute the three angular values           //对于每个临近点对，计算其三个角度值

  3. bin all the results in an output histogram             //将所有结果装箱至输出直方图

要从k邻域计算单个PFH表示，可以使用
computePointPFHSignature (const pcl::PointCloud<PointInT> &cloud,
                          const pcl::PointCloud<PointNT> &normals,
                          const std::vector<int> &indices,
                          int nr_split,
                          Eigen::VectorXf &pfh_histogram);
//其中cloud是包含点的输入点云，normals是包含法线的输入点云，indices表示从cloud开始的k个近邻的集合，nr_split为每个特征区间用于分箱过程的细分数，而pfh_histogram是作为浮点值数组输出的输出结果
// 直方图

Note:由于效率原因，PFHEstimation中的计算方法不会检查法线是否包含NaN或无限值。将此类值传递给compute()将导致未定义的输出。建议至少在程序设计期间或设置参数时检查法线。这可以通过在对
            compute()的调用之前插入以下代码来完成，在写代码是，应设置预处理步骤和参数，来避免法线为无限值或产生错误
for (int i = 0; i < normals->size(); i++)
{
  if (!pcl::isFinite<pcl::Normal>((*normals)[i]))
  {
    PCL_WARN("normals[%d] is not finite\n", i);
  }
}

*/
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

int mainFun2() {

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPLYFile("bunny_D01_L01.ply", *cloud);
   pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
   ne.setInputCloud(cloud);//设定点云法线估计输入
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
//它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
   ne.setSearchMethod(tree1);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
   ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
   ne.compute(*cloud_normals);//计算发现的结果存储于输出中


// Create the FPFH estimation class, and pass the input dataset+normals to it,创建一个FPFH的估计类，并设定输入的点云数据集和法线
pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
fpfh.setInputCloud (cloud);
fpfh.setInputNormals (cloud_normals);
// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
//创建kd树用于搜索邻域
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
fpfh.setSearchMethod (tree);

// Output datasets，创建输出对象
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

// Use all neighbors in a sphere of radius 5cm，设定搜索邻域范围为5cm
// IMPORTANT: the radius used here has to be larger than the radius ued to estimate the surface normals!!!
fpfh.setRadiusSearch (0.05);

std::cout<<"start compute "<<endl;
   for (int i = 0; i < cloud_normals->size(); i++)
   {
       if (!pcl::isFinite<pcl::Normal>((*cloud_normals)[i]))
       {
           PCL_WARN("normals[%d] is not finite\n", i);
       }
   }
// Compute the features
fpfh.compute (*fpfhs);
   std::cout<<"end compute "<<endl;
// fpfhs->size () should have the same size as the input cloud->size ()*
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals);
   pcl::visualization::PCLPlotter plotter;
   plotter.addFeatureHistogram(*fpfhs,300);
   plotter.plot();
   while (!viewer.wasStopped ())
   {
       viewer.spinOnce ();
   }
   return 0;
}









































