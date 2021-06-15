////
//// Created by cheung on 2021/6/15.
////
///*基于多项式重构的平滑和法线估计
// * 本教程解释了如何使用移动最小二乘法(MLS)表面重建方法来平滑和重新采样噪声数据。
// *
// * 使用统计分析很难消除某些数据不规则性（由小的测量距离误差所引起）。要创建完整的模型，必须考虑数据中的光泽表面和遮挡。在无法获取
// * 额外扫描的情况下，解决方案是使用重采样算法，该算法尝试通过周围数据点之间的高阶多项式插值来重新创建表面的缺失部分。通过执行重采样，
// * 可以纠正这些小错误，并且可以平滑因多次扫描配准而导致的“双壁”伪影。
// * 见https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html#moving-least-squares 图1
// * 在上图的左侧，我们看到了由两个注册点云组成的数据集中的效果并且估计表面法线。由于对齐错误，产生的法线是嘈杂的。
// * 而在上图的右侧，我们看到在使用移动最小二乘法算法平滑后，同一数据集中的表面法线估计效果，和规制每个点的曲率作为重采样前后
// * 特征值关系的度量，我们得到：
// * 见https://pcl.readthedocs.io/projects/tutorials/en/latest/resampling.html#moving-least-squares 表1
// */
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
//
//int
//main (int argc, char** argv)
//{
//    // Load input file into a PointCloud<T> with an appropriate type
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    // Load bun0.pcd -- should be available with the PCL archive in test
//    pcl::io::loadPCDFile ("bun0.pcd", *cloud);
//
//    // Create a KD-Tree
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//
//    // Output has the PointNormal type in order to store the normals calculated by MLS
//    pcl::PointCloud<pcl::PointNormal> mls_points;
//
//    // Init object (second point type is for the normals, even if unused)
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//    //如果不需要正常估计，可以跳过这一步
//    mls.setComputeNormals (true);
//
//    // Set parameters，设定参数，
//    mls.setInputCloud (cloud);
//    mls.setPolynomialOrder (2);//多项式拟合可以被禁用以加速平滑
//    mls.setSearchMethod (tree);
//    mls.setSearchRadius (0.03);
//
//    // Reconstruct
//    mls.process (mls_points);
//
//    // Save output
//    pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);//如果法线和原始维度需要在同一点云中，则必须连接字段
//}
//
