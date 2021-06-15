////
//// Created by cheung on 2021/6/15.
////
///*无序点云的快速三角剖分
// * 本教程介绍人如何在具有法线的PointCloud上运行贪婪曲面三角剖分算法，以基于局部邻域的投影获得三角形网格。
// */
///*算法和背景
// 该方法的工作原理是维护一个可以从中生长网格的点列表(“边缘”点)并扩展它直到连接所有可能的点。它可以处理来自一次或多次
// 扫描以及具有多个连接部分的无组织点。如果表面局部光滑并且具有不同点密度的区域之间存在平滑过渡，则效果最佳。
// 通过沿点的法线投影点的局部邻域并连接未连接的点，在本地执行三角剖分。因此，可以设置以下参数：
//    （1）setMaximumNearestNeighbors(unsigned)和setMu(double)控制邻域的大小。前者定义了搜索的邻居数量，
//        而后者指定了要考虑的点的最大可接受距离，相对于最近点的距离(以适应不断变化的密度)。典型值为50-100和2.5-3
//        （或网格未1.5）
//    （2）setSearchRadius(double)实际上是每个三角形的最大边长。这必须由用户设置，以便允许应该可能的最大三角形。
//    （3）setMinimumAngle(double)和setMaximumAngle(double)是每个三角形的最小和最大角度。虽然不能保证第一个参数，
//        但第二个参数的典型值为10和120
//    （4）setMaximumSurfaceAgle(double)和setNormalConsistency(bool)旨在处理存在在锐边或拐角以及曲面的两侧彼此非常靠
//        近的情况。为了实现这一点，如果点的法线偏离超过指定的角度，则点不会连接到当前点(请注意-大多数表面法线估计方法即使在锐边
//        处也能在法线角度之间产生平滑过渡)。如果未设置法线一致性标志，则该角度计算为法线定义的线之间的角度（不考虑法线的方向），
//        因为并非所有法线估计方法都可以保证一致定向的法线。通常，45度（以弧度为单位）和false适用于大多数数据集。
// */
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/io/vtk_io.h>
//
//int
//main (int argc, char** argv)
//{
//    // Load input file into a PointCloud<T> with an appropriate type
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
//    //* the data should be available in cloud
//
//    // Normal estimation*
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud);
//    n.setInputCloud (cloud);
//    n.setSearchMethod (tree);
//    n.setKSearch (20);
//    n.compute (*normals);
//    //* normals should not contain the point normals + surface curvatures
//
//    // Concatenate the XYZ and normal fields*，该方法需要法线，因此使用PCL的标准方法对其进行估计
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//    //* cloud_with_normals = cloud + normals
//
//    // Create search tree*，由于坐标和法线需要在同一个点云中，我们创建一个PointNormal类型的点云
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);
//
//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;
//
//    // Set the maximum distance between connected points (maximum edge length)，设定搜索半径
//    gp3.setSearchRadius (0.025);
//
//    // Set typical values for the parameters设定参数值
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (100);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);
//
//    // Get result设定输入对象、搜索方法、并且最终将结果给到triangles
//    gp3.setInputCloud (cloud_with_normals);
//    gp3.setSearchMethod (tree2);
//    gp3.reconstruct (triangles);
//
//    // Additional vertex information,对于每个点，可以检索包含连接组件的ID及其"状态"
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();
//    saveVTKFile ("mesh.vtk", triangles);
//    // Finish
//    return (0);
//}