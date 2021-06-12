////
//// Created by cheung on 2021/6/12.
////
///*在本教程中，我们将学习如何使用pcl::RegionGrowing类中实现的区域增长算法。这个算法的目的是合并在平滑约束方面足够接近的点。因此，
// * 该算法的输出是簇集，其中每个簇是一组点，这些点被认为是同一光滑表面的一部分。该算法的工作基于点法线之间角度的比较
// */
///*
//理论入门
// 首先，区域增长算法对点按照曲率值进行排序。之所以要这么做，是因为该区域从具有最小曲率值的点开始增长。
// 这样做的原因是曲率最小的点位于平坦区域（从最平坦区域增长可以减少总段数）
// 经过处理之后，我们有了有序的点云。直到点云中没有未标记的点为止，算法选取曲率值最小的点并且开始进行区域的增长。这个过程如下：
//    （1）选取的点被添加到称为种子的集合中
//    （2）对于每个种子点，算法找到它的相邻点
//            测试每个相邻点的法线和当前种子点的法线之间的角度。如果角度小于阈值，则将当前点添加到当前区域
//            之后，测试每个相邻点的曲率值，如果曲率小于阈值，则将该点添加到种子点集中。
//            将当前种子点从种子点集中移除
//        如果种子点集变空，则意味着算法已经扩大了区域，并且从头开始重复该过程。
//见https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html#region-growing-segmentation 伪代码
// */
//#include <iostream>
//#include <vector>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/search.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/region_growing.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
//    {
//        std::cout << "Cloud reading failed." << std::endl;
//        return (-1);
//    }
//
//    //使用pcl::NormalEstimation类来估计法线
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod (tree);
//    normal_estimator.setInputCloud (cloud);
//    normal_estimator.setKSearch (50);
//    normal_estimator.compute (*normals);
//    //
//    pcl::IndicesPtr indices (new std::vector <int>);
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
//    pass.filter (*indices);
//    //实例化RegionGrowing,他有两个参数--PointT(点类型),NormalT(使用的法线类型)
//    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//    reg.setMinClusterSize (50);//最小集群值
//    reg.setMaxClusterSize (1000000);//最大集群值，在分割完成后，所有点数少于最小值或多于最大值的簇都将被丢弃
//    //这两个值的默认值为1和as mach as possible
//
//    //设定搜索方法和搜索临近点个数
//    reg.setSearchMethod (tree);
//    reg.setNumberOfNeighbours (30);
//    reg.setInputCloud (cloud);
//    //reg.setIndices (indices);
//    reg.setInputNormals (normals);
//    //这里是算法初始化最重要的地方，它们负责平滑约束。
//    // 第一种方法以弧度为单位设置角度，该角度将用作法线偏差的允许范围。如果点法之间的偏差小于平滑度阈值，则建议它们在同一集群中。
//    //第二个负责阈值。如果两个点的法线偏差很小，则测试他们的曲率之间的差异，如果该值小于曲率阈值，则算法将使用新添加的点继续集群的增长
//    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold (1.0);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//
//    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
//    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
//    std::cout << "These are the indices of the points of the initial" <<
//              std::endl << "cloud that belong to the first cluster:" << std::endl;
//    int counter = 0;
//    while (counter < clusters[0].indices.size ())
//    {
//        std::cout << clusters[0].indices[counter] << ", ";
//        counter++;
//        if (counter % 10 == 0)
//            std::cout << std::endl;
//    }
//    std::cout << std::endl;
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//    }
//
//    return (0);
//}