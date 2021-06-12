////
//// Created by cheung on 2021/6/12.
////
///*基于颜色的区域生长分割算法
// * 在本教程中，我们将学习如何使用pcl::RegionGrowingRGB类中的实现的基于颜色的区域增长算法。
// * 基于颜色的算法和基于曲率的算法主要有两个主要区别。第一个是它使用颜色而非法线。第二个是它使用合并算法进行过分割和欠分割控制。
// * 分割后，尝试合并颜色相近的簇。两个相邻的平均颜色差异很小的簇被合并在一起。然后进行第二个合并步骤。在此步骤中，每个集群都通过
// * 其包含的点数进行验证。如果这个值小于用户定义的阈值，则当前集群将与最近的相邻集群合并。
// */
//#include <iostream>
//#include <thread>
//#include <vector>
//
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/search.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/segmentation/region_growing_rgb.h>
//
//using namespace std::chrono_literals;
//
//int
//main (int argc, char** argv)
//{
//    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("region_growing_rgb_tutorial.pcd", *cloud) == -1 )
//    {
//        std::cout << "Cloud reading failed." << std::endl;
//        return (-1);
//    }
//
//    pcl::IndicesPtr indices (new std::vector <int>);
//    pcl::PassThrough<pcl::PointXYZRGB> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.0, 1.0);
//    pass.filter (*indices);
//    //pcl::RegionGrowingRGB实例化，这里我们不使用第二个法线参数，也就是说，我们在这个实例中，并没哟使用法线来分割点云
//    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//    //设定输入、索引和搜索方法
//    reg.setInputCloud (cloud);
//    reg.setIndices (indices);
//    reg.setSearchMethod (tree);
//    //设定距离阈值，用于确定点是否相邻。如果该点位于小于给定阈值的距离处，则认为他是相邻的。该值用于集群邻居搜索。
//    reg.setDistanceThreshold (10);
//    //设定颜色阈值，用于划分集群
//    reg.setPointColorThreshold (6);
//    //设定区域颜色阈值，用于集群合并
//    reg.setRegionColorThreshold (5);
//    //设定最小的集群数
//    reg.setMinClusterSize (600);
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract (clusters);
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud (colored_cloud);
//    while (!viewer.wasStopped ())
//    {
//        std::this_thread::sleep_for(100us);
//    }
//
//    return (0);
//}