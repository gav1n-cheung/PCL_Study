////
//// Created by cheung on 2021/6/13.
////
///*条件欧几里得聚类
// * 本教程介绍如何使用pcl::ConditionalEuclideanClustering类：基于欧几里得距离和需要保持的用户可自定义条件对点进行聚类的算法
// * 此类使用与欧几里得聚类提取、区域增长分割和基于颜色的区域增长分割中使用的相同的贪婪/区域增长/泛洪填充方法。与其他类相比，使用此类
// * 的优点是聚类的约束（纯欧几里得、平滑度和RGB）现在可由用户自定义。一些优缺点包括：没有初始种子系统，没有过度和欠分割控制，以及从
// * 主计算循环调用条件函数的时间效率较低的事实。
// */
///*理论入门
// 欧几里得集群提取和区域生长分割的教程已经解释了非常准确的生长算法。除了这些这些解释之外，唯一需要满足的条件是将邻居合并到当前集群中
// 所需的条件，现在可以完全自定义。
// 随着集群的增长，他将评估集群内部的点和附近的候选点之间的用户定义条件。候选点（最近邻点）是使用围绕集群中每个点的欧几里得半径搜索找到的。
// 对于生长的集群中的每个点。该条件需要至少与它的一个邻居一起成立，而不是与其所有临近点一起成立。
// Conditional Euclidean Clustering类可以根据大小约束自动过滤集群。分类为过小或过大的聚类之后仍然可以搜索。
// */
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/console/time.h>
//
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/segmentation/conditional_euclidean_clustering.h>
//
//typedef pcl::PointXYZI PointTypeIO;
//typedef pcl::PointXYZINormal PointTypeFull;
//
//bool
//enforceIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
//{
//    if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//        return (true);
//    else
//        return (false);
//}
//
//bool
//enforceCurvatureOrIntensitySimilarity (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
//{
//    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
//    if (std::abs (point_a.intensity - point_b.intensity) < 5.0f)
//        return (true);
//    if (std::abs (point_a_normal.dot (point_b_normal)) < 0.05)
//        return (true);
//    return (false);
//}
//
//bool
//customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
//{
//    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
//    if (squared_distance < 10000)
//    {
//        if (std::abs (point_a.intensity - point_b.intensity) < 8.0f)
//            return (true);
//        if (std::abs (point_a_normal.dot (point_b_normal)) < 0.06)
//            return (true);
//    }
//    else
//    {
//        if (std::abs (point_a.intensity - point_b.intensity) < 3.0f)
//            return (true);
//    }
//    return (false);
//}
//
//int
//main (int argc, char** argv)
//{
//    // Data containers used
//    pcl::PointCloud<PointTypeIO>::Ptr cloud_in (new pcl::PointCloud<PointTypeIO>), cloud_out (new pcl::PointCloud<PointTypeIO>);
//    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals (new pcl::PointCloud<PointTypeFull>);
//    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
//    pcl::search::KdTree<PointTypeIO>::Ptr search_tree (new pcl::search::KdTree<PointTypeIO>);
//    pcl::console::TicToc tt;
//
//    // Load the input point cloud
//    std::cerr << "Loading...\n", tt.tic ();
//    pcl::io::loadPCDFile ("Statues_4.pcd", *cloud_in);
//    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_in->size () << " points\n";
//
//    // Downsample the cloud using a Voxel Grid class
//    std::cerr << "Downsampling...\n", tt.tic ();
//    pcl::VoxelGrid<PointTypeIO> vg;
//    vg.setInputCloud (cloud_in);
//    vg.setLeafSize (80.0, 80.0, 80.0);
//    vg.setDownsampleAllData (true);
//    vg.filter (*cloud_out);
//    std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->size () << " points\n";
//
//    // Set up a Normal Estimation class and merge data in cloud_with_normals
//    //实例化条件欧式聚类，该类使用true初始化。这将允许提取过小或过大的簇。如果没有这个初始化类，他会节省一些计算时间和
//    //内存。占总点云数不到0.1%的集群被认为太小。占点云总数20%以上的集群被认为是太大。
//    std::cerr << "Computing normals...\n", tt.tic ();
//    pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
//    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
//    ne.setInputCloud (cloud_out);//使用派生自PCLBase的方法指定输入数据
//    ne.setSearchMethod (search_tree);
//    ne.setRadiusSearch (300.0);
//    ne.compute (*cloud_with_normals);
//    std::cerr << ">> Done: " << tt.toc () << " ms\n";
//
//    // Set up a Conditional Euclidean Clustering class
//    std::cerr << "Segmenting to clusters...\n", tt.tic ();
//    pcl::ConditionalEuclideanClustering<PointTypeFull> cec (true);
//    cec.setInputCloud (cloud_with_normals);
//    cec.setConditionFunction (&customRegionGrowing);
//    cec.setClusterTolerance (500.0);
//    cec.setMinClusterSize (cloud_with_normals->size () / 1000);
//    cec.setMaxClusterSize (cloud_with_normals->size () / 5);
//    cec.segment (*clusters);
//    cec.getRemovedClusters (small_clusters, large_clusters);
//    std::cerr << ">> Done: " << tt.toc () << " ms\n";
//
//    // Using the intensity channel for lazy visualization of the output
//    for (int i = 0; i < small_clusters->size (); ++i)
//        for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
//            (*cloud_out)[(*small_clusters)[i].indices[j]].intensity = -2.0;
//    for (int i = 0; i < large_clusters->size (); ++i)
//        for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
//            (*cloud_out)[(*large_clusters)[i].indices[j]].intensity = +10.0;
//    for (int i = 0; i < clusters->size (); ++i)
//    {
//        int label = rand () % 8;
//        for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
//            (*cloud_out)[(*clusters)[i].indices[j]].intensity = label;
//    }
//
//    // Save the output point cloud
//    std::cerr << "Saving...\n", tt.tic ();
//    pcl::io::savePCDFile ("output.pcd", *cloud_out);
//    std::cerr << ">> Done: " << tt.toc () << " ms\n";
//
//    return (0);
//}