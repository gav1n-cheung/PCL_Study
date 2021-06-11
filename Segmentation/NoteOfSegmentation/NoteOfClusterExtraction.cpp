////
//// Created by cheung on 2021/6/11.
////
///*欧几里得聚类提取
// * 在本教程中，我们将学习如何使用 提取欧几里得聚类。
// */
///*
//    理论入门：
//    聚类方法需要将无组织的点云模型P划分为更小的部分，从而减小整体P的整体处理时间。
//    欧几里得意义上的简单数据聚类方法可以通过使用空间的3D网格细分来实现，使用固定宽度的框，或者更一般地使用八叉树数据结构。
//    这种特殊表示的构建速度非常的快，对于需要占用空间的体积表示或每个结果3D框（或八叉树叶）中的数据可以用不同的结构进行近似的情况
//    非常有用。然而，在更一般的意义上，我们可以利用最近的邻居来实现一种本质上类似于泛洪填充算法的聚类技术。
//
//    假设我们有一个点云，上面有一张桌子和一个物体，我们想要找到并分割位于平面上的单个对象点簇。假设我们使用Kd树结构来寻找最近的邻居，
//    那么算法的步骤将是：
//        （1）为输入点云数据集创建Kd树表示P
//        （2）建立一个空的集群列表C，以及一个需要检查的点队列Q
//        （3）然后对于每个点pi属于P，执行以下步骤：
//            添加pi到当前队列Q
//            对于每个点，pi执行以下操作
//                在半径为pi r< dth的球体搜索pki点临近点的集合
//                对于每个临近点pki<Pki，检查该点是否已经被处理，如果没有则将其添加进Q
//            当处理完所有点的列表后，添加Q到集群列表C中，并重置Q为空列表
//        （4）当所有点pi<P都被处理并且是点集群列表C的一部分时，算法终止
// */
//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//
//
//int
//main (int argc, char** argv)
//{
//    // Read in the cloud data
//    pcl::PCDReader reader;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//    reader.read ("table_scene_lms400.pcd", *cloud);
//    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
//
//    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//    pcl::VoxelGrid<pcl::PointXYZ> vg;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//    vg.filter (*cloud_filtered);
//    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
//
//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PCDWriter writer;
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.02);
//
//    int i=0, nr_points = (int) cloud_filtered->size ();
//    while (cloud_filtered->size () > 0.3 * nr_points)
//    {
//        // Segment the largest planar component from the remaining cloud
//        seg.setInputCloud (cloud_filtered);
//        seg.segment (*inliers, *coefficients);
//        if (inliers->indices.size () == 0)
//        {
//            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//            break;
//        }
//
//        // Extract the planar inliers from the input cloud
//        pcl::ExtractIndices<pcl::PointXYZ> extract;
//        extract.setInputCloud (cloud_filtered);
//        extract.setIndices (inliers);
//        extract.setNegative (false);
//
//        // Get the points associated with the planar surface
//        extract.filter (*cloud_plane);
//        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//
//        // Remove the planar inliers, extract the rest
//        extract.setNegative (true);
//        extract.filter (*cloud_f);
//        *cloud_filtered = *cloud_f;
//    }
//
//    // Creating the KdTree object for the search method of the extraction，为搜索方法创建kdtree对象
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud_filtered);
//    //创建一个PointIndices向量，它包含vector<int>中的实际索引信息。
//    //每个检测到的集群的索引都保存在这里。cluster_indices是一个向量，其中包含每个检测到的集群的一个PointIndices实例。
//    //因此，cluster_indices[0]包含我们点云中第一个集群的所有索引
//    std::vector<pcl::PointIndices> cluster_indices;
//    //这里我们创建了一个点类型为PointXYZ的EuclideanClusterExtraction对象。我们也在这里设置提取的参数和变量。
//    //这里最重要的参数是setClusterTolerance，他将决定集群的大小，实际的设定值我们需要测试。
//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//    ec.setClusterTolerance (0.02); // 2cm
//    ec.setMinClusterSize (100);
//    ec.setMaxClusterSize (25000);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud_filtered);
//    ec.extract (cluster_indices);
//    //我们从点云中提取集群并将索引保存在cluster_indices中。为了将每个簇从vector中分离出来，我们必须便利cluster_indices
//    //为每个条目创建一个新的PointCloud并将当前簇的所有点卸任PointCloud
//    int j = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
//        cloud_cluster->width = cloud_cluster->size ();
//        cloud_cluster->height = 1;
//        cloud_cluster->is_dense = true;
//
//        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
//        std::stringstream ss;
//        ss << "cloud_cluster_" << j << ".pcd";
//        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//        j++;
//        pcl::io::savePCDFileASCII("add_cloud.pcd",*cloud_cluster);
//    }
//
//    return (0);
//}
