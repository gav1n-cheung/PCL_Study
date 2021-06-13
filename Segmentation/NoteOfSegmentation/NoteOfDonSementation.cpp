////
//// Created by cheung on 2021/6/13.
////
///*基于法线的差异分割
// * 在本教程中，我们将学习如何使用pcl::DifferenceOfNormalsEstimation类中实现的法线差功能对无组织点云进行基于尺度的分割。
// * 该算法对给定的输入点云执行基于尺度的分割，找到属于给定尺度参数的点。
// */
///*理论入门
// 法线差（DoN）提供了一种计算效率高的多尺度方法来处理大型无组织3D点云。这个想法在概念上非常简单，但在分割具有广泛尺度变化的场景时
// 出奇的有效。对于点云P中的每个点p，两个单位点法线n^(p,rl)，n^(p,rs)用不同的半径估计rl>rs。这些点的归一化（矢量）差异定义了
// 运算符。
// 正式定义了差分法线运算符
// 见https://pcl.readthedocs.io/projects/tutorials/en/latest/don_segmentation.html#don-segmentation 公式1
// 该变量解释见公式下
// 请注意，算子的响应时归一化的矢量场，因此是可定向的(结果方向是一个关键特征)，但是算子的番薯通常提供更容易处理的量，并且始终在
// 范围内(0,1)
// 见https://pcl.readthedocs.io/projects/tutorials/en/latest/don_segmentation.html#don-segmentation 图1
// DoN背后的主要动机是观察到在任何给定半径处估计的表面法线在支撑半径的尺度上反映了表面的基本几何形状。尽管有许多不同的方法来
// 估计表面法线，但法线总是用支撑半径（或通过固定数量的邻居）来估计。该支撑半径决定了法线所代表的表面结构中的比例。
// 图1以一维的形式说明了这种效果。用小支撑半径rs估计的法线n^和切线受小尺度表面结构的影响（同样受噪声影响）。另一方面，用大支撑半径
// rl估计的法线和切平面受小尺度结构的影响较小，并表示较大尺度表面结构的几何形状。
//
// 使用法线差进行分割
// 对于分割，我们只需执行以下操作：
// （1）使用大的支持半径rl估计每个点的法线
// （2）使用小支撑半径rs估计每个点的法线
// （3）对于每个点计算其法线的归一化差异
// （4）过滤产生的矢量场以隔离属于感兴趣的尺度/区域的点。
// */
///**
// * @file don_segmentation.cpp
// * Difference of Normals Example for PCL Segmentation Tutorials.
// *
// * @author Yani Ioannou
// * @date 2012-09-24
// */
//#include <string>
//
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/organized.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/segmentation/extract_clusters.h>
//
//#include <pcl/features/don.h>
//
//using namespace pcl;
//
//int
//main (int argc, char *argv[])
//{
//    ///The smallest scale to use in the DoN filter.
//    double scale1;
//
//    ///The largest scale to use in the DoN filter.
//    double scale2;
//
//    ///The minimum DoN magnitude to threshold by
//    double threshold;
//
//    ///segment scene into clusters with given distance tolerance using euclidean clustering
//    double segradius;
//
//    if (argc < 6)
//    {
//        std::cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << std::endl;
//        exit (EXIT_FAILURE);
//    }
//
//    /// the file to read from.
//    std::string infile = argv[1];
//    /// small scale
//    std::istringstream (argv[2]) >> scale1;
//    /// large scale
//    std::istringstream (argv[3]) >> scale2;
//    std::istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
//    std::istringstream (argv[5]) >> segradius;   // threshold for radius segmentation
//
//    // Load cloud in blob format
//    pcl::PCLPointCloud2 blob;
//    pcl::io::loadPCDFile (infile.c_str (), blob);
//    pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
//    pcl::fromPCLPointCloud2 (blob, *cloud);
//
//    // Create a search tree, use KDTreee for non-organized data.
//    //我们创建一个搜索树。对于有组织的数据(即深度图像)，一个更快的搜索树是OrganizedNeighbor搜索树。
//    //对于无组织的数据，KDTree是一个不错的选择
//    pcl::search::Search<PointXYZRGB>::Ptr tree;
//    if (cloud->isOrganized ())
//    {
//        tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
//    }
//    else
//    {
//        tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
//    }
//
//    // Set the input pointcloud for the search tree
//    tree->setInputCloud (cloud);
//
//    if (scale1 >= scale2)
//    {
//        std::cerr << "Error: Large scale must be > small scale!" << std::endl;
//        exit (EXIT_FAILURE);
//    }
//
//    // Compute normals using both small and large scales at each point
//    //计算法线--我们可以使用不同的方法来加速法线计算
//    pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
//    ne.setInputCloud (cloud);
//    ne.setSearchMethod (tree);
//
//    /**
//     * NOTE: setting viewpoint is very important, so that we can ensure
//     * normals are all pointed in the same direction!
//     */
//    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
//
//    // calculate normals with the small scale
//    //我们使用大半径和小半径的法线来估计计算法线。如果法线估计仅限于一定数量的邻居，它可能不是基于给定半径的完整表面
//    //因此不适合法线差特征
//    std::cout << "Calculating normals for scale..." << scale1 << std::endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
//
//    ne.setRadiusSearch (scale1);
//    ne.compute (*normals_small_scale);
//
//    // calculate normals with the large scale
//    std::cout << "Calculating normals for scale..." << scale2 << std::endl;
//    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
//
//    ne.setRadiusSearch (scale2);
//    ne.compute (*normals_large_scale);
//
//    // Create output cloud for DoN results
//    //使用法线估计执行实际的法线差特征计算。法线差的结果是一个向量场，我们复制得到的结果
//    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
//    copyPointCloud (*cloud, *doncloud);
//
//    std::cout << "Calculating DoN... " << std::endl;
//    // Create DoN operator，实例化一个新的pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal>
//    //类来计算法线向量场的插值
//    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
//    don.setInputCloud (cloud);//设定输入点云
//    don.setNormalScaleLarge (normals_large_scale);//估计点云中的法线的类型
//    don.setNormalScaleSmall (normals_small_scale);//矢量场输出类型
//
//    if (!don.initCompute ())
//    {
//        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
//        exit (EXIT_FAILURE);
//    }
//
//    // Compute DoN
//    don.computeFeature (*doncloud);
//
//    // Save DoN features
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);
//
//    // Filter by magnitude
//    std::cout << "Filtering out DoN mag <= " << threshold << "..." << std::endl;
//
//    // Build the condition for filtering，建立过滤器使得点云数据减小
//    pcl::ConditionOr<PointNormal>::Ptr range_cond (
//            new pcl::ConditionOr<PointNormal> ()
//    );
//    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
//            new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
//    );
//    // Build the filter
//    pcl::ConditionalRemoval<PointNormal> condrem;
//    condrem.setCondition (range_cond);
//    condrem.setInputCloud (doncloud);
//
//    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);
//
//    // Apply filter
//    condrem.filter (*doncloud_filtered);
//
//    doncloud = doncloud_filtered;
//
//    // Save filtered output
//    std::cout << "Filtered Pointcloud: " << doncloud->size () << " data points." << std::endl;
//
//    writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);
//
//    // Filter by magnitude。使用聚类算法来分割结果，这里我们使用欧几里得聚类，阈值等于小半径参数
//    std::cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << std::endl;
//
//    pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
//    segtree->setInputCloud (doncloud);
//
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointNormal> ec;
//
//    ec.setClusterTolerance (segradius);
//    ec.setMinClusterSize (50);
//    ec.setMaxClusterSize (100000);
//    ec.setSearchMethod (segtree);
//    ec.setInputCloud (doncloud);
//    ec.extract (cluster_indices);
//
//    int j = 0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
//    {
//        pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
//        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            cloud_cluster_don->points.push_back ((*doncloud)[*pit]);
//        }
//
//        cloud_cluster_don->width = cloud_cluster_don->size ();
//        cloud_cluster_don->height = 1;
//        cloud_cluster_don->is_dense = true;
//
//        //Save cluster
//        std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->size () << " data points." << std::endl;
//        std::stringstream ss;
//        ss << "don_cluster_" << j << ".pcd";
//        writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
//    }
//}
