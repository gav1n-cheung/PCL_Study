////
//// Created by cheung on 2021/6/10.
////
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//// Types,提前定义变量类型
//typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<PointNT> PointCloudT;
//typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
//typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
//
//// Align a rigid object to a scene with clutter and occlusions
//int
//main (int argc, char **argv)
//{
//    // Point clouds，实例化数据容器，检查输入参数并且加载对象和场景点云。尽管我们已经定义了包含法线的基本点类型，但是我们
//    //只为对象预先定义了法线（通常是这种情况）。我们将估计下面情景的正常信息。
//    PointCloudT::Ptr object (new PointCloudT);
//    PointCloudT::Ptr object_aligned (new PointCloudT);
//    PointCloudT::Ptr scene (new PointCloudT);
//    FeatureCloudT::Ptr object_features (new FeatureCloudT);
//    FeatureCloudT::Ptr scene_features (new FeatureCloudT);
//
//    // Get input object and scene
//    if (argc != 3)
//    {
//        pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
//        return (1);
//    }
//
//    // Load object and scene
//    pcl::console::print_highlight ("Loading point clouds...\n");
//    if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
//        pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
//    {
//        pcl::console::print_error ("Error loading object/scene file!\n");
//        return (1);
//    }
//
//    // Downsample，为了加快处理速度，我们使用PCL的pcl::VoxelGrid<pcl::VoxelGrid>类将对象和场景点云下采样到五毫米的分辨率
//    pcl::console::print_highlight ("Downsampling...\n");
//    pcl::VoxelGrid<PointNT> grid;
//    const float leaf = 0.005f;
//    grid.setLeafSize (leaf, leaf, leaf);
//    grid.setInputCloud (object);
//    grid.filter (*object);
//    grid.setInputCloud (scene);
//    grid.filter (*scene);
//
//    // Estimate normals for scene，估计场景缺失的表面法线。计算用于匹配的特征需要表面法线。
//    pcl::console::print_highlight ("Estimating scene normals...\n");
//    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//    nest.setRadiusSearch (0.01);
//    nest.setInputCloud (scene);
//    nest.compute (*scene);
//
//    // Estimate features，对于下采样点云中的每个点，提取FPFH描述符
//    pcl::console::print_highlight ("Estimating features...\n");
//    FeatureEstimationT fest;
//    fest.setRadiusSearch (0.025);
//    fest.setInputCloud (object);
//    fest.setInputNormals (object);
//    fest.compute (*object_features);
//    fest.setInputCloud (scene);
//    fest.setInputNormals (scene);
//    fest.compute (*scene_features);
//
//    // Perform alignment，我们现在准备设置对齐过程。我们使用类
//    // pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT>实现了一个高效的RANSAC姿态估计循环。这是通过使用
//    //pcl:`CorrespondenceRejectorPoly <pcl::registration::CorrespondenceRejectorPoly>`尽早消除不良姿势假设来实现的。
//
//    pcl::console::print_highlight ("Starting alignment...\n");
//    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
//    align.setInputSource (object);
//    align.setSourceFeatures (object_features);
//    align.setInputTarget (scene);
//    align.setTargetFeatures (scene_features);
//    align.setMaximumIterations (50000); // Number of RANSAC iterations
//    align.setNumberOfSamples (3);
//    // Number of points to sample for generating/prerejecting a pose，样本数--对象和场景之间要采样的点对应数。至少需要3个点来计算姿势
//    align.setCorrespondenceRandomness (5);
//    // Number of nearest features to use，对应随机性，我们可以在N个最佳匹配之间随机选择，而不是将每个对象FPFH描述符与其
//    //在场景中最近的匹配特征进行匹配。这增加了必要的迭代次数，但也使得算法对异常值匹配具有鲁棒性
//    align.setSimilarityThreshold (0.9f);
//    // Polygonal edge length similarity threshold。多边形相似度阈值，该值越贪婪，从而变得更快，但是，这也增加了在存在噪声
//    //时消除良好姿势的风险
//    align.setMaxCorrespondenceDistance (2.5f * leaf);
//    // Inlier threshold，欧几里得距离阈值，用于确定变换后的对象点是否与最近的场景点正确对齐。
//    align.setInlierFraction (0.25f);
//    // Required inlier fraction for accepting a pose hypothesis，在许多实际场景中，场景中被观察对象的大部分不可见，要么是由于杂乱
//    //遮挡或两者兼而有之，在这情况下，我们需要考虑不将所有对象点与场景对齐的姿势假设。使用内点阈值确定正确对齐点的绝对数量，如果该数量
//    //与对象中点总数的比例高于制定的内点分数，则我们认为姿势假设为有效。
//    {
//        pcl::ScopeTime t("Alignment");
//        align.align (*object_aligned);
//    }
//
//    if (align.hasConverged ())
//    {
//        // Print results
//        printf ("\n");
//        Eigen::Matrix4f transformation = align.getFinalTransformation ();
//        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//        pcl::console::print_info ("\n");
//        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//        pcl::console::print_info ("\n");
//        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
//
//        // Show alignment
//        pcl::visualization::PCLVisualizer visu("Alignment");
//        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
//        visu.spin ();
//    }
//    else
//    {
//        pcl::console::print_error ("Alignment failed!\n");
//        return (1);
//    }
//
//    return (0);
//}
//
