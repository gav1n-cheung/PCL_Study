////
//// Created by cheung on 2021/6/15.
////
///*为平面模型构建凹包或凸包多边形
// * 在本教程中，我们将学习如何为平面支持的一组点计算简单地2D外壳多边形（凹面或凸面）
// */
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/surface/concave_hull.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
//            cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
//            cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCDReader reader;
//
//    reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
//    // Build a filter to remove spurious NaNs
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0, 1.1);
//    pass.filter (*cloud_filtered);
//    std::cerr << "PointCloud after filtering has: "
//              << cloud_filtered->size () << " data points." << std::endl;
//
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    //创建了一个分段对象并设置了一些参数。我们使用SACMODEL_PLANE来分割这个点云，找到这个模型
//    //的方法是SAC_RANSAC。
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);//使用SACMODEL_PLANE分割点云
//    seg.setMethodType (pcl::SAC_RANSAC);//使用SAC_RANSAC寻找模型
//    seg.setDistanceThreshold (0.01);
//
//    seg.setInputCloud (cloud_filtered);
//    seg.segment (*inliers, *coefficients);//实际分割
//    //此函数将所有内点（在平面上）存储到inliers，并将平面(a*x+b*y+c*z=d)的系数中
//    std::cerr << "PointCloud after segmentation has: "
//              << inliers->indices.size () << " inliers." << std::endl;
//
//    // Project the model inliers，将内点投影到平面模型上并创建另一个点云。
//    //我们可以做到这一点的一种方法是仅提取我们之前找到的内点，但在这种情况下，我们将使用我们
//    //之前找到的系数。我们设置我们正在寻找的模型类型，然后设置系数，然后对象直到那个点从
//    //cloud_filtered投影到cloud_projected
//    pcl::ProjectInliers<pcl::PointXYZ> proj;
//    proj.setModelType (pcl::SACMODEL_PLANE);
//    // proj.setIndices (inliers);
//    proj.setInputCloud (cloud_filtered);
//    proj.setModelCoefficients (coefficients);
//    proj.filter (*cloud_projected);
//    std::cerr << "PointCloud after projection has: "
//              << cloud_projected->size () << " data points." << std::endl;
//
//    // Create a Concave Hull representation of the projected inliers
//    //创建了ConcaveHull并执行了重建
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::ConcaveHull<pcl::PointXYZ> chull;
//    chull.setInputCloud (cloud_projected);
//    chull.setAlpha (0.1);
//    chull.reconstruct (*cloud_hull);
//
//    std::cerr << "Concave hull has: " << cloud_hull->size ()
//              << " data points." << std::endl;
//
//    pcl::PCDWriter writer;
//    writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);
//
//    return (0);
//}
