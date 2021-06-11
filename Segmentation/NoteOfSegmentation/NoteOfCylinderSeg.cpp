////
//// Created by cheung on 2021/6/11.
////
///*圆柱模型分割
// *本教程举例说明了如何为圆柱模型进行分割。
// * （由于数据中存在噪声，圆柱模型并不完美）
// */
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//
//typedef pcl::PointXYZ PointT;
//
//int
//main (int argc, char** argv)
//{
//    // All the objects needed
//    pcl::PCDReader reader;
//    pcl::PassThrough<PointT> pass;
//    pcl::NormalEstimation<PointT, pcl::Normal> ne;
//    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
//    pcl::PCDWriter writer;
//    pcl::ExtractIndices<PointT> extract;
//    pcl::ExtractIndices<pcl::Normal> extract_normals;
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
//
//    // Datasets
//    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
//
//    // Read in the cloud data
//    reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
//    std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;
//
//    // Build a passthrough filter to remove spurious NaNs
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0, 1.5);
//    pass.filter (*cloud_filtered);
//    std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;
//
//    // Estimate point normals
//    ne.setSearchMethod (tree);
//    ne.setInputCloud (cloud_filtered);
//    ne.setKSearch (50);
//    ne.compute (*cloud_normals);
//
//    // Create the segmentation object for the planar model and set all the parameters
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.03);
//    seg.setInputCloud (cloud_filtered);
//    seg.setInputNormals (cloud_normals);
//    // Obtain the plane inliers and coefficients
//    seg.segment (*inliers_plane, *coefficients_plane);
//    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
//
//    // Extract the planar inliers from the input cloud
//    extract.setInputCloud (cloud_filtered);
//    extract.setIndices (inliers_plane);
//    extract.setNegative (false);
//
//    // Write the planar inliers to disk
//    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//    extract.filter (*cloud_plane);
//    std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//    writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
//
//    // Remove the planar inliers, extract the rest
//    extract.setNegative (true);
//    extract.filter (*cloud_filtered2);
//    extract_normals.setNegative (true);
//    extract_normals.setInputCloud (cloud_normals);
//    extract_normals.setIndices (inliers_plane);
//    extract_normals.filter (*cloud_normals2);
//
//    // Create the segmentation object for cylinder segmentation and set all the parameters
//    //我们使用RANSAC来获取圆柱系数，并且我们对每个内点到模型的距离阈值不超过5cm。此外，我们将表面法线影响设置为0.1的权重，
//    // 并将圆柱模型的半径限制为小于10cm
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_CYLINDER);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMaxIterations (10000);
//    seg.setDistanceThreshold (0.05);
//    seg.setRadiusLimits (0, 0.1);
//    seg.setInputCloud (cloud_filtered2);
//    seg.setInputNormals (cloud_normals2);
//
//    // Obtain the cylinder inliers and coefficients
//    seg.segment (*inliers_cylinder, *coefficients_cylinder);
//    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
//
//    // Write the cylinder inliers to disk
//    extract.setInputCloud (cloud_filtered2);
//    extract.setIndices (inliers_cylinder);
//    extract.setNegative (false);
//    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
//    extract.filter (*cloud_cylinder);
//    if (cloud_cylinder->points.empty ())
//        std::cerr << "Can't find the cylindrical component." << std::endl;
//    else
//    {
//        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
//        writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
//    }
//    return (0);
//}