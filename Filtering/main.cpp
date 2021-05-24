//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//int
// main (int argc, char** argv)
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//
//  // Fill in the cloud data
//  cloud->width  = 5;
//  cloud->height = 1;
//  cloud->points.resize (cloud->width * cloud->height);
//
//  for (auto& point: *cloud)
//  {
//    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//  }
//
//  std::cerr << "Cloud before filtering: " << std::endl;
//  for (const auto& point: *cloud)
//    std::cerr << "    " << point.x << " "
//                        << point.y << " "
//                        << point.z << std::endl;
//
//  // Create the filtering object
//  pcl::PassThrough<pcl::PointXYZ> pass;
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("y");
//  pass.setFilterLimits (0.0, 1.0);
//  //pass.setFilterLimitsNegative (true);
//  pass.filter (*cloud_filtered);
//
//  std::cerr << "Cloud after filtering: " << std::endl;
//  for (const auto& point: *cloud_filtered)
//    std::cerr << "    " << point.x << " "
//                        << point.y << " "
//                        << point.z << std::endl;
//  pcl::visualization::PCLVisualizer viewer ("PCL Viewer");
//  viewer.setBackgroundColor(0,0,0);
////  viewer.addPointCloud<pcl::PointXYZ>(cloud);
//
//  viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered);
//
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce ();
//    }
//  return (0);
//}
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//
//int
//main (int argc, char** argv)
//{
//  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
//  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
//
//  // Fill in the cloud data
//  pcl::PCDReader reader;
//  // Replace the path below with the path where you saved your file
//  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
//
//  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
//
//  // Create the filtering object
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*cloud_filtered);
//
//  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
//
//  pcl::PCDWriter writer;
//  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
//         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
//
//  return (0);
//}
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Fill in the cloud data
//    pcl::PCDReader reader;
//    // Replace the path below with the path where you saved your file
//    reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
//
//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;
//
//    // Create the filtering object
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*cloud_filtered);
//
//    std::cerr << "Cloud after filtering: " << std::endl;
//    std::cerr << *cloud_filtered << std::endl;
//
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
//
//    sor.setNegative (true);
//    sor.filter (*cloud_filtered);
//    writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
//
//    return (0);
//}
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/filters/project_inliers.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Fill in the cloud data
//    cloud->width  = 5;
//    cloud->height = 1;
//    cloud->points.resize (cloud->width * cloud->height);
//
//    for (auto& point: *cloud)
//    {
//        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//    }
//
//    std::cerr << "Cloud before projection: " << std::endl;
//    for (const auto& point: *cloud)
//        std::cerr << "    " << point.x << " "
//                  << point.y << " "
//                  << point.z << std::endl;
//
//    // Create a set of planar coefficients with X=Y=0,Z=1
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//    coefficients->values.resize (4);
//    coefficients->values[0] = coefficients->values[1] = 0;
//    coefficients->values[2] = 1.0;
//    coefficients->values[3] = 0;
//
//    // Create the filtering object
//    pcl::ProjectInliers<pcl::PointXYZ> proj;
//    proj.setModelType (pcl::SACMODEL_PLANE);
//    proj.setInputCloud (cloud);
//    proj.setModelCoefficients (coefficients);
//    proj.filter (*cloud_projected);
//
//    std::cerr << "Cloud after projection: " << std::endl;
//    for (const auto& point: *cloud_projected)
//        std::cerr << "    " << point.x << " "
//                  << point.y << " "
//                  << point.z << std::endl;
//
//    return (0);
//}
//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/extract_indices.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Fill in the cloud data
//    pcl::PCDReader reader;
//    reader.read ("table_scene_lms400.pcd", *cloud_blob);
//
//    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
//
//    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloud_blob);
//    sor.setLeafSize (0.01f, 0.01f, 0.01f);
//    sor.filter (*cloud_filtered_blob);
//
//    // Convert to the templated PointCloud
//    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
//
//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
//
//    // Write the downsampled version to disk
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
//
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (1000);
//    seg.setDistanceThreshold (0.01);
//
//    // Create the filtering object
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//
//    int i = 0, nr_points = (int) cloud_filtered->size ();
//    // While 30% of the original cloud is still there
//    while (cloud_filtered->size () > 0.3 * nr_points)
//    {
//        // Segment the largest planar component from the remaining cloud
//        seg.setInputCloud (cloud_filtered);
//        seg.segment (*inliers, *coefficients);
//        if (inliers->indices.size () == 0)
//        {
//            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//            break;
//        }
//
//        // Extract the inliers
//        extract.setInputCloud (cloud_filtered);
//        extract.setIndices (inliers);
//        extract.setNegative (false);
//        extract.filter (*cloud_p);
//        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
//
//        std::stringstream ss;
//        ss << "table_scene_lms400_plane_" << i << ".pcd";
//        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
//
//        // Create the filtering object
//        extract.setNegative (true);
//        extract.filter (*cloud_f);
//        cloud_filtered.swap (cloud_f);
//        i++;
//    }
//
//    return (0);
//}
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int
main (int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 5;
    cloud->height = 1;
    cloud->resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    if (strcmp(argv[1], "-r") == 0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(true);
        // apply filter
        outrem.filter (*cloud_filtered);
    }
    else if (strcmp(argv[1], "-c") == 0){
        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
                                                                  pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                                                                                          pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud_filtered);
    }
    else{
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    // display pointcloud after filtering
    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto& point: *cloud_filtered)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;
    return (0);
}