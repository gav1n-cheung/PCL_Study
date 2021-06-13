////////////#include <iostream>
////////////#include <pcl/ModelCoefficients.h>
////////////#include <pcl/io/pcd_io.h>
////////////#include <pcl/point_types.h>
////////////#include <pcl/sample_consensus/method_types.h>
////////////#include <pcl/sample_consensus/model_types.h>
////////////#include <pcl/segmentation/sac_segmentation.h>
////////////
////////////int
////////////main (int argc, char** argv)
////////////{
////////////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
////////////
////////////    // Fill in the cloud data
////////////    cloud->width  = 15;
////////////    cloud->height = 1;
////////////    cloud->points.resize (cloud->width * cloud->height);
////////////
////////////    // Generate the data
////////////    for (auto& point: *cloud)
////////////    {
////////////        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
////////////        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
////////////        point.z = 1.0;
////////////    }
////////////
////////////    // Set a few outliers
////////////    (*cloud)[0].z = 2.0;
////////////    (*cloud)[3].z = -2.0;
////////////    (*cloud)[6].z = 4.0;
////////////
////////////    std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
////////////    for (const auto& point: *cloud)
////////////        std::cerr << "    " << point.x << " "
////////////                  << point.y << " "
////////////                  << point.z << std::endl;
////////////
////////////    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
////////////    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
////////////    // Create the segmentation object
////////////    pcl::SACSegmentation<pcl::PointXYZ> seg;
////////////    // Optional
////////////    seg.setOptimizeCoefficients (true);
////////////    // Mandatory
////////////    seg.setModelType (pcl::SACMODEL_PLANE);
////////////    seg.setMethodType (pcl::SAC_RANSAC);
////////////    seg.setDistanceThreshold (0.01);
////////////
////////////    seg.setInputCloud (cloud);
////////////    seg.segment (*inliers, *coefficients);
////////////
////////////    if (inliers->indices.size () == 0)
////////////    {
////////////        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
////////////        return (-1);
////////////    }
////////////
////////////    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
////////////              << coefficients->values[1] << " "
////////////              << coefficients->values[2] << " "
////////////              << coefficients->values[3] << std::endl;
////////////
////////////    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
////////////    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
////////////        for (const auto& idx: inliers->indices)
////////////            std::cerr << idx << "    " << cloud->points[idx].x << " "
////////////                      << cloud->points[idx].y << " "
////////////                      << cloud->points[idx].z << std::endl;
////////////
////////////    return (0);
////////////}
//////////#include <pcl/ModelCoefficients.h>
//////////#include <pcl/point_types.h>
//////////#include <pcl/io/pcd_io.h>
//////////#include <pcl/filters/extract_indices.h>
//////////#include <pcl/filters/voxel_grid.h>
//////////#include <pcl/features/normal_3d.h>
//////////#include <pcl/kdtree/kdtree.h>
//////////#include <pcl/sample_consensus/method_types.h>
//////////#include <pcl/sample_consensus/model_types.h>
//////////#include <pcl/segmentation/sac_segmentation.h>
//////////#include <pcl/segmentation/extract_clusters.h>
//////////
//////////
//////////int
//////////main (int argc, char** argv)
//////////{
//////////    // Read in the cloud data
//////////    pcl::PCDReader reader;
//////////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//////////    reader.read ("table_scene_lms400.pcd", *cloud);
//////////    std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
//////////
//////////    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//////////    pcl::VoxelGrid<pcl::PointXYZ> vg;
//////////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//////////    vg.setInputCloud (cloud);
//////////    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//////////    vg.filter (*cloud_filtered);
//////////    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
//////////
//////////    // Create the segmentation object for the planar model and set all the parameters
//////////    pcl::SACSegmentation<pcl::PointXYZ> seg;
//////////    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//////////    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//////////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//////////    pcl::PCDWriter writer;
//////////    seg.setOptimizeCoefficients (true);
//////////    seg.setModelType (pcl::SACMODEL_PLANE);
//////////    seg.setMethodType (pcl::SAC_RANSAC);
//////////    seg.setMaxIterations (100);
//////////    seg.setDistanceThreshold (0.02);
//////////
//////////    int i=0, nr_points = (int) cloud_filtered->size ();
//////////    while (cloud_filtered->size () > 0.3 * nr_points)
//////////    {
//////////        // Segment the largest planar component from the remaining cloud
//////////        seg.setInputCloud (cloud_filtered);
//////////        seg.segment (*inliers, *coefficients);
//////////        if (inliers->indices.size () == 0)
//////////        {
//////////            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//////////            break;
//////////        }
//////////
//////////        // Extract the planar inliers from the input cloud
//////////        pcl::ExtractIndices<pcl::PointXYZ> extract;
//////////        extract.setInputCloud (cloud_filtered);
//////////        extract.setIndices (inliers);
//////////        extract.setNegative (false);
//////////
//////////        // Get the points associated with the planar surface
//////////        extract.filter (*cloud_plane);
//////////        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//////////
//////////        // Remove the planar inliers, extract the rest
//////////        extract.setNegative (true);
//////////        extract.filter (*cloud_f);
//////////        *cloud_filtered = *cloud_f;
//////////    }
//////////
//////////    // Creating the KdTree object for the search method of the extraction
//////////    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//////////    tree->setInputCloud (cloud_filtered);
//////////
//////////    std::vector<pcl::PointIndices> cluster_indices;
//////////    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//////////    ec.setClusterTolerance (0.02); // 2cm
//////////    ec.setMinClusterSize (100);
//////////    ec.setMaxClusterSize (25000);
//////////    ec.setSearchMethod (tree);
//////////    ec.setInputCloud (cloud_filtered);
//////////    ec.extract (cluster_indices);
//////////
//////////    int j = 0;
//////////    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//////////    {
//////////        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
//////////        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//////////            cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
//////////        cloud_cluster->width = cloud_cluster->size ();
//////////        cloud_cluster->height = 1;
//////////        cloud_cluster->is_dense = true;
//////////
//////////        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
//////////        std::stringstream ss;
//////////        ss << "cloud_cluster_" << j << ".pcd";
//////////        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//////////        j++;
//////////    }
//////////
//////////    return (0);
//////////}
////////#include <iostream>
////////#include <vector>
////////#include <pcl/point_types.h>
////////#include <pcl/io/pcd_io.h>
////////#include <pcl/search/search.h>
////////#include <pcl/search/kdtree.h>
////////#include <pcl/features/normal_3d.h>
////////#include <pcl/visualization/cloud_viewer.h>
////////#include <pcl/filters/passthrough.h>
////////#include <pcl/segmentation/region_growing.h>
////////
////////int
////////main (int argc, char** argv)
////////{
////////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
////////    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
////////    {
////////        std::cout << "Cloud reading failed." << std::endl;
////////        return (-1);
////////    }
////////
////////    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
////////    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
////////    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
////////    normal_estimator.setSearchMethod (tree);
////////    normal_estimator.setInputCloud (cloud);
////////    normal_estimator.setKSearch (50);
////////    normal_estimator.compute (*normals);
////////
////////    pcl::IndicesPtr indices (new std::vector <int>);
////////    pcl::PassThrough<pcl::PointXYZ> pass;
////////    pass.setInputCloud (cloud);
////////    pass.setFilterFieldName ("z");
////////    pass.setFilterLimits (0.0, 1.0);
////////    pass.filter (*indices);
////////
////////    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
////////    reg.setMinClusterSize (50);
////////    reg.setMaxClusterSize (1000000);
////////    reg.setSearchMethod (tree);
////////    reg.setNumberOfNeighbours (30);
////////    reg.setInputCloud (cloud);
////////    //reg.setIndices (indices);
////////    reg.setInputNormals (normals);
////////    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
////////    reg.setCurvatureThreshold (1.0);
////////
////////    std::vector <pcl::PointIndices> clusters;
////////    reg.extract (clusters);
////////
////////    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
////////    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
////////    std::cout << "These are the indices of the points of the initial" <<
////////              std::endl << "cloud that belong to the first cluster:" << std::endl;
////////    int counter = 0;
////////    while (counter < clusters[0].indices.size ())
////////    {
////////        std::cout << clusters[0].indices[counter] << ", ";
////////        counter++;
////////        if (counter % 10 == 0)
////////            std::cout << std::endl;
////////    }
////////    std::cout << std::endl;
////////
////////    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
////////    pcl::visualization::CloudViewer viewer ("Cluster viewer");
////////    viewer.showCloud(colored_cloud);
////////    while (!viewer.wasStopped ())
////////    {
////////    }
////////
////////    return (0);
////////}
//////#include <iostream>
//////#include <thread>
//////#include <vector>
//////
//////#include <pcl/point_types.h>
//////#include <pcl/io/pcd_io.h>
//////#include <pcl/search/search.h>
//////#include <pcl/search/kdtree.h>
//////#include <pcl/visualization/cloud_viewer.h>
//////#include <pcl/filters/passthrough.h>
//////#include <pcl/segmentation/region_growing_rgb.h>
//////
//////using namespace std::chrono_literals;
//////
//////int
//////main (int argc, char** argv)
//////{
//////    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//////
//////    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//////    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("region_growing_rgb_tutorial.pcd", *cloud) == -1 )
//////    {
//////        std::cout << "Cloud reading failed." << std::endl;
//////        return (-1);
//////    }
//////
//////    pcl::IndicesPtr indices (new std::vector <int>);
//////    pcl::PassThrough<pcl::PointXYZRGB> pass;
//////    pass.setInputCloud (cloud);
//////    pass.setFilterFieldName ("z");
//////    pass.setFilterLimits (0.0, 1.0);
//////    pass.filter (*indices);
//////
//////    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//////    reg.setInputCloud (cloud);
//////    reg.setIndices (indices);
//////    reg.setSearchMethod (tree);
//////    reg.setDistanceThreshold (10);
//////    reg.setPointColorThreshold (6);
//////    reg.setRegionColorThreshold (5);
//////    reg.setMinClusterSize (600);
//////
//////    std::vector <pcl::PointIndices> clusters;
//////    reg.extract (clusters);
//////
//////    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//////    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//////    viewer.showCloud (colored_cloud);
//////    while (!viewer.wasStopped ())
//////    {
//////        std::this_thread::sleep_for(100us);
//////    }
//////
//////    return (0);
//////}
////#include <iostream>
////#include <vector>
////#include <pcl/io/pcd_io.h>
////#include <pcl/point_types.h>
////#include <pcl/visualization/cloud_viewer.h>
////#include <pcl/filters/passthrough.h>
////#include <pcl/segmentation/min_cut_segmentation.h>
////
////int main (int argc, char** argv)
////{
////    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
////    if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("min_cut_segmentation_tutorial.pcd", *cloud) == -1 )
////    {
////        std::cout << "Cloud reading failed." << std::endl;
////        return (-1);
////    }
////
////    pcl::IndicesPtr indices (new std::vector <int>);
////    pcl::PassThrough<pcl::PointXYZ> pass;
////    pass.setInputCloud (cloud);
////    pass.setFilterFieldName ("z");
////    pass.setFilterLimits (0.0, 1.0);
////    pass.filter (*indices);
////
////    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
////    seg.setInputCloud (cloud);
////    seg.setIndices (indices);
////
////    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
////    pcl::PointXYZ point;
////    point.x = 68.97;
////    point.y = -18.55;
////    point.z = 0.57;
////    foreground_points->points.push_back(point);
////    seg.setForegroundPoints (foreground_points);
////
////    seg.setSigma (0.25);
////    seg.setRadius (3.0433856);
////    seg.setNumberOfNeighbours (14);
////    seg.setSourceWeight (0.8);
////
////    std::vector <pcl::PointIndices> clusters;
////    seg.extract (clusters);
////
////    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;
////
////    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
////    pcl::visualization::CloudViewer viewer ("Cluster viewer");
////    viewer.showCloud(colored_cloud);
////    while (!viewer.wasStopped ())
////    {
////    }
////
////    return (0);
////}
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/progressive_morphological_filter.h>
//
//int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointIndicesPtr ground (new pcl::PointIndices);
//
//    // Fill in the cloud data
//    pcl::PCDReader reader;
//    // Replace the path below with the path where you saved your file
//    reader.read<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);
//
//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;
//
//    // Create the filtering object
//    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
//    pmf.setInputCloud (cloud);
//    pmf.setMaxWindowSize (20);
//    pmf.setSlope (1.0f);
//    pmf.setInitialDistance (0.5f);
//    pmf.setMaxDistance (3.0f);
//    pmf.extract (ground->indices);
//
//    // Create the filtering object
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud (cloud);
//    extract.setIndices (ground);
//    extract.filter (*cloud_filtered);
//
//    std::cerr << "Ground cloud after filtering: " << std::endl;
//    std::cerr << *cloud_filtered << std::endl;
//
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);
//
//    // Extract non-ground returns
//    extract.setNegative (true);
//    extract.filter (*cloud_filtered);
//
//    std::cerr << "Object cloud after filtering: " << std::endl;
//    std::cerr << *cloud_filtered << std::endl;
//
//    writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);
//
//    return (0);
//}
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>

int
main ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Generate cloud data
    int noise_size = 5;
    int sphere_data_size = 10;
    cloud->width = noise_size + sphere_data_size;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    // 1.1 Add noise
    for (std::size_t i = 0; i < noise_size; ++i)
    {
        (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    // 1.2 Add sphere:
    double rand_x1 = 1;
    double rand_x2 = 1;
    for (std::size_t i = noise_size; i < noise_size + sphere_data_size; ++i)
    {
        // See: http://mathworld.wolfram.com/SpherePointPicking.html
        while (pow (rand_x1, 2) + pow (rand_x2, 2) >= 1)
        {
            rand_x1 = (rand () % 100) / (50.0f) - 1;
            rand_x2 = (rand () % 100) / (50.0f) - 1;
        }
        double pre_calc = sqrt (1 - pow (rand_x1, 2) - pow (rand_x2, 2));
        (*cloud)[i].x = 2 * rand_x1 * pre_calc;
        (*cloud)[i].y = 2 * rand_x2 * pre_calc;
        (*cloud)[i].z = 1 - 2 * (pow (rand_x1, 2) + pow (rand_x2, 2));
        rand_x1 = 1;
        rand_x2 = 1;
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto& point: *cloud)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    // 2. filter sphere:
    // 2.1 generate model:
    // modelparameter for this sphere:
    // position.x: 0, position.y: 0, position.z:0, radius: 1
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize (4);
    sphere_coeff.values[0] = 0;
    sphere_coeff.values[1] = 0;
    sphere_coeff.values[2] = 0;
    sphere_coeff.values[3] = 1;

    pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
    sphere_filter.setModelCoefficients (sphere_coeff);
    sphere_filter.setThreshold (0.05);
    sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
    sphere_filter.setInputCloud (cloud);
    sphere_filter.filter (*cloud_sphere_filtered);

    std::cerr << "Sphere after filtering: " << std::endl;
    for (const auto& point: *cloud_sphere_filtered)
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);
}