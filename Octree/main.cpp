//#include <pcl/point_cloud.h>
//#include <pcl/octree/octree_search.h>
//
//#include <iostream>
//#include <vector>
//#include <ctime>
//
//int
//main (int argc, char** argv)
//{
//    srand ((unsigned int) time (NULL));
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Generate pointcloud data
//    cloud->width = 1000;
//    cloud->height = 1;
//    cloud->points.resize (cloud->width * cloud->height);
//
//    for (std::size_t i = 0; i < cloud->size (); ++i)
//    {
//        (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//        (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//        (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    }
//
//    float resolution = 128.0f;
//
//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
//
//    octree.setInputCloud (cloud);
//    octree.addPointsFromInputCloud ();
//
//    pcl::PointXYZ searchPoint;
//
//    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
//    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);
//
//    // Neighbors within voxel search
//
//    std::vector<int> pointIdxVec;
//
//    if (octree.voxelSearch (searchPoint, pointIdxVec))
//    {
//        std::cout << "Neighbors within voxel search at (" << searchPoint.x
//                  << " " << searchPoint.y
//                  << " " << searchPoint.z << ")"
//                  << std::endl;
//
//        for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
//            std::cout << "    " << (*cloud)[pointIdxVec[i]].x
//                      << " " << (*cloud)[pointIdxVec[i]].y
//                      << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
//    }
//
//    // K nearest neighbor search
//
//    int K = 10;
//
//    std::vector<int> pointIdxNKNSearch;
//    std::vector<float> pointNKNSquaredDistance;
//
//    std::cout << "K nearest neighbor search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with K=" << K << std::endl;
//
//    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//    {
//        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//            std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x
//                      << " " << (*cloud)[ pointIdxNKNSearch[i] ].y
//                      << " " << (*cloud)[ pointIdxNKNSearch[i] ].z
//                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
//    }
//
//    // Neighbors within radius search
//
//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;
//
//    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
//
//    std::cout << "Neighbors within radius search at (" << searchPoint.x
//              << " " << searchPoint.y
//              << " " << searchPoint.z
//              << ") with radius=" << radius << std::endl;
//
//
//    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//    {
//        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//            std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x
//                      << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y
//                      << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z
//                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//    }
//
//}
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
    srand ((unsigned int) time (NULL));

    // Octree resolution - side length of octree voxels
    float resolution = 32.0f;

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );

    // Generate pointcloud data for cloudA
    cloudA->width = 128;
    cloudA->height = 1;
    cloudA->points.resize (cloudA->width * cloudA->height);

    for (std::size_t i = 0; i < cloudA->size (); ++i)
    {
        (*cloudA)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
        (*cloudA)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
        (*cloudA)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
    }

    // Add points from cloudA to octree
    octree.setInputCloud (cloudA);
    octree.addPointsFromInputCloud ();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );

    // Generate pointcloud data for cloudB
    cloudB->width = 128;
    cloudB->height = 1;
    cloudB->points.resize (cloudB->width * cloudB->height);

    for (std::size_t i = 0; i < cloudB->size (); ++i)
    {
        (*cloudB)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
        (*cloudB)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
        (*cloudB)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
    }

    // Add points from cloudB to octree
    octree.setInputCloud (cloudB);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
                  << (*cloudB)[newPointIdxVector[i]].y << " "
                  << (*cloudB)[newPointIdxVector[i]].z << std::endl;

}
