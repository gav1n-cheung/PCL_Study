////#include <pcl/point_types.h>
////#include <pcl/io/pcd_io.h>
////#include <pcl/kdtree/kdtree_flann.h>
////#include <pcl/surface/mls.h>
////
////int
////main (int argc, char** argv)
////{
////    // Load input file into a PointCloud<T> with an appropriate type
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
////    // Load bun0.pcd -- should be available with the PCL archive in test
////    pcl::io::loadPCDFile ("bun0.pcd", *cloud);
////
////    // Create a KD-Tree
////    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
////
////    // Output has the PointNormal type in order to store the normals calculated by MLS
////    pcl::PointCloud<pcl::PointNormal> mls_points;
////
////    // Init object (second point type is for the normals, even if unused)
////    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
////
////    mls.setComputeNormals (true);
////
////    // Set parameters
////    mls.setInputCloud (cloud);
////    mls.setPolynomialOrder (2);
////    mls.setSearchMethod (tree);
////    mls.setSearchRadius (0.03);
////
////    // Reconstruct
////    mls.process (mls_points);
////
////    // Save output
////    pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
////}
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
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);
//
//    seg.setInputCloud (cloud_filtered);
//    seg.segment (*inliers, *coefficients);
//    std::cerr << "PointCloud after segmentation has: "
//              << inliers->indices.size () << " inliers." << std::endl;
//
//    // Project the model inliers
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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*，该方法需要法线，因此使用PCL的标准方法对其进行估计
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*，由于坐标和法线需要在同一个点云中，我们创建一个PointNormal类型的点云
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)，设定搜索半径
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters设定参数值
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result设定输入对象、搜索方法、并且最终将结果给到triangles
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information,对于每个点，可以检索包含连接组件的ID及其"状态"
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    pcl::io::saveVTKFile ("mesh.vtk", triangles);
    // Finish
    return (0);
}