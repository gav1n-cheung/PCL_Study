//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/centroid.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//
////点云法线估计，该方法的数据集是在terminal中通过命令行读入的，请编译后运行，或在clion中配置运行参数
//
//int main(int argc,char **argv) {
//    //读入点云数据
//    std::vector<int> filenames;
//    filenames=pcl::console::parse_file_extension_argument(argc,argv,".ply");
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile(argv[filenames[0]], *cloud);
//
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
//    ne.setInputCloud(cloud);//设定点云法线估计输入
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
//    //它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
//    ne.setSearchMethod(tree);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
//    ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
//    ne.compute(*cloud_normals);//计算发现的结果存储于输出中
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));//新建viewer
////    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");//将原始点云置入viewer
//    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 20, 0.03, "normals");//将法线点云数据置入viewer
//    while (!viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
//}

//积分图像进行法线估计，请将上面的main函数注释掉，再将下面的main函数注释去掉后运行该程序

////
//// Created by cs18 on 5/13/21.
////
//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/features/integral_image_normal.h>
//
//int
//main ()
//{
//    // load point cloud,加载数据集
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);
//
//    // estimate normals，估计法线
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//
//    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//    ne.setMaxDepthChangeFactor(0.02f);
//    ne.setNormalSmoothingSize(10.0f);
//    ne.setInputCloud(cloud);
//    ne.compute(*normals);
//
//    // visualize normals
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0.0, 0.0, 0.5);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
//
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce ();
//    }
//    return 0;
//}

//点特征直方图的计算，请将上方的main函数注释掉，再将下面的代码注释取消
//
//#include <pcl/point_types.h>
//#include <pcl/features/pfh.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//
//
//int main(){
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
//pcl::io::loadPLYFile("bun000.ply", *cloud);
//pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//ne.setMaxDepthChangeFactor(0.02f);
//ne.setNormalSmoothingSize(10.0f);
//ne.setInputCloud(cloud);
//ne.compute(*normals);
//
//
////... read, pass in or create a point cloud with normals ...
////... (note: you can create a single PointCloud<PointNormal> if you want) ...
//
//// Create the PFH estimation class, and pass the input dataset+normals to it
//pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
//pfh.setInputCloud (cloud);
//pfh.setInputNormals (normals);
//// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
//
//// Create an empty kdtree representation, and pass it to the PFH estimation object.
//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
////pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
//pfh.setSearchMethod (tree);
//
//// Output datasets
//pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
//
//// Use all neighbors in a sphere of radius 5cm
//// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//pfh.setRadiusSearch (0.05);
//    std::cout<<"pfhs start compute!"<<endl;
//// Compute the features
//pfh.compute (*pfhs);
//std::cout<<"pfhs computed!"<<endl;
//
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0.0, 0.0, 0.5);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
//    pcl::visualization::PCLPlotter plotter;
//    plotter.addFeatureHistogram(*pfhs,300);
//    plotter.plot();
//
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce ();
//    }
//    return 0;
//
//// pfhs->size () should have the same size as the input cloud->size ()*
//}
//#include <pcl/point_types.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/point_types.h>
//#include <pcl/features/pfh.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/features/integral_image_normal.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/centroid.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//int main() {
//
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("bunny_D01_L01.ply", *cloud);
//    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
//    ne.setInputCloud(cloud);//设定点云法线估计输入
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
////它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
//    ne.setSearchMethod(tree1);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
//    ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
//    ne.compute(*cloud_normals);//计算发现的结果存储于输出中
//
////... read, pass in or create a point cloud with normals ...
////... (note: you can create a single PointCloud<PointNormal> if you want) ...
//
//// Create the FPFH estimation class, and pass the input dataset+normals to it
//pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//fpfh.setInputCloud (cloud);
//fpfh.setInputNormals (cloud_normals);
//// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
//
//// Create an empty kdtree representation, and pass it to the FPFH estimation object.
//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//
//fpfh.setSearchMethod (tree);
//
//// Output datasets
//pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
//
//// Use all neighbors in a sphere of radius 5cm
//// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//fpfh.setRadiusSearch (0.05);
//
//std::cout<<"start compute "<<endl;
//    for (int i = 0; i < cloud_normals->size(); i++)
//    {
//        if (!pcl::isFinite<pcl::Normal>((*cloud_normals)[i]))
//        {
//            PCL_WARN("normals[%d] is not finite\n", i);
//        }
//    }
//// Compute the features
//fpfh.compute (*fpfhs);
//    std::cout<<"end compute "<<endl;
//// fpfhs->size () should have the same size as the input cloud->size ()*
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0.0, 0.0, 0.5);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals);
//    pcl::visualization::PCLPlotter plotter;
//    plotter.addFeatureHistogram(*fpfhs,300);
//    plotter.plot();
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce ();
//    }
//    return 0;
//}
//
//#include <pcl/point_types.h>
//#include <pcl/features/vfh.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
//
//int main(){
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::io::loadPLYFile("bunny_D01_L01.ply", *cloud);
//pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
//ne.setInputCloud(cloud);//设定点云法线估计输入
//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
////它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
//ne.setSearchMethod(tree1);
//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
//ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
//ne.compute(*normals);//计算发现的结果存储于输出中
//
////  ... read, pass in or create a point cloud with normals ...
////  ... (note: you can create a single PointCloud<PointNormal> if you want) ...
//
//  // Create the VFH estimation class, and pass the input dataset+normals to it
//  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
//  vfh.setInputCloud (cloud);
//  vfh.setInputNormals (normals);
//  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);
//
//  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//  vfh.setSearchMethod (tree);
//
//  // Output datasets
//  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
//    for (int i = 0; i < normals->size(); i++) {
//        if (!pcl::isFinite<pcl::Normal>((*normals)[i])) {
//            PCL_WARN("normals[%d] is not finite\n", i);
//        }
//    }
//  // Compute the features
//  std::cout<<"start compute"<<endl;
//  vfh.compute (*vfhs);
//  std::cout<<"end compute"<<endl;
//
//  // vfhs->size () should be of size 1*
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0.0, 0.0, 0.5);
//    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
//    pcl::visualization::PCLPlotter plotter;
//    plotter.addFeatureHistogram(*vfhs,300);
//    plotter.plot();
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce ();
//    }
//    return 0;
//}
/* \author Bastian Steder */

#include <iostream>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
    std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
              << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
              << "-m           Treat all unseen points to max range\n"
              << "-s <float>   support size for the interest points (diameter of the used sphere - "
                 "default "<<support_size<<")\n"
              << "-o <0/1>     switch rotational invariant version of the feature on/off"
              <<               " (default "<< (int)rotation_invariant<<")\n"
              << "-h           this help\n"
              << "\n\n";
}

void
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                              look_at_vector[0], look_at_vector[1], look_at_vector[2],
                              up_vector[0], up_vector[1], up_vector[2]);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage (argv[0]);
        return 0;
    }
    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
        std::cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
    int tmp_coordinate_frame;
    if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
        std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
    }
    if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
        std::cout << "Setting support size to "<<support_size<<".\n";
    if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
        std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
    angular_resolution = pcl::deg2rad (angular_resolution);

    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
    if (!pcd_filename_indices.empty ())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
        {
            std::cerr << "Was not able to open file \""<<filename<<"\".\n";
            printUsage (argv[0]);
            return 0;
        }
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                                   point_cloud.sensor_origin_[1],
                                                                   point_cloud.sensor_origin_[2])) *
                            Eigen::Affine3f (point_cloud.sensor_orientation_);
        std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
        if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
            std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
    }
    else
    {
        setUnseenToMaxRange = true;
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x=-0.5f; x<=0.5f; x+=0.01f)
        {
            for (float y=-0.5f; y<=0.5f; y+=0.01f)
            {
                PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
                point_cloud.points.push_back (point);
            }
        }
        point_cloud.width = point_cloud.size ();  point_cloud.height = 1;
    }

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters ();
    setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices[i]%range_image.width,
    //keypoint_indices[i]/range_image.width);

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
    keypoints.resize (keypoint_indices.size ());
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
        keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> keypoint_indices2;
    keypoint_indices2.resize (keypoint_indices.size ());
    for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
        keypoint_indices2[i]=keypoint_indices[i];
    pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
    narf_descriptor.getParameters ().support_size = support_size;
    narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36> narf_descriptors;
    narf_descriptor.compute (narf_descriptors);
    std::cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
              <<keypoint_indices.size ()<< " keypoints.\n";

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }
}
//#include <vector>
//#include <thread>
//
//#include <pcl/features/moment_of_inertia_estimation.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std::chrono_literals;
//
//int main (int argc, char** argv)
//{
//    if (argc != 2)
//        return (0);
////读入点云
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//    if (pcl::io::loadPCDFile (argv[1], *cloud) == -1)
//        return (-1);
////实例化MomentOfInertiaEstimation
//    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
//    feature_extractor.setInputCloud (cloud);
//    feature_extractor.compute ();
////声明变量
//    std::vector <float> moment_of_inertia;
//    std::vector <float> eccentricity;
//    pcl::PointXYZ min_point_AABB;
//    pcl::PointXYZ max_point_AABB;
//    pcl::PointXYZ min_point_OBB;
//    pcl::PointXYZ max_point_OBB;
//    pcl::PointXYZ position_OBB;
//    Eigen::Matrix3f rotational_matrix_OBB;
//    float major_value, middle_value, minor_value;
//    Eigen::Vector3f major_vector, middle_vector, minor_vector;
//    Eigen::Vector3f mass_center;
////如何访问计算的描述符和其他功能
//    feature_extractor.getMomentOfInertia (moment_of_inertia);
//    feature_extractor.getEccentricity (eccentricity);
//    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
//    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
//    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
//    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
//    feature_extractor.getMassCenter (mass_center);
////创建PCLVisualizer用于结果可视化的类的实例。在这里，我们还添加了云和用于可视化的AABB。我们设置渲染属性，以便使用线框显示多维数据库，因为默认情况下使用实心多维数据集。
//    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
////OBB的可视化稍微复杂一些。因此，这里我们从旋转矩阵创建一个四元数，设置OBB的位置，并将其传递给可视化窗口。
//    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat (rotational_matrix_OBB);
//    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
////启动可视化，特征向量的可视化
//    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
//    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
//    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
//    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
//    viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
//    viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
//    viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
//
//    while(!viewer->wasStopped())
//    {
//        viewer->spinOnce (100);
//        std::this_thread::sleep_for(100ms);
//    }
//
//    return (0);
//}
#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    if (argc != 4)
        return (-1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    if (pcl::io::loadPCDFile (argv[1], *cloud) == -1){
            std::cout<<"输入点云数据失败"<<std::endl;
            return (-1);
    }

    pcl::PointIndicesPtr indices (new pcl::PointIndices);
    std::ifstream indices_file;
    indices_file.open (argv[2], std::ifstream::in);
    for (std::string line; std::getline (indices_file, line);)
    {
        std::istringstream in (line);
        unsigned int index = 0;
        in >> index;
        indices->indices.push_back (index - 1);
    }
    indices_file.close ();

    std::vector <pcl::Vertices> triangles;
    std::ifstream triangles_file;
    triangles_file.open (argv[3], std::ifstream::in);
    for (std::string line; std::getline (triangles_file, line);)
    {
        pcl::Vertices triangle;
        std::istringstream in (line);
        unsigned int vertex = 0;
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);
        in >> vertex;
        triangle.vertices.push_back (vertex - 1);
        triangles.push_back (triangle);
    }

    float support_radius = 0.0285f;
    unsigned int number_of_partition_bins = 5;
    unsigned int number_of_rotations = 3;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method (new pcl::search::KdTree<pcl::PointXYZ>);
    search_method->setInputCloud (cloud);

    pcl::ROPSEstimation <pcl::PointXYZ, pcl::Histogram <135> > feature_estimator;
    feature_estimator.setSearchMethod (search_method);
    feature_estimator.setSearchSurface (cloud);
    feature_estimator.setInputCloud (cloud);
    feature_estimator.setIndices (indices);
    feature_estimator.setTriangles (triangles);
    feature_estimator.setRadiusSearch (support_radius);
    feature_estimator.setNumberOfPartitionBins (number_of_partition_bins);
    feature_estimator.setNumberOfRotations (number_of_rotations);
    feature_estimator.setSupportRadius (support_radius);

    pcl::PointCloud<pcl::Histogram <135> >::Ptr histograms (new pcl::PointCloud <pcl::Histogram <135> > ());
    feature_estimator.compute (*histograms);

    pcl::visualization::PCLVisualizer viewer ("pcl-viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud);

    std::cout<<"compute over"<<std::endl;

    return (0);
}