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

#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

int main(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPLYFile("bunny_D01_L01.ply", *cloud);
pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;//新建点云法线估计变量ne
ne.setInputCloud(cloud);//设定点云法线估计输入
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ> ());//创建一个空的kdtree表示形式，并将其传递给法线估计对象，
//它的内容将根据给定的输入数据集填充到对象内部（因为并没有其他可供搜索的表面）
ne.setSearchMethod(tree1);
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//创建新的点云类型变量normals作为输出变量
ne.setRadiusSearch(0.03);//设定临近点搜索半径为0.03m
ne.compute(*normals);//计算发现的结果存储于输出中

//  ... read, pass in or create a point cloud with normals ...
//  ... (note: you can create a single PointCloud<PointNormal> if you want) ...

  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (normals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    for (int i = 0; i < normals->size(); i++) {
        if (!pcl::isFinite<pcl::Normal>((*normals)[i])) {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
    }
  // Compute the features
  std::cout<<"start compute"<<endl;
  vfh.compute (*vfhs);
        std::cout<<"end compute"<<endl;

  // vfhs->size () should be of size 1*
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,normals);
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*vfhs,300);
    plotter.plot();
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}