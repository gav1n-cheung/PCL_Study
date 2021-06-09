////#include <iostream>
////#include <pcl/io/pcd_io.h>
////#include <pcl/point_types.h>
////#include <pcl/registration/icp.h>
////#include <pcl/visualization/pcl_visualizer.h>
////
////int main(int argc,char** argv){
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5,1));
////    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
////
////    for (auto& point:*cloud_in ) {
////        point.x=1024*rand()/(RAND_MAX+1.0f);
////        point.y=1024*rand()/(RAND_MAX+1.0f);
////        point.z=1024*rand()/(RAND_MAX+1.0f);
////    }
////    std::cout<<"Saved "<<cloud_in->size()<<" data points to input:"<<std::endl;
////    for (auto& point:*cloud_in) std::cout<<point<<std::endl;
////
////    *cloud_out=*cloud_in;
////    std::cout<<"Transformed "<<cloud_in->size()<<" data points:"<<std::endl;
////    for(auto& point :*cloud_out ) point.x+=0.7f;
////    std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
////    for(auto& point :*cloud_out )  std::cout<<point<<std::endl;
////    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
////    icp.setInputSource(cloud_in);
////    icp.setInputTarget(cloud_out);
////
////    pcl::PointCloud<pcl::PointXYZ> Final;
////    icp.align(Final);
////
////    std::cout<<"has converged:"<<icp.hasConverged()<<"score:"<<
////             icp.getInputSource()<<std::endl;
////    std::cout<<icp.getFinalTransformation()<<std::endl;
////    pcl::io::savePCDFileASCII("final.pcd",Final);
////    return (0);
////}
//
//Docs » How to incrementally register pairs of clouds Edit on GitHub
//        How to incrementally register pairs of clouds
//        This document demonstrates using the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.
//
//The idea is to transform all the clouds in the first cloud’s frame.
//This is done by finding the best transform between each consecutive cloud, and accumulating these transforms over the whole set of clouds.
//Your data set should consist of clouds that have been roughly pre-aligned in a common frame (e.g. in a robot’s odometry or map frame) and overlap with one another.
//We provide a set of clouds at github.com/PointCloudLibrary/data/tree/master/tutorials/pairwise/.
//The code
///*
// * Software License Agreement (BSD License)
// *
// *  Copyright (c) 2010, Willow Garage, Inc.
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of Willow Garage, Inc. nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// * $Id$
// *
// */
//
///* \author Radu Bogdan Rusu
// * adaptation Raphael Favier*/
//
//#include <pcl/make_shared.h>  // for pcl::make_shared
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>
//
//#include <pcl/io/pcd_io.h>
//
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>
//
//#include <pcl/features/normal_3d.h>
//
//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
//#include <pcl/registration/transforms.h>
//
//#include <pcl/visualization/pcl_visualizer.h>
//
//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;
//
////convenient typedefs
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//
//// This is a tutorial so we can afford having global variables
////our visualizer
//pcl::visualization::PCLVisualizer *p;
////its left and right viewports
//int vp_1, vp_2;
//
////convenient structure to handle our pointclouds
//struct PCD
//{
//    PointCloud::Ptr cloud;
//    std::string f_name;
//
//    PCD() : cloud (new PointCloud) {};
//};
//
//struct PCDComparator
//{
//    bool operator () (const PCD& p1, const PCD& p2)
//    {
//        return (p1.f_name < p2.f_name);
//    }
//};
//
//
//// Define a new point representation for < x, y, z, curvature >
//class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
//{
//    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
//public:
//    MyPointRepresentation ()
//    {
//        // Define the number of dimensions
//        nr_dimensions_ = 4;
//    }
//
//    // Override the copyToFloatArray method to define our feature vector
//    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
//    {
//        // < x, y, z, curvature >
//        out[0] = p.x;
//        out[1] = p.y;
//        out[2] = p.z;
//        out[3] = p.curvature;
//    }
//};
//
//
//////////////////////////////////////////////////////////////////////////////////
///** \brief Display source and target on the first viewport of the visualizer
// *
// */
//void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
//{
//    p->removePointCloud ("vp1_target");
//    p->removePointCloud ("vp1_source");
//
//    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
//    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
//    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
//    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
//
//    PCL_INFO ("Press q to begin the registration.\n");
//    p-> spin();
//}
//
//
//////////////////////////////////////////////////////////////////////////////////
///** \brief Display source and target on the second viewport of the visualizer
// *
// */
//void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
//{
//    p->removePointCloud ("source");
//    p->removePointCloud ("target");
//
//
//    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
//    if (!tgt_color_handler.isCapable ())
//        PCL_WARN ("Cannot create curvature color handler!");
//
//    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
//    if (!src_color_handler.isCapable ())
//        PCL_WARN ("Cannot create curvature color handler!");
//
//
//    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
//    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
//
//    p->spinOnce();
//}
//
//////////////////////////////////////////////////////////////////////////////////
///** \brief Load a set of PCD files that we want to register together
//  * \param argc the number of arguments (pass from main ())
//  * \param argv the actual command line arguments (pass from main ())
//  * \param models the resultant vector of point cloud datasets
//  */
//void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
//{
//    std::string extension (".pcd");
//    // Suppose the first argument is the actual test model
//    for (int i = 1; i < argc; i++)
//    {
//        std::string fname = std::string (argv[i]);
//        // Needs to be at least 5: .plot
//        if (fname.size () <= extension.size ())
//            continue;
//
//        std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
//
//        //check that the argument is a pcd file
//        if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
//        {
//            // Load the cloud and saves it into the global list of models
//            PCD m;
//            m.f_name = argv[i];
//            pcl::io::loadPCDFile (argv[i], *m.cloud);
//            //remove NAN points from the cloud
//            std::vector<int> indices;
//            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
//
//            models.push_back (m);
//        }
//    }
//}
//
//
//////////////////////////////////////////////////////////////////////////////////
///** \brief Align a pair of PointCloud datasets and return the result
//  * \param cloud_src the source PointCloud
//  * \param cloud_tgt the target PointCloud
//  * \param output the resultant aligned source PointCloud
//  * \param final_transform the resultant transform between source and target
//  */
//void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
//{
//    //
//    // Downsample for consistency and speed
//    // \note enable this for large datasets
//    PointCloud::Ptr src (new PointCloud);
//    PointCloud::Ptr tgt (new PointCloud);
//    pcl::VoxelGrid<PointT> grid;
//    if (downsample)
//    {
//        grid.setLeafSize (0.05, 0.05, 0.05);
//        grid.setInputCloud (cloud_src);
//        grid.filter (*src);
//
//        grid.setInputCloud (cloud_tgt);
//        grid.filter (*tgt);
//    }
//    else
//    {
//        src = cloud_src;
//        tgt = cloud_tgt;
//    }
//
//
//    // Compute surface normals and curvature
//    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
//    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
//
//    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//    norm_est.setSearchMethod (tree);
//    norm_est.setKSearch (30);
//
//    norm_est.setInputCloud (src);
//    norm_est.compute (*points_with_normals_src);
//    pcl::copyPointCloud (*src, *points_with_normals_src);
//
//    norm_est.setInputCloud (tgt);
//    norm_est.compute (*points_with_normals_tgt);
//    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
//
//    //
//    // Instantiate our custom point representation (defined above) ...
//    MyPointRepresentation point_representation;
//    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
//    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
//    point_representation.setRescaleValues (alpha);
//
//    //
//    // Align
//    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
//    reg.setTransformationEpsilon (1e-6);
//    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
//    // Note: adjust this based on the size of your datasets
//    reg.setMaxCorrespondenceDistance (0.1);
//    // Set the point representation
//    reg.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));
//
//    reg.setInputSource (points_with_normals_src);
//    reg.setInputTarget (points_with_normals_tgt);
//
//
//
//    //
//    // Run the same optimization in a loop and visualize the results
//    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
//    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
//    reg.setMaximumIterations (2);
//    for (int i = 0; i < 30; ++i)
//    {
//        PCL_INFO ("Iteration Nr. %d.\n", i);
//
//        // save cloud for visualization purpose
//        points_with_normals_src = reg_result;
//
//        // Estimate
//        reg.setInputSource (points_with_normals_src);
//        reg.align (*reg_result);
//
//        //accumulate transformation between each Iteration
//        Ti = reg.getFinalTransformation () * Ti;
//
//        //if the difference between this transformation and the previous one
//        //is smaller than the threshold, refine the process by reducing
//        //the maximal correspondence distance
//        if (std::abs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
//            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
//
//        prev = reg.getLastIncrementalTransformation ();
//
//        // visualize current state
//        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
//    }
//
//    //
//    // Get the transformation from target to source
//    targetToSource = Ti.inverse();
//
//    //
//    // Transform target back in source frame
//    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
//
//    p->removePointCloud ("source");
//    p->removePointCloud ("target");
//
//    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
//    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
//    p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
//    p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);
//
//    PCL_INFO ("Press q to continue the registration.\n");
//    p->spin ();
//
//    p->removePointCloud ("source");
//    p->removePointCloud ("target");
//
//    //add the source to the transformed target
//    *output += *cloud_src;
//
//    final_transform = targetToSource;
//}
//
//
///* ---[ */
//int main (int argc, char** argv)
//{
//    // Load data
//    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
//    loadData (argc, argv, data);
//
//    // Check user input
//    if (data.empty ())
//    {
//        PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
//        PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
//        return (-1);
//    }
//    PCL_INFO ("Loaded %d datasets.", (int)data.size ());
//
//    // Create a PCLVisualizer object
//    p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
//    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
//
//    PointCloud::Ptr result (new PointCloud), source, target;
//    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
//
//    for (std::size_t i = 1; i < data.size (); ++i)
//    {
//        source = data[i-1].cloud;
//        target = data[i].cloud;
//
//        // Add visualization data
//        showCloudsLeft(source, target);
//
//        PointCloud::Ptr temp (new PointCloud);
//        PCL_INFO ("Aligning %s (%zu) with %s (%zu).\n", data[i-1].f_name.c_str (), static_cast<std::size_t>(source->size ()), data[i].f_name.c_str (), static_cast<std::size_t>(target->size ()));
//        pairAlign (source, target, temp, pairTransform, true);
//
//        //transform current pair into the global transform
//        pcl::transformPointCloud (*temp, *result, GlobalTransform);
//
//        //update the global transform
//        GlobalTransform *= pairTransform;
//
//        //save aligned pair, transformed into the first cloud's frame
//        std::stringstream ss;
//        ss << i << ".pcd";
//        pcl::io::savePCDFile (ss.str (), *result, true);
//
//    }
//}
///* ]--- */
#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration= false;
//打印刚性变换矩阵
void print4X4Matrix(const Eigen::Matrix4Xd & matrix){
    printf("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
//查看器的回调。当查看器窗口位于顶部时，只要按下某个键，都会调用此函数。如果空格键被按下，则将bool值设为真。
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent event,
                           void* nothing ){
    if (event.getKeySym()=="space"&&event.keyDown()) next_iteration= true;
}
int main(int argc,char * argv[]) {
    //用于存储数据的3个点云
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudT::Ptr cloud_tr(new PointCloudT);
    PointCloudT::Ptr cloud_icp(new PointCloudT);
    //检查程序的参数，设置初始ICP迭代次数并尝试加载PLY文件
    if (argc < 2) {
        printf("Usage :\n");
        printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR ("Provide one ply file.\n");
        return (-1);
    }
    int iterations = 1;
    if (argc > 2) {
        iterations = atoi(argv[2]);
        if (iterations < 1) {
            PCL_ERROR("Number of initial iteration must be >=1\n");
            return (-1);
        }
    }
    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0) {
        PCL_ERROR("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << "(" << cloud_in->size() << "points) in " << time.toc() << "ms \n"
              << std::endl;


    //使用刚性矩阵变换来变换原始点云。其中cloud_in是原始点云，cloud_tr是平移的点云，cloud_icp是旋转的点云,cloud_tr是我们将用于显示的备份
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    double theta = M_PI / 8;
    transformation_matrix(0, 0) = std::cos(theta);
    transformation_matrix(0, 1) = -sin(theta);
    transformation_matrix(1, 0) = sin(theta);
    transformation_matrix(1, 1) = std::cos(theta);
    transformation_matrix(2, 3) = 0.4;
    std::cout << "Applying this rigid transformation to :cloud_in -> cloud_icp" << std::endl;
    print4X4Matrix(transformation_matrix);

    pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;

    //创建ICP对象，我们设置ICP算法的参数。setMaximumIterations(iterations)设置要执行的初始迭代次数(默认值=1).
    //然后我们将点云转换为cloud_icp.当第一次对齐之后，我们将ICP最大迭代次数设置为1，以便下次使用ICP对象。
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_in);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << "ms" << std::endl;
    //检查ICP算法是否收敛，若不收敛，则跳出程序。如果成功，我们将变换矩阵存储在4*4矩阵中，然后打印刚性变换矩阵。
    if (icp.hasConverged()) {
        std::cout << "\n ICP has converged,score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\n ICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4X4Matrix(transformation_matrix);
    } else {
        PCL_ERROR("\n ICP has not converged.\n");
        return (-1);
    }
    //可视化
    pcl::visualization::PCLVisualizer viewer("ICP demo");

    int v1(0);
    int v2(0);
    //创建两个窗口
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 0.5, 1.0, v2);
    //我们在垂直分离的可视化器中创建两个窗口。bckgr_gray_level和txt_gray_lvl是变量，可以轻松地从白色背景和黑色文本/点云切换到黑色背景和白色文本/点云。
    float bckgr_gray_level = 0.0;
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int) 255 * txt_gray_lvl,
                                                                              (int) 255 * txt_gray_lvl,
                                                                              (int) 255 * txt_gray_lvl);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 100, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_tr, 100, 20, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_icp_v1", v2);
    //我们为每个窗口的点云添加描述，便于用户理解。我们需要字符串流ss将整数迭代转换为字符串。
    viewer.addText("White: Original point cloud\nGreen:Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl,
                   txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed:ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl,
                   "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iteration_cnt", v2);
    //我们根据bckgr_gray_lvl设置两个窗口的背景颜色。要获取相机参数，我们只需要在窗口中按下"C"即可。
    //我们将参数复制到此函数中以保存相机位置/方向/焦点。当查看器窗口位于顶部时，函数registerKeyboardCallback允许我们在用户按下键盘时调用函数。
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);
//    viewer.registerKeyboardCallback(
//            reinterpret_cast<void (*)(const pcl::visualization::KeyboardEvent &, void *)>(&keyboardEventOccurred),
//            (void *) NULL);
    while (!viewer.wasStopped()) {
        viewer.spinOnce();//如果没有按下任何按键，这是正常行为，用户等待退出
        if (next_iteration) {
            time.tic();//如果用户按下键盘的任意键，则调用KeboardEventOccurred函数，这个函数会检查输入的键是否为空格。
            //如果是，则将bool next_iteration设置为true,允许查看器循环进入代码的下一部分：调用ICP对象来对齐网格。
            //我们已经配置了此对象的输入/出点云。并且我们设置了最大迭代次数为1.
            icp.align(*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << "ms" << std::endl;
            if (icp.hasConverged()) {

                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4X4Matrix(
                        transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,
                                  "iterations_cnt");
                viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            } else {
                PCL_ERROR("\n ICP has converged.\n");
                return (-1);
            }
            //和往常一样，我们检查ICP是否收敛，如果不收敛，我们就退出程序。
            //printf("\033[11A");在终端中上升11行从而覆盖显示的最后的一个矩阵。简而言之，它允许替换文本而非写新的行，使输出更具
            //可读性。我们可以增加迭代来更新可视化器中的文本值。
            //我们要显示从原始点云变换到ICP所做的当前对齐的刚性变换。
            //函数getFinalTransformation()返回在迭代期间完成的刚性变换。这意味着您已经进行了10次迭代，
            // 此函数将返回矩阵来将点云从迭代10转换为11
        }
        next_iteration = false;
    }
    return (0);
}
