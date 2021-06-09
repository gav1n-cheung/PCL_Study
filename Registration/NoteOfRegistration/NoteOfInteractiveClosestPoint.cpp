//
// Created by cheung on 2021/6/9.
//
//#include <iostream>
//#include <string>
//
//#include <pcl/io/ply_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/time.h>
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//bool next_iteration= false;
////打印刚性变换矩阵
//void print4X4Matrix(const Eigen::Matrix4Xd & matrix){
//    printf("Rotation matrix :\n");
//    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
//    printf ("Translation vector :\n");
//    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
//}
////查看器的回调。当查看器窗口位于顶部时，只要按下某个键，都会调用此函数。如果空格键被按下，则将bool值设为真。
//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent event,
//                           void* nothing ){
//    if (event.getKeySym()=="space"&&event.keyDown()) next_iteration= true;
//}
//int main(int argc,char * argv[]) {
//    //用于存储数据的3个点云
//    PointCloudT::Ptr cloud_in(new PointCloudT);
//    PointCloudT::Ptr cloud_tr(new PointCloudT);
//    PointCloudT::Ptr cloud_icp(new PointCloudT);
//    //检查程序的参数，设置初始ICP迭代次数并尝试加载PLY文件
//    if (argc < 2) {
//        printf("Usage :\n");
//        printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
//        PCL_ERROR ("Provide one ply file.\n");
//        return (-1);
//    }
//    int iterations = 1;
//    if (argc > 2) {
//        iterations = atoi(argv[2]);
//        if (iterations < 1) {
//            PCL_ERROR("Number of initial iteration must be >=1\n");
//            return (-1);
//        }
//    }
//    pcl::console::TicToc time;
//    time.tic();
//    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0) {
//        PCL_ERROR("Error loading cloud %s.\n", argv[1]);
//        return (-1);
//    }
//    std::cout << "\nLoaded file " << argv[1] << "(" << cloud_in->size() << "points) in " << time.toc() << "ms \n"
//              << std::endl;
//
//
//    //使用刚性矩阵变换来变换原始点云。其中cloud_in是原始点云，cloud_tr是平移的点云，cloud_icp是旋转的点云,cloud_tr是我们将用于显示的备份
//    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//
//    double theta = M_PI / 8;
//    transformation_matrix(0, 0) = std::cos(theta);
//    transformation_matrix(0, 1) = -sin(theta);
//    transformation_matrix(1, 0) = sin(theta);
//    transformation_matrix(1, 1) = std::cos(theta);
//    transformation_matrix(2, 3) = 0.4;
//    std::cout << "Applying this rigid transformation to :cloud_in -> cloud_icp" << std::endl;
//    print4X4Matrix(transformation_matrix);
//
//    pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
//    *cloud_tr = *cloud_icp;
//
//    //创建ICP对象，我们设置ICP算法的参数。setMaximumIterations(iterations)设置要执行的初始迭代次数(默认值=1).
//    //然后我们将点云转换为cloud_icp.当第一次对齐之后，我们将ICP最大迭代次数设置为1，以便下次使用ICP对象。
//    time.tic();
//    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    icp.setMaximumIterations(iterations);
//    icp.setInputSource(cloud_icp);
//    icp.setInputTarget(cloud_in);
//    icp.align(*cloud_icp);
//    icp.setMaximumIterations(1);
//    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << "ms" << std::endl;
//    //检查ICP算法是否收敛，若不收敛，则跳出程序。如果成功，我们将变换矩阵存储在4*4矩阵中，然后打印刚性变换矩阵。
//    if (icp.hasConverged()) {
//        std::cout << "\n ICP has converged,score is " << icp.getFitnessScore() << std::endl;
//        std::cout << "\n ICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix = icp.getFinalTransformation().cast<double>();
//        print4X4Matrix(transformation_matrix);
//    } else {
//        PCL_ERROR("\n ICP has not converged.\n");
//        return (-1);
//    }
//    //可视化
//    pcl::visualization::PCLVisualizer viewer("ICP demo");
//
//    int v1(0);
//    int v2(0);
//    //创建两个窗口
//    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort(0.5, 0.0, 0.5, 1.0, v2);
//    //我们在垂直分离的可视化器中创建两个窗口。bckgr_gray_level和txt_gray_lvl是变量，可以轻松地从白色背景和黑色文本/点云切换到黑色背景和白色文本/点云。
//    float bckgr_gray_level = 0.0;
//    float txt_gray_lvl = 1.0 - bckgr_gray_level;
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int) 255 * txt_gray_lvl,
//                                                                              (int) 255 * txt_gray_lvl,
//                                                                              (int) 255 * txt_gray_lvl);
//    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 100, 20);
//    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
//
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_tr, 100, 20, 20);
//    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_icp_v1", v2);
//    //我们为每个窗口的点云添加描述，便于用户理解。我们需要字符串流ss将整数迭代转换为字符串。
//    viewer.addText("White: Original point cloud\nGreen:Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl,
//                   txt_gray_lvl, "icp_info_1", v1);
//    viewer.addText("White: Original point cloud\nRed:ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl,
//                   "icp_info_2", v2);
//
//    std::stringstream ss;
//    ss << iterations;
//    std::string iterations_cnt = "ICP iterations = " + ss.str();
//    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iteration_cnt", v2);
//    //我们根据bckgr_gray_lvl设置两个窗口的背景颜色。要获取相机参数，我们只需要在窗口中按下"C"即可。
//    //我们将参数复制到此函数中以保存相机位置/方向/焦点。当查看器窗口位于顶部时，函数registerKeyboardCallback允许我们在用户按下键盘时调用函数。
//    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
//    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024);
//    viewer.registerKeyboardCallback(
//            reinterpret_cast<void (*)(const pcl::visualization::KeyboardEvent &, void *)>(&keyboardEventOccurred),
//            (void *) NULL);
//    while (!viewer.wasStopped()) {
//        viewer.spinOnce();//如果没有按下任何按键，这是正常行为，用户等待退出
//        if (next_iteration) {
//            time.tic();//如果用户按下键盘的任意键，则调用KeboardEventOccurred函数，这个函数会检查输入的键是否为空格。
//            //如果是，则将bool next_iteration设置为true,允许查看器循环进入代码的下一部分：调用ICP对象来对齐网格。
//            //我们已经配置了此对象的输入/出点云。并且我们设置了最大迭代次数为1.
//            icp.align(*cloud_icp);
//            std::cout << "Applied 1 ICP iteration in " << time.toc() << "ms" << std::endl;
//            if (icp.hasConverged()) {
//
//                printf("\033[11A");  // Go up 11 lines in terminal output.
//                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//                print4X4Matrix(
//                        transformation_matrix);  // Print the transformation between original pose and current pose
//
//                ss.str("");
//                ss << iterations;
//                std::string iterations_cnt = "ICP iterations = " + ss.str();
//                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,
//                                  "iterations_cnt");
//                viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//            } else {
//                PCL_ERROR("\n ICP has converged.\n");
//                return (-1);
//            }
//            //和往常一样，我们检查ICP是否收敛，如果不收敛，我们就退出程序。
//            //printf("\033[11A");在终端中上升11行从而覆盖显示的最后的一个矩阵。简而言之，它允许替换文本而非写新的行，使输出更具
//            //可读性。我们可以增加迭代来更新可视化器中的文本值。
//            //我们要显示从原始点云变换到ICP所做的当前对齐的刚性变换。
//            //函数getFinalTransformation()返回在迭代期间完成的刚性变换。这意味着您已经进行了10次迭代，
//            // 此函数将返回矩阵来将点云从迭代10转换为11
//        }
//        next_iteration = false;
//    }
//    return (0);
//}









































