#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // 点云坐标变换
#include <pcl/visualization/pcl_visualizer.h>

// 如果使用者没有提供预期的参数，则该函数在terminal中打印出帮助信息
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int main(int argc, char **argv)
{

    // Show help 展示帮助信息
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    // 我们在参数中寻找.ply或者.pcd文件，
    std::vector<int> filenames;//创建输入的文件名
    bool file_is_pcd = false;//创建判断位来获取输入文件的格式

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");//解析文件拓展名信息，返回一个带有文件名索引的向量

    if (filenames.size() != 1)//如果filenames的元素数不为1，即没有找到相应的文件名，则执行以下代码
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");//再次解析文件拓展名，返回一个带有文件名索引的向量
        if (filenames.size() != 1)//如果还是为空，则显示帮助信息，退出程序
        {
            showHelp(argv[0]);
            return -1;
        }
        else//否则，则将bool标志位置为true，则我们通过这一标志位就得知了，文件是.pcd格式的。
        {
            file_is_pcd = true;
        }
    }

    // Load file | Works with PCD and PLY files 在参数中寻找pcd或者ply文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());//新建一个PointXYZ格式的点云类型

    if (file_is_pcd)//如果文件为.pcd格式的
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)//将文件读入source_cloud内，如果点云数量小于0，则在terminal输出错误信息和帮助信息，退出程序
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    else//如果文件为.ply格式的，做出类似于.pcd处理的操作
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }

    /* Reminder: how transformation matrices work（变换矩阵的工作原理） :
           |-------> This column is the translation,最后一列就是移动变换
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left，首先在最左侧定义一个3x3单位矩阵，他们是旋转矩阵
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)，我们不适用最后一行，通常都是置为(0,0,0,1)
    METHOD #1: Using a Matrix4f ====================================
    This is the "manual" method, perfect to understand but error prone !这是一种“手动”的方法，很容易理解却比较容易出错
  */
    // 创建4x4单位阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)，定义一个旋转矩阵，它的一种常见形式如连接wiki所示
    float theta = M_PI / 4; // The angle of rotation in radians，旋转角度设定为pi/4，即45°
    transform_1(0, 0) = std::cos(theta);
    transform_1(0, 1) = -sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = std::cos(theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.在x轴上定义一个2.5米的变换
    transform_1(0, 3) = 2.5;

    // Print the transformation，在terminal中打印输出，我们使用了方法1--使用4x4变换矩阵,并且将矩阵输出出来，我们可以看到，新的点云数据向右平移了2.5M,并且绕z轴向左旋转了45°
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f ====================================第二种方法
    This method is easier and less error prone，这种方法更简单，并且更不容易出错
  */
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();//定义一个4*4的齐次矩阵变换

    // Define a translation of 2.5 meters on the x axis.首先在x平移2.5m
    transform_2.translation() << 2.5, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis，和以前的变换，但我们这次可以很简单的指定旋转角度，旋转轴等参数（基于Eigen库）
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));//

    // Print the transformation，打印出使用的方法和使用的矩阵
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation，将变换提取出来，创建一个新点云数据来可视化结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // You can either apply transform_1 or transform_2; they are the same，你可以选择变换一或变换二来变换原始点云数据，他们的结果是相同，变换矩阵也是相同的，但实现起来明显变换二更加方便
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

    // Visualization  可视化
    printf("\nPoint cloud colors :  white  = original point cloud\n"
           "                        red  = transformed point cloud\n");//在terminal中输出：白色为原始点云，红色为变换后点云
    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");//将可视化窗口命名为该字符串

    // Define R,G,B colors for the point cloud，为点云上色，原始点云设为白色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler，我们将点云添加进可视化窗口，并且处理点云颜色
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red，将变换后点云设为红色
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);//添加坐标系到可视化窗口
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey，将可视化窗口背景颜色设为黑色
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");//设置点云渲染属性：点云尺寸设置为2
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");//同上
    //viewer.setPosition(800, 400); // Setting visualiser window position

    //按Q退出点云可视化界面
    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return 0;
}