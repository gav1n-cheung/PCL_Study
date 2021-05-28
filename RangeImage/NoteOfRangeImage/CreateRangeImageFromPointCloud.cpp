//
// Created by cs18 on 5/28/21.
//
/*如何从点云创建范围图像
本教程演示了如何从点云和给定的传感器位置创建距离图像，该代码创建了一个示例矩形点云，该矩形浮在观察者的前面
*/
#include <pcl/range_image/range_image.h>

int mainFun1 (int argc, char** argv) {
    //创建新的点云数据
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  
  // Generate the data，填充数据
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }
  pointCloud.width = pointCloud.size();
  pointCloud.height = 1;
  
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution，我们想要创建一张在点云前方的并且角分辨率为1度的距离图像
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians，分辨率为1°
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians横向最大角度为360度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians竖向最大角度为180度
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//将相机的位置定在原点处
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//告诉系统x朝右，y朝下，z轴超前。另一种选择是LASER_FRAME,其中x朝前，y朝左，z朝上。
  float noiseLevel=0.00;//使用普通的z缓冲区穿件距离图像。但是，如果要对落在同一单元格中的平均点进行平均，则可以使用更高的值。0.05表示，到最近点的最大距离为5cm的所有点都用于计算范围。
  float minRange = 0.0f;//如果minRange大于0，则所有更接近的点都被忽略
  int borderSize = 1;//如果borderSize大于0时将在图像周围留下未观察到的点的边界
  
  pcl::RangeImage rangeImage;
  //将设定的参数给到点云 创建图像
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";
}
