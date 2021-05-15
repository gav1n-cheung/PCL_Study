//
// Created by cs18 on 5/15/21.
//


/*估计一组点的VFH签名
本文档描述了视点特征直方图（VFH）描述符，他是针对于点簇的新的表现形式，用于点簇识别和6DOF姿势估计的问题。
下图显示了VFH识别和姿势估计的示例。给定一组训练数据，然后使用点云查询/测试模型，见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation）图1

理论基础：
视点特征直方图（VFH）的根源是FPFH，由于其速度和判别能力，我们决定利用FPFH强大的识别结果，但在保留视点不变性的同时增加视点方差。
我们对物体识别和姿势识别的问题做出了改进：扩展了整个物体簇的FPFH估计值，并计算了视点方向在每个点处估计的法线之间的附加统计量。为此，我们使用了将视点方向直接混合到FPFH中的相对
法线角度计算中的关键思想。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图2）
通过收集视点方向与每个法线所成角度的直方图来计算视点分量。需要注意的是，我们并不是指每个法线的视角，因为不会是比例不变的，而是指中心视点方向之间平移到每个法线的角度，这是VFH的第一部分，
而第二部分则是按照FPFH测量 relative pan, tilt and yaw angles，但现在在中心点的视点方向与曲面上的每个法线之间进行测量。
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图3）
因此，新的部分特征称为VFH，下图通过包含两个部分的新功能展示了此设想：
（1）视点方向分量
（2）由拓展的FPFH组成的表面形状分量 
见（https://pcl.readthedocs.io/projects/tutorials/en/latest/vfh_estimation.html#vfh-estimation 图4）












































*/

