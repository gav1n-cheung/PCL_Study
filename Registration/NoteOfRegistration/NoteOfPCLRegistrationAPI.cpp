//
// Created by cheung on 2021/6/8.
//
/*PCL配准API
 将各种3D点云数据视图始终对齐到一个完整模型中的问题称为配准。它的目标是在全局坐标框架中找到单独获取的视图的相对位置和方向，使得它们
 之间的相交区域完美重叠。因此，对于从不同视图获取的每组点云数据集，我们需要一个能够将它们对齐到单个点云模型中的系统，以便可以应用
 后续处理步骤，例如分割和对象重建。
 （见 https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api 图1）
 图1使用倾斜的2D激光单元获取了一组六个单独的数据集。由于单独的扫描仪仅代表周围世界的一小部分，因此必须找到将它们注册到一起的方法，
 从而创建完整的点云模型，
 如https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api 图2
 PCL注册库中的算法工作的动机是在给定的输入数据集中找到正确的点对应关系，并且估计可以将每个单独数据集旋转和转换为一致的全局坐标框架
 的刚性变换。如果输入数据集中的点对应关系是完全已知的，那么这种配准范式就很容易解决。这意味着一个数据集中的选定点列表必须从特征表示的
 角度和另一个数据集中的点列表重合。此外，如果估计的对应关系是完美的，那么配准问题就有一个封闭形式的解决方案。

 PCL包含一组强大的算法，允许估计多组对应关系，并且包括拒绝不良对应关系的方法，并以稳健的方式从其推断估计变换。以下部分将分别介绍它们
 中的每一个。

 配准概述
 我们有时将一对点云数据配准在一起的问题称为成对配准，其输出通常是一个刚性变换矩阵（4*4），表示必须应用于其中一个数据集的旋转和平移
 (我们暂时称其为source-源)，从而使其与其他数据集完美对齐（我们称其为target/model-目标）。
 成对配准的执行步骤如下所示，
 见https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api 图3
 这里我们表示的是算法的一次迭代，我们可以决定循环任意或者所有步骤。
 步骤如下：
    （1）从一组点中，确定最能代表两个数据集中场景的兴趣点（即关键点-keypoints）
    （2）在每个关键点，计算一个特征描述符（Feature Descriptors）
    （3）根据特征和位置之间的相似性，从特征描述符集及其两个数据集中的XYZ位置，估计一组对应关系
    （4）假设数据是有噪声的，并且所有对应关系都是有效的，我们就应该排除那些对配准过程产生负面影响的不良对应关系
    （5）从剩余的一组良好的对应关系中，估计一个运动变换

配准模块
 我们来看一下管道的单独步骤
 关键点：key points
    关键点是在场景中具有特殊属性的兴趣点。PCL中有许多不同的关键点，如NARF等等。或者，我们也可以将每个点或子集作为关键点。
    比如，将两个kincet数据集直接输入到对应估计中的问题是每帧中在300k个点，因此我们可以有300k^2个对应。
 特征描述符：Feature Descriptors\
    基于发现的关键点，我们要对其提取特征，这样我们就可以组织信息从而生成向量来将特征相互比较，同样，有许多功能选项可供选择，
    例如NARF,FPFH等等。
 对应估计：Correspondences estimation
    给定来自两次采集扫描的两组特征向量，我们必须找到相应的特征，从而找到数据中的重叠部分。根据特征类型，我们可以使用不同的方法来
    查找对应的关系。
    对于点匹配（使用xyz坐标作为特征），有组织和无组织的数据存在不同的方法：
    （1）直接匹配
    （2）kdTree最近邻搜索(FLANN)
    （3）在有组织的数据的图像空间中搜索
    （4）在有组织数据的索引空间中搜索
    对于特征匹配（不使用点的坐标，而是使用某些特征），仅存在以下方法：
    （1）直接匹配
    （2）kdTree最近邻搜索(FLANN)
 除了搜索之外，还区分了两种类型的对应关系估计：
    （1）直接对应估计（默认）：为点云A中的每个点都搜索点云B中的对应
    （2）“Reciprocal”：对应估计搜索从点云A到点云B和从B到A的对应关系，并且只使用交集。
关系拒绝
 当然并非所有估计的对应关系都是正确的。由于错误的对应关系会对最终转换的估计产生负面影响，因此需要拒绝它们。这可以使用RANSAC或
 通过减少数量并仅使用找到的对应关系的特定百分比来完成。
 一种特殊情况是一对多的对应关系，其中模型中的一个点对应于源中的多个点。这些可以通过仅使用距离最小的一个或通过检查附近的其他匹配来过滤。

变换估计：
 最后一步是实际计算转换：
 （1）基于对应关系评估一些错误度量
 （2）估计相机姿势之间的（刚性）变换（运动估计）并且最小化度量误差
 （3）优化点的结构
 （4）使用刚性变换将源旋转/平移到目标上，并可能使用所有点或点的子集或关键点运行内部ICP循环
 （5）迭代直到满足某个收敛标准
*/
