//
// Created by cs18 on 5/25/21.
//

/*
我们在PCL普遍使用PCD文件格式

为什么要使用这个文件格式？

PCD文件格式并非是要重新造轮子，而是要补充由于某种原因不支持PCL的N维点云格式。
现在的PCD文件不是第一个支持3D点云数据的文件类型，现有的其他文件格式比如：PLY,STL,OBJ.X3D等等

PCD版本
在发布点云库(PCL)1.0版本之前，PCD文件格式可能具有不同的修订版号。这些用PCD_Vx编号（如PCD_V5等），并代表PCD文件的版本号0.x。但是PCL中PCD文件格式的正式入口点是PCL_V7。

文件格式标题
每个PCD文件都包含一个标头，该标头标识并且声明存储在文件中的点云数据的某些属性。PCD的标头必须以ASCII编码。
Note：使用新行（\n）分隔PCD文件中指定的每个标头条目以及ascii点数据。
从0.7版本开始，PCD标头包含以下条目：
    VERSION-指定PCD文件版本
    FIELDS-指定点可以具有的每个维度/字段的名称。比如
        FIELDS x y z                                # XYZ data     XYZ数据
        FIELDS x y z rgb                            # XYZ + colors      XYZ+RGB数据
        FIELDS x y z normal_x normal_y normal_z     # XYZ + surface normals      XYZ+表面法线数据
        FIELDS j1 j2 j3                             # moment invariants   不变矩
        ...
    SIZE-指定每个维度的大小（以字节为单位）。比如
        unsigned char/char has 1 byte   无符号字符/字符 为 1字节
        unsigned short/short has 2 bytes    无符号短整型/短整型 2字节
        unsigned int/int/float has 4 bytes    无符号整型/整型 4字节
        double has 8 bytes  双精度浮点数 8字节
    TYPE-将每个维度的类型指定为char。当前接受的类型是：
        I-表示带符号的类型int8(char),int16(short)和int32(int)
        U-表示无符号类型uint8(unsigned char),uint16(unsigned short),uint32(unsigned int)
        F-表示浮点类型
    COUNT-指定每个维度有多少个元素。例如，x数据通常具有1个元素，但还想VFH（视点特征直方图）这样的特征描述符有308个。基本上，这是在每个点引入n维直方图描述符并将它们视为单个连续内存块的方法。
    在 默认情况下，如果不存在COUNT,则所有尺寸的计数均设置为1.
    WIDTH-以点数指定点云数据集的宽度。WIDTH有两个含义，
        是未组织点云数据集中的点的总数量，此时HEIGHT为1
        是已组织的点云数据集中的点的宽度。此时HEIGHT为点的高度
            NOTE:一个有组织的点云数据集是赋予点云类型一个有组织的图像（或矩阵）状的结构，其中，数据被划分为行和列的点云名称。这种点云的示例包括来自立体摄像机或飞行时间摄像机的数据。
                        有组织的数据集的优势在于，通过了解相邻点（例如像素）之间的关系，最近临近点的操作会更加高效，从而加快了计算速度并且降低了PCL中某些算法的成本。
        WIDTH 640   #d代表每一行都包含640个点
    HEIGHT-以点数指定点云数据集的高度。HEIGHT有两个含义：
        它可以指定组织的点云数据集的高度（总行数）
        对于未组织的点云数据集，我们将其设置为1，用于检测数据集是否是已组织的。
        比如：
        WIDTH 640       # Image-like organized structure, with 480 rows and 640 columns，类似于图像组织方式结构，有480行和640列
        HEIGHT 480      # thus 640*480=307200 points total in the dataset，则总共有480*640=307200个点
        在比如：
        WIDTH 307200
        HEIGHT 1        # unorganized point cloud dataset with 307200 points，无组织点云数据集共有307200个点
    VIEWPOINT-为数据集中的点指定采集视点。以后可能用它来构建不同坐标系之间的变换，或者用于辅助需要一致方向的特征（例如表面法线）
        视点信息被指定为平移(tx,ty,tz)+四元数(qw,qx,qy,qz)。默认为：
        VIEWPOINT 0 0 0 1 0 0 0
    POINTS-指定云中的总点数。从0.7版开始，这个选项有些多余，有可能未来版本将删除这个属性。
    DATA-指定存储点云数据的数据类型。从0.7版本开始，支持两种数据类型：ascii和binary。
        Note：标头最后一行（DATA）之后的下一个字节被视为点云数据的一部分，因此被解释为点云数据的一部分
    
    标头条目必须按照上述顺序精确指定，即：
    VERSION
    FIELDS
    SIZE
    TYPE
    COUNT
    WIDTH
    HEIGHT
    VIEWPOINT
    POINTS
    DATA

    资料存储类型
        从0.7版本开始，.PCD文件格式使用两种不同的模式来存储数据：
            以ASCII形式，每个点都换行：
                p_1
                p_2
                p_3
                p_4
                ...
                p_n
                从1.0.1版本开始，NaN的字符串表示形式为nan
            以二进制形式，其中数据是pcl::PointCloud.points数组/向量的完整内存副本。在linux系统中，我们使用mmap/munmap操作对数据进行最快的读/写访问
        以最简单的ascii格式存储点云数据，将点、行、空格或制表符上的每个点分隔开，上面没有任何其他字符，并以二进制转储格式存储，这使得我们能够同时兼顾两个方面的优势：
        速度，取决于基础应用程序。ascii格式允许用户打开点云文件并使用gnuplot等标准软件对齐进行绘制，或使用sed,awk等工具对其进行操作。
    与其他文件格式相比的优势：
     将PCD作为点云处理的文件格式优点包括：
         存储和处理组织的点云数据集的能力-这对于实时应用以及增强现实，机器人等研究领域极为重要
         二进制mmap/munmap数据类型是将数据加载和保存到磁盘的最快方法
         存储不同数据类型（char,int,short,double.float等）使点云数据在存储和处理方面更加灵活高效。无效的点云点通常存储为nan格式
         特征描述符的n维直方图-对于3D感知和计算机视觉应用很重要
    另一个优点是，通过控制文件格式，我们可以使其最佳的适应PCL，从而获得PCL应用程序的最高性能，而不是将其他文件格式适应为PCL作为处理类型。
*/
