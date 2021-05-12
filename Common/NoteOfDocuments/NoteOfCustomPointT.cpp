//
// Created by cs18 on 5/12/21.
//

#include "NoteOfCustomPointT.h"

/*PCL带有多种预定义的点类型，包括XYZ数据、SEE结构和更复杂的n维直方图表示形式。这些类型应该足以支持PCL中实现的所有算法和方法，但是在某种情况下，
用户希望定义新的类型。

1、为何是PointT类型
         *在早期，对于点云的共识是他是一种复杂的n维机构，需要能够表示不同类型的信息。但是，用户应该知道并且理解需要传递哪些类型的信息，以便于代码的理解、调试和优化。
         *一个简单的例子是-->对XYZ数据的简单操作。对于启用了SSE的处理器，最有效的方法是将3个维度存储为浮点数，然后再添加一个用于填充的浮点数。
                 struct PointXYZ
                {
                  float x;
                  float y;
                  float z;
                  float padding;
                };
         *但是，作为示例，如果用户正在考虑在嵌入式平台上编译PCL，这样的结构可能会浪费内存。因此，可以使用没有最后一个浮点数的更简单的PointXYZ结构。
         *此外，如果您的应用程序要求PointXYZPRGBNormal包含每个点的三维坐标、颜色信息和表面法线，则用上述方法定义的结构而言，都是很容易表示的。由于PCL中所有算法都应该是
         *模块化的，因此除了结构定义外，无需进行其他更改。
2、PCL提供了哪些PointT类型
         *为了尽可能的涵盖我们可能想到的所有可能情况，我们在PCL中定义了许多点类型，在我们去定义自己的点类型前，我们最好查看一下现有的点列表，有可能我们需要的点类型已经存在了。
         * （1）PointXYZ：float  x,y,z
         *              这是最常用的数据类型之一，因为他仅包含了三维x,y,z坐标信息。三个浮点数用一个附加的浮点数填充来进行SSE对齐。用户可以通过points[i].data[0]或points[i].x以访问x坐标。
                union
                {
                  float data[4];
                  struct
                  {
                    float x;
                    float y;
                    float z;
                  };
                };
         *（2）PointXYZI：float x,y,z,intensity(强度)
         *简单的XYZ+强度的点类型。在理想情况下，这四个成员变量将创建一个于SSE对齐的单一结构。但是，由于大多数的点操作会将data[4]数组的最后一个分量设置为0或1（用于转换），因此我们不能
         *使强度成为统一结构的成员，因为其内容将被覆盖。例如，两个点之间的点积会将其第四个分量设置为0，否则该点积就没有意义了，以此类推。
         *因此，为了SS3对齐，我们用3个额外的浮点填充强度。做这样的处理有可能在存储处理方面效率低下，但是在内存对齐方法则表现良好。
         union
                {
                  float data[4];
                  struct
                  {
                    float x;
                    float y;
                    float z;
                  };
                };
                union
                {
                  struct
                  {
                    float intensity;
                  };
                  float data_c[4];
                };
         *（3）PointXYZRGBA：float x,y,z; std::unint32_t rgba
         *与PointXYZI相似，不同之处在于rgba包含打包成无符号32位整数的RGBA信息。由于使用了联合声明，还可以按照名称分别访问颜色通道。
                     union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      union
                      {
                        struct
                        {
                          std::uint8_t b;
                          std::uint8_t g;
                          std::uint8_t r;
                          std::uint8_t a;
                        };
                        float rgb;
                      };
                      std::uint32_t rgba;
                    };
         *（4）PointXYZRGB：float x,y,z;std::uint32_t rgba;
         *          与PointXYZRGBA相同
         *（5）PointXY：float x,y;
         *          简单的2D x,y点结构
                 struct
                {
                  float x;
                  float y;
                };
         *（6）InrerestPoint：浮点数x,y,z,强度
                    与PointXYZ相似，不同之处在于强度包含了关键点强度的度量。
                union
                {
                  float data[4];
                  struct
                  {
                    float x;
                    float y;
                    float z;
                  };
                };
                union
                {
                  struct
                  {
                    float strength;
                  };
                  float data_c[4];
                };
        （7）Normal ：float normal[3],curbature(曲率);
        另一种最常用的数据类型，"法线"结构表示给定点处的表面法线，以及一个"曲率"度量值（在同一调度中获得，与贴面的特征值之间的关系相同）
        由于在PCL中对表面法线进行的操作非常普遍，因此我们用第四个分量来填充他，以便于进行SSE对齐并提高计算效率。用户可以访问points[i].data_n[0]或points[i].normal[0]
        或points[i].normal_x,以访问法线向量的第一个坐标。同样的，曲率不能以正常数据操作就被覆盖的结构来存储。
                union
                {
                  float data_n[4];
                  float normal[3];
                  struct
                  {
                    float normal_x;
                    float normal_y;
                    float normal_z;
                  };
                }
                union
                {
                  struct
                  {
                    float curvature;
                  };
                  float data_c[4];
                };
        （8）PointNormal：float x,y,z ;float normal[3],curvature;
        保存XYZ数据以及表面法线和曲率的点结构
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      float data_n[4];
                      float normal[3];
                      struct
                      {
                        float normal_x;
                        float normal_y;
                        float normal_z;
                      };
                    };
                    union
                    {
                      struct
                      {
                        float curvature;
                      };
                      float data_c[4];
                    };
        （9）PointXYZRGBNormal：float x,y,z,normal[3],curvature;std::uint32_t rgba;
        包含XYZ数据和RGBA颜色以及表面法线和曲率的点结构。（尽管名称内没有包括A,但该类型确实包含Aloha通道）
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      float data_n[4];
                      float normal[3];
                      struct
                      {
                        float normal_x;
                        float normal_y;
                        float normal_z;
                      };
                    }
                    union
                    {
                      struct
                      {
                        union
                        {
                          union
                          {
                            struct
                            {
                              std::uint8_t b;
                              std::uint8_t g;
                              std::uint8_t r;
                              std::uint8_t a;
                            };
                            float rgb;
                          };
                          std::uint32_t rgba;
                        };
                        float curvature;
                      };
                      float data_c[4];
                    };
        （10）PointXYZINormal：float x,y,z,intensity,normal[3],curvature
        保存XYZ,强度值以及表面法线和曲率的点结构
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      float data_n[4];
                      float normal[3];
                      struct
                      {
                        float normal_x;
                        float normal_y;
                        float normal_z;
                      };
                    }
                    union
                    {
                      struct
                      {
                        float intensity;
                        float curvature;
                      };
                      float data_c[4];
                    };
        （11）PointWithRange：float x,y,z(union with float point[4]),range;
        与PointXYZI相似，range包含从采集视点到某个点的距离的度量。
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      struct
                      {
                        float range;
                      };
                      float data_c[4];
                    };
        （12）PointWithViewpoint-float x，y，z，vp_x，vp_y，vp_z；

        与PointXYZI相似，除了vp_x，vp_y和vp_z包含获取视点作为3D点。
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      struct
                      {
                        float vp_x;
                        float vp_y;
                        float vp_z;
                      };
                      float data_c[4];
                    };
        （13）MomentInvariants：float j1，j2，j3；
        简单点类型，将3个矩不变式保存在曲贴面上
                    struct
                    {
                      float j1, j2, j3;
                    };
        （14）PrincipalRadiiRSD ：float r_min, r_max;
        简单点类型，将2 RSD半径保持在表面斑块处
        （15）Boundary ：std::uint8_t boundary_point;
        简单点类型，用于确定点是否位于曲面边界上
                struct
                {
                  std::uint8_t boundary_point;
                };
        （16）PrincipalCurvatures：float Principal_curvature [3]，pc1，pc2；
        简单点类型，用于保存给定点的主曲率。
                struct
                {
                  union
                  {
                    float principal_curvature[3];
                    struct
                    {
                      float principal_curvature_x;
                      float principal_curvature_y;
                      float principal_curvature_z;
                    };
                  };
                  float pc1;
                  float pc2;
                };
        （17）PFHSignature125：float pfh [125]
        简单点类型，用于保存给定点的PFH（点特征直方图）
                struct
                {
                  float histogram[125];
                };
        （18）FPFHSignature33：float fpfh [33];

        保存给定点的FPFH（快速特征直方图）的简单点类型
                    struct
                    {
                      float histogram[33];
                    };
        （19）VFHSignature308 ：float vfh [308];

        保留给定点的VFH（视点特征直方图）的简单点类型。
                    struct
                    {
                      float histogram[308];
                    };
        （20）Narf36 ：float x, y, z, roll, pitch, yaw; float descriptor[36]
        简单点类型，用于保存给定点的NARF（正常对齐的半径特征
                struct
                {
                  float x, y, z, roll, pitch, yaw;
                  float descriptor[36];
                };
        （21）BorderDescription - int x, y; BorderTraits traits
        简单点类型，用于保存给定点的边框类型。
                    struct
                    {
                      int x, y;
                      BorderTraits traits;
                    };
        （22）IntensityGradient - float gradient[3]
        简单点类型，用于保存给定点的强度梯度。
                struct
                {
                  union
                  {
                    float gradient[3];
                    struct
                    {
                      float gradient_x;
                      float gradient_y;
                      float gradient_z;
                    };
                  };
                };
        （23）Histogram - float histogram[N]
        通用nD直方图占位符。
                template <int N>
                struct Histogram
                {
                  float histogram[N];
                };
        （24）PointWithScale - float x, y, z, scal
        与PointXYZI相似，不同之处是比例尺包含考虑将某个点用于几何运算的比例尺（例如，用于其最近邻居计算的球体的半径，窗口大小等）。
                struct
                {
                  union
                  {
                    float data[4];
                    struct
                    {
                      float x;
                      float y;
                      float z;
                    };
                  };
                  float scale;
                };
        （25）PointSurfel - float x, y, z, normal[3], rgba, radius, confidence, curvature
        一种复杂点类型，包含XYZ数据，表面法线以及RGB信息，比例，置信度和表面曲率。
                    union
                    {
                      float data[4];
                      struct
                      {
                        float x;
                        float y;
                        float z;
                      };
                    };
                    union
                    {
                      float data_n[4];
                      float normal[3];
                      struct
                      {
                        float normal_x;
                        float normal_y;
                        float normal_z;
                      };
                    };
                    union
                    {
                      struct
                      {
                        std::uint32_t rgba;
                        float radius;
                        float confidence;
                        float curvature;
                      };
                      float data_c[4];
                    };
3、点类型应该如何显示
        由于点云类型的体积很大，并且因为他是一个模板库，所以在一个源文件中包含许多PCL算法可能会减慢编译过程。
        为了加快包含和链接PCL的用户代码的执行，我们使用显式模板实例化，方法是包括所有可能的组合，其中可以使用PCL中已经定义的点类型来调用所有算法。
        这意味着一旦PCL被编译为库，任何用户代码都不需编译模块化代码，从而加快了编译时间。



 */
