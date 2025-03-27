#include <ow_core/types.h>
#include <ow_core/geometry/geometry_2d.h>
#include <ow_core/math.h>

ow::Matrix2 rot2(ow::Scalar alpha)
{
  return (ow::Matrix2() << 
    cos(alpha), -sin(alpha),
    sin(alpha), cos(alpha)).finished();
}

int main(int argc, char* argv[]) 
{
  ow::Points2d vertices1(2,4);
  vertices1 <<
  -0.08,   0.12,   0.12,  -0.08,
  -0.005, -0.005,  0.155,  0.155;

  ow::Points2d vertices2(2,4);
  vertices2 << 
  -0.08,   0.12,   0.12,  -0.08,
  -0.155, -0.155,  0.005,  0.005;

  std::cout << "vertices1=\n" << vertices1 << std::endl;
  std::cout << "vertices2=\n" << vertices2 << std::endl;

  // compute the union polygon
  ow::Points2d vertices3 = ow::union_polygon_no_overlap(vertices1, vertices2);

  std::cout << "size=(" << vertices3.rows() << "," << vertices3.cols() << ")" << std::endl;
  std::cout << "vertices3=\n" << vertices3 << std::endl;

  return 0;
}