#include <ow_core/types.h>
#include <ow_core/math/quaternion.h>

int main(int argc, char* argv[])
{
  ow::AngularPosition p;
  ow::CartesianPosition X;

  Eigen::AngleAxisd aa = 
    Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX());

  p = aa;
  X.orientation() = aa;

  std::cout << p.toString() << std::endl;
  std::cout << X.toString() << std::endl;


  // expected vector
  double x = 3.14159265;
  double y = 1.60217662;
  double z = 2.71828182;

  // expected quaternion
  double qx = 0.6891011;
  double qy = 0.5945444;
  double qz = 0.3514613;
  double qw = 0.2193891;

  ow::CartesianPosition X_expected;
  X_expected << x,y,z,qx,qy,qz,qw; 

  std::cout << X_expected.toString() << std::endl;

  ow::CartesianPosition X2(X_expected);

  std::cout << X_expected.toString() << std::endl;
  std::cout << X2.toString() << std::endl;

  std::cout << X_expected.orientation().toString() << std::endl;
  std::cout << X2.orientation().toString() << std::endl;

  ow::CartesianPosition X3(x,y,z,qw,qx,qy,qz);
  std::cout << X3.toString() << std::endl;

  return 0;
};