#include <ow_core/types.h>

#include <ow_core/types/matrix.h>

#include <ow_core/type_references/matrix_ref.h>
#include <ow_core/type_references/vector_ref.h>
#include <ow_core/types/vector_dof.h>

#include <ow_core/type_references/linear_acceleration_ref.h>

#include <ow_core/math.h>

#include <iostream>

using namespace std;

template <typename Derived1, typename Derived2>
void testfnc(
  ow_core::CartesianBase<Derived1>& a,
  ow_core::CartesianBase<Derived2>& b)
{
  typedef typename Derived1::Scalar Scalar;

  std::cout << "a linear=" << a.linear().toString() << std::endl;
  std::cout << "a angular=" << a.angular().toString() << std::endl;
  std::cout << "b linear=" << b.linear().toString() << std::endl;
  std::cout << "b angular=" << b.angular().toString() << std::endl;
  a.linear() = static_cast<const ow_core::Vector3<Scalar>& >(b.linear());
  a.angular() = static_cast<const ow_core::Vector3<Scalar>& >(b.angular());
}

int main()
{
  ow::SpatialVector sv;
  sv << 1, 1, 1, 2, 2, 2;   // angular 1, linear 2

  ow::CartesianVelocity cv = sv;
  ow::CartesianAcceleration ca = sv;
  ow::Wrench w = sv;

  std::cout << "sa =" << sv.toString() << std::endl;
  std::cout << "cv=" << cv.toString() << std::endl;
  std::cout << "ca=" << ca.toString() << std::endl;
  std::cout << "w= " << w.toString() << std::endl;

  ow::SpatialVector sv2;
  ow::CartesianAcceleration ca2;
  ca2 << 1, 1, 1, 2, 2, 2;
  sv2 = ca2;

  std::cout << "ca2=" << ca2.toString() << std::endl;
  std::cout << "sv2= " << sv2.toString() << std::endl;

  //----------------------------------------------------------------------------

  ow::JointPosition q;
  ow::JointAcceleration qP;
  ow::VectorDof vecDof = ow::VectorDof::Zero();

  Eigen::Vector3d test = Eigen::Vector3d::Ones();
  std::cout << ow_core::eigenToString(test) << std::endl;

  // q = qP;
  q = vecDof;
  qP = vecDof;

  ow::Rotation3 Rot = ow::Rotation3::Ry();

  ow::Wrench W = ow::Wrench::Zero();
  ow::Force F;
  F = Rot*W.force();
  std::cout << "Test1=" << W.force().toString() << std::endl;

  ow::Force F2(W.force());
  std::cout << "Test3=" << F2.toString() << std::endl;

  ow::CartesianAcceleration CA = ow::CartesianAcceleration::Ones();
  ow::LinearAcceleration LA;
  LA = Rot*CA.linear();
  std::cout << "Test2=" << CA.linear().toString() << std::endl;


  // Testing Matrix Type
  Eigen::Matrix3d m_test = Eigen::Matrix3d::Zero();
  ow_core::LinearAccelerationRef<Eigen::Matrix3d> lin_ref(m_test);
  std::cout << "LinearAccelerationRef=" << lin_ref << std::endl;


  ow_core::Matrix<double, 4, 2> MM = ow_core::Matrix<double, 4, 2>::Ones();
  std::cout << "M=\n" << MM.toString() << std::endl;
  
  Eigen::Matrix4d memory = Eigen::Matrix4d::Zero();
  ow_core::MatrixRef<Eigen::Matrix4d, 4, 2> M_ref(memory);
  M_ref = MM;

  std::cout << "M_ref=\n" << M_ref.toString() << std::endl;

  std::cout << "memory=\n" << memory << std::endl;

  //----------------------------------------------------------------------------
  // Testing Rotation3 Type

  ow::Rotation3 R = ow::Rotation3::Rx();
  std::cout << "R=\n" << R.toString() << std::endl;

  geometry_msgs::Quaternion q_msgs;
  q_msgs = R;

  std::cout << "q_msgs=" << q_msgs.w << " " << q_msgs.x 
            << " " << q_msgs.y << " " << q_msgs.z << std::endl;

  ow::Rotation3 R1 = ow::Rotation3::Rx(M_PI/4);
  ow::Rotation3 R2 = ow::Rotation3::Ry(5*M_PI/4);
  ow::Rotation3 R3 = R1*R2;

  std::cout << "R3=\n" << R3.toString() << std::endl;
  tf::Quaternion q_tf;
  q_tf = R3;
  std::cout << "q_tf=" << q_tf.w() << " " << q_tf.x() 
            << " " << q_tf.y() << " " << q_tf.z() << std::endl;

  //----------------------------------------------------------------------------
  // Testing VectorDof Type

  ow_core::VectorDof<ow::Scalar, 6> v = 
    ow_core::VectorDof<ow::Scalar, 6>::Zero();
  std::cout << "v=" << v.toString() << std::endl;

  //----------------------------------------------------------------------------

  // Matrix_Ref test
  Eigen::Matrix3d M = Eigen::Matrix3d::Ones();
  ow_core::MatrixRef<Eigen::Matrix3d, 2, 2> m_ref(M,0,0);

  cout << "M=\n" << M << endl;
  cout << "M_ref=\n" << m_ref << endl << endl;

  Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d b = Eigen::Matrix3d::Zero();
  ow_core::MatrixRef<Eigen::Matrix3d, 3, 3> m_copy(a,0,0);
  cout << "m_copy_a=\n" << m_copy << endl;
  m_copy = b;
  cout << "m_copy_a=\n" << m_copy << endl << endl;


  // Vector_Ref test
  Eigen::Vector3d c= Eigen::Vector3d::Ones();
  ow_core::VectorRef<Eigen::Vector3d, 2> v_ref(c);
  cout << "Vector_c=\n" << c << endl;
  cout << "Vector_c_ref=\n" << v_ref << endl << endl;

  // Rotation3Ref test
  Eigen::Matrix4d r = Eigen::Matrix4d::Ones();
  ow_core::Rotation3Ref<Eigen::Matrix4d> r3_ref(r, 1, 1);
  cout << "rotation=\n" << r << endl;
  cout << "rotation3=\n" << r3_ref << endl << endl;

  return 0;
}