/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 14.02.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_core/types.h>
#include <ow_core/test_utilities/gtest.h>   

/*!
 * \brief The AngularPositionTestSuite class.
 *
 * The TestFixture for AngularPosition.
 *
 */
class AngularPositionTestSuite : 
  public ::testing::Test
{
public:

  typedef Eigen::Matrix<ow::Scalar,4,1> Vector4;
  typedef Eigen::Quaternion<ow::Scalar> Quaternion;
  typedef Eigen::AngleAxis<ow::Scalar> AngleAxis;

protected:
  double x_;
  double y_;
  double z_;
  double w_;
  
  // eigen types
  Vector4 vec_eigen_expected_;
  Quaternion Q_eigen_expected_;
  AngleAxis aa_eigen_expected_;
  Quaternion Q_eigen_identity_;

  // ow types
  ow::AngularPosition Q_expected_;
  ow_core::AngularPositionRef<Vector4> Q_ref_expected_;
  ow::Rotation3 R_expected_;

  // msgs
  geometry_msgs::Quaternion Q_msg_;
  tf::Quaternion Q_tf_;
  tf::Matrix3x3 R_tf_;

  AngularPositionTestSuite() : 
    x_(0.6891011),
    y_(0.5945444),
    z_(0.3514613),
    w_(0.2193891),
    vec_eigen_expected_(x_,y_,z_,w_),
    Q_eigen_expected_(w_,x_,y_,z_),
    aa_eigen_expected_(Q_eigen_expected_),
    Q_eigen_identity_(Quaternion::Identity()),
    Q_expected_(w_,x_,y_,z_),
    Q_ref_expected_(vec_eigen_expected_),
    R_expected_(Q_eigen_expected_),
    Q_tf_(x_, y_, z_, w_),
    R_tf_(Q_tf_)
  {
    Q_msg_.x = x_;
    Q_msg_.y = y_;
    Q_msg_.z = z_;
    Q_msg_.w = w_;
  }
};

TEST_F(AngularPositionTestSuite, constructorDefault)
{
  ow::AngularPosition Q;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_identity_, "Q_identity"));
}

TEST_F(AngularPositionTestSuite, constuctorFromScalar)
{
  ow::AngularPosition Q(w_,x_,y_,z_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_,"));
}

TEST_F(AngularPositionTestSuite, constuctorFromEigenVector)
{
  ow::AngularPosition Q(vec_eigen_expected_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, constuctorFromEigenQuaternion)
{
  ow::AngularPosition Q(Q_eigen_expected_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, constuctorFromEigenAngleAxis)
{
  ow::AngularPosition Q(aa_eigen_expected_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, constuctorFromRotationMatrix)
{
  ow::AngularPosition Q(R_expected_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, constuctorQuaternionRef)
{
  ow::AngularPosition Q(Q_ref_expected_);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorAngularPosition)
{
  ow::AngularPosition Q = Q_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorAngularPositionRef)
{
  ow::AngularPosition Q = Q_ref_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorEigenVector)
{
  ow::AngularPosition Q = vec_eigen_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorAngleAxis)
{
  ow::AngularPosition Q;
  Q = aa_eigen_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorRotationMatrix)
{
  ow::AngularPosition Q;
  Q = R_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorQuaternionMsg)
{
  ow::AngularPosition Q;
  Q = Q_msg_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorQuaternionTF)
{
  ow::AngularPosition Q;
  Q = Q_tf_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, assignmentOperatorMatrixTF)
{
  ow::AngularPosition Q;
  Q = R_tf_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_eigen_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionOperatorAngleAxis)
{
  AngleAxis aa;
  aa = Q_expected_;
  EXPECT_TRUE(
    ow_test::eigenRotationNear(aa, "aa", aa_eigen_expected_, "aa_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionOperatorRotationMatrix)
{
  ow::Rotation3 R;
  R = Q_expected_;
  EXPECT_TRUE(
    ow_test::eigenMatrixNear(R, "R", R_expected_, "R_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionOperatorQuaternionMsg)
{
  geometry_msgs::Quaternion Q_msg;
  Q_msg = Q_expected_;
  ow::AngularPosition Q(Q_msg.w, Q_msg.x, Q_msg.y, Q_msg.z);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionOperatorQuaternionTF)
{
  tf::Quaternion Q_tf;
  Q_tf = Q_expected_;
  ow::AngularPosition Q(Q_tf.w(), Q_tf.x(), Q_tf.y(), Q_tf.z());
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionOperatorQuaternionMatrix)
{
  tf::Matrix3x3 R_tf;
  R_tf = Q_expected_;

  tf::Quaternion Q_tf;
  R_tf.getRotation(Q_tf);
  ow::AngularPosition Q(Q_tf.w(), Q_tf.x(), Q_tf.y(), Q_tf.z());
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionFunctionOperatorQuaternionMsg)
{
  geometry_msgs::Quaternion Q_msg;
  Q_msg = Q_expected_.toQuaternionMsg();
  ow::AngularPosition Q(Q_msg.w, Q_msg.x, Q_msg.y, Q_msg.z);
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionFunctionOperatorQuaternionTF)
{
  tf::Quaternion Q_tf;
  Q_tf = Q_expected_.toQuaternionTF();
  ow::AngularPosition Q(Q_tf.w(), Q_tf.x(), Q_tf.y(), Q_tf.z());
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}

TEST_F(AngularPositionTestSuite, conversionFunctionOperatorQuaternionMatrix)
{
  tf::Matrix3x3 R_tf;
  R_tf = Q_expected_.toMatrixTF();

  tf::Quaternion Q_tf;
  R_tf.getRotation(Q_tf);
  ow::AngularPosition Q(Q_tf.w(), Q_tf.x(), Q_tf.y(), Q_tf.z());
  EXPECT_TRUE(
    ow_test::eigenRotationNear(Q, "Q", Q_expected_, "Q_expected_"));
}


/*TEST_F(AngularPositionTestSuite, typeGuardsRotation3)
{ 
  // seam to test copy assignment and not assignement operator? 
  EXPECT_TRUE((
    ow_test::isConvertible<ow::AngularPosition, ow::Rotation3>(
      "ow::AngularPosition", "ow::Rotation3")));
}*/

/*TEST_F(AngularPositionTestSuite, typeGuardsRotation3Ref)
{ 
  // seam to test copy assignment and not assignement operator? 
  EXPECT_TRUE((
    ow_test::isConvertible<ow_core::AngularPositionRef<Vector4>, 
      ow::Rotation3>("ow::AngularPositionRef", "ow::Rotation3")));
}*/

TEST_F(AngularPositionTestSuite, typeGuardsAngularVelocity)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::AngularPosition, 
      ow::AngularVelocity>("ow::AngularPosition", "ow::AngularVelocity")));
}

TEST_F(AngularPositionTestSuite, typeGuardsAngularAcceleration)
{
  EXPECT_FALSE((
    ow_test::isConvertible<ow::AngularPosition, 
      ow::AngularAcceleration>("ow::AngularPosition", 
        "ow::AngularAcceleration")));
}

TEST_F(AngularPositionTestSuite, toStringFunction)
{
  std::ostringstream os;
  os << vec_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == Q_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}