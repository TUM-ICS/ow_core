/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Huiwen Pan
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 27.07.2020
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
 * \brief The AngularVelocityTestSuite class.
 *
 * The TestFixture for AngularVelocity.
 *
 */
class AngularVelocityTestSuite : 
  public ::testing::Test
{
public:

  typedef Eigen::Matrix<ow::Scalar,3,1> EigenVector3;

protected:
  double x_;
  double y_;
  double z_;

  
  // eigen types
  EigenVector3 vel_eigen_expected_;

  // ow types
  ow::AngularVelocity ang_vel_expected_;
  ow_core::AngularVelocityRef<EigenVector3> vel_ref_expected_;

  // msgs
  tf::Vector3 x_tf_;
  geometry_msgs::Vector3 x_msg_;
  geometry_msgs::Point p_msg_;


  AngularVelocityTestSuite() : 
    x_(0.6891011),
    y_(0.5945444),
    z_(0.3514613),
    vel_eigen_expected_(x_,y_,z_),
    ang_vel_expected_(x_,y_,z_),
    vel_ref_expected_(vel_eigen_expected_),
    x_tf_(x_, y_, z_)
  {
    x_msg_.x = x_;
    x_msg_.y = y_;
    x_msg_.z = z_;

    p_msg_.x = x_;
    p_msg_.y = y_;
    p_msg_.z = z_;
  }
};

/*!
 * Tests for constructor.
 */

TEST_F(AngularVelocityTestSuite, constuctorFromAngularVelocity)
{
  ow::AngularVelocity x(ang_vel_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, constuctorFromScalar)
{
  ow::AngularVelocity x(x_, y_, z_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, constuctorFromEigen)
{
  ow::AngularVelocity x(vel_eigen_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

/*!
 * Tests for assignment operator.
 */

TEST_F(AngularVelocityTestSuite, assignmentOperatorAngularVelocity)
{
  ow::AngularVelocity x = ang_vel_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}


TEST_F(AngularVelocityTestSuite, assignmentOperatorEigen)
{
  ow::AngularVelocity x = vel_eigen_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, assignmentOperatorVector3TF)
{
  ow::AngularVelocity x;
  x = x_tf_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, assignmentOperatorPointMsg)
{
  ow::AngularVelocity x;
  x = p_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, assignmentOperatorVector3Msg)
{
  ow::AngularVelocity x;
  x = x_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

/*!
 * Tests for type conversion.
 */

TEST_F(AngularVelocityTestSuite, conversionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = ang_vel_expected_;

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, conversionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = ang_vel_expected_;
  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, conversionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = ang_vel_expected_;

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, conversionFunctionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = ang_vel_expected_.toVectorTF();

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, conversionFunctionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = ang_vel_expected_.toPointMsg();

  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

TEST_F(AngularVelocityTestSuite, conversionFunctionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = ang_vel_expected_.toVector3Msg();

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", ang_vel_expected_, 
                                       "ang_vel_expected_"));
}

/*!
 *
 * The Tests for Type Guard.
 *
 */

TEST_F(AngularVelocityTestSuite, typeGuardsLinearVelocity)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::AngularVelocity, ow::LinearVelocity>(
      "ow::AngularVelocity", "ow::LinearVelocity")));
}

TEST_F(AngularVelocityTestSuite, typeGuardsAngularPosition)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::AngularVelocity, ow::AngularPosition>(
      "ow::AngularVelocity", "ow::AngularPosition")));
}

TEST_F(AngularVelocityTestSuite, typeGuardsAngularAcceleration)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::AngularVelocity, ow::AngularAcceleration>(
      "ow::AngularVelocity", "ow::AngularAcceleration")));
}



TEST_F(AngularVelocityTestSuite, toStringFunction)
{
  std::ostringstream os;
  os << vel_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == ang_vel_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}