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
 * \date 23.07.2020
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
 * \brief The LinearVelocityTestSuite class.
 *
 * The TestFixture for LinearVelocity.
 *
 */
class LinearVelocityTestSuite : 
  public ::testing::Test
{
public:
  typedef Eigen::Matrix<ow::Scalar,3,1> EigenVector3;

protected:
  double x_;
  double y_;
  double z_;
  
  EigenVector3 linear_vel_eigen_expected_;

  ow::LinearVelocity linear_vel_expected_;

  tf::Vector3 x_tf_;
  geometry_msgs::Vector3 x_msg_;
  geometry_msgs::Point p_msg_;

  LinearVelocityTestSuite() : 
    x_(1.23456789),
    y_(2.34567890),
    z_(0.12345678),
    linear_vel_eigen_expected_(x_,y_,z_),
    linear_vel_expected_(x_, y_, z_),
    x_tf_(x_,y_,z_)
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

TEST_F(LinearVelocityTestSuite, constuctorFromLinearVelocity)
{
  ow::LinearVelocity x(linear_vel_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, constuctorFromScalar)
{
  ow::LinearVelocity x(x_, y_, z_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, constuctorFromEigen)
{
  ow::LinearVelocity x(linear_vel_eigen_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

/*!
 * Tests for assignment operator.
 */

TEST_F(LinearVelocityTestSuite, assignmentOperatorLinearVelocity)
{
  ow::LinearVelocity x = linear_vel_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}


TEST_F(LinearVelocityTestSuite, assignmentOperatorEigen)
{
  ow::LinearVelocity x = linear_vel_eigen_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, assignmentOperatorVector3TF)
{
  ow::LinearVelocity x;
  x = x_tf_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, assignmentOperatorPointMsg)
{
  ow::LinearVelocity x;
  x = p_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, assignmentOperatorVector3Msg)
{
  ow::LinearVelocity x;
  x = x_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

/*!
 * Tests for type conversion.
 */

TEST_F(LinearVelocityTestSuite, conversionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = linear_vel_expected_;

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, conversionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = linear_vel_expected_;
  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, conversionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = linear_vel_expected_;

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, conversionFunctionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = linear_vel_expected_.toVectorTF();

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, conversionFunctionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = linear_vel_expected_.toPointMsg();

  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

TEST_F(LinearVelocityTestSuite, conversionFunctionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = linear_vel_expected_.toVector3Msg();

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_vel_expected_, 
                                       "linear_vel_expected_"));
}

/*!
 *
 * The Tests for Type Guard.
 *
 */

TEST_F(LinearVelocityTestSuite, typeGuardsLinearAcceleration)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearVelocity, ow::LinearAcceleration>(
      "ow::LinearVelocity", "ow::LinearAcceleration")));
}

TEST_F(LinearVelocityTestSuite, typeGuardsLinearPosition)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearVelocity, ow::LinearPosition>(
      "ow::LinearVelocity", "ow::LinearPosition")));
}

TEST_F(LinearVelocityTestSuite, typeGuardsMoment)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearVelocity, ow::Moment>(
      "ow::LinearVelocity", "ow::Moment")));
}

TEST_F(LinearVelocityTestSuite, typeGuardsAngularVelocity)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearVelocity, ow::AngularVelocity>(
      "ow::LinearVelocity", "ow::AngularVelocity")));
}


TEST_F(LinearVelocityTestSuite, toStringFunction)
{
  std::ostringstream os;
  os << linear_vel_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == linear_vel_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}