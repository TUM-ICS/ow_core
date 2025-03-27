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
 * \brief The LinearPositionTestSuite class.
 *
 * The TestFixture for LinearPosition.
 *
 */
class LinearPositionTestSuite : 
  public ::testing::Test
{
public:
  typedef Eigen::Matrix<ow::Scalar,3,1> EigenVector3;

protected:
  double x_;
  double y_;
  double z_;
  
  EigenVector3 linear_pos_eigen_expected_;

  ow::LinearPosition linear_pos_expected_;

  tf::Vector3 x_tf_;
  geometry_msgs::Vector3 x_msg_;
  geometry_msgs::Point p_msg_;

  LinearPositionTestSuite() : 
    x_(1.23456789),
    y_(2.34567890),
    z_(0.12345678),
    linear_pos_eigen_expected_(x_,y_,z_),
    linear_pos_expected_(x_, y_, z_),
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

TEST_F(LinearPositionTestSuite, constuctorFromLinearPosition)
{
  ow::LinearPosition x(linear_pos_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, constuctorFromScalar)
{
  ow::LinearPosition x(x_, y_, z_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, constuctorFromEigen)
{
  ow::LinearPosition x(linear_pos_eigen_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

/*!
 * Tests for assignment operator.
 */

TEST_F(LinearPositionTestSuite, assignmentOperatorLinearPosition)
{
  ow::LinearPosition x = linear_pos_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}


TEST_F(LinearPositionTestSuite, assignmentOperatorEigen)
{
  ow::LinearPosition x = linear_pos_eigen_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, assignmentOperatorVector3TF)
{
  ow::LinearPosition x;
  x = x_tf_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, assignmentOperatorPointMsg)
{
  ow::LinearPosition x;
  x = p_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, assignmentOperatorVector3Msg)
{
  ow::LinearPosition x;
  x = x_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

/*!
 * Tests for type conversion.
 */

TEST_F(LinearPositionTestSuite, conversionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = linear_pos_expected_;

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, conversionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = linear_pos_expected_;
  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, conversionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = linear_pos_expected_;

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, conversionFunctionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = linear_pos_expected_.toVectorTF();

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, conversionFunctionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = linear_pos_expected_.toPointMsg();

  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

TEST_F(LinearPositionTestSuite, conversionFunctionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = linear_pos_expected_.toVector3Msg();

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", linear_pos_expected_, 
                                       "linear_pos_expected_"));
}

/*!
 *
 * The Tests for Type Guard.
 *
 */

TEST_F(LinearPositionTestSuite, typeGuardsLinearAcceleration)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearPosition, ow::LinearAcceleration>(
      "ow::LinearPosition", "ow::LinearAcceleration")));
}

TEST_F(LinearPositionTestSuite, typeGuardsLinearVelocity)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearPosition, ow::LinearVelocity>(
      "ow::LinearPosition", "ow::LinearVelocity")));
}

TEST_F(LinearPositionTestSuite, typeGuardsMoment)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearPosition, ow::Moment>(
      "ow::LinearPosition", "ow::Moment")));
}

TEST_F(LinearPositionTestSuite, typeGuardsAngularPosition)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::LinearPosition, ow::AngularPosition>(
      "ow::LinearPosition", "ow::AngularPosition")));
}


TEST_F(LinearPositionTestSuite, toStringFunction)
{
  std::ostringstream os;
  os << linear_pos_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == linear_pos_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}