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
 * \brief The Vector3TestSuite class.
 *
 * The TestFixture for Vector3.
 *
 */
class Vector3TestSuite : 
  public ::testing::Test
{
public:
  typedef Eigen::Matrix<ow::Scalar,3,1> EigenVector3;

protected:
  double x_;
  double y_;
  double z_;
  
  EigenVector3 x_eigen_expected_;

  ow::Vector3 x_expected_;
  ow_core::Vector3Ref<EigenVector3> x_ref_expected_;

  tf::Vector3 x_tf_;
  geometry_msgs::Vector3 x_msg_;
  geometry_msgs::Point p_msg_;

  Vector3TestSuite() : 
    x_(2.00231930436),
    y_(6.62607015),
    z_(6.6743015),
    x_eigen_expected_(x_,y_,z_),
    x_expected_(x_, y_, z_),
    x_ref_expected_(x_eigen_expected_),
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

TEST_F(Vector3TestSuite, constuctorFromVector3)
{
  ow::Vector3 x(x_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, constuctorFromScalar)
{
  ow::Vector3 x(x_, y_, z_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, constuctorFromEigen)
{
  ow::Vector3 x(x_eigen_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, constuctorFromVector3Ref)
{
  ow::Vector3 x(x_ref_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorVector3)
{
  ow::Vector3 x = x_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorVector3Ref)
{
  ow::Vector3 x = x_ref_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorEigen)
{
  ow::Vector3 x = x_eigen_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorVector3TF)
{
  ow::Vector3 x;
  x = x_tf_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorPointMsg)
{
  ow::Vector3 x;
  x = p_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, assignmentOperatorVector3Msg)
{
  ow::Vector3 x;
  x = x_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = x_expected_;

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = x_expected_;
  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = x_expected_;

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionFunctionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = x_expected_.toVectorTF();

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionFunctionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = x_expected_.toPointMsg();

  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, conversionFunctionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = x_expected_.toVector3Msg();

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", x_expected_, "x_expected_"));
}

TEST_F(Vector3TestSuite, typeGuardsLinearPosition)
{ 
  EXPECT_TRUE((
    ow_test::isConvertible<ow::Vector3, ow::LinearPosition>(
      "ow::Vector3", "ow::LinearPosition")));
}

TEST_F(Vector3TestSuite, typeGuardsLinearState)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::Vector3, ow::LinearState>(
      "ow::Vector3", "ow::LinearState")));
}

TEST_F(Vector3TestSuite, toStringFunction)
{
  std::ostringstream os;
  os << x_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == x_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}