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
 * \date 13.07.2020
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
 * \brief The ForceTestSuite class.
 *
 * The TestFixture for Force.
 *
 */
class ForceTestSuite : 
  public ::testing::Test
{
public:
  typedef Eigen::Matrix<ow::Scalar,3,1> EigenVector3;

protected:
  double x_;
  double y_;
  double z_;
  
  EigenVector3 force_eigen_expected_;

  ow::Force force_expected_;
  ow_core::ForceRef<EigenVector3> x_ref_expected_;

  tf::Vector3 x_tf_;
  geometry_msgs::Vector3 x_msg_;
  geometry_msgs::Point p_msg_;

  ForceTestSuite() : 
    x_(2.00231930436),
    y_(6.62607015),
    z_(6.6743015),
    force_eigen_expected_(x_,y_,z_),
    force_expected_(x_, y_, z_),
    x_ref_expected_(force_eigen_expected_),
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

TEST_F(ForceTestSuite, constuctorFromForce)
{
  ow::Force x(force_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, constuctorFromScalar)
{
  ow::Force x(x_, y_, z_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, constuctorFromEigen)
{
  ow::Force x(force_eigen_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, constuctorFromForceRef)
{
  ow::Force x(x_ref_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

/*!
 * Tests for assignment operator.
 */

TEST_F(ForceTestSuite, assignmentOperatorForce)
{
  ow::Force x = force_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, assignmentOperatorForceRef)
{
  ow::Force x = x_ref_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, assignmentOperatorEigen)
{
  ow::Force x = force_eigen_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, assignmentOperatorVector3TF)
{
  ow::Force x;
  x = x_tf_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, assignmentOperatorPointMsg)
{
  ow::Force x;
  x = p_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, assignmentOperatorVector3Msg)
{
  ow::Force x;
  x = x_msg_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

/*!
 * Tests for type conversion.
 */

TEST_F(ForceTestSuite, conversionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = force_expected_;

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, conversionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = force_expected_;
  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, conversionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = force_expected_;

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, conversionFunctionToVectorTF)
{
  tf::Vector3 x_tf;
  x_tf = force_expected_.toVectorTF();

  EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, conversionFunctionToPointMsg)
{
  geometry_msgs::Point p_msg;
  p_msg = force_expected_.toPointMsg();

  EigenVector3 x(p_msg.x, p_msg.y, p_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

TEST_F(ForceTestSuite, conversionFunctionToVector3Msg)
{
  geometry_msgs::Vector3 x_msg;
  x_msg = force_expected_.toVector3Msg();

  EigenVector3 x(x_msg.x, x_msg.y, x_msg.z);
  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", force_expected_, "force_expected_"));
}

/*!
 *
 * The Tests for Type Guard.
 *
 */

TEST_F(ForceTestSuite, typeGuardsLinearPosition)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::Force, ow::LinearPosition>(
      "ow::Force", "ow::LinearPosition")));
}

TEST_F(ForceTestSuite, typeGuardsLinearAcceleration)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::Force, ow::LinearAcceleration>(
      "ow::Force", "ow::LinearAcceleration")));
}

TEST_F(ForceTestSuite, typeGuardsMoment)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::Force, ow::Moment>(
      "ow::Force", "ow::Moment")));
}

TEST_F(ForceTestSuite, typeGuardsLinearState)
{ 
  EXPECT_FALSE((
    ow_test::isConvertible<ow::Force, ow::LinearState>(
      "ow::Force", "ow::LinearState")));
}

TEST_F(ForceTestSuite, toStringFunction)
{
  std::ostringstream os;
  os << force_eigen_expected_.transpose();
  EXPECT_TRUE(os.str() == force_expected_.toString());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}