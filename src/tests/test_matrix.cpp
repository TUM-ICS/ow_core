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
 * \date 22.07.2020
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
#include <ow_core/type_references/matrix_ref.h>
#include <ow_core/test_utilities/gtest.h>  

/*!
 * \brief The MatrixTestSuite class.
 *
 * The TestFixture for Matrix.
 *
 */
class MatrixTestSuite : 
  public ::testing::Test
{
public:
  typedef Eigen::Matrix<ow::Scalar,3,3> EigenMatrix3x3;
  typedef Eigen::Matrix<ow::Scalar,4,4> EigenMatrix4x4;

protected:

  EigenMatrix3x3 matrix_eigen_expected_3x3_;
  EigenMatrix4x4 matrix_eigen_expected_4x4_;

  ow_core::Matrix<ow::Scalar, 3, 3> matrix_expected_;
  ow_core::MatrixRef<EigenMatrix4x4,3,3> matrix_ref_expected_;


  MatrixTestSuite():
  matrix_expected_(Eigen::Matrix3d::Ones()),
  matrix_eigen_expected_3x3_(Eigen::Matrix3d::Ones()),
  matrix_eigen_expected_4x4_(Eigen::Matrix4d::Ones()),
  matrix_ref_expected_(matrix_eigen_expected_4x4_, 0,0)
  {
                              

  }
};

/*!
 * Tests for constructor.
 */

TEST_F(MatrixTestSuite, constuctorFromMatrix)
{
  ow::Matrix3 x(matrix_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_expected_,
              "matrix_expected_"));
}

TEST_F(MatrixTestSuite, constuctorFromEigen)
{
  ow::Matrix3 x(matrix_eigen_expected_3x3_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_eigen_expected_3x3_, 
                                       "matrix_expected_"));
}

TEST_F(MatrixTestSuite, constuctorFromMatrixRef)
{
  ow::Matrix3 x(matrix_ref_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_ref_expected_, 
                                       "matrix_expected_"));
}

/*!
 * Tests for assignment operator.
 */

TEST_F(MatrixTestSuite, assignmentOperatorMatrix)
{
  ow::Matrix3 x = matrix_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_expected_, 
                                       "matrix_expected_"));
}

TEST_F(MatrixTestSuite, assignmentOperatorMatrixRef)
{
  ow::Matrix3 x = matrix_ref_expected_;

  EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_ref_expected_, 
                                       "matrix_expected_"));
}



/*!
 * Tests for type conversion.
 */

// TEST_F(MatrixTestSuite, conversionToVectorTF)
// {
//   tf::Vector3 x_tf;
//   x_tf = matrix_expected_;

//   EigenVector3 x(x_tf.x(), x_tf.y(), x_tf.z());
//   EXPECT_TRUE(ow_test::eigenMatrixNear(x, "x", matrix_expected_, "matrix_expected_"));
// }


/*!
 *
 * The Tests for Type Guard.
 *
 */

TEST_F(MatrixTestSuite, typeGuardsVector3)
{ 
  EXPECT_TRUE((
    ow_test::isConvertible<ow::Matrix3, ow::Vector3>(
      "ow::Matrix", "ow::Vector3")));
}

TEST_F(MatrixTestSuite, typeGuardsRotation3)
{ 
  EXPECT_TRUE((
    ow_test::isConvertible<ow::Matrix3, ow::Rotation3>(
      "ow::Matrix", "ow::Rotation3")));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}