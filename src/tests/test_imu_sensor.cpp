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
#include <ow_core/types/inertial_measurment_unit_sensor.h>
#include <ow_core/test_utilities/gtest.h>  

/*!
 * \brief The IMUSensor class.
 *
 * The TestFixture for Force.
 *
 */
class IMUSensorFixture : 
  public ::testing::Test
{
public:
  typedef ow_core::InertialMeasurmentUnitSensor<ow::Scalar> IMUSensor;

protected:
  double x_;
  double y_;
  double z_;

  ow::LinearAcceleration linear_acc_expected_;
  ow::AngularVelocity angular_vel_expected_;
  ow::AngularPosition angular_pos_expected_;

  IMUSensorFixture() :
    linear_acc_expected_(3.0, 4.1, 1.0),
    angular_vel_expected_(4.1, 0.4, 7.7)
  {
  }
};

/*!
 * Tests for constructor.
 */

TEST_F(IMUSensorFixture, constuctorFromWrench)
{
  IMUSensor imu(linear_acc_expected_, angular_vel_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(imu.xPP(), "xPP", linear_acc_expected_, "linear_acc_expected_"));
  EXPECT_TRUE(ow_test::eigenMatrixNear(imu.omega(), "omega", angular_vel_expected_, "angular_vel_expected_"));
  EXPECT_TRUE(ow_test::eigenRotationNear(imu.Q(), "omega", ow::AngularPosition::Identity(), "angular_pos_expected_"));
}

TEST_F(IMUSensorFixture, constuctorFromDefault)
{
  IMUSensor imu = IMUSensor::Default();

  EXPECT_TRUE(ow_test::eigenMatrixNear(imu.xPP(), "xPP", ow::LinearAcceleration::Zero(), "linear_acc_expected_"));
  EXPECT_TRUE(ow_test::eigenMatrixNear(imu.omega(), "omega", ow::AngularVelocity::Zero(), "angular_vel_expected_"));
  EXPECT_TRUE(ow_test::eigenRotationNear(imu.Q(), "omega", ow::AngularPosition::Identity(), "angular_pos_expected_"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}