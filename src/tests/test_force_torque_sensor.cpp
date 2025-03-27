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
#include <ow_core/types/force_torque_sensor.h>
#include <ow_core/test_utilities/gtest.h>  

/*!
 * \brief The ForceTorqueSensorTestSuite class.
 *
 * The TestFixture for Force.
 *
 */
class ForceTorqueSensorTestSuite : 
  public ::testing::Test
{
public:
  typedef ow_core::ForceTorqueSensor<ow::Scalar> FTSensor;

protected:
  double x_;
  double y_;
  double z_;
  
  ow::Wrench wrench_expected_;
  ow::Wrench wrench_offset_expected_;

  ForceTorqueSensorTestSuite() :
    wrench_expected_(3.0, 4.1, 1.0, 5.1, 1.4, 8.7),
    wrench_offset_expected_(2.0, 3.1, 0.0, 4.1, 0.4, 7.7)
  {
  }
};

/*!
 * Tests for constructor.
 */

TEST_F(ForceTorqueSensorTestSuite, constuctorFromWrench)
{
  FTSensor ft(wrench_expected_, "", wrench_offset_expected_);

  EXPECT_TRUE(ow_test::eigenMatrixNear(ft.W(), "W", wrench_expected_, "wrench_expected_"));
  EXPECT_TRUE(ow_test::eigenMatrixNear(ft.WOffset(), "WOffset", wrench_offset_expected_, "wrench_offset_expected_"));
}

TEST_F(ForceTorqueSensorTestSuite, constuctorFromDefault)
{
  FTSensor ft = FTSensor::Default();

  EXPECT_TRUE(ow_test::eigenMatrixNear(ft.W(), "W", ow::Wrench::Zero(), "wrench_expected_"));
  EXPECT_TRUE(ow_test::eigenMatrixNear(ft.WOffset(), "WOffset", ow::Wrench::Zero(), "wrench_offset_expected_"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}