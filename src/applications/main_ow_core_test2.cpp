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


#include <unistd.h>
#include <stdio.h>
#include <iostream>

#include <ow_core/type_references/angular_position_ref.h>
#include <ow_core/type_references/linear_position_ref.h>
#include <ow_core/types/cartesian_position.h>
#include <ow_core/type_references/linear_velocity_ref.h>
#include <ow_core/types/cartesian_velocity.h>
#include <ow_core/types/rotation3.h>

#include <ow_core/math.h>

int main(int argc, char* argv[])
{
  ow::AngularPosition test;
  test = Eigen::AngleAxisd(0.25*M_PI, ow::Vector3(0.1, 0.3, -0.2).normalized());

  ow::Rotation3 test2;
  test2 = Eigen::AngleAxisd(0.25*M_PI, ow::Vector3(0.1, 0.3, -0.2).normalized());

  ow::Vector3 test3 = test.eulerYPR();
  ow::Vector3 test4 = test.eulerAngles(2, 1, 0);
  std::cout << "EULER ANGLES=" << test3.toString() << std::endl;
  std::cout << "EULER ANGLES=" << test4.toString() << std::endl;

  ow::Rotation3 test5 = ow::Rotation3::RPY(test3[2], test3[1], test3[0]);
  ow::Rotation3 test6 = ow::Rotation3::RPY(test4[2], test4[1], test4[0]);
  std::cout << "test5=\n" << test5.toString() << std::endl;
  std::cout << "test6=\n" << test6.toString() << std::endl;

  ow::Rotation3 R;
  ow::CartesianPosition Xpos;
  Xpos.setIdentity();

  R = Xpos.angular()*ow::Rotation3::Rz(0.0);

  Eigen::AngleAxisd aa = Eigen::AngleAxisd::Identity();
  Eigen::Quaterniond Qd = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d Rd = Eigen::Matrix3d::Identity();
  R = Qd*ow::Rotation3::Rz(0.0);

  //----------------------------------------------------------------------------
  ow::VectorX test_vec(10);
  test_vec.setZero();
  Eigen::Block<ow::VectorX> test_vec_sub(test_vec, 5, 0, 3, 1);
  test_vec_sub.setOnes();
  std::cout << "test_vec=" << test_vec.transpose() << std::endl;
  std::cout << "size=" << test_vec.size() << std::endl;
  std::cout << "size=" << test_vec_sub.size() << std::endl;

  //----------------------------------------------------------------------------
  ow::CartesianPosition Xpos1 = ow::CartesianPosition::Zero();
  ow::CartesianPosition Xpos2 = ow::CartesianPosition::Zero();
  Xpos2.angular() = ow::Rotation3::Rx(2.781)*ow::Rotation3::Rz(-0.78212);

  ow::CartesianVector err = ow::cartesianError(Xpos1, Xpos2);
  std::cout << "cart err=" << err.toString() << std::endl;
  std::cout << "cart err.linear=" << err.linear().toString() << std::endl;
  std::cout << "cart err.angular=" << err.angular().toString() << std::endl;

  //----------------------------------------------------------------------------
  ow::AngularPosition Q1 = ow::AngularPosition::Identity();
  ow::AngularPosition Q2;
  Q2 = ow::Rotation3::Rx(2.781)*ow::Rotation3::Rz(-0.78212);

  ow::Vector3 e;
  ow::quaternionLogError(e, Q1, Q2);
  std::cout << "err=" << e.toString() << std::endl;

  //----------------------------------------------------------------------------
  ow::HomogeneousTransformation H = ow::HomogeneousTransformation::Identity();
  H.position() << 1.0, 2.0, 3.0;
  H.orientation() = ow::Rotation3::Rx(1.21334*M_PI/4);

  ow::Wrench w1 = ow::Wrench::Zero();
  ow::Wrench w2;
  w2 = ow::changeRefFrame(w1, H);

  ow::CartesianState cstate1 = ow::CartesianState::Zero();
  ow::CartesianState cstate2;
  ow::CartesianPosition cpos;
  cstate2 = ow::changeRefFrame(cstate1, H);

  cpos = H;

  ow::Vector3 x_vec(1.0, 2.0, 3.0);
  ow::Vector3 y_vec = H*x_vec;
  ow::Vector3 y2_vec = Xpos1*x_vec;

  std::cout << "w2=" << w2.toString() << std::endl;
  std::cout << "cstate2=\n" << cstate2.toString() << std::endl;
  std::cout << "cpos=" << cpos.toString() << std::endl;

  std::cout << "error:" << std::endl;
  ow::CartesianVector error = ow::cartesianError(cstate1.pos(), cstate2.pos());

  //----------------------------------------------------------------------------

  ow_core::Rotation3<double> R1 = ow_core::Rotation3<double>::Identity();
  std::cout << R1.toString() << std::endl;

  ow_core::Rotation3<double> R2;
  R2 = Eigen::AngleAxisd(0.25*M_PI, Eigen::Vector3d::UnitX());
  std::cout << R2.toString() << std::endl;

  typedef Eigen::Matrix<double, 7, 1> PositionVec;
  PositionVec X;
  X.setZero();

  ow_core::LinearPositionRef<PositionVec> x(X);
  std::cout << x.toString() << std::endl;

  geometry_msgs::Point x_msg;
  x_msg.x = 1.0;
  x_msg.y = 2.0;
  x_msg.z = 3.0;
  x = x_msg;
  std::cout << x.toString() << std::endl;
  x_msg = x;

  tf::Vector3 x_tf(10, 20, 30);
  x = x_tf;
  std::cout << x.toString() << std::endl;
  x_tf = x;

  //----------------------------------------------------------------------------
  ow::HomogeneousTransformation H1_w = ow::HomogeneousTransformation::Identity();
  H1_w.position() << 1.0, 2.0, 3.0;
  H1_w.orientation() = ow::Rotation3::Rx(1.21334*M_PI/4);

  ow::HomogeneousTransformation H2_w = ow::HomogeneousTransformation::Identity();
  H2_w.position() << 1.0, 2.0, 5.0;
  H2_w.orientation() = ow::Rotation3::Rx(1.21334*M_PI/4);

  ow::HomogeneousTransformation H_2_1 = H1_w.inverse()*H2_w;
  std::cout << "H_2_1=\n" << H_2_1.toString() << std::endl;

  ow::CartesianPosition X1_w(H1_w);
  ow::CartesianPosition X2_w(H2_w);

  ow::CartesianPosition X_2_1;
  X_2_1.angular() = X1_w.angular().inverse()*X2_w.angular();
  X_2_1.position() = X1_w.angular().inverse()*X2_w.linear() - X1_w.angular().inverse()*X1_w.linear();
  
  ow::HomogeneousTransformation H_2_1_test;
  H_2_1_test = X_2_1;
  std::cout << "H_2_1_test=\n" << H_2_1_test.toString() << std::endl;

  ow::CartesianPosition Xw_1 = X1_w.inverse();
  ow::CartesianPosition X_2_1_ = Xw_1*X2_w;

  ow::HomogeneousTransformation H_2_1_test2;
  H_2_1_test2 = X_2_1_;
  std::cout << "H_2_1_test2=\n" << H_2_1_test2.toString() << std::endl;

  usleep(1000 * 1000);
  return 0;
}
