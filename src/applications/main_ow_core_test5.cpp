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

#include <ow_core/types.h>
#include <ow_core/math/transform.h>

int main(int argc, char* argv[])
{
  std::cout << "Testing the change reference frame functions." << std::endl;
  std::cout << "Check LinearState:" << std::endl;

  ow::LinearPosition lp1 = ow::LinearPosition::Zero();
  ow::LinearState s1 = ow::LinearState::Zero();
  ow::LinearState s2 = ow::LinearState::Zero();
  lp1 << 1,2,3;
  s1.pos() << 1, 2, 3;

  ow::HomogeneousTransformation T1 = ow::HomogeneousTransformation::Identity();
  T1.orientation() = ow::Rotation3::RPY(0.4*M_PI, 0.1*M_PI, 1.1*M_PI);

  ow::CartesianPosition X1;
  ow::CartesianState Xstate;
  X1 = T1;
  Xstate.pos() = T1;

  ow::CartesianPosition X2;

  ow::LinearPosition lp_a = ow::changeRefFrame(lp1, T1);
  ow::LinearPosition lp_b = ow::changeRefFrame(s1.pos(), T1);
  X2.pos() = ow::changeRefFrame(lp1, X1);
  ow::LinearPosition lp_d = ow::changeRefFrame(s1.pos(), X1);
  ow::LinearPosition lp_e = ow::changeRefFrame(s1.pos(), Xstate.pos());

  std::cout << "lp_a=" << lp_a.toString() << std::endl;
  std::cout << "lp_b=" << lp_b.toString() << std::endl;
  std::cout << "lp_c=" << X2.pos().toString() << std::endl;
  std::cout << "lp_d=" << lp_d.toString() << std::endl;

  T1.position() = ow::Vector3(1.0, 0.0, 0.0);
  T1.orientation() = ow::Rotation3::Rx(90.0/180.0*M_PI);

  std::cout << T1.toString() << std::endl;

  s1.pos() = ow::Vector3(0.0, 1.0, 0.0);
  s1.vel() = ow::Vector3(0.0, 1.0, 0.0);

  std::cout << s1.toString() << std::endl;
  s2 = ow::changeRefFrame(s1, T1);
  std::cout << s2.toString() << std::endl;

  std::cout << "Check Wrench:" << std::endl;
  ow::Wrench w1;
  ow::Wrench w2 = ow::Wrench::Zero();

//  w1.force() = ow::Vector3(0.0, 1.0, 0.0);
//  w1.moment() = ow::Vector3(0.0, 1.0, 0.0);

//  std::cout << w1.toString() << std::endl;

//  w2 = ow::changeRefFrame(w2,w1,T1);

//  std::cout << w2.toString() << std::endl;



  std::cout << "Check ZeroMomentPoint:" << std::endl;


  ow::LinearPosition p1;
  ow::LinearPosition p2 = ow::LinearPosition::Zero();

  p1 = ow::Vector3(0.0, 1.0, 0.0);

  std::cout << p1.toString() << std::endl;

  p2 = ow::changeRefFrame(p1, T1);

  std::cout << p2.toString() << std::endl;

  std::cout << "Check Force:" << std::endl;


  const ow::Force f1 = ow::Vector3(0.0, 1.0, 0.0);
  ow::Force f2 = ow::Force::Zero();

  geometry_msgs::Vector3 v;
  v.x = 1.0;
  v.y = 0.0;
  v.z = 0.0;

  f2 = v;
  f2 = w1.force();

  ow::Force f3 = w1.force();

  std::cout << f2.toString() << std::endl;

  v = w1.force().toVector3Msg();

//  w1.force() = v;

//  f1 = ow::Vector3(0.0, 1.0, 0.0);

  std::cout << f1.toString() << std::endl;

  f2 = ow::changeRefFrame(f1,T1);

  std::cout << f1.toString() << std::endl;
  std::cout << f2.toString() << std::endl;


  f2 = ow::changeRefFrame(w1.force(),T1);

  std::cout << f2.toString() << std::endl;

  w2.force() = ow::Force::Zero();
  std::cout << w2.toString() << std::endl;

  w2.force() = ow::changeRefFrame(w1.force(),T1);

  std::cout << w2.toString() << std::endl;

  w2.force() = ow::changeRefFrame(w1.force(),T1);

  std::cout << w2.toString() << std::endl;


  usleep(1000 * 1000);
  return 0;
}
