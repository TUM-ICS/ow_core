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
#include <ow_core/types/vector_dof.h>

int main(int argc, char* argv[])
{
    ow::LinearPosition zmp(1.0, 2.0, 3.0);
    ow::ZeroMomentPointVelocity zmpP(1.0, 2.0, 3.0);
    ow::ZeroMomentPointAcceleration zmpPP(1.0, 2.0, 3.0);
    ow::LinearState state(zmp, zmpP, zmpPP);

    std::cout << "zmp=" << state.x().toString() << std::endl;
    std::cout << "zmpP=" << state.pP().toString() << std::endl;
    std::cout << "zmpPP=" << state.pPP().toString() << std::endl;

    ow::VectorDof v;
    v = ow::JointPosition::Random();

    std::cout << v.toString() << std::endl;

    v.setZero();
    v.setLinSpaced(-1,1);

    std::cout << v.toString() << std::endl;

    ow::JointState js;

    js = ow::JointState::Zero();

    ow::LinearAcceleration xPP;
    xPP = Eigen::Vector3d::Zero();

    std::cout << xPP.toString() << std::endl;

    ow::AngularPosition Q;
    ow::AngularVelocity omega;

    Eigen::Quaterniond QE;
    Q = QE;

    ow_core::ForceTorqueSensors fts;
    ow_core::IForceTorqueSensors* intf = &fts;
    std::cout << "before" << std::endl;

    fts.leftFoot() = ow::Wrench::Ones();

    std::cout << "after" << std::endl;

//    ow_core::RobotOutPorts out;

    usleep(1000*1000);
    return 0;
}


