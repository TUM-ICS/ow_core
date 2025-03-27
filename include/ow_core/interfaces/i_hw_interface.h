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


#ifndef OPEN_WALKER_CORE_INTERFACES_I_HW_INTERFACE_H
#define OPEN_WALKER_CORE_INTERFACES_I_HW_INTERFACE_H

#include <ow_core/types.h>
#include <ow_core/interfaces/module_base.h>

namespace ow {

/*!
 * \brief The IHwInterface class.
 *
 * This is the Interface class that abstracts the robot and gives access to 
 * its jointstate and sensor readings.
 *
 * IHwInterface is used by the controller_base class.
 * 
 * This class is special since it directly couples to the
 * ros_control::interface in the class ros_control_plugin.h
 * 
 */
class IHwInterface : 
  public GenericModuleBase
{
public:
  IHwInterface(const std::string& name) :
    GenericModuleBase(name)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~IHwInterface()
  {
  }

  /*!
   * \brief Output port function.
   *
   * \return
   *    JointState of the last robot command.
   */
  virtual const ow::JointState& lastJointStateCommand() const = 0;

  /*!
   * \brief Output port function.
   *
   * \return
   *    JointState of the current robot command.
   */
  virtual const ow::JointState& jointStateCommand() const = 0;

  /*!
   * \brief Output port function.
   *
   * \return
   *    JointState of the real robot.
   */
  virtual const ow::JointState& jointStateReal() const = 0;

  virtual bool hasImuSensor() const = 0;

  /*!
   * \brief Output port function.
   *
   * \return
   *    ImuSensor of the robot.
   */
  virtual const ow::ImuSensor& imu() const = 0;

  /*!
   * \brief Output port function.
   *
   * \return
   *    force torque wrench of the robot.
   */
  virtual const ow::Wrench& forceTorqueRight() const = 0;

  /*!
   * \brief Output port function.
   *
   * \return
   *    force torque wrench of the robot.
   */
  virtual const ow::Wrench& forceTorqueLeft() const = 0;

};

}

#endif // OPEN_WALKER_CORE_INTERFACES_I_INVERSE_KINEMATICS_H
