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


#ifndef OPEN_WALKER_CORE_INTERFACES_CONTROLLER_BASE_H_
#define OPEN_WALKER_CORE_INTERFACES_CONTROLLER_BASE_H_

#include <memory>

#include <ow_core/interfaces/generic_module_base.h>

#include <ow_core/interfaces/i_hw_interface.h>

namespace ow {

/*!
 * \brief The ControllerBase class.
 * 
 * This provides the basic functionallity to start, stop and update a
 * openwalker controller.
 */
class ControllerBase :
  public GenericModuleBase
{
public:
  typedef GenericModuleBase Base;

public:
  /*!
   * \brief Constructor.
   */
  ControllerBase(std::string name) : 
    GenericModuleBase(name)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~ControllerBase()
  {
  }

  /** 
  * \brief start the controller
  *
  * \param starting time
  */
  bool startRequest(const IHwInterface& robot, const ros::Time& time) 
  {
    if(Base::setStartState(time))
    {
      start(robot, time);
      return true;
    }
    return false;
  }

  /** 
  * \brief stop the controller
  *
  * \param stopping time
  */
  bool stopRequest(const IHwInterface& robot, const ros::Time& time) 
  {
    if(Base::setStopState(time))
    {
      stop(robot, time);
      return true;
    }
    return false;
  }

  /** 
  * \brief performs update step of the controller, called periodically
  *
  * \param current jointstate_
  * \param current time
  * \param current deltatime since last call
  */
  ow::JointState updateRequest(
    const IHwInterface& robot,
    const ros::Time& time,
    const ros::Duration& dt) 
  {
    if(Base::isRunning())
      return update(robot, time, dt);
    
    ROS_ERROR("%s::updateRequest call but not initalized.", name_.c_str());
    return ow::JointState::Zero();
  }

protected:
  /** 
  * \brief start the controller, called befor update, optional
  *
  * \param starting time
  */
  virtual void start(const IHwInterface& robot, const ros::Time& time)
  {
  }

  /** 
  * \brief performs update step of the controller, called periodically
  *
  * \param current jointstate
  * \param current time
  * \param current deltatime since last call
  */
  virtual ow::JointState update(
    const IHwInterface& robot, 
    const ros::Time& time, 
    const ros::Duration& dt) = 0;

  /** 
  * \brief stop the controller, called befor stopping, optional
  *
  * \param stopping time
  */
  virtual void stop(const IHwInterface& robot, const ros::Time& time)
  {
  }

};

typedef std::shared_ptr<ControllerBase> ControllerBasePtr;

}

#endif // OPEN_WALKER_CORE_INTERFACES_CONTROLLER_BASE_H_
