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


#ifndef OPEN_WALKER_CORE_MODEL_BASE_H_
#define OPEN_WALKER_CORE_MODEL_BASE_H_

#include <ow_core/types.h>

#include <ow_core/interfaces/generic_module_base.h>

namespace ow {

/*!
 * \brief The ModuleBase class.
 * 
 * This provides the basic functionallity to start, stop and update a 
 * openwalker module.
 */
class ModuleBase : 
  public GenericModuleBase
{
public:
  typedef GenericModuleBase Base;

public:
  /**
   * @brief Construct a new Module Base object
   * 
   * @param name the name of this module
   */
  ModuleBase(const std::string& name) : 
    GenericModuleBase(name)
  {
  }

  /**
   * @brief Destroy the Module Base object
   * 
   */
  virtual ~ModuleBase()
  {
  }

  /**
   * @brief start this module
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   * @return true successfully initalized
   * @return false error during initalization
   */
  bool startRequest(ow::Flags& flags, const ros::Time& time) 
  {
    if(Base::setStartState(time))
    {
      start(flags, time);
      return true;
    }
    return false;
  }

  /**
   * @brief stop this module
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   * @return true true stopped successfully
   * @return false error during stopping 
   */
  bool stopRequest(ow::Flags& flags, const ros::Time& time) 
  {
    if(Base::setStopState(time))
    {
      start(flags, time);
      return true;
    }
    return false;
  }

  /**
   * @brief performs update step of the module, called periodically
   * 
   * \note For now this function is not used, instead each module implements
   *  its own update function.
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   * @param dt the elapsed time since last update call
   *
  void updateRequest(ow::Flags& flags, const ros::Time& time, const ros::Duration& dt) 
  {
    if(Base::isRunning()) 
    {
      update(flags, time, dt);
      return;
    }
    ROS_ERROR("%s::updateRequest call but not initalized.", name_.c_str());
  }*/

protected:

  /**
   * @brief start the module, called befor update
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   */
  virtual void start(ow::Flags& flags, const ros::Time& time)
  {
  }

  /**
   * @brief performs update step of the module, called periodically
   * 
   * \note For now this function is not used, instead each module implements
   *  its own update function.
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   * @param dt the elapsed time since last update call
   *
  virtual void update(
    ow::Flags& flags, 
    const ros::Time& time, 
    const ros::Duration& dt) = 0;*/

  /**
   * @brief stop the module, called befor stopping
   * 
   * @param flags the current status flags of the robot
   * @param time the current robot time
   */
  virtual void stop(ow::Flags& flags, const ros::Time& time)
  {
  }

};

}

#endif // OPEN_WALKER_CORE_MODEL_BASE_H_
