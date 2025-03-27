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


#ifndef OPEN_WALKER_CORE_GENERIC_MODEL_BASE_H_
#define OPEN_WALKER_CORE_GENERIC_MODEL_BASE_H_

#include <ros/node_handle.h>
#include <ros/time.h>

#include <ow_core/plugins/i_generic_class.h>
#include <ow_core/common/parameter.h>

namespace ow {

/*!
 * \brief The GenericModuleBase class.
 * 
 * This provides the basic functionallity to initialize a module.
 * Keeps track of module state.
 */
class GenericModuleBase : 
  ow_core::IGenericClass
{
public:
  /*!
   * \brief Current state of the module.
   */
  enum State {CONSTRUCTED, INITIALIZED, RUNNING};

protected:
  State state_;                 //!< the current module state
  std::string name_;            //!< the module name
  ros::Time start_time_;        //!< start time of the module
  ros::Time stop_time_;         //!< stop time of the module

public:
  /*!
   * \brief Constructor.
   */
  GenericModuleBase(const std::string& name) : 
    name_(name),
    state_(CONSTRUCTED)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~GenericModuleBase()
  {
  }

  /** 
  * \brief return if is running
  *
  * \return running flag
  */
  bool isInitialized() const 
  { 
    return state_ == INITIALIZED; 
  }

  /** 
  * \brief return if is running
  *
  * \return running flag
  */
  bool isRunning() const 
  { 
    return state_ == RUNNING; 
  }

  /** 
   * \brief get state name
   */
  std::string name() const 
  { 
    return name_;
  }

  /** 
   * \brief get state name
   */
  std::string classType() const
  {
    return name_;
  }

  /**
   * \brief get start time
   */
  ros::Time startTime() const 
  {
    return start_time_;
  }

  /**
   * \brief get stop time
   */
  ros::Time stopTime() const 
  {
    return stop_time_;
  }

  /** 
  * \brief init interal\parameter
  *
  * \param ros nh for namespace
  */
  bool initRequest(const ow::Parameter& parameter, ros::NodeHandle& nh) 
  {
    if(init(parameter, nh)) {
      state_ = INITIALIZED;
      return true;
    }
    return false;
  }

protected:
  /** 
  * \brief init interal\parameter
  *
  * \param ros nh for namespace
  */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh) = 0;


  /** 
  * \brief start the module
  *
  * \param starting time
  */
  bool setStartState(const ros::Time& time) 
  {
    if(state_ == INITIALIZED) 
    {
      state_ = RUNNING;
      start_time_ = time;
      return true;
    }
    return false;
  }

  /** 
  * \brief stop the module
  *
  * \param stopping time
  */
  bool setStopState(const ros::Time& time) 
  {
    if(state_ == RUNNING) 
    {
      stop_time_ = time;
      state_ = INITIALIZED;
      return true;
    }
    return false;
  }
};

}

#endif // OPEN_WALKER_CORE_GENERIC_MODEL_BASE_H_
