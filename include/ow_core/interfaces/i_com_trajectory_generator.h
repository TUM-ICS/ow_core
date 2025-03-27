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


#ifndef OPEN_WALKER_CORE_INTERFACES_I_COM_TRAJECTORY_GENERATOR_H
#define OPEN_WALKER_CORE_INTERFACES_I_COM_TRAJECTORY_GENERATOR_H

#include <ow_core/types.h>
#include <ow_core/interfaces/module_base.h>

namespace ow {

/*!
 * \brief The ICOMTrajectoryGenerator class.
 *
 * Interface class.
 *
 */
class ICOMTrajectoryGenerator : 
  public ModuleBase
{
public:
  ICOMTrajectoryGenerator(const std::string& name) :
    ModuleBase(name)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~ICOMTrajectoryGenerator()
  {
  }

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the Reference trajectory for the ZMP.
  */
  virtual const ow::LinearState& ZMP_w() const = 0;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the Reference trajectory for the DCM.
  */
  virtual const ow::LinearState& DCM_w() const = 0;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the Reference trajectory for the CoM. 
  *    This includes both po-sition and orientation of a virtual link located 
  *    at the CoMwith the same oriantation of the base link.
  */
  virtual const ow::CartesianState& X_com_w() const = 0;
};

}

#endif // OPEN_WALKER_CORE_INTERFACES_I_COM_TRAJECTORY_GENERATOR_H
