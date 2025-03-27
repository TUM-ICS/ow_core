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


#ifndef OPEN_WALKER_CORE_INTERFACES_I_COM_ESTIMATOR_H
#define OPEN_WALKER_CORE_INTERFACES_I_COM_ESTIMATOR_H

#include <ow_core/types.h>
#include <ow_core/interfaces/module_base.h>

namespace ow {

/*!
 * \brief The ICOMEstimator class.
 *
 * Interface class.
 *
 */
class ICOMEstimator : 
  public ModuleBase
{
public:
  ICOMEstimator(const std::string& name) :
    ModuleBase(name)
  {
  }

  /*!
   * \brief Virtual destructor.
   */
  virtual ~ICOMEstimator()
  {
  }

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState for the filtered pose of the real CoM.
  */
  virtual const ow::CartesianState& Xf_com_w() const = 0;

  /*!
  * \brief Output port function.
  *
  * \return
  *    DCMState of the real robot
  */
  virtual const ow::LinearState& DCMr_w() const = 0;

};

}

#endif // OPEN_WALKER_CORE_INTERFACES_I_COM_ESTIMATOR_H
