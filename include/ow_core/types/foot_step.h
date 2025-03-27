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


#ifndef OPEN_WALKER_CORE_FOOT_STEP_H
#define OPEN_WALKER_CORE_FOOT_STEP_H

#include <ow_core/types/cartesian_position.h>
#include <ow_core/types/foot_id.h>

#include <ow_msgs/FootStep.h>

namespace ow_core{

/*!
 * \brief The FootStep class.
 *
 *  This class is a container for:
 *    - Footstep position.
 *    - Foot id (Left,Right).
 *    - Final step flag (true or false)
 *    - n step. Number of step in the plan.
 */
template<typename _Scalar>
class FootStep 
{
public:
  typedef _Scalar Scalar;
  typedef CartesianPosition<Scalar> Position;

  /*!
   * \brief Construct as Zero
   */
  static const FootStep<_Scalar>& Zero()
  {
    static const FootStep v;
    return v;
  }

protected:
  Position      X_;
  FootId        foot_id_;
  bool          final_step_;
  unsigned int  n_step_;

public:
  /*!
    * \brief Default Constructor.
    */
  FootStep():
    X_(Position::Zero()),
    foot_id_(FootId::LEFT),
    final_step_(false),
    n_step_(0)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  FootStep(const FootStep& other) :
    X_(other.X_),
    foot_id_(other.foot_id_),
    final_step_(other.final_step_),
    n_step_(other.n_step_)
  {
  }


  /*!
    * \brief Constructor from sub elements
   */
  explicit FootStep(const Position& X,
                    const FootId& foot_id = FootId::LEFT,
                    const bool& final_step = false,
                    const unsigned int& n_step = 0) :
    X_(X),
    foot_id_(foot_id),
    final_step_(final_step),
    n_step_(n_step)
  {
  }

  Position& X()
  {
    return X_;
  }

  Position& pos()
  {
    return X_;
  }

  const Position& X() const
  {
    return X_;
  }

  const Position& pos() const
  {
    return X_;
  }

  FootId& footId()
  {
    return foot_id_;
  }

  const FootId& footId() const
  {
    return foot_id_;
  }

  bool& finalStep()
  {
    return final_step_;
  }

  const bool& finalStep() const
  {
    return final_step_;
  }

  unsigned int& nStep()
  {
    return n_step_;
  }

  const unsigned int& nStep() const
  {
    return n_step_;
  }

  /*!
   * \brief Assignment of ow_msgs::FootStep.
   */
  FootStep& operator=(const ow_msgs::FootStep& msg)
  {
    X_ = msg.position;
    foot_id_ = FootId(static_cast<FootId::Value>(msg.foot_id.data));
    final_step_ = msg.final_step.data;
    n_step_ = msg.n_step.data;
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::FootStep.
   */
  operator ow_msgs::FootStep() const
  {
    ow_msgs::FootStep msg;
    msg.position = X_;
    msg.foot_id.data = static_cast<int>(foot_id_);
    msg.final_step.data = final_step_;
    msg.n_step.data = n_step_;
    return msg;
  }  

  /*!
   * \brief Conversion to ow_msgs::FootStep.
   */
  ow_msgs::FootStep toFootStepMsg() const
  {
    return static_cast<ow_msgs::FootStep>(*this);
  }

  std::string toString() const
  {
    std::stringstream out;
    out << "nStep: " << n_step_ << "\n";
    out << "Pose: " << X_.toString() << "\n";
    out << "foot: " << foot_id_.toString() << "\n";
    out << "final: " << final_step_ << "\n";
    return out.str();
  }

};

}

#endif // OPEN_WALKER_CORE_FOOT_STEP_H
