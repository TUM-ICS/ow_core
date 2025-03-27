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

#ifndef OPEN_WALKER_CORE_DCM_POINT_SET_H
#define OPEN_WALKER_CORE_DCM_POINT_SET_H

#include <ow_core/types/linear_position.h>
#include <ow_core/types/angular_position.h>
#include <ow_core/types/linear_velocity.h>
#include <ow_core/types/foot_id.h>

#include <ow_msgs/DCMPointSet.h>

namespace ow_core{

/*!
 * \brief The DCMPointSet class.
 *
 *  This class is a container for:
 *    - DCMPointSet position.
 *    - Foot id (Left,Right).
 *    - Final step flag (true or false)
 *    - n step. Number of step in the plan.
 */
template<typename _Scalar>
class DCMPointSet 
{
public:

  typedef LinearPosition<_Scalar> Vrp;
  typedef LinearPosition<_Scalar> Dcm;
  typedef AngularPosition<_Scalar> Quaternion;
  typedef LinearPosition<_Scalar> Dsi;
  typedef LinearVelocity<_Scalar> DsiP;
  typedef LinearPosition<_Scalar> Dsf;
  typedef LinearVelocity<_Scalar> DsfP;

public:
  /*!
   * \brief Construct as Default
   */
  static const DCMPointSet<_Scalar>& Default()
  {
    static const DCMPointSet v;
    return v;
  }

  /*!
   * \brief Construct as Zero
   */
  static const DCMPointSet<_Scalar>& Zero()
  {
    static const DCMPointSet v;
    return v;
  }

protected:
  Vrp vrp_;
  Dcm dcm_;
  Quaternion Q_;
  Dsi dsi_;
  DsiP dsiP_;
  Dsf dsf_;
  DsfP dsfP_;
  FootId foot_id_;
  unsigned int n_point_;

public:
  /*!
    * \brief Default Constructor.
    */
  DCMPointSet():
    vrp_(Vrp::Zero()),
    dcm_(Dcm::Zero()),
    Q_(Quaternion::Identity()),
    dsi_(Dsi::Zero()),
    dsiP_(DsiP::Zero()),
    dsf_(Dsf::Zero()),
    dsfP_(DsfP::Zero()),
    foot_id_(FootId::LEFT),
    n_point_(0)
  {
  }

  /*!
   * \brief Copy constructor.
   */
  DCMPointSet(const DCMPointSet& other) :
    vrp_(other.vrp_),
    dcm_(other.dcm_),
    Q_(other.Q_),
    dsi_(other.dsi_),
    dsiP_(other.dsiP_),
    dsf_(other.dsf_),
    dsfP_(other.dsfP_),
    foot_id_(other.foot_id_),
    n_point_(other.n_point_)
  {
  }

  /*!
    * \brief Constructor from sub elements
   */
  DCMPointSet(const Vrp& vrp,
              const Dcm& dcm,
              const Quaternion& Quat,
              const Dsi& dsi,
              const DsiP& dsiP,
              const Dsf& dsf,
              const DsfP& dsfP,
              const FootId& foot_id = FootId::LEFT,
              const unsigned int nPoint = 0) :
    vrp_(vrp),
    dcm_(dcm),
    Q_(Quat),
    dsi_(dsi),
    dsiP_(dsiP),
    dsf_(dsf),
    dsfP_(dsfP),
    foot_id_(foot_id),
    n_point_(nPoint)
  {
  }

  /*!
   * \brief Assignment from DCMPointSet.
   */
  DCMPointSet& operator=(const DCMPointSet& other)
  {
    vrp_ = other.vrp();
    dcm_ = other.dcm();
    Q_ = other.Q();
    dsi_ = other.dsi();
    dsiP_ = other.dsiP();
    dsf_ = other.dsf();
    dsfP_ = other.dsfP();
    foot_id_ = other.footId();
    n_point_ = other.nPoint();
    return *this;
  }

  Vrp& vrp()
  {
    return vrp_;
  }

  const Vrp& vrp() const
  {
    return vrp_;
  }

  Dcm& dcm()
  {
    return dcm_;
  }

  const Dcm& dcm() const
  {
    return dcm_;
  }

  Quaternion& Q()
  {
    return Q_;
  }

  const Quaternion& Q() const
  {
    return Q_;
  }

  Dsi& dsi()
  {
    return dsi_;
  }

  const Dsi& dsi() const
  {
    return dsi_;
  }

  DsiP& dsiP()
  {
    return dsiP_;
  }

  const DsiP& dsiP() const
  {
    return dsiP_;
  }

  Dsf& dsf()
  {
    return dsf_;
  }

  const Dsf& dsf() const
  {
    return dsf_;
  }

  DsfP& dsfP()
  {
    return dsfP_;
  }

  const DsfP& dsfP() const
  {
    return dsfP_;
  }

  FootId& footId()
  {
    return foot_id_;
  }

  const FootId& footId() const
  {
    return foot_id_;
  }

  unsigned int& nPoint()
  {
    return n_point_;
  }

  const unsigned int& nPoint() const
  {
    return n_point_;
  }

  /*!
   * \brief Assignment of ow_msgs::DCMPointSet.
   */
  DCMPointSet& operator=(const ow_msgs::DCMPointSet& msg)
  {
    vrp_ = msg.vrp;
    dcm_ = msg.dcm;
    Q_ = msg.hip_orientation;
    dsi_ = msg.dsi;
    dsiP_ = msg.dsiP;
    dsf_ = msg.dsf;
    dsfP_ = msg.dsfP;
    foot_id_ = FootId(static_cast<FootId::Value>(msg.foot_id.data));
    n_point_ = static_cast<unsigned int>(msg.n_step.data);
    return *this;
  }

  /*!
   * \brief Conversion to ow_msgs::DCMPointSet.
   */
  operator ow_msgs::DCMPointSet() const
  {
    ow_msgs::DCMPointSet msg;
    msg.vrp = vrp_;
    msg.dcm = dcm_;
    msg.hip_orientation = Q_;
    msg.dsi = dsi_;
    msg.dsiP = dsiP_;
    msg.dsf = dsf_;
    msg.dsfP = dsfP_;
    msg.foot_id.data = static_cast<int>(foot_id_);
    msg.n_step.data = static_cast<int>(n_point_);
    return msg;
  }

  /*!
   * \brief Conversion to ow_msgs::DCMPointSet.
   */
  ow_msgs::DCMPointSet toDCMPointSetMsg() const
  {
    return static_cast<ow_msgs::DCMPointSet>(*this);
  }

  /*!
   * \brief Conversion to string
   */
  std::string toString() const
  {
    std::stringstream out;
    out << "vrp:     " << vrp_.toString() << "\n";
    out << "dcm:     " << dcm_.toString() << "\n";
    out << "Q:       " << Q_.toString() << "\n";
    out << "dsi_:    " << dsi_.toString() << "\n";
    out << "dsiP_:   " << dsiP_.toString() << "\n";
    out << "dsf_:    " << dsf_.toString() << "\n";
    out << "dsfP_:   " << dsfP_.toString() << "\n";
    out << "foot_id_:" << foot_id_.toString() << "\n";
    out << "n_step:  " << n_point_ << "\n";
    return out.str();
  }

};

}

#endif // OPEN_WALKER_CORE_DCM_POINT_SET_H
