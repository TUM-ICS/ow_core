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


#ifndef OPEN_WALKER_CORE_TYPES_H
#define OPEN_WALKER_CORE_TYPES_H

/*! \file types.h
 *  \brief Contains all the types.
 */

// configuration
#include <ow_core/configuration.h>
#include <ow_core/common/eigen_typedef_macros.h>

// basic types
#include <ow_core/types/matrix.h>
#include <ow_core/types/rotation3.h>
#include <ow_core/types/vector3.h>

// states
#include <ow_core/types/angular_state.h>
#include <ow_core/types/linear_state.h>
#include <ow_core/types/cartesian_state.h>
#include <ow_core/types/joint_state.h>

// special types
#include <ow_core/types/spatial_vector.h>
#include <ow_core/types/cartesian_vector.h>
#include <ow_core/types/homogeneous_transformation.h>

// walking types
#include <ow_core/types/flags.h>
#include <ow_core/types/foot_step.h>
#include <ow_core/types/dcm_point_set.h>

// sensor types
#include <ow_core/types/inertial_measurment_unit_sensor.h>
#include <ow_core/types/force_torque_sensor.h>


// the namespace for the configured types
namespace ow
{
  /*! Type for Scalars */
  typedef OW_TYPES_SCALAR Scalar;

  /*! Basic Types */
  typedef ow_core::Rotation3<Scalar> Rotation3;
  typedef ow_core::Vector3<Scalar> Vector3;

  typedef ow_core::VectorDof<Scalar,OW_VECTOR_DOF_ROWS> VectorDof;
  typedef ow_core::JointPosition<Scalar,OW_VECTOR_DOF_ROWS> JointPosition;
  typedef ow_core::JointVelocity<Scalar,OW_VECTOR_DOF_ROWS> JointVelocity;
  typedef ow_core::JointAcceleration<Scalar,OW_VECTOR_DOF_ROWS> JointAcceleration;
  typedef ow_core::JointEffort<Scalar,OW_VECTOR_DOF_ROWS> JointEffort;
  typedef ow_core::JointState<Scalar,OW_VECTOR_DOF_ROWS> JointState;

  typedef ow_core::JointState<Scalar> JointStateX;
  typedef ow_core::JointPosition<Scalar> JointPositionX;
  typedef ow_core::JointVelocity<Scalar> JointVelocityX;
  typedef ow_core::JointAcceleration<Scalar> JointAccelerationX;
  typedef ow_core::JointEffort<Scalar> JointEffortX;

  typedef ow_core::AngularPosition<Scalar> AngularPosition;
  typedef ow_core::AngularVelocity<Scalar> AngularVelocity;
  typedef ow_core::AngularAcceleration<Scalar> AngularAcceleration;
  typedef ow_core::Moment<Scalar> Moment;
  typedef ow_core::AngularState<Scalar> AngularState;

  typedef ow_core::LinearPosition<Scalar> LinearPosition;
  typedef ow_core::LinearVelocity<Scalar> LinearVelocity;
  typedef ow_core::LinearAcceleration<Scalar> LinearAcceleration;
  typedef ow_core::Force<Scalar> Force;
  typedef ow_core::LinearState<Scalar> LinearState;

  typedef ow_core::HomogeneousTransformation<Scalar> HomogeneousTransformation;

  typedef ow_core::CartesianPosition<Scalar> CartesianPosition;
  typedef ow_core::CartesianVelocity<Scalar> CartesianVelocity;
  typedef ow_core::CartesianAcceleration<Scalar> CartesianAcceleration;
  typedef ow_core::Wrench<Scalar> Wrench;
  typedef ow_core::CartesianState<Scalar> CartesianState;

  typedef ow_core::SpatialVector<Scalar> SpatialVector;
  typedef ow_core::CartesianVector<Scalar> CartesianVector;

  /*! Walking specific types */
  typedef ow_core::FootId FootId;
  typedef ow_core::Flags Flags;

  typedef ow_core::DCMPointSet<Scalar> DCMPointSet;
  typedef std::vector<DCMPointSet> DCMPointSetList;

  typedef ow_core::FootStep<Scalar> FootStep;
  typedef std::vector<FootStep> FootStepList;

  typedef ow_core::InertialMeasurmentUnitSensor<Scalar> ImuSensor;
  typedef ow_core::ForceTorqueSensor<Scalar> FTSensor;

  /*! Eigen specific types */
  OPEN_WALKER_CORE_TYPEDEFS_ALL_SIZES(unsigned int, ui)
  OPEN_WALKER_CORE_TYPEDEFS_ALL_SIZES(int, i)

  typedef ow_core::Matrix<Scalar, 6, 1> Vector6;
  typedef ow_core::Matrix<Scalar, 7, 1> Vector7;
  typedef ow_core::Matrix<Scalar, 2, 1> Vector2;

  typedef ow_core::Matrix<Scalar, 2, 2> Matrix2;
  typedef ow_core::Matrix<Scalar, 3, 3> Matrix3;
  typedef ow_core::Matrix<Scalar, 4, 4> Matrix4;
  typedef ow_core::Matrix<Scalar, 6, 6> Matrix6;

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

  typedef Vector2 Point2d;
  typedef Vector3 Point3d;
  typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> Points2d;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Points3d;

  template<typename Derived> 
  using Ref = Eigen::Ref<Derived>;
}

#endif  // OPEN_WALKER_CORE_TYPES_H
