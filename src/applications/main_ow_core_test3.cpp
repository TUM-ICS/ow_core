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
#include <ow_core/algorithms.h>

int main(int argc, char* argv[])
{
  // ------------------------------------------------------------------
  // FiniteDifference: Scalar Input
  // ------------------------------------------------------------------
  double f_s = 1.0;

  ow_core::ScalarFiniteDifference<double> diff = 
    ow_core::ScalarFiniteDifference<double>::FirstOrderAccurarcyThree(f_s);

  double x = 0;

  std::vector<double> x_vec(6);
  for(int i = 0; i < x_vec.size(); ++i) {
    x_vec[i] = i;
  }
  //std::iota(x_vec.begin(), x_vec.end(), 0);
  
  for(int i = 0; i < x_vec.size(); ++i) {
    double y = diff.update(x_vec[i]);
    if(diff.valid()) {
      std::cout << "valid ouput:" << std::endl;
    }
    std::cout << "[" << i << "]: y=" << y << std::endl;
  }

  // ------------------------------------------------------------------
  // FiniteDifference: Matrix Input
  // ------------------------------------------------------------------
  typedef Eigen::Matrix2d M;

  // new user initalization
  ow_core::MatrixAlgorithm<M> diff_matrix(
    ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(f_s));

  // old user initalization
  // ow_core::FiniteDifference<M> diff_matrix(1, ow_core::algo::DIFF_FIRST_ORDER_ACC_THREE);

  for(int i = 0; i < x_vec.size(); ++i) {
    M X = x_vec[i]*M::Ones();
    M Y = diff_matrix.update(X);
    if(diff_matrix.valid()) {
      std::cout << "valid ouput:" << std::endl;
    }
    std::cout << "[" << i << "]: Y=\n" << Y << std::endl;
  }

  // ------------------------------------------------------------------
  // FiniteDifference: LinearState
  // ------------------------------------------------------------------
  ow::LinearState state = ow::LinearState::Zero();

  // new user initalization
  ow::StateDifferentiator<ow::LinearState> updater(
    ow_core::ScalarFiniteDifference<ow::Scalar>::FirstOrderAccurarcyThree(f_s)
  );

  // old user initalization
  // ow_core::ScalarFiniteDifference<ow::Scalar> diff2(1, ow_core::algo::DIFF_FIRST_ORDER_ACC_THREE);
  // ow::StateDifferentiator<ow::LinearState> updater(diff2);

  for(int i = 0; i < x_vec.size(); ++i) {
    state.pos() = x_vec[i]*ow::LinearPosition::Ones();

    state = updater.update(state);
    if(updater.valid()) {
      std::cout << "valid ouput:" << std::endl;
    }
    std::cout << "[" << i << "]: state.vel=" << state.vel().toString() << std::endl;
    std::cout << "[" << i << "]: state.acc=" << state.acc().toString() << std::endl;
  }

  /*ow::AngularPosition angular = ow::AngularPosition::Identity();
  ow_core::FiniteDifference<Eigen::Vector4d> test(1,1,3);
  angular.coeffs() = test.update(angular.coeffs());*/

  usleep(1000 * 1000);
  return 0;
}
