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

#include <ow_core/algorithms/matrix_algorithm.h>

#include <iostream>
#include <stdio.h>
#include <fstream>

#include <ow_core/types.h>
#include <ow_core/algorithms.h>
#include <ctime>

int main()
{
  // ------------------------------------------------------------------
  // Exponential Decay Filter: Scalar Input
  /* ------------------------------------------------------------------
  int n = 6;
  std::vector<double> x_in(n, 0.0); x_in[0] = 1.0;
  std::vector<double> x_out(n, 0.0);

  ow_core::ScalarExponentialDecayFilter<double> scalar_exp_f(0.6);
  for(int i = 0; i < x_in.size(); ++i)
    x_out[i] = scalar_exp_f.update(x_in[i]);

  std::cout << std::endl << "-----------------------" << std::endl;
  std::cout << "Scalar Exp Decay Filter" << std::endl;
  for(int i = 0; i < x_out.size(); ++i)
    printf("t[%d]: in=%4.2f -> out=%4.2f\n", int(i), x_in[i], x_out[i]);

  // ------------------------------------------------------------------
  // Exponential Decay Filter: Matrix Input
  // ------------------------------------------------------------------
  std::vector<Eigen::Matrix2d> X_in(n, Eigen::Matrix2d::Zero()); X_in[0] = Eigen::Matrix2d::Ones();
  std::vector<Eigen::Matrix2d> X_out(n, Eigen::Matrix2d::Zero()); 

  ow_core::ExponentialDecayFilter<Eigen::Matrix2d> matrix_exp_f(0.6);
  for(int i = 0; i < X_in.size(); ++i)
    X_out[i] = matrix_exp_f.update(X_in[i]);

  std::cout << std::endl << "-----------------------" << std::endl;
  std::cout << "Matrix Exp Decay Filter" << std::endl;
  for(int i = 0; i < x_out.size(); ++i) {
    std::cout << "t[" << i << "]:" << std::endl;
    std::cout << "in=\n" << X_in[i] << std::endl;
    std::cout << "out=\n" << X_out[i] << std::endl;
  }*/



  // ------------------------------------------------------------------
  // Butterworth HighPass Filter: Scalar Input
  // ------------------------------------------------------------------
  int n_iter = 100000;
  double f_cutoff = 5;
  double f_sample = 200;

  // make filter
  ow::ScalarButterWorthFilter scalar_bw_lp_f = 
    ow::ScalarButterWorthFilter::LowPassSecondOrder(f_sample, f_cutoff);

  // build test signal
  double f1, f2, t_end;
  f1 = 0.5;
  f2 = 10;
  t_end = 5.0;
  std::vector<double> t_samples(t_end*f_sample);
  std::vector<double> x_samples(t_end*f_sample);
  std::vector<double> y_samples(t_end*f_sample);
  for(int i = 0; i < t_samples.size(); ++i) 
  {
    t_samples[i] = i/f_sample;
    x_samples[i] = 2.0*std::cos(2*M_PI*f1*t_samples[i]) 
      + 4.0*std::sin(2*M_PI*f2*t_samples[i] + 0.2);
  }

  // pull through
  for(int i = 0; i < t_samples.size(); ++i)
    y_samples[i] = scalar_bw_lp_f.update(x_samples[i]);

  // output
  std::cout << std::endl << "-----------------------" << std::endl;
  std::cout << "Butterworth Lowpass Filter Scalar" << std::endl;
  for(int i = 0; i < t_samples.size(); ++i)
  {
    printf("i[%d]: t=%4.2f x=%4.2f -> y=%4.5f\n", 
      int(i), t_samples[i], x_samples[i], y_samples[i]);
  }

  std::ofstream myfile;
  myfile.open("/home/simon/ros/workspaces/tmp/tests/filter.txt");
  for(int i = 0; i < t_samples.size(); ++i)
  {
    myfile << t_samples[i] << " " << x_samples[i] << " " << y_samples[i] << std::endl;
  }
  myfile.close();

  /* ------------------------------------------------------------------
  // Butterworth Lowpass Filter: Matrix Input
  // ------------------------------------------------------------------
  typedef ow_core::MatrixAlgorithm<Eigen::Matrix3d> Algo;

  // initalization via AlgorithmType enum
  Algo matrix_bw_lp_f(
    ow::ScalarButterWorthFilter::HighPassSecondOrder(f_sample, f_cutoff));

  // initalization via Algorithm
  // roger
  Algo matrix_bw_lp_f_2(
    ow::ScalarButterWorthFilter::LowPassSecondOrder(f_sample, f_cutoff));

  std::vector<Eigen::Matrix3d> XX_samples(t_end*f_sample);
  std::vector<Eigen::Matrix3d> YY_samples(t_end*f_sample);
  for(int i = 0; i < t_samples.size(); ++i)
    XX_samples[i] = x_samples[i]*Eigen::Matrix3d::Ones();

  for(int i = 0; i < t_samples.size(); ++i)
    YY_samples[i] = matrix_bw_lp_f.update(XX_samples[i]);

  std::cout << std::endl << "-----------------------" << std::endl;
  std::cout << "Butterworth Lowpass Filter Matrix" << std::endl;
  for(int i = 0; i < 6; ++i) {
    printf("i[%d]: t=%4.2f y=\n", int(i), t_samples[i]);
    std::cout << YY_samples[i] << std::endl;
  }*/

  return 0;
}