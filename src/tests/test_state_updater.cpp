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

#include <ow_core/types.h>
#include <ow_core/algorithms.h>
#include <ow_core/algorithms/state_differentiator.h>

#include <ow_core/test_utilities/gtest.h>  
#include <vector>

/*!
 * \brief The StateDifferentiatorTestSuite class.
 *
 * The TestFixture for StateDifferentiatorTestSuite.
 */
class StateDifferentiatorTestSuite :
  public ::testing::Test
{
public:
  typedef std::vector<ow::CartesianState> CartesianStateVector;

protected:
  // trajectory parameter
  double dt_;                         // delta t
  double fs_;                         // sample freqency
  double t_task_;                     // total time of the trajectory 
  int n_samples_;

  // trajectory start and goal
  ow::Rotation3 Ri_, Rf_;             // inital final Rotation
  ow::AngularPosition Qi_, Qf_;       // inital final Quaternion
  ow::LinearPosition xi_;             // inital Positon

  // cartesian state updater
  ow::StateDifferentiator<ow::CartesianState> updater_;

  // expected results
  int valid_cnt_expected_;            // num of samples befor valid flag true

  StateDifferentiatorTestSuite() : 
    dt_(0.01),
    fs_(1.0/dt_),
    t_task_(20.0),
    n_samples_(t_task_/dt_),
    Ri_(ow::Rotation3::Identity()),
    Rf_(ow::Rotation3::Rx(M_PI/2)*ow::Rotation3::Ry(5*M_PI/3)),
    Qi_(Ri_),
    Qf_(Rf_),
    xi_(-0.25, -0.25, 0.25),
    updater_(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs_)),
    valid_cnt_expected_(3)
  {
  }

  void generate(
    CartesianStateVector& sol_ana,
    CartesianStateVector& sol_num,
    int n)
  {
    n = std::min(std::max(0, n), n_samples_);

    ow::CartesianState state_ana;
    ow::CartesianState state_num;

    for(int i = 0; i < n; ++i)
    {
      ow::Scalar t = i*dt_;

      // timelaw
      ow::Scalar s, sP, sPP;
      computeTimeLaw(t, t_task_, s, sP, sPP);

      // linear path and derivatives wrt s
      ow::LinearPosition x, xsP, xsPP;
      computePathFigure(s, xi_, x, xsP, xsPP);

      // linear path and derivatives wrt t
      state_ana.X().linear() = x;
      state_ana.XP().linear() = xsP*sP;
      state_ana.XPP().linear() = xsPP*sP*sP + xsP*sPP;

      // angular path and derivatives wrt s
      ow::AngularPosition Q, QsP, QsPP;
      Q = ow::slerp(Qi_, Qf_, s);
      QsP = ow::slerpP(Qi_, Qf_, s);
      QsPP = ow::slerpPP(Qi_, Qf_, s);

      // angular path derivatives wrt t
      ow::AngularPosition QP, QPP;
      QP.coeffs() = QsP.coeffs()*sP;
      QPP.coeffs() = QsPP.coeffs()*sP*sP + QsP.coeffs()*sPP;

      // propagate to angular velocities/acceleration
      ow::AngularVelocity omega;
      ow::AngularAcceleration alpha;
      ow::quaternion2AngularVelocityInertial(omega, QP, Q);
      ow::quaternion2AngularAccelerationInertial(alpha, QPP, Q);
    
      state_ana.X().angular() = Q;
      state_ana.XP().angular() = omega;
      state_ana.XPP().angular() = alpha;

      // numerical soluation based on cartesian position
      state_num.X() = state_ana.X();
      state_num = updater_.update(state_num);

      // save results
      sol_ana.push_back(state_ana);
      sol_num.push_back(state_num);
    }
  }

  /*!
  * \brief Compute the timelaw of the trajectory 
  *
  * Maps current time t to parameterization s in [0,1] 
  * and its derivatives sP, sPP.
  * Here: fith order polynomial.
  */
  void computeTimeLaw(
    ow::Scalar t, 
    ow::Scalar tf, 
    ow::Scalar& s,
    ow::Scalar& sP,
    ow::Scalar& sPP)
  {
    typedef ow::Scalar Scalar;

    Scalar tf3 = tf*tf*tf;
    Scalar tf4 = tf3*tf;
    Scalar tf5 = tf4*tf;

    Scalar t2 = t*t;
    Scalar t3 = t2*t;
    Scalar t4 = t3*t;
    Scalar t5 = t4*t;

    s = 6/tf5*t5 - 15/tf4*t4 + 10/tf3*t3;
    sP = 30/tf5*t4 - 60/tf4*t3 + 30/tf3*t2;
    sPP = 120/tf5*t3 - 180/tf4*t2 + 60/tf3*t;
  }

  /*!
  * \brief Compute the path of the trajectory 
  *
  * Computes the current LinearPosition based on the parameterization s
  * and its geometric derivatives.
  */
  void computePathFigure(
    ow::Scalar s,
    const ow::LinearPosition& xi,
    ow::LinearPosition& xs,
    ow::LinearPosition& xsP,
    ow::LinearPosition& xsPP)
  {
    xs << std::sin(3*s)*std::cos(s), std::sin(3*s)*std::sin(s), 0;
    xsP << 3*std::cos(3*s)*std::cos(s) - std::sin(3*s)*std::sin(s),
          std::cos(s)*std::sin(3*s) + 3*std::sin(s)*std::cos(3*s), 0;
    xsPP << -10*std::cos(s)*std::sin(3*s) - 6*std::sin(s)*std::cos(3*s),
            6*std::cos(s)*std::cos(3*s) - 10*std::sin(s)*std::sin(3*s), 0;
    xs += xi;
  }

  /*!
  * \brief Compair two cartesian states
  */
  ::testing::AssertionResult cartesianStateNear(
    const ow::CartesianState& s1, const std::string& s1_name,
    const ow::CartesianState& s2, const std::string& s2_name,
    double prec = ow_test::PRECISION)
  {
    ::testing::AssertionResult res1 = 
      ow_test::eigenMatrixNear(
        s1.pos().linear(), s1_name+"::x", 
          s2.pos().linear(), s2_name+"::x", prec);

    ::testing::AssertionResult res2 = 
      ow_test::eigenRotationNear(
        s1.pos().angular(), s1_name+"::Q", 
          s2.pos().angular(), s2_name+"::Q", prec);

    ::testing::AssertionResult res3 = 
      ow_test::eigenMatrixNear(
        s1.vel(), s1_name+"::XP", 
          s2.vel(), s2_name+"::XP", prec);

    ::testing::AssertionResult res4 = 
      ow_test::eigenMatrixNear(
        s1.acc(), s1_name+"::XPP", 
          s2.acc(), s2_name+"::XPP", prec);

    ::testing::AssertionResult res(res1 & res2 & res3 & res4);
    res << res1.message() << res2.message() 
        << res3.message() << res4.message();
    return res;
  }
};

TEST_F(StateDifferentiatorTestSuite, validOutputFlag)
{
  // test if valid flag is turned on after specific num of iterations
  // feed some trash in updater and check when the valid flag becomes true

  ow::CartesianState state = ow::CartesianState::Zero();
  int valid_cnt = -1;

  for(int i = 0; i < valid_cnt_expected_+1; ++i) {
    updater_.update(state);
    if(updater_.valid()) {
      valid_cnt = i;
      break;
    }
  }

  EXPECT_EQ(valid_cnt, valid_cnt_expected_);
}

TEST_F(StateDifferentiatorTestSuite, resetWithoutInput)
{
  // test if reset to zero works
  // generate solution for a couple of steps then reset and observe ouput

  ow::CartesianState state, state_expected;
  CartesianStateVector sol_ana, sol_num;
  generate(sol_ana, sol_num, 300);

  state_expected.setZero();

  state = updater_.reset();

  EXPECT_TRUE(
    cartesianStateNear(
      state, "s", state_expected, "s_expected"));
}

TEST_F(StateDifferentiatorTestSuite, resetWithInput)
{
  // test if reset to some given state works
  // generate solution for a couple of steps then reset and observe ouput

  ow::CartesianState state, state_expected;
  CartesianStateVector sol_ana, sol_num;
  generate(sol_ana, sol_num, 300);

  state_expected.setZero();
  state_expected.pos() = sol_ana.back().pos();

  state = updater_.reset(sol_ana.back());

  EXPECT_TRUE(
    cartesianStateNear(
      state, "s", state_expected, "s_expected"));
}

TEST_F(StateDifferentiatorTestSuite, addValues)
{
  // test add value function works
  // generate solution for a couple of steps, then add them to updater.
  // Run a single update step and check if internal state is correct.

  ow::CartesianState state, state_expected;
  CartesianStateVector sol_ana, sol_num;
  generate(sol_ana, sol_num, 1000);

  updater_.reset();

  int n = 600;
  for(int i = 0; i < n; ++i) 
  {
    updater_.add(sol_ana[i]);
  }

  state_expected = sol_ana[n];
  state = updater_.update(sol_ana[n]);

  EXPECT_TRUE(
    cartesianStateNear(
      state, "s", state_expected, "s_expected", 10e-5));
}

TEST_F(StateDifferentiatorTestSuite, update)
{
  // test if update function works
  // generate solution for a couple of steps then compair some key points
  // of the analytically generated and numerical trajectory

  CartesianStateVector sol_ana, sol_num;
  generate(sol_ana, sol_num, 1000);

  std::vector<int> indices = {100, 200, 500, 800};

  int n = 600;
  for(int i = 0; i < indices.size(); ++i) 
  {
    int idx = indices[i];
    std::string s = "s[" + std::to_string(idx) + "]";
    std::string s_expected = "s_expected[" + std::to_string(idx) + "]";

    EXPECT_TRUE(
    cartesianStateNear(
      sol_num[idx], s, sol_ana[idx], s_expected));
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}