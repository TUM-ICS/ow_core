#include <ow_core/types.h>
#include <ow_core/math/quaternion.h>
#include <ow_core/algorithms.h>

#include <vector>
#include <iostream>

// time evolution along fith order polynomial
void timelaw_poly_fifth(
  ow::Scalar t, 
  ow::Scalar tf, 
  ow::Scalar& s,
  ow::Scalar& sP,
  ow::Scalar& sPP);

// path evolution along some 3d figure
void path_sinty_figure(
  ow::Scalar s,
  const ow::LinearPosition& xi,
  ow::LinearPosition& xs,
  ow::LinearVelocity& xsP,
  ow::LinearAcceleration& xsPP);

/*!
 * - Test Quaternion Differntation: Q -> omega -> alpha
 * - Test Quaternion Integration: omega -> Q
 * - Use Slerp to interpolate a trajectory: Q, omega, alpha
 * - Then compair the numerical diff omega & alpha with the trajectory
 *   omega & alpha. Compair Q with integrated Q_integ
 * 
 * --> Seams to work
 */
int main(int argc, char* argv[])
{
  ow::CartesianVelocity vel1, vel2;
  vel1 = vel2;

  // setup params
  ow::Scalar dt = 0.01;               // delta t
  ow::Scalar fs = ow::Scalar(1)/dt;   // sample freqency
  ow::Scalar t_task = 20.0;           // total time of the trajectory

  // setup a inital and final Rotation for interpolation
  ow::Rotation3 Ri, Rf;
  Ri << -1, 0, 0, 0,-1, 0, 0, 0, 1;
  Rf <<  0, 1, 0, 0, 0,-1,-1, 0 ,0;
  // convert to angular position
  ow::AngularPosition Qi(Ri);
  ow::AngularPosition Qf(Rf);
  // a start position
  ow::LinearPosition xi(-0.25, -0.25, 0.25);

  // setup numerical differentiation for quaternion coeffs
  typedef ow_core::MatrixAlgorithm<ow::LinearPosition> xDiff;
  typedef ow_core::MatrixAlgorithm<ow::LinearVelocity> xPDiff;
  typedef ow_core::MatrixAlgorithm<ow::AngularPosition> QDiff;
  typedef ow_core::MatrixAlgorithm<ow::AngularVelocity> OmegaDiff;

  // setup a test states
  ow::LinearState linear_state = ow::LinearState::Zero();
  ow::AngularState angular_state = ow::AngularState::Zero();
  ow::CartesianState cartesian_state = ow::CartesianState::Zero();

  // diff coeffs of Q -> QP
  QDiff diff_Q(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs));
  // diff coeffs of QP -> QPP
  QDiff diff_QP(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs));
  // diff omega directly -> alpha
  OmegaDiff diff_Omega(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs));

  xDiff diff_x(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs));
  // diff coeffs of QP -> QPP
  xPDiff diff_xP(ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs));

  // test state updater for AngularState and CartesianState
  ow::StateDifferentiator<ow::AngularState> angular_diff(
    ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs)
  );
  ow::StateDifferentiator<ow::CartesianState> cartesian_diff(
    ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs)
  );
  ow::StateDifferentiator<ow::LinearState> linear_diff(
    ow::ScalarFiniteDifference::FirstOrderAccurarcyThree(fs)
  );

  ow::StateIntegrator<ow::LinearState> linear_integ(1.0/fs);
  ow::StateIntegrator<ow::AngularState> angular_integ(1.0/fs);
  ow::StateIntegrator<ow::CartesianState> cartesian_integ(1.0/fs);

  //----------------------------------------------------------------------------
  // simulate a trajectory with slerp
  int n = t_task/dt;

  // integrated quaternion
  ow::AngularPosition Q_integ(Qi);

  // memory stack
  std::vector<ow::Scalar> t_vec(n);
  std::vector<ow::AngularPosition> Q_vec(n), Q_integ_vec(n);
  std::vector<ow::AngularVelocity> omega_vec(n), omega_hat_vec(n);
  std::vector<ow::AngularAcceleration> alpha_vec(n), alpha_hat_vec(n);
  std::vector<ow::LinearVelocity> xP_vec(n), xP_hat_vec(n);
  std::vector<ow::LinearAcceleration> xPP_vec(n), xPP_hat_vec(n);

  std::vector<ow::LinearState> linear_state_vec(n), linear_state_integ_vec(n);
  std::vector<ow::AngularState> angular_state_vec(n), angular_state_integ_vec(n);
  std::vector<ow::CartesianState> cartesian_state_vec(n);

  ow::LinearState linear_state_integ = ow::LinearState::Zero();
  ow::AngularState angular_state_integ = ow::AngularState::Zero();
  ow::CartesianState cartesian_state_integ = ow::CartesianState::Zero();

  for(int i = 0; i < n; ++i)
  {
    Eigen::Vector3d::UnitX();

    ow::Scalar t = i*dt;

    // timelaw
    ow::Scalar s, sP, sPP;
    timelaw_poly_fifth(t, t_task, s, sP, sPP);

    // linear part
    ow::LinearPosition x;
    ow::LinearVelocity xsP, xP;
    ow::LinearAcceleration xsPP, xPP;
    path_sinty_figure(s, xi, x, xsP, xsPP);

    // linear path and derivatives wrt t
    xP = xsP*sP;
    xPP = xsPP*sP*sP + xsP*sPP;

    // numerical differntiation
    ow::LinearPosition xP_hat, xPP_hat;
    xP_hat = diff_x.update(x);
    xPP_hat = diff_x.update(xP_hat);

    // orientation part: slerp quaternion interpolation, derivatives wrt parameter s
    ow::AngularPosition Q, QsP, QsPP;
    Q = ow::slerp(Qi, Qf, s);
    QsP = ow::slerpP(Qi, Qf, s);
    QsPP = ow::slerpPP(Qi, Qf, s);

    // analytical quaternion derivatives wrt time from slerp
    ow::AngularPosition QP, QPP;
    QP.coeffs() = sP*QsP.coeffs();
    QPP.coeffs() = sP*sP*QsPP.coeffs() + sPP*QsP.coeffs();

    // numerical differentiation of coeffs
    ow::AngularPosition QP_hat, QPP_hat;
    QP_hat = diff_Q.update(Q);
    QPP_hat = diff_QP.update(QP_hat);

    // quaternion propagation to angular velocities/acceleration
    ow::AngularVelocity omega;
    ow::AngularAcceleration alpha;
    ow::quaternion2AngularVelocityInertial(omega, QP, Q);
    ow::quaternion2AngularAccelerationInertial(alpha, QPP, Q);

    // quaternion propagation to angular velocities/acceleration
    ow::AngularVelocity omega_hat;
    ow::AngularAcceleration alpha_hat;
    ow::quaternion2AngularVelocityInertial(omega_hat, QP_hat, Q);
    ow::quaternion2AngularAccelerationInertial(alpha_hat, QPP_hat, Q);

    // integrate the angular velocity back to a Quaternion and compair
    ow::AngularPosition Q_integ_new;
    ow::quaternionIntegration(Q_integ_new, Q_integ, omega, dt);
    Q_integ = Q_integ_new;

    //--------------------------------------------------------------------------
    // test state differentiator
    linear_state.pos() = x;
    linear_diff.update(linear_state);

    angular_state.pos() = Q;
    angular_diff.update(angular_state);

    cartesian_state.X().linear() = x;
    cartesian_state.pos().orientation() = Q;
    cartesian_state = cartesian_diff.update(cartesian_state);

    //--------------------------------------------------------------------------
    // test state integrator

    if(i == 0)
    {
      // set initial position/velocity
      linear_state_integ = linear_state;
      angular_state_integ = angular_state;
      cartesian_state_integ = cartesian_state;
    }
    angular_state_integ.acc().setZero();
    angular_state_integ.vel() = angular_state.vel();
    angular_integ.update(angular_state_integ);

    linear_state_integ.acc().setZero();
    linear_state_integ.vel() = linear_state.vel();
    linear_integ.update(linear_state_integ);

    cartesian_integ.update(cartesian_state_integ);

    // save results in vector
    t_vec[i] = t;                           // time
    Q_vec[i] = Q;                           // Orientation
    Q_integ_vec[i] = Q_integ;               // Integrated Orientation

    omega_vec[i] = omega;                   // angular velo slerp
    omega_hat_vec[i] = omega_hat;           // angular velo numerical diff.
    alpha_vec[i] = alpha;                   // angular acc slerp
    alpha_hat_vec[i] = alpha_hat;           // angular acc numerical diff.

    xP_vec[i] = xP;
    xP_hat_vec[i] = static_cast<const ow_core::Vector3<ow::Scalar>& >(xP_hat);

    xPP_vec[i] = xPP;
    xPP_hat_vec[i] = static_cast<const ow_core::Vector3<ow::Scalar>& >(xPP_hat);

    linear_state_vec[i] = linear_state;
    linear_state_integ_vec[i] = linear_state_integ;
    angular_state_vec[i] = angular_state;   // complete state
    angular_state_integ_vec[i] = angular_state_integ;
    cartesian_state_vec[i] = cartesian_state;
  }

  for(int i = 0; i < n; i+=100)
  {
    /*std::cout << "[" << i << "]: t=" << t_vec[i] << std::endl
      << "l acc=" << linear_state_vec[i].acc().toString() << std::endl
      << "l vel=" << linear_state_vec[i].vel().toString() << std::endl
      << "l pos=" << linear_state_vec[i].pos().toString() << std::endl;

    std::cout
      << "l acc=" << linear_state_integ_vec[i].acc().toString() << std::endl
      << "l vel=" << linear_state_integ_vec[i].vel().toString() << std::endl
      << "l pos=" << linear_state_integ_vec[i].pos().toString() << std::endl;

    std::cout << "[" << i << "]: t=" << t_vec[i] << std::endl
      << "a acc=" << angular_state_vec[i].acc().toString() << std::endl
      << "a vel=" << angular_state_vec[i].vel().toString() << std::endl
      << "a pos=" << angular_state_vec[i].pos().toString() << std::endl;

    std::cout
      << "a acc=" << angular_state_integ_vec[i].acc().toString() << std::endl
      << "a vel=" << angular_state_integ_vec[i].vel().toString() << std::endl
      << "a pos=" << angular_state_integ_vec[i].pos().toString() << std::endl;*/

    // compair integral to actual
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " Q=           " << Q_vec[i].toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " Q_integ=     " << Q_integ_vec[i].toString() << std::endl;

    /* compair angular velo
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " omega=       " << omega_vec[i].toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " omega_hat=   " << omega_hat_vec[i].toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " omega_state1=" << angular_state_vec[i].vel().toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " omega_state2=" << cartesian_state_vec[i].vel().angular().toString() << std::endl;

    // compair angular acc
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " alpha=       " << alpha_vec[i].toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " alpha_hat=   " << alpha_hat_vec[i].toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " alpha_state1=" << angular_state_vec[i].acc().toString() << std::endl;
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " alpha_state2=" << cartesian_state_vec[i].acc().angular().toString() << std::endl;

    // plot complete trajectory
    std::cout << "[" << i << "]: t=" << t_vec[i]
      << " X=           " << std::endl << cartesian_state_vec[i].toString() << std::endl;*/

    std::cout << "---------------" << std::endl;
  }

  //----------------------------------------------------------------------------
};

void timelaw_poly_fifth(
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

  s = Scalar(6)/tf5*t5 - Scalar(15)/tf4*t4 + Scalar(10)/tf3*t3;
  sP = Scalar(30)/tf5*t4 - Scalar(60)/tf4*t3 + Scalar(30)/tf3*t2;
  sPP = Scalar(120)/tf5*t3 - Scalar(180)/tf4*t2 + Scalar(60)/tf3*t;
}

void path_sinty_figure(
  ow::Scalar s,
  const ow::LinearPosition& xi,
  ow::LinearPosition& xs,
  ow::LinearVelocity& xsP,
  ow::LinearAcceleration& xsPP)
{
  xs << std::sin(3*s)*std::cos(s), std::sin(3*s)*std::sin(s), 0;
  xsP << 3*std::cos(3*s)*std::cos(s) - std::sin(3*s)*std::sin(s),
          3*std::sin(3*s)*std::sin(s) - std::sin(3*s)*std::cos(s), 0;
  xsPP << -9*std::sin(3*s)*std::cos(s) - 3*std::cos(3*s)*std::sin(s)
          - ( 3*std::cos(3*s)*std::sin(s) + std::sin(3*s)*std::cos(s) ),
          9*std::cos(3*s)*std::sin(s) + 3*std::sin(3*s)*std::cos(s) 
          - (3*std::sin(3*s)*std::cos(s) - std::sin(3*s)*std::sin(s)), 0;
  xs += xi;
}