#include <ow_core/trajectory/polynomial.h>
#include <fstream>

#include <ow_core/trajectory/trajectories.h>
#include <ow_core/trajectory/state_trajectory.h>

#include <ow_core/math/quaternion.h>

// test fixed order (6 and 3) for scalar and matrix case
int main()
{
  //----------------------------------------------------------------------------
  // create a 6 order polynomial for the scalar case

  ow::Scalar period = 10;
  ow::Scalar x_start = 1.0;
  ow::Scalar xP_start = 0.0;
  ow::Scalar xPP_start = 0.0;
  ow::Scalar x_end = 1.0;
  ow::Scalar xP_end = 0.0;
  ow::Scalar xPP_end = 0.0;
  ow::Scalar x_middle = 6.0;

  ow_core::Polynomial<ow::Scalar> poly6
    = ow_core::Polynomial<ow::Scalar>::Polynomial6Order(
      period, x_start, xP_start, xPP_start, x_end, xP_end, xPP_end, x_middle);

  ow_core::Polynomial<ow::Scalar> poly3
    = ow_core::Polynomial<ow::Scalar>::Polynomial3Order(
      period, x_start, xP_start, x_end, xP_end);

  //----------------------------------------------------------------------------
  // create time vector 
  int len = (poly6.endTime() - poly6.startTime())/0.01;
  std::vector<ow::Scalar> time_vec;
  for(int i = 0; i < len; ++i)
    time_vec.push_back(i*0.01);

  // evaluate all derivatives and value for all times in time vec 
  std::vector<std::vector<ow::Scalar> > resutls6_vec_all;
  resutls6_vec_all = poly6.evaluateAll(time_vec, 2);

  std::vector<std::vector<ow::Scalar> > resutls3_vec_all;
  resutls6_vec_all = poly3.evaluateAll(time_vec, 2);

  //----------------------------------------------------------------------------
  // write for a matplotlib plot
  std::ofstream myfile;
  myfile.open("/home/simon/ros/workspaces/tmp/tests/polynomial6.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "; 
    for(int j = 0; j < resutls6_vec_all[i].size(); ++j) 
    {
      myfile << resutls6_vec_all[i][j] << " ";
    }
    myfile << std::endl;
  }
  myfile.close();

  //----------------------------------------------------------------------------
  // create a 6 order polynomial for the matrix case

  ow::LinearPosition x_start_(5, 5, 5);
  ow::LinearPosition xP_start_(0, 0, 0);
  ow::LinearPosition xPP_start_(0, 0, 0);
  ow::LinearPosition x_end_(6, 6, 6);
  ow::LinearPosition xP_end_(0, 0, 0);
  ow::LinearPosition xPP_end_(0, 0, 0);
  ow::LinearPosition x_middle_(5.5, 5.5, 5.5);

  ow_core::PolynomialTrajectory<ow::LinearPosition, ow::Scalar>
    multi_dim_poly6 = ow_core::Polynomial6Order(
      period, x_start_, xP_start_, xPP_start_,
        x_end_, xP_end_, xPP_end_, x_middle_);

  ow_core::PolynomialTrajectory<ow::LinearPosition, ow::Scalar>
    multi_dim_poly3 = ow_core::Polynomial3Order(
      period, x_start_, xP_start_, x_end_, xP_end_);

  ow_core::PolynomialTrajectory<ow::LinearPosition, ow::Scalar>
    multi_dim_poly5 = ow_core::Polynomial5Order(
      period, x_start_, xP_start_, xPP_start_, x_end_, xP_end_, xPP_end_);

  // compute secorder derivative
  std::vector<std::vector<ow::LinearPosition> > pos6_vec_all, pos3_vec_all;
  pos6_vec_all = multi_dim_poly6.evaluateAll(time_vec, 2);
  pos3_vec_all = multi_dim_poly3.evaluateAll(time_vec, 2);

  // comute the arc length of the polynomial
  ow::Scalar len1 = multi_dim_poly6.arcLength(0, period);
  ow::Scalar len2 = multi_dim_poly3.arcLength(0, period);
  ow::Scalar len3 = multi_dim_poly5.arcLength(0, period);
  std::cout << "arc_length_1=" << len1 << std::endl;
  std::cout << "arc_lenght_2=" << len2 << std::endl;
  std::cout << "arc_lenght_3=" << len3 << std::endl;

  ow::Scalar t1 = multi_dim_poly5.inverseArcLength(0, len3/2.);
  std::cout << "len=" << len3/2. << " t1=" << t1 << std::endl;

  ow::Scalar t2 = multi_dim_poly5.inverseArcLength(0, len3);
  std::cout << "len=" << len3 << " t2=" << t2 << std::endl;

  ow::Scalar t3 = multi_dim_poly5.inverseArcLength(0, 2*len3);
  std::cout << "len=" << len3 << " t2=" << t3 << std::endl;

  //----------------------------------------------------------------------------
  // write for a matplotlib plot

  myfile.open("/home/simon/ros/workspaces/ow/data/polynomial6_multi.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "; 
    for(int j = 0; j < pos6_vec_all[i].size(); ++j) 
    {
      myfile << pos6_vec_all[i][j].toString() << " ";
    }
    myfile << std::endl;
  }
  myfile.close();

  //----------------------------------------------------------------------------
  // create a 6 order polynomial for the cartesian pose

  ow::CartesianPosition X_start_(1, 1, 1, 1, 0, 0, 0);
  ow::CartesianPosition XP_start_(0, 0, 0, 0, 0, 0, 0);
  ow::CartesianPosition XPP_start_(0, 0, 0, 0, 0, 0, 0);
  ow::CartesianPosition X_end_(1, 1, 1, 0, 1, 0, 0);
  ow::CartesianPosition XP_end_(0, 0, 0, 0, 0, 0, 0);
  ow::CartesianPosition XPP_end_(0, 0, 0, 0, 0, 0, 0);
  ow::CartesianPosition X_middle_(6, 6, 6, 0.37845, 0.3052338, 0.6179029, 0.6179029);

  // feed into interpolator
  ow_core::StateTrajectory<ow::CartesianState> 
    state_traj(ow_core::Polynomial6Order(
        period, X_start_, XP_start_, XPP_start_,
        X_end_, XP_end_, XPP_end_, X_middle_));

  // add an extra second
  len = 1.0/0.01;
  for(int i = 0; i < len; ++i)
    time_vec.push_back(time_vec.back() + 0.01);

  // interpolate the complete state
  std::vector<ow::CartesianState> c_results = state_traj.evaluate(time_vec);

  myfile.open("/home/simon/ros/workspaces/ow/data/cartesian_state.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "
      << c_results[i].pos().toString() << " "
      << c_results[i].vel().toString() << " "
      << c_results[i].acc().toString() << std::endl;
  }
  myfile.close();

  return 0;
}
