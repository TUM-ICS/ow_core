#include <ow_core/trajectory/trajectories.h>
#include <ow_core/trajectory/spline_trajectory.h>
#include <ow_core/trajectory/state_trajectory.h>
#include <fstream>

#include <ow_core/types.h>

// test cubic spline for arbitary number of points
int main()
{  
  //----------------------------------------------------------------------------
  std::vector<ow::Scalar> segments = {0, 1, 3, 5, 6, 9, 10};  
  std::vector<ow::LinearPosition> samples = {
    ow::LinearPosition(1, 1, 1),
    ow::LinearPosition(0, 0, 0),  // support point
    ow::LinearPosition(5, 5, 5),
    ow::LinearPosition(6, 6, 6),
    ow::LinearPosition(7, 7, 7),
    ow::LinearPosition(0, 0, 0),  // support point
    ow::LinearPosition(10, 10, 10)
  };

  for(unsigned int i = 0; i < segments.size() - 1; ++i)
    std::cout << "idx=" << i << " start=" << segments[i] << " end="
              << segments[i+1] << std::endl;

  ow_core::PolynomialTrajectory<ow::LinearPosition, ow::Scalar>
    spline = ow_core::CubicSpline(segments, samples);

  for(size_t i = 0; i < spline.numberOfSegments(); ++i)
  {
    std::cout << "spline: idx=" << i << " start=" << spline.startTime(i) << " end="
              << spline.endTime(i) << std::endl;
  }

  std::cout << "start_time=" << spline.startTime() << std::endl;
  std::cout << "end_time=" << spline.endTime() << std::endl;
  std::cout << "arc_length=" << spline.arcLength(0.0, 6.0) << std::endl;

  double step = 0.01;
  int len = (spline.endTime() - spline.startTime())/step;
  std::vector<double> time_vec;
  for(int i = 0; i < len; ++i)
    time_vec.push_back(step*i);

  std::vector<ow::LinearPosition> resutls_vec = spline.evaluate(time_vec);

  std::vector<std::vector<ow::LinearPosition> > resutls_vec_all = 
    spline.evaluateAll(time_vec, 2);

  std::ofstream myfile;
  myfile.open("/home/simon/ros/workspaces/tmp/tests/spline_curve.txt");
  for(int i = 0; i < time_vec.size(); ++i)
  {
    myfile << time_vec[i] << " "; 
    for(int j = 0; j < resutls_vec_all[i].size(); ++j) 
    {
      myfile << resutls_vec_all[i][j].toString() << " ";
    }
    myfile << std::endl;
  }
  myfile.close();

  //----------------------------------------------------------------------------
  // same test but this time with our statebase class
 
  // create cartesian pose samples
  std::vector<ow::AngularPosition> a_samples = {
    ow::AngularPosition(1, 0, 0, 0),
    ow::AngularPosition::Zero(),  // support point
    ow::AngularPosition(0.7042004, 0, 0.7100013, 0),
    ow::AngularPosition(0.4388107, 0, 0.6353917, 0.6353917),
    ow::AngularPosition(0.37845, 0.3052338, 0.6179029, 0.6179029),
    ow::AngularPosition::Zero(),  // support point
    ow::AngularPosition(1, 0, 0, 0)
  };
  
  // feed into interpolator
  ow_core::StateTrajectory<ow::AngularState> 
    state_traj(ow_core::CubicSpline(segments, a_samples));

  // interpolate the state
  std::vector<ow::AngularState> c_results = state_traj.evaluate(time_vec);
  return 0;
}
