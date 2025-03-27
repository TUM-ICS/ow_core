#include <iostream>
#include <ow_core/math/pseudo_inverse.h>
#include <ctime>

int main()
{
  int n_iter = 100000;

  double elapsed_a, elapsed_b, elapsed_c;
  clock_t begin, end;

  std::cout << "Timing dynamically sized matrix" << std::endl;
  for(int i = 1; i < 30; i+=2)
  {
    Eigen::MatrixXd J = Eigen::MatrixXd::Random(6, i);
    Eigen::MatrixXd Jinv1, Jinv2, Jinv3;

    // a
    begin = clock();
    for(int k = 0; k < n_iter; ++k) 
    {
      Jinv1 = ow::pinvSVD(J);
    }
    end = clock();
    elapsed_a = double(end - begin) / CLOCKS_PER_SEC * 1000.0*1000.0;
    
    // b
    begin = clock();
    for(int k = 0; k < n_iter; ++k) 
    {
      Jinv2 = ow::pinvGen(J);
    }
    end = clock();
    elapsed_b = double(end - begin) / CLOCKS_PER_SEC * 1000.0*1000.0;

    // c
    begin = clock();
    for(int k = 0; k < n_iter; ++k) 
    {
      Jinv3 = ow::pinv(J);
    }
    end = clock();
    elapsed_c = double(end - begin) / CLOCKS_PER_SEC * 1000.0*1000.0;

    // print:
    std::cout << "Size (" << J.rows() << " " << J.cols() << "): " << std::endl
              << "  svd = " << elapsed_a/double(n_iter) << std::endl
              << "  gen = " << elapsed_b/double(n_iter) << std::endl
              << " pinv = " << elapsed_c/double(n_iter) << std::endl;
  }

  //----------------------------------------------------------------------------

  std::cout << "Timing static sized matrix" << std::endl;

  Eigen::Matrix<double, 6, 9> J_static = Eigen::Matrix<double, 6, 9>::Random();
  Eigen::Matrix<double, 9, 6> Jinv1_static, Jinv2_static;

  // a
  begin = clock();
  for(int k = 0; k < n_iter; ++k) 
  {
    Jinv1_static = ow::pinvSVD(J_static);
  }
  end = clock();
  elapsed_a = double(end - begin) / CLOCKS_PER_SEC * 1000.0;
  
  // b
  begin = clock();
  for(int k = 0; k < n_iter; ++k) 
  {
    Jinv2_static = ow::pinvGen(J_static);
  }
  end = clock();
  elapsed_b = double(end - begin) / CLOCKS_PER_SEC * 1000.0;

  std::cout << "Size (" << J_static.rows() << " " << J_static.cols() << "): " << std::endl
            << "  a = " << elapsed_a << std::endl
            << "  b = " << elapsed_b << std::endl;

  return 0;
}