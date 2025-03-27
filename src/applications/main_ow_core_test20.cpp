#include <ow_core/types.h>

int main(int argc, char* argv[]) 
{
  std::vector<double> test;

  ow_core::Force<double> f;
  f.setZero();
  
  ow_core::Moment<double> mu;
  mu.setZero();

  return 0;
}