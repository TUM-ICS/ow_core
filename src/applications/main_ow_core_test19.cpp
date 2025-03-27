#include <ow_core/types.h>
#include <ow_core/math.h>
#include <ow_core/algorithms.h>

int main(int argc, char* argv[]) 
{
  ow::Scalar T_episode = 6;
  ow::Scalar dt = 0.01;
  size_t N = T_episode/dt;

  ow_core::StateIntegrator<ow::CartesianState> integrator(dt);

  ow::Scalar yaw_rate = 100;

  ow::CartesianState X = ow::CartesianState::Zero();
  X.pos().angular() = ow::Rotation3::Ry(-0.1);

  for(size_t i = 0; i < N; ++i)
  {
    X.vel().angular().z() = yaw_rate*dt; 
    integrator.update(X);

    //ow::checkFlipQuaternionSign(X.pos().angular());

    ow::CartesianVector err = ow::cartesianError(ow::CartesianPosition::Identity(), X.pos());



    std::cout << "i" << i << " -------" << std::endl;
    std::cout << "Q=  " << X.pos().angular().toString() << std::endl;
    std::cout << "Eul=" << X.pos().angular().eulerYPR().toString() << std::endl;
    std::cout << "err=" << err.angular().transpose() << std::endl;
    //std::cout << "Rot=\n" << X.pos().angular().toRotationMatrix() << std::endl;
  }



}
