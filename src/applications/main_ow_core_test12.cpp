#include <ow_core/types.h>

int main()
{
  // this is fixed size
  ow::JointState joint_state;
  ow::JointPosition joint_pos;

  // this is dynamic
  ow::JointStateX joint_state_x;
  ow::JointPositionX joint_pos_x;

  // resize the dynamic types
  int n = 10;
  joint_state_x.resize(n);
  joint_pos_x.resize(n);

  for(int i = 0; i < n; ++i) 
  {
    joint_state_x.pos()[i] = i;
    joint_pos_x[i] = i;
  }
  std::cout << "state_dynamic:\n"
    << joint_state_x.toString();

  std::cout << "pos_dynamic:\n"
    << joint_pos_x.toString() << "\n";

  //----------------------------------------------------------------------------

  // resize once more
  joint_state_x.setZero(OW_ROBOT_DOF);
  joint_pos_x.setZero(OW_ROBOT_DOF);

  // assign to fixed size
  joint_state.pos().ref(0, OW_ROBOT_DOF) = joint_state_x.pos();
  joint_state.vel().ref(0, OW_ROBOT_DOF) = joint_state_x.vel();
  joint_state.acc().ref(0, OW_ROBOT_DOF) = joint_state_x.acc();

  joint_pos.ref(0, OW_ROBOT_DOF) = joint_pos_x;

  return 0;
}