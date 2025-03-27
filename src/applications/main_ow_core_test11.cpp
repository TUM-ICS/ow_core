#include <ow_core/types.h>
#include <ow_core/type_references/joint_ref.h>
#include <ow_core/type_references/joint_state_ref.h>

/**
 * A test class working on a JoinStateReference
 */
class TestJointStateRef
{
protected:
  ow_core::JointStateRef<ow::JointState> q_state;

public:
  TestJointStateRef(ow_core::JointStateRef<ow::JointState> q_state) : 
    q_state(q_state)
  {
  }

  void computeJointValues()
  {
    q_state.pos().setConstant(10);
    std::cout << "JointState non const INSIDE THE CLASS: " << q_state.pos().toString() << std::endl;
    std::cout << q_state.toString() << std::endl;
  }

  void computeJointValues() const
  {
    // q_state.pos().setConstant(10); // const is still working!
    std::cout << "JointState const INSIDE THE CLASS:     " << q_state.pos().toString() << std::endl;
    std::cout << q_state.toString() << std::endl;
  }
};

void joint_pos_fnc(ow_core::JointRef<ow::JointPosition::Base> joint_ref)
{
  std::cout << "INSIDE fnc= " << joint_ref.toString() << std::endl;
}

void joint_pos_fnc_2(const ow::JointPositionX& joint_pos)
{
  std::cout << "INSIDE fnc 2= " << joint_pos.toString() << std::endl;
}

void test_fnc(const Eigen::VectorXd& test)
{
  std::cout << "INSIDE test fnc= " << test << std::endl;
}

int main()
{
  ow::JointState joint_state = ow::JointState::Zero();
  TestJointStateRef test_class(joint_state.ref(2,3));

  // test normal fnc
  test_class.computeJointValues();

  // test fnc on const obj
  const TestJointStateRef& test_class_ref = test_class;
  test_class_ref.computeJointValues();

  // test assignment ref -> vec
  ow_core::JointPosition<ow::Scalar> q_sub;
  q_sub = joint_state.ref(2,3).pos();
  std::cout << "q_sub=" << q_sub.toString() << std::endl;

  // test assignment vec -> ref
  joint_state.ref(2,3).pos() = ow_core::JointPosition<ow::Scalar, 3>::Constant(30);

  ow_core::JointPosition<ow::Scalar, 3> joint_pos = ow_core::JointPosition<ow::Scalar, 3>::Ones();
  ow::JointPositionX joint_pos_dyn = ow::JointPositionX::Ones(3);

  joint_state.ref(2,3).pos() = joint_pos;
  joint_state.ref(2,3).pos() = joint_pos_dyn;

  joint_pos_fnc_2(joint_state.ref(2,3).pos());

  Eigen::Matrix<ow::Scalar, 10, 1> mem;
  mem.setConstant(5);
  Eigen::Block<Eigen::Matrix<ow::Scalar, 10, 1> > mem_block(mem, 3, 0, 3, 1);
  test_fnc(mem_block);

  // problem no type safety broken
  //joint_state.ref(2,3).vel() = ow_core::JointVelocity<ow::Scalar, 3>::Constant(-1);

  std::cout << "joint_state.pos=" << joint_state.pos().toString() << std::endl;

  joint_pos_fnc(joint_state.ref(2,3).pos());



  return 0;
}