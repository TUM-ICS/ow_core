#include <gtest/gtest.h>

class MyClass
{
public:
  MyClass()
  {}

  int someFnc()
  {
    return 42;
  }

};


class MyClassTestSuite : public ::testing::Test
{
protected:
  MyClass* obj;
  int expected;

protected:

  MyClassTestSuite() :
    expected(42)
  {
    obj = new MyClass;
  }

  ~MyClassTestSuite()
  {
    delete obj;
  }

};

// simple data type comparison
TEST_F(MyClassTestSuite, someFnc)
{
  ASSERT_EQ(obj->someFnc(), expected);
}

// main:
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
