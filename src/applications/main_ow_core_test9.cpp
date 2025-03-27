#include <iostream>
#include <Eigen/Dense>
#include <vector>

/**
 * Testing templates of templates 
 * for filter class
 */

/**
 * Example Scalar Algorithm 1
 * 
 * Three template arguments
 * Two constructor elements
 */
template<
  typename _Scalar, 
  int _Type = 0,
  int _Order = 2>
class FilterA
{
public:

  enum {
    Type = _Type,
    Order = _Order
  };

  typedef _Scalar Scalar;

protected:
  Scalar f_s_, fcut_1_, fcut_2_;

public:
  FilterA(Scalar f_s,
          Scalar fcut_1, 
          Scalar fcut_2 = 0) :
    fcut_1_(fcut_1),
    fcut_2_(fcut_2),
    f_s_(f_s)
  {
  }

  void update(Scalar x) 
  {
    std::cout << "update FilterA" << std::endl;
  }
};

/**
 * Example Scalar Algorithm 2
 * 
 * One template arguments
 * One constructor elements
 */
template<
  typename _Scalar>
class FilterB
{
public:
  typedef _Scalar Scalar;

protected:
  Scalar alpha_;

public:
  FilterB(Scalar alpha) :
    alpha_(alpha)
  {
  }

  void update(Scalar x) 
  {
    std::cout << "update FilterB" << std::endl;
  }
};

/**
 * Matrix Algorithm test class.
 * 
 * Template parameters:
 *  - _Derived: Data Type
 *  - _Algorithm: Algorithm
 *  - Ts remaining template arguments of Algorithm
 */
template <typename _Derived, 
  template<typename, int...> typename _Algorithm, 
  int...Ts> 
class MatrixAlgorithm
{
public:
  enum
  {
    Rows = _Derived::RowsAtCompileTime,
    Cols = _Derived::ColsAtCompileTime,
    Size = Rows*Cols,
  };

  typedef _Derived Derived;
  typedef typename Derived::Scalar Scalar;
  typedef _Algorithm<Scalar,Ts...> Algorithm; 

protected:
  std::vector<Algorithm> algos_;    //!< vector of algos_.
  Derived y;                        //!< output

public:
  /**
   * Take Algorithm as input
   */
  MatrixAlgorithm(const Algorithm& algorithm)
  {
    setup(algorithm);
  }

  void update(Scalar x) 
  {
    for(size_t i = 0; i < Size; ++i)
    {
      algos_[i].update(x);
    }
  }

protected:
  /**
   * copy algo in vector
   */
  virtual void setup(const Algorithm& algorithm)
  {
    for(size_t i = 0; i < Size; ++i)
    {
      algos_.push_back(algorithm);
    }
  }
};

int main()
{
  typedef Eigen::Matrix<double,6,2> Matrix;

  // first filter
  MatrixAlgorithm<Matrix, FilterA, 0, 2> matrix_filter_1({ 1.0, 2.0, 3.0});
  matrix_filter_1.update(0.0);

  // second filter
  MatrixAlgorithm<Matrix, FilterB> matrix_filter_2(1.0);
  matrix_filter_2.update(0.0);

  return 0;
}