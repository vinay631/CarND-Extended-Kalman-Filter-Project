#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, 2, 1>  Vector2f;
using namespace std;

class StringException : public std::exception
{
public:
   std::string s;
   StringException(std::string ss) : s(ss) {}
   ~StringException() throw () {} // Updated
   const char* what() const throw() { return s.c_str(); }
};


class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /* A helper method to transform Polar coordinates to Cartesian */
  Vector2f PolarToCartesian(double rho, double phi);

};

#endif /* TOOLS_H_ */
