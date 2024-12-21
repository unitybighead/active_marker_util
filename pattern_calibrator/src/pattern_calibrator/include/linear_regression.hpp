#ifndef LINEAR_REGRESSION_HPP_
#define LINEAR_REGRESSION_HPP_

#include <Eigen/Dense>
#include <vector>

class LinearRegression {
 public:
  LinearRegression(std::string filename);
  std::vector<std::uint8_t> calc_intercepts();

 private:
  std::string filename_ = "";
  Eigen::MatrixXd X_;
  Eigen::MatrixXd Y_;

  void load_CSV();
};

#endif