#ifndef LINEAR_REGRESSION_HPP_
#define LINEAR_REGRESSION_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

class LinearRegression {
 public:
  explicit LinearRegression(const std::string& filename);

  std::vector<std::uint8_t> calc_intercepts();

 private:
  std::string filename_;
  Eigen::MatrixXd X_;
  Eigen::MatrixXd Y_;

  void loadCSV();
};

#endif  // LINEAR_REGRESSION_HPP_
