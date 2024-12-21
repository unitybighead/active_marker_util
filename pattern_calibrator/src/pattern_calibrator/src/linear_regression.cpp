#include "linear_regression.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

LinearRegression::LinearRegression(std::string filename)
    : filename_(filename) {}

void LinearRegression::load_CSV() {
  std::ifstream file(filename_);
  if (!file.is_open()) {
    std::cerr << "Error: Cannot open file " << filename_ << std::endl;
    exit(1);
  }

  std::vector<std::vector<double>> data;
  std::string line;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;

    while (std::getline(ss, value, ',')) {
      row.push_back(std::stod(value));
    }

    data.push_back(row);
  }

  file.close();

  size_t rows = data.size();
  size_t cols = data[0].size();

  X_ = Eigen::MatrixXd(rows, 1);
  Y_ = Eigen::MatrixXd(rows, cols - 1);

  for (size_t i = 0; i < rows; ++i) {
    X_(i, 0) = data[i][0];
    for (size_t j = 1; j < cols; ++j) {
      Y_(i, j - 1) = data[i][j];
    }
  }
}

std::vector<std::uint8_t> LinearRegression::calc_intercepts() {
  load_CSV();
  size_t numTargets = Y_.cols();
  std::vector<std::uint8_t> intercepts;

  for (size_t i = 0; i < numTargets; ++i) {
    Eigen::VectorXd y = Y_.col(i);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(X_.rows());
    Eigen::MatrixXd X_design(X_.rows(), 2);
    X_design << ones, X_;

    Eigen::VectorXd beta =
        (X_design.transpose() * X_design).inverse() * X_design.transpose() * y;
    intercepts.push_back(beta(0));

    std::cerr << "Intercepts: " << std::endl;
    for (const auto &intercept : intercepts) {
      std::cerr << std::to_string(intercept) << std::endl;
    }
  }

  return intercepts;
}