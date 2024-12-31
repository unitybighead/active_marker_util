#include "linear_regression.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

LinearRegression::LinearRegression(const std::string& filename)
    : filename_(filename) {}

void LinearRegression::loadCSV() {
  std::ifstream file(filename_);
  if (!file.is_open()) {
    throw std::runtime_error("Error: Cannot open file " + filename_);
  }

  std::vector<std::vector<double>> data;
  std::string line;

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;
    std::vector<double> row;

    while (std::getline(ss, value, ',')) {
      try {
        row.push_back(std::stod(value));
      } catch (const std::invalid_argument&) {
        throw std::runtime_error("Error: Invalid data format in CSV.");
      }
    }
    data.push_back(row);
  }

  file.close();

  if (data.empty()) {
    throw std::runtime_error("Error: Empty CSV file.");
  }

  size_t rows = data.size();
  size_t cols = data[0].size();
  if (cols < 2) {
    throw std::runtime_error("Error: Insufficient columns in CSV file.");
  }

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
  loadCSV();
  size_t numTargets = Y_.cols();
  std::vector<std::uint8_t> intercepts;

  for (size_t i = 0; i < numTargets; ++i) {
    Eigen::VectorXd y = Y_.col(i);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(X_.rows());
    Eigen::MatrixXd X_design(X_.rows(), 2);
    X_design << ones, X_;

    Eigen::VectorXd beta = (X_design.transpose() * X_design)
                               .ldlt()
                               .solve(X_design.transpose() * y);

    intercepts.push_back(static_cast<std::uint8_t>(beta(0)));
  }

  std::cerr << "Calculated intercepts:" << std::endl;
  for (const auto& intercept : intercepts) {
    std::cerr << static_cast<int>(intercept) << std::endl;
  }

  return intercepts;
}
