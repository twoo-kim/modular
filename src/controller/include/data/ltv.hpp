#pragma once
#include <Eigen/Dense>

struct LTVData {
  Eigen::MatrixXd A, B;   // Linear matrix of dx_{k+1} = A dx_{k} + B du_{k}
  double t;               // Time
  
  LTVData(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, const double time) {
    A = a;
    B = b;
    t = time;
  }
};