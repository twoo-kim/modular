#pragma once
#include <Eigen/Dense>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <time.h>
#include <complex>

#include "simulation/viewer.hpp"
#include "data/config.hpp"
#include "utils/utils.hpp"
#include "utils/xml_utils.hpp"
using namespace std::complex_literals;

class DynamicsSimulator {
public:
  DynamicsSimulator(SimConfig &config);
  ~DynamicsSimulator();

  // Simulation step
  void step();

  // Get data
  Eigen::VectorXd getState();
  Eigen::Vector3d getForce();
  std::vector<Eigen::VectorXcd> computeResponse(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
  
  // Update actuator
  void setAmplitude(const double &amp) { t_ = 0.0; amp_ = amp; }
  void setFrequency(const double &freq) { t_ = 0.0; freq_ = freq; };

  int nq, nv, nx, ndx, nu;  // Dimension

private:
  // Compute actuator control input
  void computeControl(const double &t);

  // Check and copy Eigen into mujoco data
  void setEvalState(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
  void copyQpos(const Eigen::VectorXd &q);
  void copyQvel(const Eigen::VectorXd &dq);

  // Dynamics
  Eigen::MatrixXd computeMass(const Eigen::VectorXd &q);
  Eigen::VectorXd computeCoriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
  Eigen::VectorXd computeBias(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);

  // Integrate with small perturbation
  Eigen::VectorXd integrate(const Eigen::VectorXd &q, int k, double eps);

  double amp_, freq_;       // Amplitude and frequency for actuator
  double t_;                // Simulation time

  // Configuration
  SimConfig config_;

  // Mujoco
  mjModel* model_{nullptr};
  mjData* data_{nullptr}, *eval_{nullptr};

  // Viewer
  std::unique_ptr<MujocoViewer> viewer_;
};