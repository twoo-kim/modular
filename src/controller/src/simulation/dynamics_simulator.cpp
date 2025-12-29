#include "simulation/dynamics_simulator.hpp"
#include <iostream>

DynamicsSimulator::DynamicsSimulator(SimConfig &config)
: config_(config) {
  // Make multi-agent simulation xml file
  std::filesystem::path xml_path = generateColonyXML(config_);
  
  // MuJoCo
  char err[1024];
  model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
  if (!model_) {
    throw std::runtime_error("mj_loadXML failed!");
  }
  data_ = mj_makeData(model_);
  eval_ = mj_makeData(model_);
  if (!data_ || !eval_) {
    throw std::runtime_error("mj_makeData failed!");
  }

  // Viewer
  if (config_.visualize) {
    viewer_ = std::make_unique<MujocoViewer>(model_, data_, config_.width, config_.height, "Mujoco");
  }

  nq = model_->nq;  nv = model_->nv;  nu = model_->nu;
  nx = nq + nv;     ndx = 2*nv;
}

DynamicsSimulator::~DynamicsSimulator() {
  mj_deleteData(data_);
  mj_deleteData(eval_);
  mj_deleteModel(model_);
}

Eigen::VectorXd DynamicsSimulator::getState() {
  Eigen::VectorXd x(nx);
  x.head(nq) = toEigen(data_->qpos, nq);
  x.tail(nv) = toEigen(data_->qvel, nv);
  return x;
}

Eigen::Vector3d DynamicsSimulator::getForce() {
  // Get force applied on the colony center
  int sid = mj_name2id(model_, mjOBJ_SENSOR, "colony_force");
  Eigen::Vector3d F = Eigen::Vector3d(data_->sensordata[sid + 0], data_->sensordata[sid + 1], data_->sensordata[sid + 2]);
  return F;
}

void DynamicsSimulator::step() {
  if (!model_ || !data_) return;
  // Get actuator control input
  computeControl(t_);

  // Step forward
  mj_step(model_, data_);
  t_ += config_.dt;

  // Render
  if (config_.visualize) {
    viewer_->render();
    viewer_->pollEvents();
  }
}

void DynamicsSimulator::computeControl(const double &t) {
  const Eigen::VectorXd q = toEigen(data_->qpos, nq);
  const Eigen::VectorXd dq = toEigen(data_->qvel, nv);
  const double omega = 2*M_PI*freq_;

  for (int i = 0; i < config_.N; ++i) {
    const double phi = q(7+2*i);
    const double dphi = dq(6+2*i);
    double desired_pos = amp_ * std::sin(omega*t);
    double ma = config_.kp * (desired_pos - phi) - config_.kd * dphi;
    data_->ctrl[i] = ma;
  }
}

Eigen::MatrixXd DynamicsSimulator::computeMass(const Eigen::VectorXd &q) {
  // Compute mass
  setEvalState(q, Eigen::VectorXd::Zero(nv));
  mj_forward(model_, eval_);

  // Make full matrix
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> M(nv, nv);
  mj_fullM(model_, M.data(), eval_->qM);
  return Eigen::MatrixXd(M);
}

Eigen::VectorXd DynamicsSimulator::computeCoriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Compute forward dynamics
  setEvalState(q, dq);
  mj_forward(model_, eval_);

  // Get bias
  Eigen::VectorXd c(nv);
  for (int i = 0; i < nv; ++i) {
    c[i] = static_cast<double>(eval_->qfrc_bias[i]);
  }
  return c;
}

Eigen::VectorXd DynamicsSimulator::computeBias(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Compute forward dynamics
  setEvalState(q, dq);
  mj_forward(model_, eval_);

  // Get bias
  Eigen::VectorXd b(nv);
  for (int i = 0; i < nv; ++i) {
    const mjtNum coriolis = eval_->qfrc_bias[i];    // coriolis
    const mjtNum passive = eval_->qfrc_passive[i];  // spring, damper, gravity, fluid
    b[i] = static_cast<double>(coriolis + passive);
  }
  return b;
}

Eigen::VectorXd DynamicsSimulator::integrate(const Eigen::VectorXd &q, int k, double eps) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  v[k] = eps;

  // Copy configuration
  copyQpos(q); copyQvel(v);

  // integrate along tangent direction k
  mj_integratePos(model_, eval_->qpos, eval_->qvel, 1.0);
  Eigen::VectorXd out = toEigen(eval_->qpos, nq);
  return out;
}

std::vector<Eigen::VectorXcd> DynamicsSimulator::computeResponse(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
// Compute temporary mass matrix
  Eigen::MatrixXd M0 = computeMass(q);

  // Compute the jacobian of the coriolis hydro; V - tau
  Eigen::MatrixXd C0 = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd K0 = Eigen::MatrixXd::Zero(nv, nv);
  
  const double epsilon = 1e-6;
  for (int k = 0; k < nv; k++) {
    Eigen::VectorXd q1 = integrate(q, k, epsilon);
    Eigen::VectorXd q2 = integrate(q, k, -epsilon);

    Eigen::VectorXd dq1 = dq; dq1[k] += epsilon;
    Eigen::VectorXd dq2 = dq; dq2[k] -= epsilon;

    const Eigen::VectorXd b1 = computeBias(q1, dq);
    const Eigen::VectorXd b2 = computeBias(q2, dq);
    const Eigen::VectorXd db1 = computeBias(q, dq1);
    const Eigen::VectorXd db2 = computeBias(q, dq2);
    K0.col(k).noalias() = (b1 - b2) / (2.0 * epsilon);
    C0.col(k).noalias() = (db1 - db2) / (2.0 * epsilon);
  }

  // std::cout << "M0\n" << M0 << std::endl;
  // std::cout << "C0\n" << C0 << std::endl;
  // std::cout << "K0\n" << K0 << std::endl;

  // Currently only consider a single actuator input(by symmetry)
  std::vector<Eigen::VectorXcd> response;
  for (int i = 0; i < config_.N; ++i) {
    double w = 2*M_PI*freq_;

    Eigen::VectorXcd q_hat, u_hat;
    Eigen::MatrixXcd temp, G;
    u_hat = Eigen::VectorXcd::Zero(nv);
    if (config_.isActivate[i]) {
      u_hat(6+2*i) = data_->ctrl[i];
    }
    temp = -w*w*M0 + 1.0i*w*C0 + K0;
    // temp += config_.reg * Eigen::MatrixXcd::Identity(nv, nv);
    
    G = temp.inverse();
    q_hat.noalias() = G*u_hat;
    response.push_back(q_hat);
  }

  return response;
}

void DynamicsSimulator::setEvalState(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  copyQpos(q); copyQvel(dq);

  mju_zero(eval_->ctrl, model_->nu);
  mju_zero(eval_->qfrc_applied, model_->nv);
  mju_zero(eval_->xfrc_applied, 6 * model_->nbody);
}

void DynamicsSimulator::copyQpos(const Eigen::VectorXd &q) {
  // Check size of the vector
  if (q.size() != nq) throw std::runtime_error("q size != model.nq");

  // Copy into mujoco data
  toMj(q, eval_->qpos);
}

void DynamicsSimulator::copyQvel(const Eigen::VectorXd &dq) {
  // Check size of the vector
  if (dq.size() != nv) throw std::runtime_error("dq size != model.nv");

  // Copy into mujoco data
  toMj(dq, eval_->qvel);
}