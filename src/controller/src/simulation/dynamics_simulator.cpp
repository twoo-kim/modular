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
  nx = nq + nv;     na = model_->na;  ndx = 2*nv + na;
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
  int adr = model_->sensor_adr[sid];
  Eigen::Vector3d F = Eigen::Vector3d(data_->sensordata[adr + 0], data_->sensordata[adr + 1], data_->sensordata[adr + 2]);
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
    if (!config_.isActivate[i]) continue;
    int j = (!config_.isFixed) ? 6 + 2*i : 2*i;
    const double phi = (!config_.isFixed) ? q(j+1) : q(j);
    const double dphi = dq(j);
    double desired_pos = amp_ * std::sin(omega*t);
    double ma = config_.kp * (desired_pos - phi) - config_.kd * dphi;
    data_->ctrl[i] = ma;
  }
}

Eigen::MatrixXd DynamicsSimulator::computeMass(const Eigen::VectorXd &q) {
  // Compute mass
  setEvalState(q, Eigen::VectorXd::Zero(nv));
  mj_forward(model_, eval_);

  // Get mass matrix
  Eigen::MatrixXd M(nv, nv);  M.setZero();

  // Make full matrix
  std::vector<mjtNum> Mfull(nv*nv);
  mj_fullM(model_, Mfull.data(), eval_->qM);
  for (int r = 0; r < nv; ++r) {
    for (int c = 0; c < nv; ++c) {
      M(r, c) = static_cast<double>(Mfull[r*nv + c]);
    }
  }
  return M;
}

Eigen::VectorXd DynamicsSimulator::computeCoriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Compute forward dynamics
  setEvalState(q, dq);
  mj_forward(model_, eval_);

  // Get bias
  return toEigen(eval_->qfrc_bias, nv);
}

Eigen::VectorXd DynamicsSimulator::computeBias(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Compute forward dynamics
  setEvalState(q, dq);
  mj_forward(model_, eval_);

  // Get bias
  Eigen::VectorXd bias = toEigen(eval_->qfrc_bias, nv);
  Eigen::VectorXd passive = toEigen(eval_->qfrc_passive, nv);
  // Eigen::VectorXd constraint = toEigen(eval_->qfrc_constraint, nv);
  return bias + passive;
}

Eigen::VectorXd DynamicsSimulator::integrate(const Eigen::VectorXd &q, int k, double eps) {
  // Copy configuration
  toMj(q, eval_->qpos);

  // Set unit tangent velocity
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  v[k] = 1.0;
  toMj(v, eval_->qvel);

  // integrate by epsilon
  mj_integratePos(model_, eval_->qpos, eval_->qvel, eps);
  return toEigen(eval_->qpos, nq);
}

Eigen::VectorXd DynamicsSimulator::getGeneralizedForce(int id, double ctrl_value, const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  setEvalState(q, dq);

  // Baseline
  mj_forward(model_, eval_);
  Eigen::VectorXd tau0 = toEigen(eval_->qfrc_actuator, nv);

  // Perturbed
  setEvalState(q, dq);
  eval_->ctrl[id] = ctrl_value;
  mj_forward(model_, eval_);
  Eigen::VectorXd tau1 = toEigen(eval_->qfrc_actuator, nv);

  // std::cout << "gen f:\t" << (tau1-tau0).transpose() << std::endl;
  return (tau1 - tau0);
}

std::vector<Eigen::VectorXcd> DynamicsSimulator::computeResponse(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Compute temporary mass matrix
  Eigen::MatrixXd M0 = computeMass(q);

  // Compute the jacobian of the coriolis hydro; V - tau_fluid
  Eigen::MatrixXd C0(nv, nv), K0(nv, nv);
  
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

  // PD control matrix
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(nv, nv);
  for (int i = 0; i < config_.N; ++i) {
    if (!config_.isActivate[i]) continue;
    int j = (!config_.isFixed) ? 6 + 2*i : 2*i;
    Kp(j, j) = config_.kp;
    Kd(j, j) = config_.kd;
  }

  // Currently only consider a single actuator input(by symmetry)
  std::vector<Eigen::VectorXcd> response;
  for (int i = 0; i < config_.N; ++i) {
    double w = 2*M_PI*freq_;

    Eigen::VectorXcd q_hat, u_hat;
    Eigen::MatrixXcd temp, G;
    u_hat = Eigen::VectorXcd::Zero(nv);
    if (config_.isActivate[i]) {
      int j = (!config_.isFixed) ? 6 + 2*i : 2*i;
      u_hat(j) = -1.0i*amp_;
    }
    temp = -w*w*M0 + 1.0i*w*(C0+Kd) + (K0+Kp);
    // temp += config_.reg * Eigen::MatrixXcd::Identity(nv, nv);
    
    G = temp.inverse();
    q_hat.noalias() = G*Kp*u_hat;
    response.push_back(q_hat);
  }

  return response;
}

std::vector<Eigen::VectorXcd> DynamicsSimulator::computeMuJoCoResponse(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  // Set eval state
  Eigen::VectorXd ctrl = toEigen(data_->ctrl, nu);
  setEvalState(q, dq, ctrl);

  // Run forward step for evaluation
  mj_forward(model_, eval_);
  
  // Decrease iteration
  const int prev_iter = model_->opt.iterations;
  model_->opt.iterations = 5;

  // Linearize MuJoCo step dx_{k+1} = A dx_k + B du_k
  std::vector<mjtNum> A_raw(ndx * ndx);
  std::vector<mjtNum> B_raw(ndx * nu);
  mjd_transitionFD(model_, eval_, 1e-6, 0, A_raw.data(), B_raw.data(), nullptr, nullptr);
  
  const Eigen::MatrixXd A = toEigen(A_raw.data(), ndx, ndx);
  const Eigen::MatrixXd B = toEigen(B_raw.data(), ndx, nu);
  
  // Restore iteration
  model_->opt.iterations = prev_iter;
  
  // PD controller gains; du = Ku*ddesired - Kx*dx
  Eigen::MatrixXd Kx = Eigen::MatrixXd::Zero(nu, ndx);
  Eigen::MatrixXd Ku = Eigen::MatrixXd::Zero(nu, nu);
  for (int i = 0; i < config_.N; ++i) {
    if (!config_.isActivate[i]) continue;
    int j = (!config_.isFixed) ? 6 + 2*i : 2*i;
    Ku(i, i) = config_.kp;
    Kx(i, j) = config_.kp;
    Kx(i, nv+j) = config_.kd;
  }

  const Eigen::MatrixXd Acl = A - B * Kx;
  const Eigen::MatrixXd Bcl = B * Ku;

  // Get frequency response
  const double dt = config_.dt;
  const double omega = 2.0*M_PI*freq_;
  const double Omega = omega*dt;
  const std::complex<double> z = std::exp(1.0i*Omega);

  Eigen::MatrixXcd inv = z * Eigen::MatrixXcd::Identity(ndx, ndx);
  inv -= Acl.cast<std::complex<double>>();
  Eigen::PartialPivLU<Eigen::MatrixXcd> lu(inv);

  std::vector<Eigen::VectorXcd> response;
  for (int i = 0; i < config_.N; ++i) {
    Eigen::VectorXcd u_hat = Eigen::VectorXcd::Zero(nu);;

    // Control input; desired pose
    if (config_.isActivate[i]) {
      // int j = (!config_.isFixed) ? 6 + 2*i : 2*i;
      u_hat(i) = -1.0i*1e-6;
    }
    
    // Solve
    const Eigen::VectorXcd rhs = (Bcl * u_hat);
    Eigen::VectorXcd x_hat = lu.solve(rhs);

    // Get q_hat
    Eigen::VectorXcd q_hat = x_hat.head(nv);
    response.push_back(q_hat);
  }

  return response;
}

void DynamicsSimulator::setEvalState(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &ctrl) {
  toMj(q, eval_->qpos);
  toMj(dq, eval_->qvel);
  toMj(ctrl, eval_->ctrl);

  mju_zero(eval_->qfrc_applied, model_->nv);
  mju_zero(eval_->xfrc_applied, 6 * model_->nbody);
}

void DynamicsSimulator::setEvalState(const Eigen::VectorXd &q, const Eigen::VectorXd &dq) {
  const Eigen::VectorXd ctrl = Eigen::VectorXd::Zero(nu);
  setEvalState(q, dq, ctrl);
}
