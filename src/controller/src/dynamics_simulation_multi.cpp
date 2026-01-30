#include "simulation/dynamics_simulator.hpp"
#include "matplotlibcpp.h"
#include <string>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>

namespace plt = matplotlibcpp;

static std::vector<double> parse_phase_list_from_argv(int argc, char** argv) {
  // Optional CLI: numbers in degrees separated by commas, e.g. --phases=0,30,60,90
  // Falls back to a default sweep if not provided.
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    const std::string key = "--phases=";
    if (arg.rfind(key, 0) == 0) {
      std::vector<double> out;
      std::string rest = arg.substr(key.size());
      std::stringstream ss(rest);
      std::string item;
      while (std::getline(ss, item, ',')) {
        try {
          out.push_back(std::stod(item));
        } catch (...) {}
      }
      if (!out.empty()) return out;
    }
  }
  // Default in degrees
  return {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
}

int main(int argc, char** argv) {
  //-------------------- Initialize ---------------------//
  // Configuration file
  std::string config_path = "/home/twkim/ros2_ws/src/modular/src/controller/config/modular_params.yaml";
  SimConfig base_config = SimConfig::load(config_path);
  base_config.printData();

  if (base_config.N != 2) {
    std::cerr << "This multi-phase runner assumes N=2 (two modules). Current N=" << base_config.N << std::endl;
  }

  // Phase list (degrees); will apply to module 1 only (index 1)
  std::vector<double> phase_deg_list = parse_phase_list_from_argv(argc, argv);

  //------------------- Data for plot -------------------//
  const double dt = base_config.dt;
  const int steps = static_cast<int>(base_config.sim_time / dt);

  // Time vector
  std::vector<double> t;
  t.reserve(steps);
  double time = 0.0;
  for (int i = 0; i < steps; ++i) { time += dt; t.push_back(time); }

  // Single amplitude/frequency like the baseline script
  const double freq = 0.5;         // [Hz]
  const double amp_deg = 45.0;     // [deg]

  // Per-scenario storage
  std::vector<std::vector<double>> forces_per_phase;
  std::vector<std::vector<double>> mod1_passive_per_phase;
  std::vector<std::vector<double>> htf_mag_mod1_per_phase;
  std::vector<int> htf_order;  // common x-axis for HTF plot

  //------------------- Run simulations -------------------//
  for (double ph_deg : phase_deg_list) {
    // Prepare config per scenario
    SimConfig cfg = base_config;
    if (cfg.phase_gap.size() < 2) cfg.phase_gap.resize(2, 0.0);
    cfg.phase_gap[0] = 0.0;                   // module 0 fixed to 0 phase
    cfg.phase_gap[1] = ph_deg * M_PI / 180.0; // module 1 gets varying phase

    std::unique_ptr<DynamicsSimulator> sim = std::make_unique<DynamicsSimulator>(cfg);
    sim->setFrequency(freq);
    sim->setAmplitude(amp_deg * M_PI / 180.0);

    std::vector<double> fz;
    fz.reserve(steps);
    std::vector<double> q_mod1_passive;
    q_mod1_passive.reserve(steps);

    for (int k = 0; k < steps; ++k) {
      sim->step();
      const Eigen::VectorXd x = sim->getState();

      // module index 1 passive joint corresponds to x(7 + 2*1 + 1) = x(10) in this model layout
      // Keep formula for clarity
      int idx_passive = 7 + 2 * 1 + 1;
      if (idx_passive < x.size()) {
        q_mod1_passive.push_back(x(idx_passive));
      } else {
        q_mod1_passive.push_back(0.0);
      }

      fz.push_back(sim->getForce().z());

      // Populate LTV data for HTF computation
      const int nq = sim->nq;
      const int nv = sim->nv;
      const Eigen::VectorXd q = x.head(nq);
      const Eigen::VectorXd dq = x.tail(nv);
      sim->computeMuJoCoResponse(q, dq);
    }

    forces_per_phase.push_back(std::move(fz));
    mod1_passive_per_phase.push_back(std::move(q_mod1_passive));

    // Compute HTF for this scenario
    Eigen::VectorXcd q_hat = sim->computeHTF();
    const int N = cfg.harmonics;
    const int H = 2*N + 1;
    const int nv = sim->nv;

    // Build order vector once
    if (htf_order.empty()) {
      htf_order.reserve(H);
      for (int h = 0; h < H; ++h) htf_order.push_back(h - N);
    }

    // Extract magnitude for module 1 passive joint across harmonics
    std::vector<double> mag_mod1;
    mag_mod1.reserve(H);
    const int idx_mod1_passive = 2*1 + 7; // same joint index used in single-case code (passive)
    for (int h = 0; h < H; ++h) {
      int idx = nv*h + idx_mod1_passive;
      std::complex<double> val = q_hat(idx);
      mag_mod1.push_back(std::abs(val));
    }
    htf_mag_mod1_per_phase.push_back(std::move(mag_mod1));
  }

  //------------------------ Plot --------------------------//
  // Force overlay
  plt::figure_size(1200, 800);
  for (size_t i = 0; i < phase_deg_list.size(); ++i) {
    std::string label = "force phase " + std::to_string(static_cast<int>(phase_deg_list[i])) + " deg";
    plt::named_plot(label, t, forces_per_phase[i]);
  }
  plt::title("Thrust-Time (multiple phase offsets)");
  plt::xlabel("Time (s)");
  plt::ylabel("Force (N)");
  plt::legend();

  // Module 1 passive angle overlay
  plt::figure_size(1200, 800);
  for (size_t i = 0; i < phase_deg_list.size(); ++i) {
    std::string label = "mod1 passive " + std::to_string(static_cast<int>(phase_deg_list[i])) + " deg";
    plt::named_plot(label, t, mod1_passive_per_phase[i]);
  }
  plt::title("Module 1 Passive Angle-Time (multiple phase offsets)");
  plt::xlabel("Time (s)");
  plt::ylabel("Angle (rad)");
  plt::legend();

  // HTF magnitude overlay for module 1 passive
  plt::figure_size(1200, 800);
  for (size_t i = 0; i < phase_deg_list.size(); ++i) {
    std::string label = "HTF |module1 passive| " + std::to_string(static_cast<int>(phase_deg_list[i])) + " deg";
    plt::named_plot(label, htf_order, htf_mag_mod1_per_phase[i]);
  }
  plt::title("LTP response magnitude-harmonic (module 1 passive)");
  plt::xlabel("Harmonic order (n*omega)");
  plt::ylabel("Magnitude");
  plt::legend();

  plt::show();

  return 0;
}
