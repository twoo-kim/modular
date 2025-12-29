#include "simulation/dynamics_simulator.hpp"
#include "matplotlibcpp.h"
#include <string>
#include <cmath>

#include <iostream>

namespace plt = matplotlibcpp;

int main(int argc, char** argv) {
  //-------------------- Initialize ---------------------//
  // Configuration file
  std::string config_path = "/home/twkim/ros2_ws/src/modular/src/controller/config/modular_params.yaml";
  SimConfig config = SimConfig::load(config_path);
  config.printData();

  // System
  std::unique_ptr<DynamicsSimulator> colony = std::make_unique<DynamicsSimulator>(config);
  
  //------------------- Data for plot -------------------//
  double time_step = config.dt;
  int steps = static_cast<int>(config.sim_time/time_step);
  int n_freq = 1,  n_amp = 1;
  std::vector<double> t(0);

  // Set frequency
  std::vector<double> frequency;
  for (int n = 0; n < n_freq; n++) {
    double freq = 0.5 + 0.25*static_cast<double>(n);
    frequency.push_back(freq);
  }

  // Set amplitude
  std::vector<double> amplitude;
  for (int n = 0; n < n_amp; n++) {
    double amp = 45.0 + 10.0*static_cast<double>(n);
    amplitude.push_back(amp);
  }

  // Cache each joint trajectory
  std::vector<std::vector<std::vector<std::vector<double>>>> q;

  // Set forces
  std::vector<std::vector<std::vector<double>>> f;
  for (auto _: amplitude) {
    std::vector<std::vector<double>> fi;
    std::vector<std::vector<std::vector<double>>> qi;
    for (auto __: frequency) {
      std::vector<double> fj(0);
      fi.push_back(fj);

      std::vector<std::vector<double>> qj;
      for (int n = 0; n < config.N*2; n++) {
        std::vector<double> qn(0);
        qj.push_back(qn);
      }
      qi.push_back(qj);
    }
    f.push_back(fi);
    q.push_back(qi);
  }

  // Time line
  double time = 0.0;
  for (int i = 0; i < steps; i++) {
    time += time_step;
    t.push_back(time);
  }

  // XYZ coordinate
  std::vector<std::vector<double>> xyz;
  for (int i = 0; i < 3; i++) {
    std::vector<double> x;
    xyz.push_back(x);
  }

  // Response check
  std::vector<std::vector<std::vector<double>>> magnitude;
  std::vector<std::vector<std::vector<double>>> phase;

  const int nq = colony->nq;
  const int nv = colony->nv;
  //------------------- Run simulation ---------------------//
  for (size_t i = 0; i < amplitude.size(); ++i) {
    double A = amplitude[i];
    colony->setAmplitude(A*M_PI/180.0);
    for (size_t j = 0; j < frequency.size(); ++j) {
      time = 0.0;
      double F = frequency[j];
      colony->setFrequency(F);

      // Response check
      std::vector<std::vector<double>> magnitude_j;
      std::vector<std::vector<double>> phase_j;
      for (int a = 0; a < config.N; ++a) {
        std::vector<double> m, p;
        magnitude_j.push_back(m);
        phase_j.push_back(p);
      }

      std::cout << "amp: " << A << "\tfreq: " << F << std::endl;
      for (int k = 0; k < steps; ++k) {
        // Step simulation
        colony->step();
        time += time_step;

        // Store joint pose
        Eigen::VectorXd x = colony->getState();
        for (int n = 0; n < config.N; ++n) {
          q[i][j][2*n].push_back(x(7+2*n));
          q[i][j][2*n+1].push_back(x(7+2*n+1));
        }
        xyz[0].push_back(x(0));
        xyz[1].push_back(x(1));
        xyz[2].push_back(x(2)-0.5);

        // Get force
        f[i][j].push_back(colony->getForce().z());

        // std::cout << time << " state:\t" << x.transpose() << std::endl;
        
        // Get response
        auto response = colony->computeResponse(x.head(nq), x.tail(nv));
        // std::cout << "q_hat0(" << time << ")\t" << response[0].transpose() << std::endl;
        // std::cout << "q_hat1(" << time << ")\t" << response[1].transpose() << std::endl;

        /********************* Single Paddle phase lag check *****************/
        // Effect of module 0 to the modules 1, 2, 3
        double abs00 = std::abs(response[0](6));
        double abs01 = std::abs(response[0](9));
        // double abs02 = std::abs(response[0](11));
        // double abs03 = std::abs(response[0](13));
        
        double arg00 = std::arg(response[0](6));
        double arg01 = std::arg(response[0](9));
        // double arg02 = std::arg(response[0](11));
        // double arg03 = std::arg(response[0](13));

        magnitude_j[0].push_back(abs00*std::cos(arg00));
        magnitude_j[1].push_back(abs01*std::cos(arg01));
        // magnitude_j[1].push_back(abs01);
        // magnitude_j[2].push_back(value);
        // magnitude_j[3].push_back(abs03);

        // phase_j[0].push_back(arg0);
        // phase_j[1].push_back(std::cos(arg01));
        // phase_j[2].push_back(arg02);
        // phase_j[3].push_back(arg03);
      } // Step loop
      magnitude.push_back(magnitude_j);
      phase.push_back(phase_j);
    } // Frequency loop
  } // Amplitude loop

  //------------------------ Plot --------------------------//
  // JOINT plot
  plt::figure_size(1200, 800);
  plt::named_plot("module0 active", t, q[0][0][0]);
  plt::named_plot("module0 passive", t, q[0][0][1]);
  // plt::named_plot("module1 active", t, q[0][0][2]);
  // plt::named_plot("module1 passive", t, q[0][0][3]);
  plt::title("Joint Angle-Time");
  plt::xlabel("Time (s)");
  plt::ylabel("Angle (rad)");
  plt::legend();
  // plt::show();

  // RESPONSE plot
  plt::figure_size(1200, 800);
  plt::named_plot("module0 active", t, q[0][0][0]);
  plt::named_plot("module0 passive", t, q[0][0][1]);
  // plt::named_plot("0 to 0 response", t, magnitude[0][0], "--");
  plt::named_plot("0 to 1 response", t, magnitude[0][1], "--");
  // plt::named_plot("0 to 2 response", t, magnitude[0][2], "--");
  // plt::named_plot("0 to 3 response", t, magnitude[0][3], "--");
  plt::title("Response-Time");
  plt::xlabel("Time (s)");
  plt::ylabel("Angle (rad)");
  plt::legend();

  // plt::figure_size(1200, 800);
  // plt::named_plot("module0 active", t, q[0][0][0]);
  // plt::named_plot("module0 passive", t, q[0][0][1]);
  // plt::named_plot("0 to 1 phase", t, phase[0][1], "--");
  // plt::named_plot("0 to 2 phase", t, phase[0][2], "--");
  // plt::named_plot("0 to 3 phase", t, phase[0][3], "--");
  // plt::title("Response Phase-Time");
  // plt::xlabel("Time (s)");
  // plt::ylabel("Angle (rad)");
  // plt::legend();

  // XYZ plot
  plt::figure_size(1200, 800);
  plt::named_plot("x", t, xyz[0]);
  plt::named_plot("y", t, xyz[1]);
  plt::named_plot("z", t, xyz[2]);
  plt::title("XYZ Position-Time");
  plt::xlabel("Time (s)");
  plt::ylabel("Position (m)");
  plt::legend();
  // plt::show();
  
  // FORCE plot
  plt::figure_size(1200, 800);
  plt::named_plot("z-force", t, f[0][0]);
  plt::title("Thrust-Time");
  plt::xlabel("Time (s)");
  plt::ylabel("Force (N)");
  plt::show();

  // std::cout << "average phase: " << sumVec(phase[0][1]) / static_cast<double>(phase[0][1].size()) << std::endl;
  // std::cout << "average force: " << sumVec(f[0][0]) / static_cast<double>(f[0][0].size()) << std::endl;

  return 0;
}
