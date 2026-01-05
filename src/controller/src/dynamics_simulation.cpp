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
  const double dt = config.dt;
  int steps = static_cast<int>(config.sim_time/dt);
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
  std::vector<std::vector<std::vector<std::vector<double>>>> Q;

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
    Q.push_back(qi);
  }

  // Time line
  double time = 0.0;
  for (int i = 0; i < steps; i++) {
    time += dt;
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

        // Store joint pose
        Eigen::VectorXd x = colony->getState();
        for (int n = 0; n < config.N; ++n) {
          Q[i][j][2*n].push_back(x(7+2*n));
          Q[i][j][2*n+1].push_back(x(7+2*n+1));
        }
        xyz[0].push_back(x(0));
        xyz[1].push_back(x(1));
        xyz[2].push_back(x(2)-0.5);

        // Get force
        f[i][j].push_back(colony->getForce().z());

        // std::cout << time << " state:\t" << x.transpose() << std::endl;
        
        // Get response
        const Eigen::VectorXd q = x.head(nq);
        const Eigen::VectorXd dq = x.tail(nv);
        auto response = colony->computeMuJoCoResponse(q, dq);
        // std::cout << "q_hat0(" << time << ")\t" << response[0].transpose() << std::endl;
        // std::cout << "q_hat1(" << time << ")\t" << response[1].transpose() << std::endl;

        /********************* Single Paddle phase lag check *****************/
        const double w = 2*M_PI*colony->freq_;
        std::complex<double> ph0 = response[0](6);
        std::complex<double> ph00 = response[0](7);
        std::complex<double> ph01 = response[0](9);
        std::complex<double> ph10 = response[1](7);
        std::complex<double> ph11 = response[1](9);
        
        // double abs0 = std::abs(response[0](6));
        // double abs00 = std::abs(response[0](7));
        // double abs01 = std::abs(response[0](9));
        // double abs02 = std::abs(response[0](11));
        // double abs03 = std::abs(response[0](13));
        // double abs1 = std::abs(response[0](8));
        // double abs10 = std::abs(response[1](7));
        // double abs11 = std::abs(response[1](9));
        
        // double arg0 = std::arg(response[0](6));
        // double arg00 = std::arg(response[0](7));
        // double arg01 = std::arg(response[0](9));
        // double arg02 = std::arg(response[0](11));
        // double arg03 = std::arg(response[0](13));
        // double arg1 = std::arg(response[0](8));
        // double arg10 = std::arg(response[1](7));
        // double arg11 = std::arg(response[1](9));

        double pred0 = std::real(ph0 * std::exp(1.0i * w * time));
        double pred00 = std::real(ph00 * std::exp(1.0i * w * time));
        double pred01 = std::real(ph01 * std::exp(1.0i * w * time));
        double pred10 = std::real(ph10 * std::exp(1.0i * w * time));
        double pred11 = std::real(ph11 * std::exp(1.0i * w * time));

        // double j0 = q(7) + pred0;
        // double j00 = q(8) + pred00;
        // double j01 = q(10) + pred01;
        // static double passive0 = 0.0, passive00 = 0.0;
        // passive0 += pred0*dt;
        // passive00 += (pred11+pred10)*dt;

        magnitude_j[0].push_back(pred00);
        magnitude_j[1].push_back(pred10);
        // magnitude_j[1].push_back(abs01*std::cos(w*time + arg01));
        // magnitude_j[1].push_back(abs02*std::cos(w*time + arg02));
        // magnitude_j[2].push_back(abs02);
        // magnitude_j[3].push_back(abs03);

        // phase_j[0].push_back(std::cos(arg00));
        // phase_j[1].push_back(std::cos(arg01));
        // phase_j[2].push_back(std::cos(arg02));
        // phase_j[3].push_back(arg03);
        time += dt;
      } // Step loop
      magnitude.push_back(magnitude_j);
      phase.push_back(phase_j);
    } // Frequency loop
  } // Amplitude loop

  //------------------------ Plot --------------------------//
  // JOINT plot
  // plt::figure_size(1200, 800);
  // plt::named_plot("module0 active", t, q[0][0][0]);
  // plt::named_plot("module0 passive", t, q[0][0][1]);
  // plt::named_plot("module1 active", t, q[0][0][2]);
  // plt::named_plot("module1 passive", t, q[0][0][3]);
  // plt::title("Joint Angle-Time");
  // plt::xlabel("Time (s)");
  // plt::ylabel("Angle (rad)");
  // plt::legend();
  // plt::show();

  // RESPONSE plot
  plt::figure_size(1200, 800);
  // plt::named_plot("module0 active", t, Q[0][0][0]);
  // plt::named_plot("module0 passive", t, Q[0][0][1]);
  plt::named_plot("0 to 0 passive response", t, magnitude[0][0], "--");
  plt::named_plot("1 to 0 passive response", t, magnitude[0][1], "--");
  // plt::named_plot("0 to 1 response", t, magnitude[0][2], "--");
  // plt::named_plot("0 to 2 response", t, magnitude[0][2], "--");
  // plt::named_plot("0 to 3 response", t, magnitude[0][3], "--");
  plt::title("Response-Time");
  plt::xlabel("Time (s)");
  plt::ylabel("Angle (rad)");
  plt::legend();

  // plt::figure_size(1200, 800);
  // plt::named_plot("module0 active", t, q[0][0][0]);
  // plt::named_plot("module0 passive", t, q[0][0][1]);
  // plt::named_plot("0 to 0 active phase", t, phase[0][0], "--");
  // plt::named_plot("0 to 0 passive phase", t, phase[0][1], "--");
  // plt::named_plot("0 to 1 phase", t, phase[0][2], "--");
  // plt::title("Response Phase-Time");
  // plt::xlabel("Time (s)");
  // plt::ylabel("Angle (rad)");
  // plt::legend();

  // XYZ plot
  // plt::figure_size(1200, 800);
  // plt::named_plot("x", t, xyz[0]);
  // plt::named_plot("y", t, xyz[1]);
  // plt::named_plot("z", t, xyz[2]);
  // plt::title("XYZ Position-Time");
  // plt::xlabel("Time (s)");
  // plt::ylabel("Position (m)");
  // plt::legend();
  // plt::show();
  
  // FORCE plot
  // plt::figure_size(1200, 800);
  // plt::named_plot("z-force", t, f[0][0]);
  // plt::title("Thrust-Time");
  // plt::xlabel("Time (s)");
  // plt::ylabel("Force (N)");
  plt::show();

  // std::cout << "average phase: " << sumVec(phase[0][1]) / static_cast<double>(phase[0][1].size()) << std::endl;
  // std::cout << "average force: " << sumVec(f[0][0]) / static_cast<double>(f[0][0].size()) << std::endl;

  return 0;
}
