#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>

/* Simulation configuration */
struct SimConfig {
  std::string xml_dir;            // Path to the xml directory
  int width;                      // Width of the window
  int height;                     // Height of the window
  bool visualize;                  // Visualize via viewer if true

  int N;                          // Number of agents/bots
  double radius;                  // Radius of the colony
  bool face_radial;
  double yaw_offset;
  bool add_colony_dummy_mass;

  double sim_time;                // Simulation time
  double dt;                      // Unit time step
  std::vector<bool> isActivate;    // Check which module actuator to activate

  double kp, kd;                  // PD gain for actuator controller
  double cn, rho, viscosity;      // Fluid dynamics coefficients
  bool isEuler;                   // Use Euler if true, and RK4 for false
  static SimConfig load(const std::string &path);
  void printData();
};

