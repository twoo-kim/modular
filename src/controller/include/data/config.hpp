#pragma once
#include <string>
#include <filesystem>
#include <yaml-cpp/yaml.h>

/* Simulation configuration */
struct SimConfig {
  std::string xml_dir;            // Path to the xml directory
  int width;                      // Width of the window
  int height;                     // Height of the window

  int N;                          // Number of agents/bots
  double radius;                  // Radius of the colony
  double frequency;               // Base frequency
  double sim_time;                // Simulation time
  
  bool face_radial;
  double yaw_offset;
  bool add_colony_dummy_mass;

  static SimConfig load(const std::string &path);
};
