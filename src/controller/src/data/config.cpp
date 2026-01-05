#include "data/config.hpp"
#include <iostream>

SimConfig SimConfig::load(const std::string &path) {
  YAML::Node file = YAML::LoadFile(path);

  SimConfig config;
  config.xml_dir = file["mujoco_xml_dir"].as<std::string>();
  config.width = file["width"].as<int>();
  config.height = file["height"].as<int>();
  config.visualize = file["visualize"].as<bool>();

  config.N = file["N"].as<int>();
  config.radius = file["RADIUS"].as<double>();
  config.face_radial = file["FACE_RADIAL"].as<bool>();
  config.yaw_offset = file["YAW_OFFSET_DEG"].as<double>();
  config.add_colony_dummy_mass = file["ADD_DUMMY_MASS"].as<bool>();

  config.sim_time = file["SIM_TIME"].as<double>();
  config.dt = file["TIME_STEP"].as<double>();
  config.isFixed = file["isFixed"].as<bool>();
  config.isActivate = file["is_activate"].as<std::vector<bool>>();
  
  config.kp = file["P_gain"].as<double>();
  config.kd = file["D_gain"].as<double>();
  config.cn = file["Cn"].as<double>();
  config.rho = file["rho"].as<double>();
  config.viscosity = file["viscosity"].as<double>();
  config.isEuler = file["isEuler"].as<bool>();

  return config;
}

void SimConfig::printData() {
  std::cout << "========== Colony Configuration ==========" << std::endl;
  std::cout << "Number of agents:\t\t" << N << std::endl;
  std::cout << "Radius of the colony:\t\t" << radius << std::endl;
  std::cout << "========== Paddle Configuration ==========" << std::endl;
  std::cout << "Fluid coefficient:\t\t" << cn << std::endl;
  std::cout << "Fluid density:\t\t\t" << rho << std::endl;
  std::cout << "Fluid viscosity:\t\t" << viscosity << std::endl;
  
  std::string integrator = (isEuler) ? "Euler" : "RK4";
  std::cout << "Integrator:\t\t\t" << integrator << std::endl;
  std::cout << "=========================================" << std::endl;
}