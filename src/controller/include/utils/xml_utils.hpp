#pragma once
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include "data/config.hpp"

// Get quaternion string from yaw
inline std::string yaw2quat(double yaw) {
  const double w = std::cos(yaw * 0.5);
  const double z = std::sin(yaw * 0.5);
  std::ostringstream quat;
  quat << w << " 0 0 " << z;
  return quat.str();
}

// Generate MCJF file that makes colony
inline std::filesystem::path generateColonyXML(SimConfig &cfg) {
  const int n = cfg.N;
  const double radius = cfg.radius;
  const std::filesystem::path xml_dir = cfg.xml_dir;
    
  // Check files path
  if (!std::filesystem::exists(xml_dir)) {
    std::filesystem::create_directories(xml_dir);
  }

  // Create robot
  std::ostringstream bots;
  for (int i = 0; i < n; i++) {
    const double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
    const double x = radius * std::cos(ang);
    const double y = radius * std::sin(ang);
    
    // Quaternion
    std::string quat;
    if (cfg.face_radial) {
      const double yaw = std::atan2(y, x) + M_PI/2.0 + cfg.yaw_offset;
      quat = yaw2quat(yaw);   
    } else {
      quat = "1 0 0 0";
    }

    const int idx = i + 1;
    bots << "\t\t<body name=\"bot" << idx << "\" pos=\""
         << x << " " << y << " 0\" quat=\"" << quat << "\">\n"
         << "\t\t\t<attach model=\"modbot\" body=\"main_body\" prefix=\"r" << idx << "_\"/>\n"
         << "\t\t</body>\n";
  }

  // Dummy mass
  std::string dummy;
  if (cfg.add_colony_dummy_mass) {
    dummy = "\t\t<geom type =\"sphere\" size=\"0.01\" density=\"1000\""
            " rgba=\"0 0 0 0\" contype=\"0\" conaffinity=\"0\"/>\n";
  }
  
  // Total XML
  std::string integrator = (cfg.isEuler) ? "Euler" : "RK4";
  std::ostringstream xml;
  xml << "<mujoco model=\"modular\">\n"
      << "\t<option timestep=\"" << cfg.dt << "\" gravity=\"0 0 0\""
      << " integrator=\"" << integrator << "\" density=\"" << cfg.rho
      << "\" viscosity=\"" << cfg.viscosity << "\" iterations=\"50\" solver=\"Newton\"/>\n"
      << R"(
  <include file="../environment.xml"/>
  <asset>
    <model name="modbot" file="../modular.xml"/>
  </asset>
  <worldbody>
    <body name="colony" pos="0 0 0.5">
    <site name="COM" pos="0 0 0"/>
    <joint name="base" type="free"/>
)";
  xml << dummy;
  xml << bots.str();
  xml << R"(
    </body>
  </worldbody>

  <sensor>
    <force  name="colony_force"  site="COM"/>
    <torque name="colony_torque" site="COM"/>
  </sensor>
</mujoco>
)";

  // Write xml file and return
  const auto out_path = xml_dir / "colony" / ("test_" + std::to_string(n) + "_circle.xml");
  std::ofstream ofs(out_path);
  if (!ofs) {
    throw std::runtime_error("Failed to write xml file");
  }
  ofs << xml.str();
  ofs.close();

  return out_path;
}

