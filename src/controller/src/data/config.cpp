#include "data/config.hpp"

SimConfig SimConfig::load(const std::string &path) {
    YAML::Node file = YAML::LoadFile(path);

    SimConfig config;
    config.xml_dir = file["mujoco_xml_dir"].as<std::string>();
    config.width = file["width"].as<int>();
    config.height = file["height"].as<int>();
    
    config.N = file["N"].as<int>();
    config.radius = file["RADIUS"].as<double>();
    config.frequency = file["FREQUENCY"].as<double>();
    config.sim_time = file["SIM_TIME"].as<double>();

    config.face_radial = file["FACE_RADIAL"].as<bool>();
    config.yaw_offset = file["YAW_OFFSET_DEG"].as<double>();
    config.add_colony_dummy_mass = file["ADD_DUMMY_MASS"].as<bool>();

    return config;
}
