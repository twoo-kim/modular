#include "simulation/mujoco_simulator.hpp"
using std::placeholders::_1;

MujocoSimulator::MujocoSimulator() : rclcpp::Node("mujoco_simulator") {
  // Params
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "default");
  this->get_parameter("config_path", config_path);
  if (config_path == "default") {
    RCLCPP_WARN(get_logger(), "No configuration path parameter!");
  }

  // Load Configuration
  config_ = SimConfig::load(config_path);

  // Make multi-agent simulation xml file
  std::filesystem::path xml_path = generateColonyXML(config_);
  if (!std::filesystem::exists(xml_path)) {
    RCLCPP_ERROR(get_logger(), "Failed to generate colony xml file!");
  }

  // MuJoCo
  char err[1024];
  model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
  if (!model_) {
    RCLCPP_ERROR(get_logger(), "mj_loadXML Failed!: %s", err);
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    RCLCPP_ERROR(get_logger(), "mj_makeData Failed!");
  }

  // Viewer
  if (config_.visualize) {
    viewer_ = std::make_unique<MujocoViewer>(model_, data_, config_.width, config_.height, "Mujoco");
  }

  // Identify modules
  modules_ = getModules();
  if (modules_.empty()) {
    RCLCPP_ERROR(get_logger(), "No modules found in model!");
  }
  if (modules_.size() != static_cast<size_t>(config_.N)) {
    RCLCPP_WARN(get_logger(), "Module size mismatch: model=%zu, N=%d", modules_.size(), config_.N);
  }

  // ROS2
  for (int idx = 0; idx < modules_.size(); idx++) {
    const std::string topic = modules_[idx].prefix + "module/control";
    auto sub = this->create_subscription<modular_msgs::msg::ControlMsg>(topic, 10, std::bind(&MujocoSimulator::controlCallback, this, _1));
    control_subs_.push_back(sub);
  }
  pose_pub_ = this->create_publisher<modular_msgs::msg::StateMsg>("/pose_topic", 1);
}

MujocoSimulator::~MujocoSimulator() {
  mj_deleteData(data_);
  mj_deleteModel(model_);
}

void MujocoSimulator::controlCallback(const modular_msgs::msg::ControlMsg::SharedPtr msg) {
  // Get module prefix; ID
  const int m_id = msg->index;
  
  // Find module
  for (auto &m: modules_) {
    int id = m.module_id;
    if (id == m_id) {
      int a_id = m.aid_motor;
      if (a_id >= 0 && a_id < model_->nu) {
        // Get control and allocate value to the corresponding control list
        data_->ctrl[a_id] = msg->ctrl;
      } else {
        RCLCPP_ERROR(get_logger(), "Invalid actuator id %d with module id %d", a_id, m_id);
      }
      // Get internal phase
      m.phi = msg->phi;
      break;
    }
  }
}

void MujocoSimulator::publishPose() {
  modular_msgs::msg::StateMsg msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  std::vector<int> index;
  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<geometry_msgs::msg::Twist> twists;
  std::vector<geometry_msgs::msg::Vector3> forces;
  std::vector<double> j_motors;
  std::vector<double> j_paddles;
  std::vector<double> phases;
  for (auto m: modules_) {
    // Order of modules
    index.push_back(m.module_id);

    // Joint values
    j_motors.push_back(data_->qpos[m.jid_motor]);
    j_paddles.push_back(data_->qpos[m.jid_paddle]);

    // Force applied to the COM
    geometry_msgs::msg::Vector3 f;
    auto d = data_->sensordata;
    int id = 3*m.sid_force;
    f.x = d[id], f.y = d[id + 1], f.z = d[id + 2];
    forces.push_back(f);

    // Internal phase
    phases.push_back(m.phi);
  }

  msg.index = index;
  msg.joint_motor = j_motors;
  msg.joint_paddle = j_paddles;
  msg.force = forces;
  msg.phase = phases;
  pose_pub_->publish(msg);
}

void MujocoSimulator::run() {
  if (!model_ || !data_) return;

  // 1. Step simulation
  mj_step(model_, data_);

  // 2. Publish pose
  publishPose();

  // 3. Render
  if (config_.visualize) {
    viewer_->render();
    viewer_->pollEvents();
  }
}

static bool checkPrefix(const std::string &s) {
  if (s.size() < 3 || s[0] != 'r') return false;
  auto us = s.find('_');
  if (us == std::string::npos || us < 2) return false;
  for (int i = 1; i < us; i++) {
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
  }
  return true;
}

std::vector<Module> MujocoSimulator::getModules() {
  std::vector<std::string> prefs;
  for (int j = 0; j < model_->njnt; j++) {
    // Get name from id
    std::string name = mj_id2name(model_, mjOBJ_JOINT, j);

    // Check prefix and store
    if (checkPrefix(name)) {
      auto us = name.find('_');
      std::string pf = name.substr(0,us+1);
      if (!contains(prefs, pf))
        prefs.push_back(pf);     // r1_, r2_, ...
    }
  }
  
  std::vector<Module> modules;
  for (const auto &pf: prefs) {
    int m_id = std::stoi(pf.substr(1));
    int jid_m = mj_name2id(model_, mjOBJ_JOINT, (pf + "motor_hinge").c_str());
    int jid_p = mj_name2id(model_, mjOBJ_JOINT, (pf + "paddle_hinge").c_str());
    int aid_m = mj_name2id(model_, mjOBJ_ACTUATOR, (pf + "motor").c_str());
    int sid_f = mj_name2id(model_, mjOBJ_SENSOR, (pf + "module_force").c_str());
    int sid_m = mj_name2id(model_, mjOBJ_SENSOR, (pf + "module_torque").c_str());
    if (jid_m < 0 || aid_m < 0) continue;

    auto m = Module(pf, m_id, jid_m, jid_p, aid_m, sid_f, sid_m);
    modules.push_back(m);
  }
  return modules;
}


