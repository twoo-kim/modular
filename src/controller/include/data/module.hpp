#pragma once
#include <string>

struct Module {
  std::string prefix;     // Prefix
  int module_id;          // unique module ID
  int jid_motor;          // motor_hinge joint ID
  int jid_paddle;         // paddle_hinge joint ID
  int aid_motor;          // actuator ID

  int sid_force;          // force sensor ID
  int sid_torque;         // torque sensor ID

  double phi;             // Internal phase

  Module() = default;
  Module(std::string p, const int m_id, const int jid_m,
    const int jid_p, const int aid_m, const int sid_f, const int sid_m) {
    prefix = p;
    module_id = m_id;
    jid_motor = jid_m;
    jid_paddle = jid_p;
    aid_motor = aid_m;
    sid_force = sid_f;
    sid_torque = sid_m;
  }
};