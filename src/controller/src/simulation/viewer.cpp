#include "simulation/viewer.hpp"

MujocoViewer::MujocoViewer(mjModel* m, mjData* d, int w, int h, const char* title)
: m_(m), d_(d) {
  if (!m_ || !d_) throw std::invalid_argument("Null model, data!");

  // Initialize
  if (!glfwInit()) throw std::runtime_error("GLFW init failed");
  window_ = glfwCreateWindow(w, h, title, nullptr, nullptr);
  if (!window_) { glfwTerminate(); throw std::runtime_error("GLFW window failed"); }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  
  // route callbacks to this instance
  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &MujocoViewer::sMouseButton);
  glfwSetCursorPosCallback(window_, &MujocoViewer::sCursorPos);
  glfwSetScrollCallback(window_, &MujocoViewer::sScroll);
  glfwSetKeyCallback(window_, &MujocoViewer::sKey);

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);

  cam_.type = mjCAMERA_FREE;
  cam_.distance = 3.0;

  mjv_makeScene(m_, &scn_, 10000);
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);
}

MujocoViewer::~MujocoViewer() {
  if (window_) {
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    glfwDestroyWindow(window_);
    glfwTerminate();
    window_ = nullptr;
  }
}

void MujocoViewer::render() {
  // update scene from current state
  mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

  // draw
  mjrRect viewport{0,0,0,0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjr_render(viewport, &scn_, &con_);

  glfwSwapBuffers(window_);
}

void MujocoViewer::pollEvents() {
  glfwPollEvents();
}

bool MujocoViewer::isRunning() const {
  return !glfwWindowShouldClose(window_);
}

void MujocoViewer::setFree() {
  cam_.type = mjCAMERA_FREE;
  cam_.trackbodyid = -1;
  cam_.fixedcamid  = -1;
}
void MujocoViewer::setFixedCam(int camid) {
  cam_.type = mjCAMERA_FIXED;
  cam_.fixedcamid = camid;
  cam_.trackbodyid = -1;
}
void MujocoViewer::setTrackBody(int bodyid) {
  cam_.type = mjCAMERA_TRACKING;
  cam_.trackbodyid = bodyid;
  cam_.fixedcamid  = -1;
}
void MujocoViewer::resetCamera() {
  mjv_defaultCamera(&cam_);
  cam_.type = mjCAMERA_FREE;
  cam_.distance = 3.0;
}

// ---- static trampolines ----
void MujocoViewer::sMouseButton(GLFWwindow* w, int b, int a, int m) {
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(w));
  if (self) self->onMouseButton(b,a,m);
}
void MujocoViewer::sCursorPos(GLFWwindow* w, double x, double y) {
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(w));
  if (self) self->onCursorPos(x,y);
}
void MujocoViewer::sScroll(GLFWwindow* w, double xoff, double yoff) {
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(w));
  if (self) self->onScroll(xoff,yoff);
}
void MujocoViewer::sKey(GLFWwindow* w, int k, int sc, int act, int mods) {
  auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(w));
  if (self) self->onKey(k,sc,act,mods);
}

// ---- instance handlers ----
void MujocoViewer::onMouseButton(int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT)   btn_left_  = (action==GLFW_PRESS);
  if (button == GLFW_MOUSE_BUTTON_MIDDLE) btn_mid_   = (action==GLFW_PRESS);
  if (button == GLFW_MOUSE_BUTTON_RIGHT)  btn_right_ = (action==GLFW_PRESS);
  glfwGetCursorPos(window_, &lastx_, &lasty_);
}

int MujocoViewer::mouseAction() const {
  if (btn_right_)  return mjMOUSE_MOVE_H;
  if (btn_mid_)    return mjMOUSE_MOVE_V;
  if (btn_left_)   return mjMOUSE_ROTATE_H;
  return mjMOUSE_NONE;
}

void MujocoViewer::onCursorPos(double xpos, double ypos) {
  if (!btn_left_ && !btn_mid_ && !btn_right_) {
    lastx_ = xpos; lasty_ = ypos;
    return;
  }
  double dx = xpos - lastx_, dy = ypos - lasty_;
  lastx_ = xpos; lasty_ = ypos;

  int width, height; glfwGetWindowSize(window_, &width, &height);
  int act = mouseAction();
  if (act != mjMOUSE_NONE)
    mjv_moveCamera(m_, act, dx/height, dy/height, &scn_, &cam_);
}

void MujocoViewer::onScroll(double, double yoff) {
  mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, 0.02 * yoff, &scn_, &cam_);
}

void MujocoViewer::onKey(int key, int, int action, int) {
  if (action != GLFW_PRESS) return;
  if (key == GLFW_KEY_R) resetCamera();
  if (key == GLFW_KEY_F) setFree();
  if (key == GLFW_KEY_T) setTrackBody(0);  // example: body 0
}
