#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>
#include <stdexcept>

class MujocoViewer {
public:
  // Constructor
  MujocoViewer(mjModel* m, mjData* d, int w, int h, const char* title);
  
  // Destructor
  ~MujocoViewer();
  
  // Rendering
  void render();
  void pollEvents();
  bool isRunning() const;

  // camera helpers
  void setFree();                // free camera
  void setFixedCam(int camid);   // use a fixed camera from XML
  void setTrackBody(int bodyid); // track a body
  void resetCamera();            // reset to defaults

private:
  // static trampolines -> instance methods
  static void sMouseButton(GLFWwindow*, int, int, int);
  static void sCursorPos(GLFWwindow*, double, double);
  static void sScroll(GLFWwindow*, double, double);
  static void sKey(GLFWwindow*, int, int, int, int);

  // instance callbacks
  void onMouseButton(int button, int action, int mods);
  void onCursorPos(double xpos, double ypos);
  void onScroll(double xoff, double yoff);
  void onKey(int key, int scancode, int action, int mods);

  // helpers
  int mouseAction() const;

  // Mujoco
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjvCamera  cam_;
  mjvOption  opt_;
  mjvScene   scn_;
  mjrContext con_;

  // OpenGL
  GLFWwindow* window_ = nullptr;
  
  // mouse state
  bool btn_left_ = false, btn_mid_ = false, btn_right_ = false;
  double lastx_ = 0.0, lasty_ = 0.0;
};