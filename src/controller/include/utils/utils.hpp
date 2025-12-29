#pragma once
#include <vector>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

// Utiltiy functions that converts mjNum and Eigen
inline Eigen::VectorXd toEigen(const mjtNum *arr, int n) {
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) {
    v[i] = static_cast<double>(arr[i]);
  }
  return v;
}

inline void toMj(const Eigen::VectorXd &v, mjtNum *arr) {
  const int n = v.size();
  for (int i = 0; i < n; ++i) {
    arr[i] = static_cast<mjtNum>(v[i]);
  }
}

// Check if element x is contained in the vector v
template <typename T>
inline bool contains(const std::vector<T> &v, const T &x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

// Change vector to Eigen VectorXd
template <typename T>
inline Eigen::VectorXd toEigen(const std::vector<T> &v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
}

// Change array(pointer) to Eigen elements
template <typename T>
inline Eigen::Vector3d toEigen(const T* arr, int id) {
  const T* v = arr + 3 * id;
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

template <typename T>
inline Eigen::Quaterniond toEigen(const T* arr, int id) {
  const T* q = arr + 4 * id;
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

// Change array to geometry_msgs
template <typename T>
inline geometry_msgs::msg::Point toGeoMsg(const T* arr, int id) {
  const T* p = arr + 3 * id;
  geometry_msgs::msg::Point pt(p[0], p[1], p[2]);
  return pt;
}
template <typename T>
inline geometry_msgs::msg::Quaternion toGeoMsg(const T* arr, int id) {
  const T* q = arr + 4 * id;
  geometry_msgs::msg::Quaternion quat(q[0], q[1], q[2], q[3]);
  return quat;
}

// Change array(pointer) to std::vector
template <typename T>
inline std::vector<T> toVector(const T* arr, int length) {
  std::vector<T> v;
  for (int i = 0; i < length; i++) {
    v.push_back(*(arr + i));
  }
  return v;
}


