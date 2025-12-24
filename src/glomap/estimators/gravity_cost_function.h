#pragma once

#include "colmap/estimators/cost_functions.h"

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace glomap {

// Cost function for gravity alignment priors.
class GravityAlignmentCostFunctor
    : public colmap::AutoDiffCostFunctor<GravityAlignmentCostFunctor, 1, 4> {
 public:
  explicit GravityAlignmentCostFunctor(const Eigen::Vector3d& gravity)
      : gravity_(gravity) {
    gravity_.normalize();
  }

  template <typename T>
  bool operator()(const T* const cam_from_world_rotation, T* residuals) const {
    // Convert quaternion to rotation matrix
    const Eigen::Matrix<T, 3, 3> rotation_matrix =
        colmap::EigenQuaternionMap<T>(cam_from_world_rotation).toRotationMatrix();

    // Extract the second column (down direction in camera frame)
    const Eigen::Matrix<T, 3, 1> cam_down = rotation_matrix.col(1);

    // Compute dot product with gravity
    const Eigen::Matrix<T, 3, 1> gravity_world = gravity_.cast<T>();
    const T dot_product = cam_down.dot(gravity_world);

    // Compute residual
    residuals[0] = T(1.0) - dot_product;

    return true;
  }

 private:
  Eigen::Vector3d gravity_;
};

}  // namespace glomap

