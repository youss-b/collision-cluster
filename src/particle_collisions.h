#pragma once

#include <Eigen/Dense>
#include <cmath>

// Per-particle-radius pairwise collision with contact-tangent spin blend.
// Shoves overlapping particles apart along the contact normal (mass-weighted
// via inv_mass, so heavier particles move less; equal masses reduce exactly to
// a 0.5/0.5 split), then blends the perpendicular angular velocity components
// at the contact. See "Collisions and spin" in the README.
inline void ApplyPairwiseCollisionsWithSpin(Eigen::Matrix3Xf& pos,
                                            int active_count,
                                            const Eigen::VectorXf& radius,
                                            const Eigen::VectorXf& inv_mass,
                                            Eigen::Matrix3Xf& angular_vel,
                                            float alpha) {
  for (int i = 0; i < active_count; ++i) {
    Eigen::Vector3f obj1 = pos.col(i);
    const float r_i = radius(i);
    const float im_i = inv_mass(i);
    for (int j = i + 1; j < active_count; ++j) {
      Eigen::Vector3f obj2 = pos.col(j);
      const float r_j = radius(j);
      const float contact = r_i + r_j;
      const Eigen::Vector3f delta = obj2 - obj1;
      const float sq = delta.squaredNorm();
      if (sq >= contact * contact || sq <= 0.0f) continue;

      const float dist = std::sqrt(sq);
      const Eigen::Vector3f n = delta / dist;
      const float overlap = contact - dist;

      const float im_j = inv_mass(j);
      const float inv_total = 1.0f / (im_i + im_j);
      const float w_i = im_i * inv_total;
      const float w_j = im_j * inv_total;

      pos.col(i) = obj1 - w_i * overlap * n;
      pos.col(j) = obj2 + w_j * overlap * n;
      obj1 = pos.col(i);

      const Eigen::Vector3f wi = angular_vel.col(i);
      const Eigen::Vector3f wj = angular_vel.col(j);
      const float wi_par = wi.dot(n);
      const float wj_par = wj.dot(n);
      const Eigen::Vector3f wi_perp = wi - wi_par * n;
      const Eigen::Vector3f wj_perp = wj - wj_par * n;

      const Eigen::Vector3f wi_perp_new =
          wi_perp + alpha * w_i * (wj_perp - wi_perp);
      const Eigen::Vector3f wj_perp_new =
          wj_perp + alpha * w_j * (wi_perp - wj_perp);

      angular_vel.col(i) = wi_par * n + wi_perp_new;
      angular_vel.col(j) = wj_par * n + wj_perp_new;
    }
  }
}
