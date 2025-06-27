#ifndef SENSFUS_KALMAN_EVOLUTION_MODEL_H
#define SENSFUS_KALMAN_EVOLUTION_MODEL_H

#include <Eigen/Dense>
#include <cmath>

#include "sensfus/types.h"

namespace sensfus {
namespace kalman {
template <size_t Dim>
struct EvolutionModel {
  static_assert(Dim == 2 || Dim == 3,
                "EvolutionModel only supported for 2D and 3D");

  static constexpr size_t kDim =
      Dim * 3;  // State vector size: position, velocity, acceleration

  const std::vector<Eigen::Matrix<ScalarType, kDim, kDim>> kF;
  const std::vector<Eigen::Matrix<ScalarType, kDim, kDim>> kD;

  // true if F and D are multidimensional e.g. support multiple states
  const bool kSupport_mh_sates;
  const double kT_seconds;             // Time step in seconds
  const double kProcessNoiseVariance;  // Process noise variance for the model

  // Standard Constructor uses piecewise constant white acceleration Model
  EvolutionModel(const double time_step_seconds = 0.05,
                 const double noise_variance = 0.01,
                 const bool support_multidimensional_states = false)
      : kSupport_mh_sates(support_multidimensional_states),
        kT_seconds(time_step_seconds),
        kProcessNoiseVariance(noise_variance) {
    // Evolution Model -> F(I Tk 0.5*Tk^2, 0 I Tk, 0 0 I)

    Eigen::Matrix<ScalarType, kDim, kDim> F;
    Eigen::Matrix<ScalarType, Dim, Dim> I;

    F.setZero();

    F.block<Dim, Dim>(0, 0) = I;                 // Identity matrix for position
    F.block<Dim, Dim>(0, Dim) = kT_seconds * I;  // Position to velocity
    F.block<Dim, Dim>(0, 2 * Dim) =
        0.5 * kT_seconds * kT_seconds * I;  // Position to acceleration
    F.block<Dim, Dim>(Dim, Dim) = I;        // Identity matrix for velocity
    F.block<Dim, Dim>(Dim, 2 * Dim) =
        kT_seconds * I;  // Velocity to acceleration
    F.block<Dim, Dim>(2 * Dim, 2 * Dim) =
        I;  // Identity matrix for acceleration

    kF = {F};

    // Constant Acceleration Rates D = noise^2(complecated stuff)
    Eigen::Matrix<ScalarType, kDim, kDim> D;
    D.setZero();

    double q = kProcessNoiseVariance * kProcessNoiseVariance;
    D.block<Dim, Dim>(0, 0) = 0.25 * std::pow(kT_seconds, 4) * q * I;
    D.block<Dim, Dim>(0, Dim) = 0.5 * std::pow(kT_seconds, 3) * q * I;
    D.block<Dim, Dim>(0, 2 * Dim) = 0.5 * kT_seconds * kT_seconds * q * I;

    D.block<Dim, Dim>(Dim, 0) = 0.5 * std::pow(kT_seconds, 3) * q * I;
    D.block<Dim, Dim>(Dim, Dim) = kT_seconds * kT_seconds * q * I;
    D.block<Dim, Dim>(Dim, 2 * Dim) = kT_seconds * q * I;

    D.block<Dim, Dim>(2 * Dim, 0) = 0.5 * kT_seconds * kT_seconds * q * I;
    D.block<Dim, Dim>(2 * Dim, Dim) = kT_seconds * q * I;
    D.block<Dim, Dim>(2 * Dim, 2 * Dim) = q * I;

    kD = {D};
  }
};

}  // namespace kalman
}  // namespace sensfus

#endif  // SENSFUS_KALMAN_EVOLUTION_MODEL_H