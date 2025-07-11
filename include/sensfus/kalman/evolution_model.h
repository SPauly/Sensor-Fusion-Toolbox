#ifndef SENSFUS_KALMAN_EVOLUTION_MODEL_H
#define SENSFUS_KALMAN_EVOLUTION_MODEL_H

#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "sensfus/types.h"

namespace sensfus {
namespace kalman {
template <size_t Dim>
class MultipleModels;  // Forward declaration

template <size_t Dim>
class EvolutionModel {
  static_assert(Dim == 2 || Dim == 3,
                "EvolutionModel only supported for 2D and 3D");

 public:
  static constexpr size_t kDim =
      Dim * 3;  // Dim*3 bc: State vector size: pos, vel, acc

  EvolutionModel(double time_step_seconds = 0.05, double noise_variance = 0.01,
                 bool support_multi_models = false)
      : kT_seconds_(time_step_seconds),
        kProcessNoiseVariance_(noise_variance),
        kSupport_mh_states_(support_multi_models) {
    UpdateMatrices();
  }

  void SetDeltaTime(double time_step_seconds) {
    if (time_step_seconds == kT_seconds_) {
      return;  // No change needed
    }
    kT_seconds_ = time_step_seconds;
    UpdateMatrices();
  }

  void SetProcessNoiseVariance(double noise_variance) {
    kProcessNoiseVariance_ = noise_variance;
    UpdateMatrices();
  }

  double GetDeltaTime() const { return kT_seconds_; }
  double GetProcessNoiseVariance() const { return kProcessNoiseVariance_; }
  bool SupportsMultiModels() const { return kSupport_mh_states_; }

  const Eigen::Matrix<ScalarType, kDim, kDim>& F() const { return F_; }
  const Eigen::Matrix<ScalarType, kDim, kDim>& D() const { return D_; }

 private:
  void UpdateMatrices() {
    // Evolution Model -> F_(I Tk 0.5*Tk^2, 0 I Tk, 0 0 I)
    Eigen::Matrix<ScalarType, Dim, Dim> I =
        Eigen::Matrix<ScalarType, Dim, Dim>::Identity();  // Identity matrix

    F_.setZero();

    F_.block<Dim, Dim>(0, 0) = I;  // Identity matrix for position
    F_.block<Dim, Dim>(0, Dim) = kT_seconds_ * I;  // Position to velocity
    F_.block<Dim, Dim>(0, 2 * Dim) =
        0.5 * kT_seconds_ * kT_seconds_ * I;  // Position to acceleration
    F_.block<Dim, Dim>(Dim, Dim) = I;         // Identity matrix for velocity
    F_.block<Dim, Dim>(Dim, 2 * Dim) =
        kT_seconds_ * I;  // Velocity to acceleration
    F_.block<Dim, Dim>(2 * Dim, 2 * Dim) =
        I;  // Identity matrix for acceleration

    // Constant Acceleration Rates D = noise^2(complecated stuff)
    D_.setZero();

    double q = kProcessNoiseVariance_ * kProcessNoiseVariance_;
    D_.block<Dim, Dim>(0, 0) = 0.25 * std::pow(kT_seconds_, 4) * q * I;
    D_.block<Dim, Dim>(0, Dim) = 0.5 * std::pow(kT_seconds_, 3) * q * I;
    D_.block<Dim, Dim>(0, 2 * Dim) = 0.5 * kT_seconds_ * kT_seconds_ * q * I;

    D_.block<Dim, Dim>(Dim, 0) = 0.5 * std::pow(kT_seconds_, 3) * q * I;
    D_.block<Dim, Dim>(Dim, Dim) = kT_seconds_ * kT_seconds_ * q * I;
    D_.block<Dim, Dim>(Dim, 2 * Dim) = kT_seconds_ * q * I;

    D_.block<Dim, Dim>(2 * Dim, 0) = 0.5 * kT_seconds_ * kT_seconds_ * q * I;
    D_.block<Dim, Dim>(2 * Dim, Dim) = kT_seconds_ * q * I;
    D_.block<Dim, Dim>(2 * Dim, 2 * Dim) = q * I;
  }

 private:
  double kT_seconds_;
  double kProcessNoiseVariance_;
  const bool kSupport_mh_states_;

  Eigen::Matrix<ScalarType, kDim, kDim> F_;  // State transition matrix
  Eigen::Matrix<ScalarType, kDim, kDim> D_;  // Evolution covariance matrix

  MultipleModels<Dim> states_;  // Multiple models for MM states
};

/// @brief Class to handle multiple models for the evolution model.
/// This class is used to store the next states with their probabilities.
/// TODO: Implement this class to handle multiple models for the evolution
template <size_t Dim>
class MultipleModels {
 private:
  // if kSupport_mh_states is true, then we need to store the next states
  // together with the probability of the next state
  std::vector<std::pair<EvolutionModel<Dim>, double>>
      next_states;  // Next states with their probabilities (probabilities sum
};

}  // namespace kalman
}  // namespace sensfus

#endif  // SENSFUS_KALMAN_EVOLUTION_MODEL_H