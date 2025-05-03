#ifndef SENSFUS_UTILS_RAND_GENERATOR_H
#define SENSFUS_UTILS_RAND_GENERATOR_H

#include <array>
#include <random>
#include <chrono>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace sensfus {
namespace utils {

/// @brief Base class for random number generators.
/// @tparam Derived Using CRTP to allow derived classes to implement the
/// sampleImpl method.
/// @tparam Scalar Type of the resulting vector used for sampling.
/// @tparam Size size of the resulting vector used for sampling.
template <typename Derived, typename Scalar = double, std::size_t Size = 2>
class RandGenerator {
 public:
  using VectorType = Eigen::Matrix<Scalar, Size, 1>;

  /// @brief Populates a VectorType with random values and specified size.
  /// @return Random values specified by the used distribution.
  VectorType sample() { return static_cast<Derived*>(this)->sampleImpl(); }

  /// @brief Populates a raw array with random values and specified size.
  /// @return Random values specified by the used distribution.
  std::array<Scalar, Size> sample_raw_array() {
    return sample().template cast<Scalar>().data();
  }

  /// @brief Returns a single random value of the specified type.
  /// @return Random value specified by the used distribution.
  Scalar sample_raw() { return static_cast<Derived*>(this)->samplerawImpl(); }

  /// @brief Populates a std::vector with random values and specified size.
  /// @param n number of Types to be sampled.
  /// @return vector with n random number vectors
  std::vector<VectorType> sampleMany(std::size_t n) {
    std::vector<VectorType> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) out.push_back(sample());
    return out;
  }
};

/// @brief Standard Normal distribution generator.
/// @tparam Scalar Type of the resulting vector used for sampling.
/// @tparam Size Size of the resulting vector used for sampling.
template <typename Scalar = double, std::size_t Size = 2>
class StdNormalGenerator
    : public RandGenerator<StdNormalGenerator<Scalar, Size>, Scalar, Size> {
 public:
  using VectorType = Eigen::Matrix<Scalar, Size, 1>;

  StdNormalGenerator() : gen(std::random_device{}()), dist(0.0, 1.0) {}

 protected:
  VectorType sampleImpl() {
    VectorType v;
    for (std::size_t i = 0; i < Size; ++i) v[i] = dist(gen);
    return v;
  }

  Scalar samplerawImpl() { return dist(gen); }

 private:
  // Grant access to base class
  friend class RandGenerator<StdNormalGenerator<Scalar, Size>, Scalar, Size>;

  std::mt19937 gen;
  std::normal_distribution<Scalar> dist;
};

}  // namespace utils
}  // namespace sensfus

#endif  // SENSFUS_UTILS_RAND_GENERATOR_H