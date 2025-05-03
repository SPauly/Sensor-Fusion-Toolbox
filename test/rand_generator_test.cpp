#include <gtest/gtest.h>

#include <numeric>
#include <eigen3/Eigen/Dense>

#include "sensfus/utils/rand_generator.h"

namespace sensfus {
namespace utils {
namespace sensfus_test {

constexpr std::size_t DIM = 4;

using Scalar = double;
using Generator = StdNormalGenerator<Scalar, DIM>;
using VectorType = Eigen::Matrix<Scalar, DIM, 1>;

TEST(StdNormalGeneratorTest, GeneratesCorrectSizeVectors) {
  Generator gen;
  VectorType sample = gen.sample();
  EXPECT_EQ(sample.size(), DIM);
}

TEST(StdNormalGeneratorTest, MultipleSamplesHaveCorrectSize) {
  Generator gen;
  auto samples = gen.sampleMany(10);
  ASSERT_EQ(samples.size(), 10);
  for (const auto& vec : samples) {
    EXPECT_EQ(vec.size(), DIM);
  }
}

TEST(StdNormalGeneratorTest, RoughlyStandardNormalDistribution) {
  Generator gen;
  constexpr std::size_t N = 10000;
  auto samples = gen.sampleMany(N);

  Eigen::Matrix<Scalar, DIM, 1> mean = VectorType::Zero();
  Eigen::Matrix<Scalar, DIM, 1> var = VectorType::Zero();

  for (const auto& v : samples) {
    mean += v;
  }
  mean /= static_cast<Scalar>(N);

  for (const auto& v : samples) {
    var += (v - mean).cwiseAbs2();
  }
  var /= static_cast<Scalar>(N);

  // Check mean is near 0 and variance is near 1 for each dimension
  for (std::size_t i = 0; i < DIM; ++i) {
    EXPECT_NEAR(mean[i], 0.0, 0.1);  // Accept small statistical error
    EXPECT_NEAR(var[i], 1.0, 0.1);
  }
}

}  // namespace sensfus_test
}  // namespace utils
}  // namespace sensfus

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}