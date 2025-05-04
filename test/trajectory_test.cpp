#include <gtest/gtest.h>
#include <sensfus/sim/trajectory.h>

namespace sensfus {
namespace sim {
namespace sensfus_test {

TEST(TrajectoryTest, ConstructFromLineVector2D) {
  std::vector<SensVec2D> points = {{1.0f, 2.0f}, {3.0f, 4.0f}, {5.0f, 6.0f}};
  Trajectory<ObjectState2D> traj(points);

  ASSERT_EQ(traj.GetSize(), 3);
  auto state0 = traj.GetState(0);
  EXPECT_FLOAT_EQ(state0(0), 1.0f);
  EXPECT_FLOAT_EQ(state0(1), 2.0f);

  auto state2 = traj.GetState(2);
  EXPECT_FLOAT_EQ(state2(0), 5.0f);
  EXPECT_FLOAT_EQ(state2(1), 6.0f);
}

TEST(TrajectoryTest, ConstructFromLineVector3D) {
  std::vector<SensVec3D> points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
  Trajectory<ObjectState3D> traj(points);

  ASSERT_EQ(traj.GetSize(), 2);
  auto state0 = traj.GetState(0);
  EXPECT_FLOAT_EQ(state0(0), 1.0f);
  EXPECT_FLOAT_EQ(state0(1), 2.0f);
  EXPECT_FLOAT_EQ(state0(2), 3.0f);

  auto state1 = traj.GetState(1);
  EXPECT_FLOAT_EQ(state1(0), 4.0f);
  EXPECT_FLOAT_EQ(state1(1), 5.0f);
  EXPECT_FLOAT_EQ(state1(2), 6.0f);
}

TEST(TrajectoryTest, GetStateOutOfBoundsReturnsLast) {
  std::vector<SensVec2D> points = {{1.0f, 2.0f}, {3.0f, 4.0f}};
  Trajectory<ObjectState2D> traj(points);

  auto state = traj.GetState(100);  // Out of bounds
  EXPECT_FLOAT_EQ(state(0), 3.0f);
  EXPECT_FLOAT_EQ(state(1), 4.0f);
}

TEST(TrajectoryTest, FromLineVectorReplacesStates) {
  std::vector<SensVec2D> points1 = {{1.0f, 2.0f}, {3.0f, 4.0f}};
  Trajectory<ObjectState2D> traj(points1);

  std::vector<SensVec2D> points2 = {{7.0f, 8.0f}};
  traj.FromLineVector(points2);

  ASSERT_EQ(traj.GetSize(), 1);
  auto state = traj.GetState(0);
  EXPECT_FLOAT_EQ(state(0), 7.0f);
  EXPECT_FLOAT_EQ(state(1), 8.0f);
}

}  // namespace sensfus_test
}  // namespace sim
}  // namespace sensfus

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}