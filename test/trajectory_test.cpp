#include <gtest/gtest.h>
#include <sensfus/sim/trajectory.h>
#include <sensfus/types.h>

namespace sensfus {
namespace sim {
namespace sensfus_test {

TEST(TrajectoryTest, ConstructFromLineVector2D) {
  std::vector<ObjectPosition2D> points = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished(),
      (ObjectPosition2D() << 5.0, 6.0).finished()};
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
  std::vector<ObjectPosition3D> points = {
      (ObjectPosition3D() << 1.0, 2.0, 3.0).finished(),
      (ObjectPosition3D() << 4.0, 5.0, 6.0).finished()};
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
  std::vector<ObjectPosition2D> points = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished()};
  Trajectory<ObjectState2D> traj(points);

  auto state = traj.GetState(100);  // Out of bounds
  EXPECT_FLOAT_EQ(state(0), 3.0f);
  EXPECT_FLOAT_EQ(state(1), 4.0f);
}

TEST(TrajectoryTest, FromLineVectorReplacesStates) {
  std::vector<ObjectPosition2D> points1 = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished()};
  Trajectory<ObjectState2D> traj(points1);

  std::vector<ObjectPosition2D> points2 = {
      (ObjectPosition2D() << 7.0, 8.0).finished()};
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