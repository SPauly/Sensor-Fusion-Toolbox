#include <gtest/gtest.h>
#include <sensfus/sim/internal/trajectory_impl.h>
#include <sensfus/types.h>

namespace sensfus {
namespace sim {
namespace sensfus_test {

using sensfus::internal::TrajectoryImpl;

TEST(TrajectoryTest, ConstructFromLineVector2D) {
  std::vector<ObjectPosition2D> points = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished(),
      (ObjectPosition2D() << 5.0, 6.0).finished()};
  TrajectoryImpl<ObjectState2D> traj(points);

  ASSERT_EQ(traj.GetSize(), 3);
  auto state0 = traj.GetState(0);
  EXPECT_DOUBLE_EQ(state0(0), 1.0);
  EXPECT_DOUBLE_EQ(state0(1), 2.0);

  auto state2 = traj.GetState(2);
  EXPECT_DOUBLE_EQ(state2(0), 5.0);
  EXPECT_DOUBLE_EQ(state2(1), 6.0);
}

TEST(TrajectoryTest, ConstructFromLineVector3D) {
  std::vector<ObjectState3D> points = {
      (ObjectState3D() << 1.0, 2.0, 3.0).finished(),
      (ObjectState3D() << 4.0, 5.0, 6.0).finished()};
  TrajectoryImpl<ObjectState3D> traj(points);

  ASSERT_EQ(traj.GetSize(), 2);
  auto state0 = traj.GetState(0);
  EXPECT_DOUBLE_EQ(state0(0), 1.0);
  EXPECT_DOUBLE_EQ(state0(1), 2.0);
  EXPECT_DOUBLE_EQ(state0(2), 3.0);

  auto state1 = traj.GetState(1);
  EXPECT_DOUBLE_EQ(state1(0), 4.0);
  EXPECT_DOUBLE_EQ(state1(1), 5.0);
  EXPECT_DOUBLE_EQ(state1(2), 6.0);
}

TEST(TrajectoryTest, GetStateOutOfBoundsReturnsLast) {
  std::vector<ObjectPosition2D> points = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished()};
  TrajectoryImpl<ObjectState2D> traj(points);

  auto state = traj.GetState(100);  // Out of bounds
  EXPECT_DOUBLE_EQ(state(0), 3.0);
  EXPECT_DOUBLE_EQ(state(1), 4.0);
}

TEST(TrajectoryTest, FromLineVectorReplacesStates) {
  std::vector<ObjectPosition2D> points1 = {
      (ObjectPosition2D() << 1.0, 2.0).finished(),
      (ObjectPosition2D() << 3.0, 4.0).finished()};
  TrajectoryImpl<ObjectState2D> traj(points1);

  std::vector<ObjectPosition2D> points2 = {
      (ObjectPosition2D() << 7.0, 8.0).finished()};
  traj.FromLineVector(points2);

  ASSERT_EQ(traj.GetSize(), 1);
  auto state = traj.GetState(0);
  EXPECT_DOUBLE_EQ(state(0), 7.0);
  EXPECT_DOUBLE_EQ(state(1), 8.0);
}

}  // namespace sensfus_test
}  // namespace sim
}  // namespace sensfus

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}