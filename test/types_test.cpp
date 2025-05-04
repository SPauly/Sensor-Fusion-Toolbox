#include <gtest/gtest.h>
#include <sensfus/types.h>

namespace sensfus {
namespace utils {
namespace sensfus_test {

TEST(ObjectStateWrapperTest, DimensionConstant2D) {
  using Wrapper2D = ObjectStateWrapper<ObjectState2D>;
  static_assert(Wrapper2D::kDim == 2, "kDim should be 2 for 2D");
  EXPECT_EQ(Wrapper2D::kDim, 2);
}

TEST(ObjectStateWrapperTest, DimensionConstant3D) {
  using Wrapper3D = ObjectStateWrapper<ObjectState3D>;
  static_assert(Wrapper3D::kDim == 3, "kDim should be 3 for 3D");
  EXPECT_EQ(Wrapper3D::kDim, 3);
}

TEST(ObjectStateWrapperTest, PositionVelocityAccelerationTypes2D) {
  using Wrapper2D = ObjectStateWrapper<ObjectState2D>;
  static_assert(
      std::is_same<typename Wrapper2D::ObjectPosition, ObjectPosition2D>::value,
      "Position type should be ObjectPosition2D");
  static_assert(
      std::is_same<typename Wrapper2D::ObjectVelocity, ObjectVelocity2D>::value,
      "Velocity type should be ObjectVelocity2D");
  static_assert(std::is_same<typename Wrapper2D::ObjectAcceleration,
                             ObjectAcceleration2D>::value,
                "Acceleration type should be ObjectAcceleration2D");
}

TEST(ObjectStateWrapperTest, PositionVelocityAccelerationTypes3D) {
  using Wrapper3D = ObjectStateWrapper<ObjectState3D>;
  static_assert(
      std::is_same<typename Wrapper3D::ObjectPosition, ObjectPosition3D>::value,
      "Position type should be ObjectPosition3D");
  static_assert(
      std::is_same<typename Wrapper3D::ObjectVelocity, ObjectVelocity3D>::value,
      "Velocity type should be ObjectVelocity3D");
  static_assert(std::is_same<typename Wrapper3D::ObjectAcceleration,
                             ObjectAcceleratio3D>::value,
                "Acceleration type should be ObjectAcceleratio3D");
}

TEST(ObjectStateWrapperTest, StateSetAndGet2D) {
  ObjectState2D state;
  state << 1, 2, 3, 4, 5, 6;
  ObjectStateWrapper<ObjectState2D> wrapper;
  wrapper.SetState(state);
  ObjectState2D retrieved = wrapper.GetState();
  for (int i = 0; i < state.size(); ++i) {
    EXPECT_EQ(retrieved(i), state(i));
  }
}

TEST(ObjectStateWrapperTest, StateSetAndGet3D) {
  ObjectState3D state;
  state << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  ObjectStateWrapper<ObjectState3D> wrapper;
  wrapper.SetState(state);
  ObjectState3D retrieved = wrapper.GetState();
  for (int i = 0; i < state.size(); ++i) {
    EXPECT_EQ(retrieved(i), state(i));
  }
}

TEST(ObjectStateWrapperTest, PositionVelocityAccelerationAccess2D) {
  ObjectState2D state;
  state << 1, 2, 3, 4, 5, 6;
  ObjectStateWrapper<ObjectState2D> wrapper(state);

  auto pos = wrapper.GetPosition();
  auto vel = wrapper.GetVelocity();
  auto acc = wrapper.GetAcceleration();

  EXPECT_EQ(pos(0), 1.0f);
  EXPECT_EQ(pos(1), 2.0f);

  EXPECT_EQ(vel(0), 3.0f);
  EXPECT_EQ(vel(1), 4.0f);

  EXPECT_EQ(acc(0), 5.0f);
  EXPECT_EQ(acc(1), 6.0f);
}

TEST(ObjectStateWrapperTest, PositionVelocityAccelerationAccess3D) {
  ObjectState3D state;
  state << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  ObjectStateWrapper<ObjectState3D> wrapper(state);

  auto pos = wrapper.GetPosition();
  auto vel = wrapper.GetVelocity();
  auto acc = wrapper.GetAcceleration();

  EXPECT_EQ(pos(0), 1.0f);
  EXPECT_EQ(pos(1), 2.0f);
  EXPECT_EQ(pos(2), 3.0f);

  EXPECT_EQ(vel(0), 4.0f);
  EXPECT_EQ(vel(1), 5.0f);
  EXPECT_EQ(vel(2), 6.0f);

  EXPECT_EQ(acc(0), 7.0f);
  EXPECT_EQ(acc(1), 8.0f);
  EXPECT_EQ(acc(2), 9.0f);
}

}  // namespace sensfus_test
}  // namespace utils
}  // namespace sensfus

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}