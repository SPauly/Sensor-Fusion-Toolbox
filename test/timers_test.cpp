#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "sensfus/utils/timers.h"

namespace sensfus {
namespace utils {
namespace sensfus_test {

class RateTimerTest : public ::testing::Test {};

TEST_F(RateTimerTest, WaitsForSpecifiedDuration) {
  using namespace std::chrono;
  auto wait_duration = milliseconds(100);

  RateTimer timer(wait_duration);
  auto start = steady_clock::now();
  timer.WaitRemaining();
  auto end = steady_clock::now();

  auto elapsed = duration_cast<milliseconds>(end - start);
  EXPECT_GE(elapsed.count(),
            wait_duration.count() - 2);  // allow small scheduling error
}

TEST_F(RateTimerTest, ReturnsImmediatelyIfAlreadyExpired) {
  using namespace std::chrono;
  auto wait_duration = milliseconds(50);

  RateTimer timer(wait_duration);
  std::this_thread::sleep_for(wait_duration + milliseconds(20));
  auto start = steady_clock::now();
  timer.WaitRemaining();
  auto end = steady_clock::now();

  auto elapsed = duration_cast<milliseconds>(end - start);
  EXPECT_LT(elapsed.count(), 10);  // Should return almost immediately
}

TEST_F(RateTimerTest, GetElapsedAndRemaining) {
  using namespace std::chrono;
  auto wait_duration = milliseconds(80);

  RateTimer timer(wait_duration);
  std::this_thread::sleep_for(milliseconds(30));
  auto elapsed = duration_cast<milliseconds>(timer.GetElapsed());
  auto remaining = duration_cast<milliseconds>(timer.GetRemaining());

  EXPECT_GE(elapsed.count(), 28);
  EXPECT_LE(remaining.count(), 52);
  EXPECT_NEAR(elapsed.count() + remaining.count(), wait_duration.count(), 5);
}
}  // namespace sensfus_test
}  // namespace utils
}  // namespace sensfus

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}