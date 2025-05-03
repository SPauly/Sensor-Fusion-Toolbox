#ifndef SENSFUS_UTILS_TIMERS_H
#define SENSFUS_UTILS_TIMERS_H

#include <chrono>
#include <thread>

namespace sensfus {
namespace utils {

/// @brief A simple timer class that allows for blocking until a specified
/// duration has passed. It can be used to implement rate limiting or to wait
/// for a specific time interval before executing a task. Usage: Instantiate the
/// RateTimer with a duration, and call WaitRemaining() to block until the
/// specified duration has passed.
class RateTimer {
 public:
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::steady_clock::duration;

  /// @brief Starts a timer that will block for the specified duration. When the
  /// call to WaitRemaining() is made
  /// @param total_wait Duration in Nanoseconds to wait
  explicit RateTimer(Duration total_wait)
      : total_wait_(total_wait), start_time_(Clock::now()) {}

  /// @brief Waits for the remaining time of the timer. If the timer has already
  /// expired, it returns immediately.
  void WaitRemaining() {
    auto now = Clock::now();
    auto elapsed = now - start_time_;
    if (elapsed < total_wait_) {
      std::this_thread::sleep_for(total_wait_ - elapsed);
    }
    // else: already exceeded, return immediately
  }

  /// @brief Gets the elapsed time since the timer was started.
  /// @return Elapsed time since the timer was started.
  Duration GetElapsed() const { return Clock::now() - start_time_; }

  /// @brief Gets the remaining time until the timer expires.
  /// @return Remaining time until the timer expires.
  Duration GetRemaining() const {
    auto now = Clock::now();
    auto elapsed = now - start_time_;
    if (elapsed < total_wait_) {
      return total_wait_ - elapsed;
    }
    return Duration(0);  // Timer has already expired
  }

 private:
  Duration total_wait_;
  Clock::time_point start_time_;
};

}  // namespace utils
}  // namespace sensfus

#endif  // SENSFUS_UTILS_TIMERS_H