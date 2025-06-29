#ifndef SENSFUS_UTILS_TIME_H
#define SENSFUS_UTILS_TIME_H

#include <chrono>
#include <cstdint>
#include <cmath>

namespace sensfus {
namespace utils {

class Time {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  // Allowed error in nanoseconds for equality
  static constexpr int64_t kDefaultAllowedErrorNs = 1000;  // 1 microsecond

  Time() : time_point_(Clock::now()) {}
  explicit Time(TimePoint tp) : time_point_(tp) {}

  // Returns current time as Time object
  static Time now() { return Time(Clock::now()); }

  // Returns time in nanoseconds since epoch (as int64_t)
  int64_t toNanoseconds() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               time_point_.time_since_epoch())
        .count();
  }

  // Returns the difference between two Time objects in seconds (as double)
  double operator-(const Time& other) const {
    auto diff_ns = toNanoseconds() - other.toNanoseconds();
    return static_cast<double>(diff_ns) * 1e-9;
  }

  // Equality operator with allowed error
  bool operator==(const Time& other) const {
    return std::abs(toNanoseconds() - other.toNanoseconds()) <=
           allowed_error_ns_;
  }

  // Inequality operator
  bool operator!=(const Time& other) const { return !(*this == other); }

  // Set allowed error in nanoseconds for equality
  static void setAllowedErrorNs(int64_t error_ns) {
    allowed_error_ns_ = error_ns;
  }

  // Get allowed error in nanoseconds
  static int64_t getAllowedErrorNs() { return allowed_error_ns_; }

 private:
  TimePoint time_point_;
  static int64_t allowed_error_ns_;
};

// Definition of static member
inline int64_t Time::allowed_error_ns_ = Time::kDefaultAllowedErrorNs;

}  // namespace utils
}  // namespace sensfus

#endif  // SENSFUS_UTILS_TIME_H