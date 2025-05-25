#ifndef SENSFUS_SIM_MATH_H
#define SENSFUS_SIM_MATH_H

#include "sensfus/types.h"

namespace sensfus {
#define M_PI 3.14159265358979323846

namespace utils {
/// @brief Fast modulo operation that returns the remainder of the input
/// when divided by the ceiling value. If the input is less than the ceiling,
/// nothing is done.
/// @param input Input value to be modified.
/// @param ceil Ceiling value to which the input is compared.
/// @return modulo of the input with respect to the ceiling value.
/// @details This function is a modified version of the proposed fast_mod
/// function by Chandler Carruth https://youtu.be/nXaxk27zwlk?t=56m34s
template <typename T = TimeStepIdType>
inline T fast_mod(const T input, const T ceil) {
  static_assert(std::is_integral<T>::value,
                "T must be an integral type supporting < and % operators");
  // apply the modulo operator only when needed
  // (i.e. when the input is greater than the ceiling)
  return input < ceil ? input : input % ceil;
}

}  // namespace utils

}  // namespace sensfus

#endif  // SENSFUS_SIM_MATH_H