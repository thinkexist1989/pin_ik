#ifndef PIN_IK_SOLVE_DEADLINE_HPP
#define PIN_IK_SOLVE_DEADLINE_HPP

#include <chrono>

namespace PIN_IK { namespace detail {
using Clock = std::chrono::steady_clock;

inline Clock::time_point deadlineAfter(Clock::time_point start, double seconds) {
  if (!(seconds > 0)) return start;
  const auto available = Clock::time_point::max() - start;
  if (seconds >= std::chrono::duration<double>(available).count())
    return Clock::time_point::max();
  return start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}
}} // namespace PIN_IK::detail

#endif
