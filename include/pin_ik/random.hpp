#ifndef PIN_IK_RANDOM_HPP
#define PIN_IK_RANDOM_HPP

#include <cstdint>
#include <random>

namespace PIN_IK { namespace detail {
// Owned by one solver/worker. Entropy is obtained only during construction.
class RandomGenerator {
public:
  RandomGenerator() : engine(initialSeed()) {}
  explicit RandomGenerator(std::uint64_t seed) : engine(seed) {}
  void seed(std::uint64_t value) { engine.seed(value); }
  double uniform(double lower, double upper) {
    const double unit = static_cast<double>(engine() >> 11) * 0x1.0p-53;
    return lower + unit * (upper - lower);
  }
private:
  static std::uint64_t initialSeed() {
    std::random_device source;
    return (static_cast<std::uint64_t>(source()) << 32) ^ source();
  }
  std::mt19937_64 engine;
};
}} // namespace PIN_IK::detail

#endif
