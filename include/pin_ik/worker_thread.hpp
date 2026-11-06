#ifndef PIN_IK_WORKER_THREAD_HPP
#define PIN_IK_WORKER_THREAD_HPP

#include <vector>

namespace PIN_IK {

enum class WorkerThread { Newton = 0, Nlopt = 1 };

struct WorkerThreadSettings {
  std::vector<int> cpus;
  // Native operating-system policy value (e.g. Linux SCHED_OTHER/SCHED_FIFO).
  int policy = 0;
  int priority = 0;
};

} // namespace PIN_IK
#endif
