#include <pin_ik/pin_ik.hpp>
#include <cerrno>
#include <system_error>

#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
#include <pthread.h>
#include <sched.h>
#endif

namespace PIN_IK {
namespace {
void checkThreadError(int error, const char* operation) {
  // pthread routines return the error number; they do not set errno.
  if (error) throw std::system_error(error, std::generic_category(), operation);
}
}

std::thread& PIN_IK::workerThread(WorkerThread worker) {
  switch (worker) {
    case WorkerThread::Newton: return task1;
    case WorkerThread::Nlopt: return task2;
  }
  throw std::invalid_argument("Invalid IK worker");
}

void PIN_IK::setWorkerThreadAffinity(WorkerThread worker, const std::vector<int>& cpus) {
  auto& thread = workerThread(worker);
  std::lock_guard<std::mutex> lock(worker_mutex);
#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
  if (cpus.empty()) throw std::invalid_argument("Worker CPU set must not be empty");
  cpu_set_t mask;
  CPU_ZERO(&mask);
  for (int cpu : cpus) {
    if (cpu < 0 || cpu >= CPU_SETSIZE) throw std::invalid_argument("Worker CPU ID out of range");
    CPU_SET(cpu, &mask);
  }
  checkThreadError(pthread_setaffinity_np(thread.native_handle(), sizeof(mask), &mask),
                   "set IK worker affinity");
#else
  (void)thread; (void)cpus;
  throw std::system_error(std::make_error_code(std::errc::not_supported), "set IK worker affinity");
#endif
}

void PIN_IK::setWorkerThreadScheduling(WorkerThread worker, int policy, int priority) {
  auto& thread = workerThread(worker);
  std::lock_guard<std::mutex> lock(worker_mutex);
#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
  const int minimum = sched_get_priority_min(policy);
  const int maximum = sched_get_priority_max(policy);
  if (minimum < 0 || maximum < 0 || priority < minimum || priority > maximum)
    throw std::invalid_argument("Invalid IK worker scheduling policy/priority");
  sched_param parameters{};
  parameters.sched_priority = priority;
  checkThreadError(pthread_setschedparam(thread.native_handle(), policy, &parameters),
                   "set IK worker scheduling");
#else
  (void)thread; (void)policy; (void)priority;
  throw std::system_error(std::make_error_code(std::errc::not_supported), "set IK worker scheduling");
#endif
}

WorkerThreadSettings PIN_IK::getWorkerThreadSettings(WorkerThread worker) {
  auto& thread = workerThread(worker);
  std::lock_guard<std::mutex> lock(worker_mutex);
#if defined(__linux__) && !defined(PIN_IK_NO_THREAD_CONFIG)
  WorkerThreadSettings settings;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  sched_param parameters{};
  checkThreadError(pthread_getaffinity_np(thread.native_handle(), sizeof(mask), &mask),
                   "get IK worker affinity");
  checkThreadError(pthread_getschedparam(thread.native_handle(), &settings.policy, &parameters),
                   "get IK worker scheduling");
  settings.priority = parameters.sched_priority;
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu)
    if (CPU_ISSET(cpu, &mask)) settings.cpus.push_back(cpu);
  return settings;
#else
  (void)thread;
  throw std::system_error(std::make_error_code(std::errc::not_supported), "get IK worker settings");
#endif
}
} // namespace PIN_IK
