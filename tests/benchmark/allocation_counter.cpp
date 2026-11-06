// Linux/glibc-only diagnostic preload. Never use this instrumentation for timing.
#include <atomic>
#include <cstddef>
extern "C" void* __libc_malloc(size_t);
extern "C" void* __libc_calloc(size_t, size_t);
extern "C" void* __libc_realloc(void*, size_t);
namespace {
std::atomic<bool> enabled{false};
std::atomic<unsigned long> count{0};
void record() { if (enabled.load(std::memory_order_relaxed)) count.fetch_add(1, std::memory_order_relaxed); }
}
extern "C" void allocation_begin() { count = 0; enabled = true; }
extern "C" unsigned long allocation_end() { enabled = false; return count.load(); }
extern "C" void* malloc(size_t size) { record(); return __libc_malloc(size); }
extern "C" void* calloc(size_t n, size_t size) { record(); return __libc_calloc(n, size); }
extern "C" void* realloc(void* p, size_t size) { record(); return __libc_realloc(p, size); }
