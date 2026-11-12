/*
 * Optional wall-clock profiler for per-physics-step plugin cost.
 *
 * Disabled unless the environment variable NEZHA_PROFILE_DIR names a directory.
 * When enabled, every timed scope appends its duration in microseconds (float32)
 * to <NEZHA_PROFILE_DIR>/<tag>_<pid>_<n>.f32. Samples are buffered and written
 * outside the timed scope, so file I/O is not counted.
 */
#ifndef NEZHA_STEP_PROFILER_HH
#define NEZHA_STEP_PROFILER_HH

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace nezha {

class StepProfiler {
 public:
  explicit StepProfiler(const std::string& tag) {
    const char* dir = std::getenv("NEZHA_PROFILE_DIR");
    if (dir == nullptr || *dir == '\0') return;
    static std::atomic<int> counter{0};
    path_ = std::string(dir) + "/" + tag + "_" + std::to_string(getpid()) +
            "_" + std::to_string(counter++) + ".f32";
    buffer_.reserve(kFlushSize);
    enabled_ = true;
  }
  ~StepProfiler() { Flush(); }

  bool Enabled() const { return enabled_; }

  void Add(float microseconds) {
    buffer_.push_back(microseconds);
    if (buffer_.size() >= kFlushSize) Flush();
  }

  void Flush() {
    if (!enabled_ || buffer_.empty()) return;
    if (FILE* f = std::fopen(path_.c_str(), "ab")) {
      std::fwrite(buffer_.data(), sizeof(float), buffer_.size(), f);
      std::fclose(f);
    }
    buffer_.clear();
  }

 private:
  static constexpr size_t kFlushSize = 2000;
  bool enabled_ = false;
  std::string path_;
  std::vector<float> buffer_;
};

// Times the enclosing scope and records it when the scope ends.
class ScopedStepTimer {
 public:
  explicit ScopedStepTimer(StepProfiler& profiler) : profiler_(profiler) {
    if (profiler_.Enabled()) start_ = std::chrono::steady_clock::now();
  }
  ~ScopedStepTimer() {
    if (!profiler_.Enabled()) return;
    const auto stop = std::chrono::steady_clock::now();
    profiler_.Add(std::chrono::duration<float, std::micro>(stop - start_).count());
  }

 private:
  StepProfiler& profiler_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace nezha

#endif  // NEZHA_STEP_PROFILER_HH
