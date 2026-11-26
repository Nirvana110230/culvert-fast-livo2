#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

inline int64_t timestamp_monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline const char* timestamp_environment(const char* name) {
  const char* value = std::getenv(name);
  if (!value || !*value) throw std::runtime_error(std::string("Missing ") + name);
  return value;
}

// Official high/low layout. The supervisor creates a fresh zeroed file per run.
class SharedTimestamp {
 public:
  struct Layout { uint64_t high; uint64_t low; };
  static_assert(sizeof(Layout) == 16, "Official timeshare layout must be 16 bytes");
  static_assert(__atomic_always_lock_free(8, nullptr), "Require lock-free 64-bit access");

  SharedTimestamp(const char* path, bool writable) {
    int fd = open(path, writable ? O_RDWR : O_RDONLY);
    if (fd < 0) throw std::system_error(errno, std::generic_category(), "open timeshare");
    struct stat status {};
    if (fstat(fd, &status) != 0 || status.st_size != sizeof(Layout)) {
      close(fd);
      throw std::runtime_error("timeshare must be a fresh 16-byte file");
    }
    void* memory = mmap(nullptr, sizeof(Layout),
                        writable ? PROT_READ | PROT_WRITE : PROT_READ,
                        MAP_SHARED, fd, 0);
    int map_error = errno;
    close(fd);
    if (memory == MAP_FAILED)
      throw std::system_error(map_error, std::generic_category(), "mmap timeshare");
    value_ = static_cast<Layout*>(memory);
  }

  ~SharedTimestamp() { munmap(value_, sizeof(Layout)); }
  SharedTimestamp(const SharedTimestamp&) = delete;
  SharedTimestamp& operator=(const SharedTimestamp&) = delete;

  uint64_t load() const { return __atomic_load_n(&value_->low, __ATOMIC_ACQUIRE); }
  void store(uint64_t stamp) { __atomic_store_n(&value_->low, stamp, __ATOMIC_RELEASE); }
  uint64_t read_valid() const {
    uint64_t stamp = load();
    if (stamp == 0) throw std::runtime_error("No LiDAR scan in this run's timeshare");
    return stamp;
  }

 private:
  Layout* value_;
};
