// Test-only emitter for exercising the exact estimator event writer under ROS.
#include "session_gravity.h"
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 4) return 1;
  try {
    auto& recorder = session_gravity::Recorder::instance();
    recorder.closeForReset();  // First initialization performs an ordinary Reset().
    recorder.initialized(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
    std::string control;
    std::getline(std::cin, control);
    if (control == "reset") recorder.closeForReset();
  } catch (const session_gravity::Error& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
  return 0;
}
